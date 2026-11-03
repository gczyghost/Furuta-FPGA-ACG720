"""One serial owner; explicit commands, ACK deadlines, telemetry/UI watchdogs."""
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import serial
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools.operation_log import AsyncJournal
from .fpga_protocol import Calibration, BoundCalibrationProfile, MAGIC, StreamDecoder, SnapshotAssembler, encode
from .fpga_motion import fast_request,servo_request,position_request,demo_recipe,continuous_servo_target
from .fpga_reset import ResetFlow,InitializeFlow
from .stop_confirmation import StopConfirmation,expected_stop_epoch
from .legacy_servo_entry import LegacyServoEntry

def angle_error(angle, target=0.):
    """Physical angular distance; keep raw continuous telemetry unchanged."""
    return abs(math.remainder(angle-target, math.tau)) if math.isfinite(angle) else math.inf

class Session:
    def __init__(self,send,emit,clock=time.monotonic,persist_reset=None,profile_identity=None,manual_control=False,build14_entry_compat=False,persist_zero=None):
        self.send,self.emit,self.clock=send,emit,clock
        self.manual_control=bool(manual_control);self.manual_command_pending=False
        self.build14_entry_compat=bool(build14_entry_compat);self.entry_preparation=None;self.entry_dispatch=False
        self.decoder=StreamDecoder(); self.assembler=SnapshotAssembler(stopped_diagnostics=True); self.queue=deque()
        self.last_diagnostic=None;self.status_row=None;self.status_at=None;self.safety_stop_pending=False
        self.pending=None; self.seq=0; self.synchronized=False; self.failed=False; self.started=False
        self.opened=clock(); self.last_rx=self.opened; self.last_hb=0.; self.latest=None
        self.readback={}; self.expected=None; self.busy=False
        self.await_state=None
        self.device_words=None;self.commission=None;self.last_progress=0.;self.candidate={}
        self.drop_wait=None;self.last_home_poll=0.
        self.motion_status_addr=23;self.motion_mode=7;self.motion_timeout=4.5;self.motion_goal=0.
        self.cycle=None;self.quiet_since=None;self.cycle_drop=False
        self.capabilities={};self.motion_version=0
        self.motion=None;self.last_motion_poll=0.
        self._demo=None;self.demo_dispatch=False
        self.showcase=None
        self.zero_calibration=None;self.zero_dispatch=False;self.persist_zero=persist_zero
        self.reset=None;self.persist_reset=persist_reset
        self.initialization=None;self.profile_identity=dict(profile_identity or {})
        self.verified_reset_epoch=None
        self.stop_reset_epoch=None  # Exact +1 epoch established by a matched RST ACK, cleanup only.
        self.pbd1=None
        self.swing_context_serial=0  # This report-capture successor only.

    def _put(self,op,addr=0,value=None,tag='command'):
        self.queue.append((op,addr,MAGIC.get(op,0) if value is None else value,tag))

    def communication_status(self):
        if self.status_row is not None and self.clock()-self.status_at<=.25:return self.status_row
        if self.latest is not None and self.clock()-self.last_rx<=.25:return self.latest
        return None

    def operation_failed(self,text):
        self.stop()
        if not self.synchronized:self._put(4,tag='sync')
        self.emit('rejected',text=text+'；未确认成功，保持诊断连接')

    @property
    def demo(self):return self._demo
    @demo.setter
    def demo(self,value):
        previous=self._demo;self._demo=value
        if (previous is None)!=(value is None):
            self.emit('sequence_state',active=value is not None,
                      sequence='continuous' if (value or previous or {}).get('finish_upright') else 'demo')

    def stop(self,preserve_demo=False):
        if self.pbd1 is not None:self.pbd1.finish('UNKNOWN','STOP interrupted readout')
        if self.zero_calibration is not None:self.emit('sequence_state',active=False,sequence='zero_calibration')
        self.zero_calibration=None;self.zero_dispatch=False
        self.showcase=None
        self.entry_preparation=None;self.entry_dispatch=False
        self.reset=None
        self.assembler.expected_epoch=None
        self.manual_command_pending=False
        self.initialization=None
        if not preserve_demo:self.demo=None
        self.motion=None
        self.emit('servo',active=False,ending=False,rpm=0)
        self.drop_wait=None
        self.cycle=None;self.quiet_since=None;self.cycle_drop=False
        self.commission=None;self.candidate={};self.emit('candidate_invalid')
        self.queue.clear(); self.pending=None; self.expected=None; self.busy=True; self.await_state=None
        self._put(2,tag='stop')
        self.emit('log',text='STOP 已排队；等待回执和停止状态')

    def fail(self,text):
        if self.failed:return
        if self.pbd1 is not None:self.pbd1.finish('UNKNOWN',text)
        if self.zero_calibration is not None:self.emit('sequence_state',active=False,sequence='zero_calibration')
        self.zero_calibration=None;self.zero_dispatch=False
        self.showcase=None
        self.entry_preparation=None;self.entry_dispatch=False
        self.demo=None;self.motion=None;self.reset=None;self.initialization=None
        self.emit('servo',active=False,ending=False,rpm=0)
        self.failed=True; self.synchronized=False; self.queue.clear(); self.pending=None; self.busy=False
        self.commission=None;self.emit('candidate_invalid')
        self.send(encode(2,0,self.seq,0))
        self.emit('error',text=text+'；已尝试 STOP，需重连，未确认关桥时不显示已停止')

    def request(self,action,data=None):
        if action=='stop': self.stop(); return
        if not self.synchronized or self.failed: raise ValueError('尚未同步 FPGA 连接')
        if self.zero_calibration is not None and not self.zero_dispatch:raise ValueError('辅助零点标定中；可STOP取消')
        if self.reset is not None:raise ValueError('软复位核验中；仅允许STOP取消')
        if action=='prepare_zero':
            from .pendulum_zero import PendulumZero
            if self.persist_zero is None:raise ValueError('辅助标定需要可靠现场与新profile保存入口')
            status=self.communication_status()
            if status is None or status.get('protocol_revision')!=4:raise ValueError('辅助标定需要v4通信状态；陈旧控制采样可按条件恢复')
            self.stop();self.zero_calibration=PendulumZero(self)
            self.emit('sequence_state',active=True,sequence='zero_calibration');return
        if action in ('soft_reset','recover'):
            if action=='soft_reset' and not self.capabilities.get(33) or self.persist_reset is None:raise ValueError('需要匹配能力与可靠现场保存入口')
            status=self.communication_status()
            if status is None:raise ValueError('通信状态过期，无法核验复位或清错')
            initial=dict(status)
            self.stop();self.reset=ResetFlow(self,self.persist_reset,initial,operation='reset' if action=='soft_reset' else 'recover');return
        if self.initialization is not None:raise ValueError('初始化核验中，仅允许STOP取消')
        if self.entry_preparation is not None:raise ValueError('B14回零准备进行中；可STOP取消')
        if self.showcase is not None and not self.demo_dispatch:raise ValueError('自动最高速演示进行中；可STOP取消')
        if action in ('servo_target','servo_end'):
            self._live_servo_request(action,data)
            return
        if self.busy: raise ValueError('上一项操作尚未完成')
        if self.demo is not None and not self.demo_dispatch:raise ValueError('演示进行中；可使用 STOP 中止')
        if action=='read_pbd1':
            from .pbd1_readout import PBD1Readout
            status=self.communication_status()
            if data is not None:raise ValueError('PBD1只支持固定只读事务')
            if self.queue or self.pending is not None or self.cycle is not None or self.motion is not None or self.commission is not None:raise ValueError('PBD1需要事务空闲')
            if status is None or status['run'] or any(status.get(k)!=0 for k in ('duty_u','duty_v','duty_w')):raise ValueError('PBD1需要近期停止PWM0通信状态')
            self.pbd1=PBD1Readout(self);self.emit('busy',value=True);return
        if action=='apply':
            status=self.communication_status()
            if status is None:raise ValueError('通信状态过期，无法写入参数')
            if status['run']:raise ValueError('请先停止，再写入参数')
        diagnostic_read=(action in ('read','apply') or self.manual_control) and self.communication_status() is not None and not self.communication_status()['run']
        if not diagnostic_read and (self.latest is None or self.clock()-self.last_rx>.25): raise ValueError('状态已过期')
        if not diagnostic_read and not self.manual_control and self.latest.get('protocol_revision')==4 and not self.latest.get('control_sample_fresh') and action!='read':
            raise ValueError('控制采样未更新，仅可查看诊断；请检查驱动电源与传感器')
        if self.manual_control and self.communication_status() is None:raise ValueError('通信状态过期')
        if self.cycle is not None:raise ValueError('自动循环进行中；可使用 STOP 中止')
        required={'fast':(26,),'position':(26,),'servo':(26,),'home':(24,),'session_home':(24,26,29),'session_swing':(29,),'drop':(17,22),'cycle':(17,19,22),'long_run':(19,)}.get(action,())
        if any(not self.capabilities.get(addr,False) for addr in required):raise ValueError('当前固件未报告此运动能力，未发送运动命令')
        if LegacyServoEntry.begin(self,action,data):return
        if action in ('apply','read','clear') and (self.communication_status() if diagnostic_read else self.latest)['run']: raise ValueError('请先停止，再操作标定或清故障')
        if action=='initialize_hold':
            if not isinstance(data,(tuple,list)) or len(data)!=2:raise ValueError('初始化需要绑定配置及完成动作')
            profile,after=data
            row=self.latest
            if not isinstance(profile,BoundCalibrationProfile) or after not in ('hold','continuous') or not profile.matches(self.profile_identity,row.get('protocol_revision')):
                raise ValueError('缺少与当前精确设备/位流匹配的已核验配置')
            if row['run'] or row['fault'] or not row['sensor_valid'] or self.motion_version!=2 or not all(self.capabilities.get(a) for a in (24,26,29)):
                raise ValueError('初始化需要无故障停止、有效采样与ENA1能力；未自动CLEAR')
            self.initialization=InitializeFlow(self,profile,after)
        elif action=='servo_begin':
            if self.motion_version!=2 or not self.capabilities.get(32):
                raise ValueError('固件未报告 SV27 持续伺服能力')
            if not self.manual_control and (not self.latest['run'] or self.latest['mode'] not in (3,4) or self.latest['fault'] or not self.latest.get('keep_session')):
                raise ValueError('持续伺服需要连续使能会话中稳定直立')
            if not self.manual_control and (self.latest.get('protocol_revision')==4 and not(self.latest.get('servo_eligible') and self.latest.get('control_sample_fresh'))):
                raise ValueError('持续伺服需要新鲜板端SV27资格')
            self.motion=dict(kind='servo',seconds=None,active_at=None,seen=False,ending=False,
                             started=self.clock(),goal=None,target=0,desired=None,last_target=-float('inf'),end_at=None)
            self.busy=True;self._put(0x12,32,0,'servo_capability');self._put(0x26,0,0,'motion_start')
        elif action=='speed':
            allowed={1,5,10,15,20,30,40,50,60,80,100,120,150,180,200}
            if type(data) is not int or abs(data) not in allowed:raise ValueError('SPEED仅接受已定义的正反30档')
            row=self.latest
            if self.motion_version!=2 or not self.capabilities.get(35) or row.get('protocol_revision')!=4:
                raise ValueError('固件未报告SPD1及对应原子状态')
            if not self.manual_control and (not row['run'] or row['mode'] not in (3,4) or row['fault'] or not row.get('keep_session') or not row.get('speed_eligible') or not row.get('control_sample_fresh')):
                raise ValueError('SPEED需要连续会话稳摆与新鲜板端资格')
            self.motion=dict(kind='speed',seen=False,active_at=None,started=self.clock(),goal=None)
            self.busy=True;self._put(0x12,35,0,'speed_capability');self._put(0x2b,0,data,'motion_start')
        elif action=='continuous':
            if self.motion_version!=2 or not all(self.capabilities.get(a) for a in (24,26,29)):raise ValueError('固件未报告连续会话能力')
            row=self.latest
            down=row['run'] and row.get('keep_session') and row['mode']==0 and row.get('home_settled')
            if not self.manual_control and (self.motion_version!=2 or not all(self.capabilities.get(a) for a in (24,26,29)) or row['fault'] or not(down or not row['run'] and row['ready'])):
                raise ValueError('起摆稳摆需要有效ENA1下垂会话或资格就绪的停止状态')
            self.demo=dict(steps=[('session_home',0.),('session_swing',None)],index=0,started=self.clock(),quiet=None,
                           dose=None,dose_poll=0.,thermal_wait=False,full=True,finish_upright=True,deadline=60)
        elif action=='peak_showcase':
            from .peak_showcase import PeakShowcase
            if not self.capabilities.get(32):raise ValueError('最高速演示需要连续SERVO能力')
            self.showcase=PeakShowcase(self)
            try:self.showcase.prepare('servo_begin')
            except ValueError:
                self.showcase=None;raise
        elif action=='demo':
            if not all(self.capabilities.get(a) for a in (17,19,22,24,26)):raise ValueError('固件未报告演示所需能力')
            full=data=='v27_full';steps=demo_recipe(data,full_session=self.motion_version==2 and bool(self.capabilities.get(29)))
            if full and (self.latest.get('protocol_revision')!=4 or not self.capabilities.get(32)):raise ValueError('当前完整演示的最高速段需要v4与连续SERVO能力')
            enabled_down=full and self.latest.get('keep_session') and self.latest['mode']==0 and self.latest.get('home_settled')
            if not self.manual_control and (self.latest['fault'] or (not enabled_down and (self.latest['run'] or not self.latest['ready'])) or not all(self.capabilities.get(x) for x in (17,19,22,24,26))):
                raise ValueError('演示需要下垂待机、资格就绪和完整运动能力')
            if not self.manual_control and (angle_error(self.latest['alpha'])>.1):raise ValueError('演示起点需摆杆下垂')
            self.demo=dict(steps=steps,index=0,started=self.clock(),quiet=None,dose=None,dose_poll=0.,thermal_wait=False,full=full,deadline=600 if full else 120)
            self.emit('log',text='演示：准备零位 → 起摆 → 稳摆快移五步 → 高速演示 → 受控落摆；最高速段前后按需回零，STOP可中止' if full else '有限范围演示开始；自动等待热预算，最多 120 秒')
        elif action in ('fast','position','servo'):
            multi=self.motion_version==2
            if action=='servo':
                rpm,seconds=servo_request(self.latest,*data,multiturn=multi,manual_control=self.manual_control)
                self.motion=dict(kind='servo',seconds=seconds,active_at=None,seen=False,ending=False,started=self.clock(),goal=None)
                op,addr,value=0x26,0,rpm
            else:
                bank,value,goal,upright=(position_request(self.latest,data,multiturn=multi,manual_control=self.manual_control) if action=='position'
                                         else fast_request(self.latest,data,multiturn=multi,manual_control=self.manual_control))
                self.motion=dict(kind='up' if upright else 'down',seen=False,started=self.clock(),goal=goal)
                op,addr=(0x27 if action=='position' else 0x25),bank
            self.busy=True;self._put(0x12,26,0,'motion_capability');self._put(op,addr,value,'motion_start')
        elif action=='commission':
            if data not in (5,6) or self.device_words is None or not self.manual_control and (not self.latest['ready'] or self.latest['run'] or self.latest['fault']):
                raise ValueError('自动标定需要已回读参数、静止待机且资格就绪')
            if data==6 and self.device_words[7]&63!=11:raise ValueError('R1 自动对齐仅支持已知 11 极对电机')
            self.busy=True;self.commission=(data,self.clock());self.candidate={};self.emit('candidate_invalid')
            self._put(0x12,8,0,'capability');self._put(0x21,data,0x43414C31,'commission_start')
        elif action=='apply':
            words=data.words(); self.expected=words; self.readback={}; self.busy=True
            for addr,value in enumerate(words):self._put(0x10,addr,value,'write')
            self._put(0x11,tag='commit')
            for addr in range(8):self._put(0x12,addr,0,'verify')
        elif action=='read':
            self.expected=None; self.readback={}; self.busy=True
            for addr in range(8):self._put(0x12,addr,0,'read')
        elif action=='clear':self.busy=True; self._put(3,tag='clear')
        elif action in ('home','session_home'):
            limit=3.141592653589793 if self.motion_version==2 else 2.8
            if not isinstance(data,(int,float)) or not -limit<=data<=limit:raise ValueError('下垂位置目标超过固件范围')
            if not self.manual_control and self.latest['fault']:raise ValueError('下垂位置移动不能在故障时启动')
            if action=='session_home' and self.latest['run'] and (not self.manual_control or self.demo_dispatch):
                if self.demo_dispatch and (self.clock()-self.last_rx>.25 or not self.latest.get('control_sample_fresh',True) or not self.latest.get('telemetry_current',True)):
                    raise ValueError('连续会话衔接需要新鲜下垂保持状态；未重发HOME')
                if not self.latest.get('keep_session') or self.latest['mode']!=0 or not self.latest.get('home_settled'):raise ValueError('连续会话回中需要下垂待机')
                # Already holding this target: encoder noise must not launch
                # another trajectory, especially below the 0.01 degree minimum.
                if abs(self.latest['q']-data)<math.radians(.01) or (abs(self.latest.get('motion_goal',self.latest['q'])-data)<math.radians(.01) and abs(self.latest['q']-data)<=.008727):return
                self.request('position',data);return
            if not self.manual_control and (not self.latest['ready'] or self.latest['run']):raise ValueError('下垂位置移动需要停止且资格就绪')
            self.busy=True;self._put(0x12,24,0,'move_capability')
            if action=='session_home':self._put(0x12,29,0,'session_capability')
            self._put(0x24,1 if action=='session_home' else 0,round(data*65536),'home')
        elif action=='session_swing':
            row=self.latest
            if not self.manual_control and (not row['run'] or not row.get('keep_session') or row['mode']!=0 or not row.get('home_settled') or abs(row['q'])>=.03 or angle_error(row['alpha'])>=.012):raise ValueError('连续起摆需要已回中下垂待机')
            self.busy=True;self._put(0x12,29,0,'session_capability')
            if self.demo_dispatch and self.demo is not None and self.demo.get('finish_upright'):
                # Evidence only, after the unchanged native quiet/dose checks.
                # The original FIFO/ACK/NACK/heartbeat/STOP paths own every read.
                self.swing_context_serial+=1
                at=self.clock();dose=self.demo['dose']
                self.emit('swing_context_queued',capture_id=self.swing_context_serial,
                          requested_monotonic=at,read_order=[25,27,30,18],atomic_permission=False,
                          completed_frame=dict(row),completed_frame_received_monotonic=self.last_rx,
                          quiet_started_monotonic=self.demo['quiet'],
                          quiet_elapsed_s=at-self.demo['quiet'] if self.demo['quiet'] is not None else None,
                          read20_received_monotonic=dose[0] if dose is not None else None,
                          read20_A2s=dose[1] if dose is not None else None,
                          read18_interpretation='bit2 drop_pending only; other bits are not SWING gates')
                for address in (25,27,30,18):self._put(0x12,address,0,'capture_swing_context')
            self._put(0x29,3,0x53574731,'arm')
        elif action=='drop':
            if data not in (-1,1):raise ValueError('落摆方向必须为正向或反向')
            if not self.manual_control and (not self.latest['run'] or self.latest['fault'] or self.latest['mode'] not in (3,4) or data not in (-1,1)):
                raise ValueError('受控落摆需要正在稳摆，且指定正向或反向')
            self.busy=True
            self._put(0x12,17,0,'drop_capability')
            self._put(0x12,22,0,'home_capability')
            self._put(0x12,18,0,'drop_qualification')
            self._put(0x22,7,0x44525031 if data==1 else 0x4452504E,'drop')
        elif action=='cycle':
            seconds,direction=data
            if not 1<=seconds<=600 or direction not in (-1,1):raise ValueError('稳摆时间应为 1..600 秒，方向应为正向或反向')
            if not self.manual_control and (not self.latest['ready'] or self.latest['run'] or self.latest['fault']):raise ValueError('自动循环需要停止且资格就绪')
            self.cycle=dict(seconds=seconds,direction=direction,started=self.clock(),hold=None)
            self.quiet_since=None;self.busy=True
            self._put(0x12,17,0,'drop_capability');self._put(0x12,22,0,'home_capability')
            self._put(0x12,19,0,'long_capability');self._put(0x23,3,0x4C4F4E31,'arm')
        elif action=='long_run':
            if not self.manual_control and (not self.latest['ready'] or self.latest['run'] or self.latest['fault']):
                raise ValueError('长时起摆需要停止且资格就绪')
            self.busy=True;self._put(0x12,19,0,'long_capability');self._put(0x23,3,0x4C4F4E31,'arm')
        elif action=='arm':
            mode,current=data
            if not self.manual_control and (not self.latest['ready'] or self.latest['run'] or self.latest['fault']): raise ValueError('设备未满足待机资格条件')
            if mode not in (1,2,3,4) or not -0.08<=current<=0.08: raise ValueError('模式或电流越界')
            self.busy=True; self._put(0x20,mode,round(current*65536) if mode==1 else 0,'arm')
        else:raise ValueError('未知操作')
        if self.manual_control and action not in ('read','apply','clear'):
            self.manual_command_pending=True
        self.emit('busy',value=self.busy)

    def _live_servo_request(self,action,data):
        m=self.motion
        if not m or m['kind']!='servo' or m['seconds'] is not None or m['active_at'] is None or m['ending']:
            raise ValueError('持续伺服尚未进入或正在减速退出')
        row=self.latest
        if self.manual_control and self.communication_status() is None:raise ValueError('通信状态过期')
        if not self.manual_control and (row is None or self.clock()-self.last_rx>.25 or row['fault'] or not row['run'] or row['mode']!=0 or row.get('motion_kind')!=3):
            raise ValueError('持续伺服状态无效或已过期')
        if action=='servo_target':
            m['desired']=continuous_servo_target(data)
        else:
            m['desired']=None;m['ending']=True;m['end_at']=self.clock()
            self.queue=deque(item for item in self.queue if item[3]!='servo_target')
            self._put(0x26,1,0,'servo_end')

    def feed(self,data):
        now=self.clock()
        for frame in self.decoder.feed(data):
            if frame['op']==0x80:
                try:row=self.assembler.feed(frame,now)
                except ValueError as exc:self.fail(str(exc)); return
                if row is not None:
                    self.status_row=dict(row);self.status_at=now
                    if row.get('diagnostic_only'):
                        self.last_diagnostic=now
                        if row['run'] and not self.safety_stop_pending:
                            self.safety_stop_pending=True;self.stop()
                            self.emit('rejected',text='运行中采样未更新，已请求STOP；保持诊断连接')
                        if self.await_state and self.await_state[0] in ('stop','clear'):
                            action,_,at=self.await_state
                            if not row['run'] and (action!='clear' or not row['fault']) and self.assembler.started>=at:
                                self.await_state=None;self.busy=self.reset is not None;self.emit('busy',value=self.busy)
                                self.safety_stop_pending=False
                        text='已连接 · 控制采样未更新，不能使能；请检查驱动电源与传感器'
                        if self.manual_control:text='已连接 · 控制采样未更新；人工命令由设备确认'
                        if row['fault']:text+=f" · 设备故障 0x{row['reason']:02X}"
                        self.emit('diagnostic_status',text=text,row=dict(row),received_monotonic=now)
                        continue
                    if self.verified_reset_epoch is not None and (row.get('reset_epoch')!=self.verified_reset_epoch or not row.get('control_sample_fresh')):
                        row['telemetry_current']=False
                    if not row.get('telemetry_current',True):
                        self.emit('sample',row=row);continue
                    if self.verified_reset_epoch is not None:self.verified_reset_epoch=None
                    self.latest=row; self.last_rx=now; self.emit('sample',row=row)
                    if self.motion_version==2:self._motion_snapshot(row,now)
                    if self.commission and row['fault'] and not self.manual_command_pending:
                        self.stop();self.emit('rejected',text=f"自动标定触发设备故障 0x{row['reason']:02X}，未应用结果")
                    if self.await_state:
                        action,mode,at=self.await_state
                        reached=(row['run'] and row['mode']==mode) if action in ('arm','home','drop','commission_start') else (not row['run'] and (action!='clear' or not row['fault']))
                        if action=='calibration':reached=not row['run'] and row['calibrated']
                        reached=reached and self.assembler.started>=at
                        if reached:
                            self.await_state=None;self.busy=self.commission is not None or self.drop_wait is not None or self.reset is not None or self.initialization is not None;self.emit('busy',value=self.busy);self.emit('log',text=action+' 已由遥测确认')
                        elif row['fault'] and action in ('arm','home','drop','commission_start'):
                            self.await_state=None;self.stop();self.emit('rejected',text='启动后设备进入故障，已请求停止')
                    if not self.started:
                        self.started=True; self._put(2,tag='connect_stop'); self._put(4,tag='sync')
                continue
            p=self.pending
            if p is None or frame['op'] not in (0x81,0x82) or frame['seq']!=p['seq'] or frame['addr']!=p['addr']:continue
            self.pending=None
            if p['tag']=='pbd1' and self.pbd1 is not None:
                handled=self.pbd1.reply(frame,now)
                if handled:
                    self.emit('command_reply',op=p['op'],addr=p['addr'],seq=p['seq'],value=frame['value'],accepted=frame['op']==0x81)
                    continue
            if self.zero_calibration is not None and p['tag'] in ('commit','verify'):
                self.zero_calibration.receipts.append(dict(tag=p['tag'],op=p['op'],addr=p['addr'],seq=p['seq'],value=frame['value'],accepted=frame['op']==0x81,received_monotonic=now))
            if p['tag']=='probe_capability' and frame['op']==0x82 and frame['value']==2:
                self.capabilities[p['addr']]=False;self.emit('capabilities',values=dict(self.capabilities));continue
            if frame['op']==0x82:
                if self.pbd1 is not None:self.pbd1.finish('UNKNOWN','interleaved command NACK')
                self.entry_preparation=None
                self.manual_command_pending=False
                self.emit('command_reply',op=p['op'],addr=p['addr'],seq=p['seq'],value=frame['value'],accepted=False)
                if p['tag'].startswith('reset_'):
                    self.operation_failed(f"设备拒绝{p['tag']}（码 {frame['value']}），未自动重试");return
                if frame['value']==1:self.fail('设备拒绝序号');return
                if self.zero_calibration is not None:self.emit('sequence_state',active=False,sequence='zero_calibration')
                self.zero_calibration=None
                self.queue.clear();self.busy=False;self.expected=None;self.drop_wait=None;self.cycle=None;self.motion=None;self.demo=None;self.showcase=None
                self.initialization=None
                self.commission=None;self.emit('candidate_invalid')
                self._put(2,tag='stop')
                self.emit('rejected',text=f"设备拒绝 {p['tag']}（码 {frame['value']}），未执行成功")
                continue
            if p['op']>=0x20:self.manual_command_pending=False
            if p['op']!=0x12 and frame['value']!=(0 if p['op']==2 else p['value']&0xFFFFFFFF):
                self.fail('命令回执值不匹配');return
            self.emit('command_reply',op=p['op'],addr=p['addr'],seq=p['seq'],value=frame['value'],accepted=True)
            if p['tag']=='commit' and p['op']==0x11:
                self.assembler.calibration_committed(now)
            if p['op']==0x12:
                self.emit('read_reply',addr=p['addr'],value=frame['value'],seq=frame['seq'],
                          epoch=(self.latest or {}).get('reset_epoch'))
            if p['tag'].startswith('reset_'):
                if self.reset is not None:self.reset.reply(p,frame)
                self.emit('busy',value=self.busy);continue
            if p['tag']=='probe_capability':
                expected={17:0x44525031,19:0x4C4F4E31,22:0x484F4D31,24:0x484F4D32,26:0x4D4F5431,29:0x454E4131,31:0x44543230,32:0x53563237,33:0x52535431,35:0x53504431}[p['addr']]
                if p['addr']==26:
                    expected=0x4D4F5432 if self.latest.get('protocol_revision') in (3,4) else 0x4D4F5431
                    self.motion_version=(2 if expected==0x4D4F5432 else 1) if frame['value']==expected else 0
                self.capabilities[p['addr']]=frame['value']==expected;self.emit('capabilities',values=dict(self.capabilities))
            elif p['tag'] in ('drop_capability','home_capability','long_capability','move_capability','motion_capability','session_capability'):
                expected={'drop_capability':0x44525031,'home_capability':0x484F4D31,'long_capability':0x4C4F4E31,'move_capability':0x484F4D32,'motion_capability':0x4D4F5431,'session_capability':0x454E4131}[p['tag']]
                if p['tag']=='motion_capability' and self.motion_version==2:expected=0x4D4F5432
                if frame['value']!=expected:self.fail('固件运动能力不匹配，未启动');return
            elif p['tag']=='servo_capability':
                if frame['value']!=0x53563237:self.fail('固件持续伺服能力不匹配，未启动');return
            elif p['tag']=='speed_capability':
                if frame['value']!=0x53504431:self.fail('固件SPEED能力不匹配，未启动');return
            elif p['tag'] in ('servo_target','servo_end') and self.motion and self.motion.get('seconds') is None:
                self.emit('servo',active=True,ending=self.motion['ending'],rpm=0 if self.motion['ending'] else p['value'])
            elif p['tag']=='motion_start':
                if self.entry_preparation is not None and p['op']==0x27:self.entry_preparation.ack_at=now
                self.emit('log',text='运动命令已接受；等待活动状态与到达确认')
            elif p['tag']=='motion_goal' and self.motion:
                word=frame['value'];self.motion['goal']=(word if word<0x80000000 else word-0x100000000)/65536
            elif p['tag']=='demo_dose' and self.demo:
                self.demo['dose']=(now,frame['value']/65536)
            elif p['tag']=='motion_status' and self.motion:
                m=self.motion;status=frame['value'];row=self.latest
                if status&8:
                    m['seen']=True
                    if m['kind']=='servo' and m['active_at'] is None:m['active_at']=now
                if m['seen'] and m['kind']=='down' and not status&8 and status&3==0:
                    self._put(0x12,25,0,'motion_down_rest')
                if m['seen'] and m['kind']!='down' and status&32 and row['mode']==4 and m['goal'] is not None:
                    if row['sensor_valid'] and abs(row['q']-m['goal'])<=.03491 and angle_error(row['alpha'], math.pi)<=.03491 and abs(row['w'])<.15:
                        self.motion=None;self.busy=False;self.emit('motion_complete',goal=m['goal']);self.emit('log',text='移动完成，保持直立位置')
            elif p['tag']=='motion_down_rest' and self.motion:
                row=self.latest;m=self.motion
                if frame['value']&3==3 and row['mode']==0 and row['sensor_valid'] and abs(row['q']-m['goal'])<=.008727 and angle_error(row['alpha'])<=.008727 and abs(row['w'])<=.05 and abs(row['v'])<=.05:
                    self.emit('motion_complete',goal=m['goal']);self.stop(preserve_demo=True)
            elif p['tag']=='showcase_drop_qualification' and self.showcase is not None:
                self.showcase.drop_qualification=(now,frame['value'])
            elif p['tag']=='drop_qualification':
                if (not self.manual_control or self.showcase is not None) and frame['value']&7!=3:
                    if self.demo is not None:
                        self.stop();self.emit('rejected',text='演示落摆资格不足，已请求 STOP；未标记完成')
                    else:
                        self.queue.clear();self.busy=False;self.emit('rejected',text='落摆姿态或电流积分资格不足，保持原模式')
                    continue
            elif p['tag']=='home_status':
                row=self.latest
                if row and row.get('protocol_revision') in (3,4) and self.motion_mode==0:self.motion_goal=row['motion_goal']
                rest=row and row['sensor_valid'] and abs(row['q']-self.motion_goal)<=.008726647 and angle_error(row['alpha'])<=.008726647 and abs(row['w'])<=.05 and abs(row['v'])<=.05
                if self.drop_wait is not None and frame['value']&3==3 and rest and row['run'] and not row['fault'] and row['mode']==self.motion_mode:
                    if row.get('keep_session'):
                        self.drop_wait=None;self.busy=False;self.emit('log',text='末端 Home 已静止，保持连续使能会话')
                    else:
                        self.emit('log',text='下垂保持已连续静止，正在请求 STOP');self.stop(preserve_demo=True)
            elif p['tag']=='capability':
                if frame['value']!=0x43414C31:self.fail('固件不支持 CAL1 自动标定');return
            elif p['tag'] in ('progress','candidate_seal'):
                status=frame['value'];mode=(status>>2)&7;active=bool(status&1);valid=bool(status&2);error=status>>24;count=(status>>8)&65535
                if self.commission is None:continue
                target=self.commission[0]
                self.emit('commission_progress',mode=target,seconds=count/2000,error=error)
                if error or (not active and not valid):
                    self.stop();self.emit('rejected',text=f'自动标定失败 / 中止：0x{error:02X}，未应用结果');continue
                if mode!=target:self.fail('标定结果模式不匹配');return
                if valid and not active and self.latest and not self.latest['run'] and not self.latest['fault']:
                    if p['tag']=='candidate_seal':
                        words=list(self.device_words)
                        if target==5:words[2:4]=[self.candidate[10],self.candidate[11]]
                        else:words[6]=self.candidate[12];words[7]=(words[7]&63)|(self.candidate[13]<<8)
                        try:Calibration.from_words(words)
                        except ValueError:self.fail('候选标定结果越界');return
                        self.commission=None;self.busy=False;self.emit('commission_result',words=words,mode=target)
                    else:
                        for addr in range(10,14):self._put(0x12,addr,0,'candidate')
                        self._put(0x12,9,0,'candidate_seal')
            elif p['tag']=='candidate':self.candidate[p['addr']]=frame['value']
            elif p['tag']=='sync':
                self.synchronized=True;self.emit('connected',text=f"FPGA F1 / 协议 v{self.assembler.revision} 已同步；未自动运行")
                if self.latest and self.latest['calibrated']:
                    self.readback={};self.expected=None;self.busy=True
                    for addr in range(8):self._put(0x12,addr,0,'read')
                for addr in (17,19,22,24,26,29,31,32,33,35):self._put(0x12,addr,0,'probe_capability')
            elif p['tag'] in ('read','verify'):
                self.readback[p['addr']]=frame['value']
                if p['addr']==7:
                    words=[self.readback[i] for i in range(8)]
                    if self.expected is not None and words!=self.expected:self.fail('标定回读与写入不一致');return
                    self.device_words=list(words)
                    self.busy=self.expected is not None or self.initialization is not None
                    if self.expected is not None:self.await_state=('calibration',0,now)
                    if self.expected is not None and self.zero_calibration is not None and self.zero_calibration.phase=='apply':
                        self.await_state=None;self.busy=False
                        self.zero_calibration.phase='health';self.zero_calibration.health_after=now
                    self.emit('calibration',words=words,verified=self.expected is not None);self.expected=None
            elif p['tag'] in ('arm','home','drop','stop','clear','commission_start'):
                if p['tag'] in ('home','drop'):
                    self.drop_wait=now;self.motion_status_addr=25 if p['tag']=='home' else 23
                    self.motion_mode=0 if p['tag']=='home' else 7;self.motion_timeout=19.5 if p['tag']=='home' else (20.5 if self.capabilities.get(31) else 4.5)
                    goal=p['value'] if p['value']<0x80000000 else p['value']-0x100000000
                    self.motion_goal=goal/65536 if p['tag']=='home' else 0.
                    if p['tag']=='drop' and self.latest.get('keep_session'):self.motion_status_addr=25;self.motion_mode=0
                self.busy=True;self.await_state=(p['tag'],0 if p['tag']=='home' else p['addr'],now)
                self.emit('log',text=f"{p['tag']} 回执接受；运行/故障状态以遥测为准")
            self.emit('busy',value=self.busy)

    def poll(self):
        if self.pbd1 is not None:
            status=self.communication_status()
            if status is None or status['run'] or any(status.get(k)!=0 for k in ('duty_u','duty_v','duty_w')):
                self.fail('PBD1读取期间停止通信状态失效');return
        if self.failed:return
        now=self.clock()
        if self.reset is not None:
            self.reset.poll()
            if self.failed:return
        if self.zero_calibration is not None:
            try:self.zero_calibration.poll()
            except Exception as exc:self.operation_failed('辅助零点标定失败：'+str(exc))
            if self.failed:return
        if self.initialization is not None:
            self.initialization.poll()
            if self.failed:return
        if self.entry_preparation is not None:
            self.entry_preparation.poll()
            if self.failed:return
        if self.showcase is not None:
            try:self.showcase.poll()
            except ValueError as exc:self.operation_failed(str(exc))
        if self.demo is not None:
            demo=self.demo;row=self.latest
            if row and row['fault'] and not self.manual_command_pending or now-demo['started']>demo['deadline']:
                self.stop();self.emit('rejected',text='演示故障或超时，已请求 STOP')
            elif self.showcase is None and not self.busy and not self.queue and self.pending is None and self.await_state is None:
                if demo['index']>=len(demo['steps']):
                    if demo.get('finish_upright'):
                        if row and now-self.last_rx<=.25 and row['sensor_valid'] and (row.get('protocol_revision')!=4 or row.get('control_sample_fresh')) and row['run'] and row.get('keep_session') and row['mode'] in (3,4) and angle_error(row['alpha'],math.pi)<.017 and abs(row['w'])<.15 and abs(row['v'])<.2:
                            self.demo=None
                            self.emit('upright_complete',row=dict(row),epoch=row.get('reset_epoch'),stamp=row['stamp'],sample_monotonic=self.last_rx)
                            self.emit('log',text='起摆稳摆已确认，持续使能；未设置自动落摆期限')
                    elif demo['full'] and not(row and row['run'] and row.get('keep_session') and row['mode']==0 and row.get('home_settled')):
                        self.stop();self.emit('rejected',text='完整演示终态未确认，已请求 STOP')
                    else:
                        self.demo=None;self.emit('demo_complete');self.emit('log',text='完整演示完成，连续使能下垂待机' if demo['full'] else '有限范围演示完成，已停止')
                else:
                    action,data=demo['steps'][demo['index']]
                    quiet=row and (row['ready'] if not row['run'] else row['mode'] in (3,4) and angle_error(row['alpha'], math.pi)<.017 and abs(row['w'])<.15 and abs(row['v'])<.2)
                    if row and row['run'] and row['mode'] in (3,4):
                        quiet=quiet and not row.get('servo_settling',False) and abs(row['q']-row['motion_goal'])<2287/65536
                    # STOP can release a small pendulum oscillation while ready
                    # remains true. Fast motion needs the stricter down dwell.
                    if row and not row['run'] and action in ('fast','position'):
                        quiet=quiet and row['sensor_valid'] and angle_error(row['alpha'])<.01 and abs(row['w'])<.1 and abs(row['v'])<.15
                    if demo['full'] and row and row['run'] and row.get('keep_session') and row['mode']==0:
                        quiet=row.get('home_settled') and row['sensor_valid'] and abs(row['q']-row['motion_goal'])<.008727 and angle_error(row['alpha'])<.008727 and abs(row['w'])<.05 and abs(row['v'])<.05
                    # Manual admission does not turn an automatic sequence's
                    # SWING acceptance into completed upright capture.
                    manual_first=self.manual_control and demo['index']==0
                    quiet=quiet and now-self.last_rx<=.25 and row['sensor_valid'] and row.get('control_sample_fresh',True) and row.get('telemetry_current',True)
                    if manual_first:quiet=True
                    if quiet:
                        if demo['quiet'] is None:demo['quiet']=now
                        thermal_ready=True
                        if row['run'] and action in ('fast','position','servo','drop','session_swing'):
                            sample=demo['dose'];limit=.49 if action=='drop' else .64
                            thermal_ready=sample is not None and now-sample[0]<=.5 and sample[1]<limit
                            if now-demo['dose_poll']>=.25:
                                self._put(0x12,20,0,'demo_dose');demo['dose_poll']=now
                            if sample is not None and sample[1]>=limit and not demo['thermal_wait']:
                                demo['thermal_wait']=True;self.emit('log',text='演示等待热预算恢复，保持稳摆；未清除累计量')
                        # Manual single commands remain direct; an automatic demo
                        # must wait for its fresh dose readback even in manual UI mode.
                        if manual_first or now-demo['quiet']>=.4 and thermal_ready:
                            try:
                                self.demo_dispatch=True;self.request(action,data)
                                demo['index']+=1;demo['quiet']=None;demo['dose']=None;demo['thermal_wait']=False
                                description={'session_home':'准备零位保持','session_swing':'起摆稳摆',
                                             'peak_showcase':'高速演示及受控落摆'}.get(action,action)
                                if action=='fast':description=f'快移 {data:+g}°'
                                self.emit('log',text=f"演示步骤 {demo['index']}/{len(demo['steps'])}：{description}")
                            except ValueError as exc:self.stop();self.emit('rejected',text='演示未跳过失败步骤：'+str(exc))
                            finally:self.demo_dispatch=False
                    else:demo['quiet']=None
        if self.motion is not None:
            m=self.motion;row=self.latest
            if row and row['fault'] and not self.manual_command_pending:self.stop();self.emit('rejected',text='运动故障，已请求 STOP')
            elif m['kind']=='speed':
                if m['active_at'] is None and now-m['started']>2:self.fail('有限SPEED未确认进入');return
                if now-m['started']>25:self.fail('有限SPEED未在25秒内到位');return
            elif m['kind']=='servo' and m['seconds'] is None:
                if m['active_at'] is None and now-m['started']>2:self.fail('持续伺服未确认进入');return
                if m['ending'] and now-m['end_at']>25:self.fail('持续伺服减速退出未在期限内完成');return
                if m['active_at'] is not None and not m['ending'] and m['desired'] is not None and now-m['last_target']>=.1:
                    if not any(item[3]=='servo_target' for item in self.queue) and not (self.pending and self.pending['tag']=='servo_target'):
                        m['target']=m['desired'];m['desired']=None;m['last_target']=now
                        self._put(0x26,0,m['target'],'servo_target')
            elif now-m['started']>19:self.fail('运动未在期限内完成');return
            elif m['kind']=='servo' and m['active_at'] is not None and not m['ending'] and now-m['active_at']>=m['seconds']:
                m['ending']=True;self._put(0x26,1,0,'servo_end')
            elif self.motion_version!=2 and not self.queue and self.pending is None and now-self.last_motion_poll>=.1:
                self._put(0x12,28,0,'motion_goal');self._put(0x12,27,0,'motion_status');self.last_motion_poll=now
        if self.cycle is not None and self.await_state is None and not self.busy:
            cycle=self.cycle;row=self.latest
            if not row or not row['run'] or row['fault']:
                self.stop();self.emit('rejected',text='自动循环运行中断，已请求 STOP')
            else:
                quiet=abs(row['q'])<.03 and angle_error(row['alpha'], math.pi)<.03 and abs(row['w'])<.3 and abs(row['v'])<.3
                if cycle['hold'] is None:
                    if quiet:
                        if self.quiet_since is None:self.quiet_since=now
                        if now-self.quiet_since>=.3:cycle['hold']=now;self.emit('log',text='已捕获稳摆，开始有限时长计时')
                    else:self.quiet_since=None
                    if now-cycle['started']>4:self.fail('起摆后未确认稳定捕获');return
                elif now-cycle['hold']>=cycle['seconds']:
                    self.cycle=None
                    self.request('drop',cycle['direction'])
                    # A finite cycle cannot remain energized after a rejected DROP.
                    self.cycle_drop=True
        if self.cycle_drop and self.drop_wait is None and not self.busy and not self.queue and self.pending is None:
            self.cycle_drop=False;self.stop();self.emit('rejected',text='自动循环落摆未启动，已请求 STOP')
        if self.drop_wait is not None:
            if self.latest and self.latest['fault']:
                self.stop();self.emit('rejected',text='落摆期间设备故障，已请求 STOP，未自动清故障')
            elif now-self.drop_wait>self.motion_timeout:self.fail('运动末端未在时限内确认静止');return
            elif self.await_state is None and not self.queue and self.pending is None and now-self.last_home_poll>=.1:
                self._put(0x12,self.motion_status_addr,0,'home_status');self.last_home_poll=now
        if self.commission and now-self.commission[1]>18:self.fail('自动标定超时，结果未应用');return
        if self.await_state and now-self.await_state[2]>.35:self.fail('回执后未收到预期状态');return
        diagnostic_idle=(self.last_diagnostic is not None and now-self.last_diagnostic<=.25 and self.latest is not None
                         and self.motion is None and self.demo is None and self.initialization is None
                         and (self.await_state is None or self.await_state[0] in ('stop','clear')) and self.cycle is None and self.commission is None)
        if self.manual_control and self.communication_status() is not None and not self.communication_status()['run']:diagnostic_idle=True
        if self.zero_calibration is not None and self.zero_calibration.phase in ('apply','health') and self.communication_status() is not None and not self.communication_status()['run']:
            diagnostic_idle=True  # CAL invalidates calibrated/control_done until commit; never a motion exemption.
        if (self.started and now-self.last_rx>.25 and not diagnostic_idle) or (not self.started and now-self.opened>2):
            self.fail('遥测超时 / 身份未确认');return
        if self.pending:
            if now-self.pending['at']>.35:
                if self.pending['tag']=='reset_command' and self.reset is not None:
                    self.pending=None;self.reset.resync();self.emit('log',text='RESET回执缺失；仅SYNC并查世代，不重发RESET')
                else:self.fail('命令回执超时，不重发运动或标定命令')
            return
        if self.synchronized and now-self.last_hb>=.2 and not (self.queue and self.queue[0][0]==2):
            self.queue.appendleft((1,0,MAGIC[1],'heartbeat'));self.last_hb=now
        if self.commission and self.await_state is None and not self.queue and now-self.last_progress>=.1:
            self._put(0x12,9,0,'progress');self.last_progress=now
        if self.queue:
            op,addr,value,tag=self.queue.popleft()
            self.pending=dict(op=op,addr=addr,seq=self.seq,value=value,tag=tag,at=now)
            self.seq=(self.seq+1)&255
            self.send(encode(op,addr,self.pending['seq'],value))
            if op>=0x20 or op in (2,3,0x11):
                stamp=(self.latest or {}).get('t',0.)+max(0.,now-self.last_rx)
                self.emit('command_sent',marker=dict(t=stamp,label=tag+' · 已发送',command=f'F1 {op:02X}:{addr} {value}'))

    def _motion_snapshot(self,row,now):
        """MOT2 completion uses goal/status/q from the same captured snapshot."""
        m=self.motion
        if m is None or row.get('protocol_revision') not in (3,4) or not row['run'] or row['fault']:return
        if row['motion_active']:
            if m['kind']=='speed' and not row.get('finite_speed_active'):return
            m['seen']=True
            if m['kind'] in ('servo','speed') and m['active_at'] is None:
                m['active_at']=now
                if m['kind']=='servo' and m['seconds'] is None:self.emit('servo',active=True,ending=False,rpm=0)
        if not m['seen']:return
        goal=row['motion_goal'];m['goal']=goal
        if m['kind']=='down':
            if row['mode']==0 and row['motion_kind']==0 and row['home_settled'] and row['home_active'] and row['sensor_valid'] and abs(row['q']-goal)<=.008727 and angle_error(row['alpha'])<=.008727 and abs(row['w'])<=.05 and abs(row['v'])<=.05:
                self.emit('motion_complete',goal=goal)
                if row.get('keep_session'):self.motion=None;self.busy=False
                else:self.stop(preserve_demo=True)
        elif row['mode']==4 and row['motion_arrived'] and row['sensor_valid'] and abs(row['q']-goal)<=.03491 and angle_error(row['alpha'], math.pi)<=.03491 and abs(row['w'])<.15:
            if m['kind']=='servo' and m['seconds'] is None:self.emit('servo',active=False,ending=False,rpm=0)
            self.motion=None;self.busy=self.entry_preparation is not None
            if self.entry_preparation is None:self.emit('motion_complete',goal=goal)
            else:self.emit('busy',value=True)
            self.emit('log',text='移动完成，保持直立位置')

class FPGAController(threading.Thread):
    def __init__(self,port,events,serial_factory=serial.Serial,output=None,profile_identity=None,bootstrap_profile=None,bootstrap_guard=None,manual_control=False,build14_entry_compat=False,backend=None):
        super().__init__(name='FPGA serial owner',daemon=True)
        self.port=port;self.events=events;self.serial_factory=serial_factory
        self.backend=backend or ('hardware' if serial_factory is serial.Serial else 'custom-unclassified')
        self.ui_activity={}
        self.manual_control=bool(manual_control)
        self.build14_entry_compat=bool(build14_entry_compat)
        self.profile_identity=dict(profile_identity or {})
        self.bootstrap_profile=bootstrap_profile;self.bootstrap_guard=bootstrap_guard
        self.commands=queue.Queue(maxsize=8);self.stopping=threading.Event();self.disconnect=threading.Event()
        self.ui_action_events=queue.SimpleQueue()
        self.servo_target_lock=threading.Lock();self.servo_target_pending=None
        self.last_ui=time.monotonic();self.output=Path(output or Path(__file__).resolve().parents[2]/'records')
        self._ui_watchdog_observation=None
    def emit(self,kind,**data):
        if kind=='sample':data['received_monotonic']=time.monotonic()
        self.events.put(dict(kind=kind,**data))
    def pulse(self):self.last_ui=time.monotonic()
    @property
    def ui_watchdog_observation(self):
        snapshot=self._ui_watchdog_observation
        return dict(snapshot) if snapshot is not None else None
    def request(self,action,data=None):
        if action=='servo_target':
            value=continuous_servo_target(data)
            with self.servo_target_lock:self.servo_target_pending=value
        else:
            if action in ('stop','servo_end'):
                with self.servo_target_lock:self.servo_target_pending=None
            if action=='stop':self.stopping.set()
            else:self.commands.put_nowait((action,data))
    def close(self):self.request('stop');self.disconnect.set()
    def trace_ui_action(self,kind,**fields):
        # UI never writes/fsyncs evidence or fabricates a heartbeat here.
        self.ui_action_events.put(dict(event='ui_action_'+kind,ui_monotonic=time.monotonic(),**fields))
    def drain_ui_actions(self,record):
        while True:
            try:item=self.ui_action_events.get_nowait()
            except queue.Empty:return
            record(**item)
    def persist_reset_snapshot(self,snapshot,log,record):
        log.checkpoint()
        path=self.output/('reset-before-'+datetime.now().strftime('%Y%m%dT%H%M%S_%f')+'.json')
        document=dict(snapshot,journal=str(log.path),saved_at=datetime.now().isoformat())
        with path.open('x',encoding='utf-8') as target:
            json.dump(document,target,ensure_ascii=False,indent=2);target.flush();os.fsync(target.fileno())
        record(event='reset_snapshot_saved',path=str(path));log.checkpoint()
        # Synchronous tee sees this only after both durability barriers succeeded,
        # before ResetFlow may queue/send the single RESET command.
        self.emit('reset_snapshot_saved',path=str(path),epoch=document['epoch'],
                  words=list(document['words']),operation=document['operation'])
        return str(path)
    def persist_zero_profile(self,document,log,record):
        from dataclasses import asdict
        phase=document['phase'];suffix=datetime.now().strftime('%Y%m%dT%H%M%S_%f')
        path=self.output/('pendulum-zero-'+phase+'-'+suffix+'.json')
        payload=dict(document,journal=str(log.path),backend=self.backend,identity=self.profile_identity,saved_at=datetime.now().isoformat())
        def durable(target,data):
            with target.open('x',encoding='utf-8') as f:
                json.dump(data,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
        durable(path,payload);log.checkpoint()
        fields=dict(path=str(path),before_words=list(document['before_words']),words=list(document['words']),
                    pend_down=document['words'][5],epoch=document['epoch'])
        if phase=='verified':
            profile=self.output/('calibration-zero-'+suffix+'.json')
            durable(profile,dict(schema='furuta-fpga-r1-calibration-v2',units='count, A/count, unsigned electrical turns/65536',
                parameters=asdict(Calibration.from_words(document['words'])),hardware_verified=self.backend=='hardware',
                verification='commit ACK + exact read8 + fresh stopped calibrated snapshot',evidence_path=str(path),identity=self.profile_identity))
            fields['profile_path']=str(profile)
        record(event='calibration_zero_'+phase,**fields);log.checkpoint()
        # EvidenceQueue.put may synchronously bind/confirm the owner's ledger.
        # Any exception propagates to the flow's STOP before apply/demo dispatch.
        self.emit('calibration_zero_'+phase,**fields)
        return fields.get('profile_path',str(path))
    def run(self):
        stream=None;log=None;session=None;stop_confirmed=False
        def record(**fields):
            if log:
                event=fields.pop('event')
                log.event(event,t=time.monotonic(),wall_time=datetime.now().isoformat(),**fields)
        try:
            self.output.mkdir(parents=True,exist_ok=True)
            log=AsyncJournal('qt_fpga_session',self.output)
            record(event='serial_open_begin',port=self.port,baud=460800,backend=self.backend,
                   serial_factory=getattr(self.serial_factory,'__module__','')+'.'+getattr(self.serial_factory,'__qualname__',type(self.serial_factory).__name__),
                   device_identity=self.profile_identity,exact_device_bound=bool(self.profile_identity))
            stream=self.serial_factory(port=self.port,baudrate=460800,timeout=.01,write_timeout=.1)
            record(event='serial_open_complete',backend=self.backend,stream_type=type(stream).__module__+'.'+type(stream).__qualname__)
            def send(data):
                record(event='tx_begin',tx_intent=data.hex())
                if stream.write(data)!=len(data):raise IOError('串口短写入')
                record(event='tx_complete',tx=data.hex())
            def persist_reset(snapshot):
                return self.persist_reset_snapshot(snapshot,log,record)
            def persist_zero(document):
                return self.persist_zero_profile(document,log,record)
            def session_emit(kind,**fields):
                if kind in ('error','rejected','connected','command_reply','reset_complete','entry_preparation','swing_context_queued'):
                    record(event='session_'+kind,**fields)
                self.emit(kind,**fields)
            if self.bootstrap_profile is not None:
                from .fpga_bootstrap import ColdBootstrap
                session=ColdBootstrap(send,session_emit,self.profile_identity,self.bootstrap_profile,self.bootstrap_guard)
            else:session=Session(send,session_emit,persist_reset=persist_reset,persist_zero=persist_zero,profile_identity=self.profile_identity,manual_control=self.manual_control,build14_entry_compat=self.build14_entry_compat)
            while not self.disconnect.is_set():
                self.drain_ui_actions(record)
                checked_at=time.monotonic();last_ui=self.last_ui;age=checked_at-last_ui
                observation=dict(checked_monotonic=checked_at,last_ui=last_ui,age_s=age,timed_out=age>.4)
                self._ui_watchdog_observation=observation
                if observation['timed_out']:
                    session.fail('界面心跳超时')
                    record(event='ui_watchdog_timeout',**observation,ui_activity=dict(getattr(self.ui_activity,'live',self.ui_activity)))
                    self.emit('ui_watchdog_timeout',**observation)
                    break
                if self.stopping.is_set():
                    self.stopping.clear()
                    with self.servo_target_lock:self.servo_target_pending=None
                    while not self.commands.empty():self.commands.get_nowait()
                    session.stop()
                try:action,data=self.commands.get_nowait()
                except queue.Empty:pass
                else:
                    record(event='request_received',action=action,data_repr=repr(data))
                    try:
                        if isinstance(session,Session):session.build14_entry_compat=self.build14_entry_compat
                        session.request(action,data)
                        record(event='request_admitted',action=action)
                    except ValueError as exc:
                        record(event='request_rejected',action=action,text=str(exc))
                        self.emit('sequence_state',active=getattr(session,'demo',None) is not None,sequence='current')
                        self.emit('rejected',text=str(exc))
                with self.servo_target_lock:
                    target=self.servo_target_pending;self.servo_target_pending=None
                if target is not None:
                    try:session.request('servo_target',target)
                    except ValueError as exc:self.emit('rejected',text=str(exc))
                data=stream.read(min(max(getattr(stream,'in_waiting',1),1),4096))
                if data:
                    record(event='rx',rx=data.hex());session.feed(data)
                session.poll()
                if session.failed:break
            # Explicit disconnect never leaves a queued ARM to be transmitted.
            self.drain_ui_actions(record)
            send(encode(2,0,session.seq,0))
        except Exception as exc:
            self.emit('error',text=str(exc))
            try:record(event='error',text=str(exc))
            except Exception:pass
        finally:
            if stream is not None:
                try:
                    data=encode(2,0,session.seq if session else 0,0)
                    # STOP must still reach the serial port when the journal has failed.
                    count=stream.write(data)
                    record(event='final_stop_complete',tx=data.hex(),bytes_written=count)
                    # Drain a fresh STOP ACK and stopped snapshot before closing.
                    # A failed session never resumes commands or clears a fault.
                    decoder=StreamDecoder()
                    baseline=(getattr(session,'status_row',None) or getattr(session,'latest',None) or {})
                    epoch=expected_stop_epoch(session,baseline)
                    confirmation=StopConfirmation(session.seq if session else 0,epoch,baseline.get('protocol_revision'))
                    deadline=time.monotonic()+.6
                    while time.monotonic()<deadline and count==len(data):
                        received=stream.read(min(max(getattr(stream,'in_waiting',1),1),4096))
                        if received:record(event='shutdown_rx',rx=received.hex())
                        for frame in decoder.feed(received):
                            row=confirmation.feed(frame,time.monotonic())
                            if row is not None:
                                plot_current=False
                                if session and session.latest:
                                    ticks=(row['stamp']-session.latest['stamp'])&0xffffffff
                                    plot_current=ticks<50_000_000 and row.get('control_sample_fresh',True)
                                    if plot_current:row['t']=session.latest['t']+ticks/50_000_000
                                record(event='shutdown_stopped_snapshot',row=row)
                                stop_confirmed=True
                                if plot_current:self.emit('sample',row=row)
                                self.emit('busy',value=False);self.emit('stop_confirmed',row=row)
                                break
                        if stop_confirmed:break
                except Exception:pass
                stream.close()
                try:record(event='serial_closed')
                except Exception:pass
            if log:
                try:log.close()
                except Exception as exc:self.emit('error',text='Journal flush failed: '+str(exc))
            self.emit('disconnected',text='串口已关闭；已收到 STOP 回执和停止快照' if stop_confirmed else '串口已关闭；停止状态若无遥测确认则未知')


