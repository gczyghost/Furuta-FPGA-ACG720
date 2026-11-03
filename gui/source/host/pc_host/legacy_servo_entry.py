"""Build14-only, explicit phase-zero preparation before a servo entry.

One position request at most; observe normal atomic rebase, never add a turn.
This verifies completion of preparation, not firmware motion eligibility bits.
"""
import math

TAU=411775/65536
NEAR=math.radians(2)

def local_zero(row):
    return abs(row['q'])<=NEAR and abs(row['motion_goal'])<=NEAR

class LegacyServoEntry:
    def __init__(self,session,action,data,row,position_requested):
        self.s=session;self.action=action;self.data=data;self.epoch=row['reset_epoch']
        self.started=session.clock();self.position_requested=position_requested;self.ack_at=None
        self.previous=None;self.stable=[];self.idle_at=None

    @classmethod
    def begin(cls,s,action,data,automatic=False):
        row=s.communication_status()
        if not ((automatic or s.build14_entry_compat and s.manual_control) and not s.entry_dispatch and
                action in (('servo_begin','drop') if automatic else ('servo_begin','servo','speed')) and row and row.get('protocol_revision')==4 and
                row.get('run') and not row.get('fault') and row.get('mode') in (3,4) and
                row.get('keep_session') and row.get('control_sample_fresh') and row.get('sensor_valid') and
                s.motion_version==2 and s.capabilities.get(26)):
            return False
        if local_zero(row) and not automatic:return False
        if action=='servo_begin' and not s.capabilities.get(32):return False
        if action=='speed' and (not s.capabilities.get(35) or type(data) is not int or abs(data) not in {1,5,10,15,20,30,40,50,60,80,100,120,150,180,200}):return False
        if action=='servo':
            from .fpga_motion import servo_request
            servo_request(row,*data,multiturn=True,manual_control=True)
        # Phase-zero but local +/-TAU must idle for the board's own rebase.
        # Sending another command would inhibit that rebase or add real travel.
        position_requested=abs(math.remainder(row['motion_goal'],TAU))>(.015 if action=='drop' else NEAR)
        flow=cls(s,action,data,row,position_requested)
        flow.automatic=automatic
        s.emit('entry_preparation',phase='start',action=action,data=data,epoch=flow.epoch,position_requested=position_requested,
               text='B14过渡入口准备：'+('先定位到物理零位，会改变位置；确认实际局部零点后续接原命令。' if position_requested else '物理零位尚待局部坐标归帧；等待板端自然归帧后续接原命令，不额外转圈。'))
        if position_requested:s.request('position',0.)
        s.entry_preparation=flow;s.busy=True;s.emit('busy',value=True)
        return True

    def abort(self,reason):
        self.s.emit('entry_preparation',phase='aborted',action=self.action,reason=reason,text='B14过渡准备取消：'+reason)
        self.s.operation_failed('B14入口准备未完成：'+reason+'；原速度命令未发送')

    def poll(self):
        s=self.s;now=s.clock();row=s.communication_status()
        if now-self.started>18:self.abort('18秒截止，不重复定位');return
        if row is None or not row.get('control_sample_fresh') or not row.get('sensor_valid'):
            self.abort('采样失鲜或无效');return
        if row.get('reset_epoch')!=self.epoch or row['fault'] or not row['run']:
            self.abort('世代、运行或故障状态改变');return
        if self.position_requested and self.ack_at is None:return
        if self.ack_at is not None and s.status_at<self.ack_at:return
        arrived=row['mode']==4 and row.get('motion_arrived') and not row.get('motion_active')
        if arrived:
            if self.idle_at is None:self.idle_at=now
            if now-self.idle_at>2:self.abort('到位后2秒仍未确认局部零点/原子归帧');return
        else:
            self.stable.clear();self.idle_at=None
        previous=self.previous
        if previous is not None and row['stamp']==previous['stamp']:return
        if previous is not None:
            if not 0<(row['stamp']-previous['stamp'])&0xffffffff<=12_500_000:
                self.abort('准备采样时间不连续');return
            turns=row['frame_turns']-previous['frame_turns']
            if turns and (abs(turns)!=1 or abs(row['q']-previous['q']+turns*TAU)>NEAR or
                          abs(row['motion_goal']-previous['motion_goal']+turns*TAU)>NEAR):
                self.abort('局部坐标与圈数不是一致的原子归帧');return
        self.previous=dict(row)
        # Continuity and atomic rebase apply to every fresh preparation sample,
        # including position travel. Arrival starts a separate stability window.
        if not arrived:return
        stable=local_zero(row) and abs(row['w'])<.15 and abs(row['v'])<.2
        if self.action=='drop':stable=stable and abs(row['q'])<.015 and abs(row['motion_goal'])<.015 and not row.get('servo_settling')
        if stable:self.stable.append(now)
        else:self.stable.clear()
        if len(self.stable)<3 or now-self.stable[-3]>.25 or s.queue or s.pending is not None:return
        if getattr(self,'automatic',False):
            if self.action=='servo_begin' and not row.get('servo_eligible'):return
            if self.action=='drop' and (s.showcase is None or not s.showcase.drop_ready(now)):return
        s.entry_preparation=None;s.motion=None;s.busy=False
        s.emit('entry_preparation',phase='complete',action=self.action,epoch=self.epoch,row=dict(row),
               text='B14入口准备完成：实际局部位置和目标已确认近零，续接原请求一次。')
        s.entry_dispatch=True
        previous_dispatch=s.demo_dispatch;s.demo_dispatch=True
        try:s.request(self.action,self.data)
        except ValueError as exc:s.operation_failed(str(exc))
        finally:s.entry_dispatch=False;s.demo_dispatch=previous_dispatch
