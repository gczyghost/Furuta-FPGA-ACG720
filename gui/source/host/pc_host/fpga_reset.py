"""Explicit RST1 transaction. RESET is sent once; a lost ACK is reconciled by epoch."""
from .fpga_protocol import hold_phase
import copy
import math

RST1=0x52535431


class ResetFlow:
    def __init__(self,session,persist,initial,operation='reset'):
        self.s=session;self.persist=persist;self.initial=copy.deepcopy(initial)
        self.phase='stop';self.started=session.clock();self.before={};self.after={}
        self.before_epoch=None;self.after_epoch=None;self.snapshot_path=None
        self.operation=operation

    def poll(self):
        s=self.s;now=s.clock()
        if now-self.started>10:s.operation_failed('软复位核验超时，未自动重试');return
        if self.phase=='health' and now-self.health_after>2:
            s.operation_failed('复位/清错读回完成，但健康资格未就绪；请检查采样和设备状态');return
        status=s.communication_status()
        if self.phase=='health' and status and not status['run'] and not status['fault'] and s.assembler.started>=self.health_after:
            if self.operation!='reset' or status.get('reset_epoch')==self.after_epoch:
                healthy=bool(status.get('control_sample_fresh',True) and status.get('sensor_valid') and status.get('ready') and now-s.last_rx<.25)
                s.device_words=[self.after[i] for i in range(8)];s.busy=False;s.reset=None
                s.emit('reset_complete',operation=self.operation,before_epoch=self.before_epoch,epoch=self.after_epoch,words=s.device_words,snapshot_path=self.snapshot_path,healthy=healthy)
                s.emit('busy',value=False);return
        if self.phase=='stop' and not s.queue and s.pending is None and s.await_state is None:
            status=s.communication_status()
            if not status or status['run']:return
            self.phase='before'
            if self.operation=='reset':s._put(0x12,33,0,'reset_capability');s._put(0x12,34,0,'reset_epoch_before')
            for addr in range(8):s._put(0x12,addr,0,'reset_before')

    def resync(self):
        self.phase='verify';self.s.synchronized=False;self.s._put(4,tag='reset_sync')

    def reply(self,p,frame):
        s=self.s;tag=p['tag'];value=frame['value']
        if tag=='reset_capability':
            if value!=RST1:s.operation_failed('RST1能力复核失败，未发送复位')
        elif tag=='reset_epoch_before':self.before_epoch=value
        elif tag=='reset_before':
            self.before[p['addr']]=value
            if p['addr']==7:
                snapshot=dict(operation=self.operation,initial=self.initial,stopped=copy.deepcopy(s.communication_status()),epoch=self.before_epoch,words=[self.before[i] for i in range(8)])
                try:
                    self.snapshot_path=self.persist(snapshot)
                    if not isinstance(self.snapshot_path,str) or not self.snapshot_path:raise ValueError('保存入口未返回现场路径')
                except Exception as exc:s.operation_failed('复位前现场保存失败：'+str(exc));return
                self.phase='reset'
                if self.operation=='reset':
                    s.assembler.expected_epoch=(self.before_epoch+1)&0xffffffff
                    s._put(0x2a,0,RST1,'reset_command')
                else:s._put(3,tag='reset_command')
        elif tag=='reset_command':
            if self.operation=='reset' and self.before_epoch is not None:
                # A matched new RST ACK supersedes any older epoch readback.
                # Clear only here; a queued/unacknowledged reset proves nothing.
                s.verified_reset_epoch=None
                s.stop_reset_epoch=(self.before_epoch+1)&0xffffffff
            self.resync()
        elif tag=='reset_sync':
            s.synchronized=True;s.last_hb=0
            if self.operation=='reset':s._put(0x12,34,0,'reset_epoch_after')
            for addr in range(8):s._put(0x12,addr,0,'reset_after')
        elif tag=='reset_epoch_after':
            self.after_epoch=value
            if value==(self.before_epoch+1)&0xffffffff:s.verified_reset_epoch=value
        elif tag=='reset_after':
            self.after[p['addr']]=value
            if p['addr']==7:
                if self.operation=='reset' and self.after_epoch!=(self.before_epoch+1)&0xffffffff:
                    s.operation_failed('复位世代未按一次复位增加，结果未确认，禁止自动重试');return
                if self.after!=self.before:s.operation_failed('复位后8字标定或物理零点改变');return
                self.phase='health';self.health_after=s.clock()


class InitializeFlow:
    """Explicit enable initialization, retaining zero points and never clearing faults."""
    def __init__(self,session,profile,after='hold'):
        self.s=session;self.profile=profile;self.after=after;self.phase='read';self.started=session.clock();self.reused=False
        session.device_words=None;session.readback={};session.expected=None;session.busy=True
        for addr in range(8):session._put(0x12,addr,0,'read')

    def poll(self):
        s=self.s;now=s.clock();row=s.latest
        if not row or now-s.last_rx>.25:return
        if row['fault'] or row['run']:s.fail('初始化期间状态不再是无故障停止，未自动恢复');return
        if now-self.started>5:s.fail('初始化未在5秒内完成，未自动重试');return
        if s.queue or s.pending is not None or s.await_state is not None:return
        words=list(self.profile.words)
        if self.phase=='read' and s.device_words is not None:
            if row['calibrated']:
                if s.device_words!=words:s.fail('现有有效参数与绑定配置不同；未覆盖物理零点');return
                self.phase='health';self.health_at=now;self.reused=True
            else:
                s.expected=words;s.readback={};s.busy=True
                for addr,value in enumerate(words):s._put(0x10,addr,value,'write')
                s._put(0x11,tag='commit')
                for addr in range(8):s._put(0x12,addr,0,'verify')
                self.phase='apply'
        elif self.phase=='apply' and s.device_words==words and row['calibrated']:
            self.phase='health';self.health_at=now
        if self.phase=='health':
            if now-self.health_at>2:s.fail('初始化后2秒内资格未就绪；未CLEAR或使能');return
            if row['ready'] and row['sensor_valid'] and row['calibrated']:
                s.initialization=None;s.busy=False
                s.emit('initialization_complete',reused=self.reused,words=words)
                if self.after=='continuous':s.request('continuous')
                else:s.request('session_home',hold_phase(row['q']))
                s.emit('busy',value=s.busy)
