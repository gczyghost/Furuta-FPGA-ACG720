"""Automatic demo completion gates; never used to admit manual commands."""
import math
from .legacy_servo_entry import LegacyServoEntry


class PeakShowcase:
    def __init__(self,s):
        self.s=s;self.started=s.clock();self.epoch=s.latest['reset_epoch'];self.stage='enter'
        self.stage_at=self.started;self.previous=None;self.platform_ticks=0
        self.drop_qualification=None;self.last_drop_read=-1.

    def command(self,action,data=None):
        s=self.s;previous=s.demo_dispatch;s.demo_dispatch=True
        try:s.request(action,data)
        finally:s.demo_dispatch=previous

    def prepare(self,action,data=None):
        s=self.s;previous=s.demo_dispatch;s.demo_dispatch=True
        try:
            if not LegacyServoEntry.begin(s,action,data,automatic=True):
                raise ValueError('自动最高速演示需要新鲜v4/MOT2直立连续会话；未发送速度命令')
        finally:s.demo_dispatch=previous

    def drop_ready(self,now):
        q=self.drop_qualification
        return q is not None and now-q[0]<.2 and q[1]&7==3

    def poll(self):
        s=self.s;now=s.clock();r=s.communication_status()
        if now-self.started>120 or not r or not r.get('control_sample_fresh') or not r['sensor_valid'] or r['fault'] or not r['run'] or r.get('reset_epoch')!=self.epoch:
            s.operation_failed('最高速演示超时/失鲜/运行或世代异常，取消后继动作');return
        if self.stage=='enter':
            if s.entry_preparation is None and s.motion and s.motion['kind']=='servo' and s.motion['active_at'] is not None:
                self.command('servo_target',200);self.stage='platform';self.stage_at=now
                s.emit('log',text='演示目标 +200 rpm：等待参考与有效实测速度平台，尚未开始1秒计时')
        elif self.stage=='platform':
            if now-self.stage_at>30:s.operation_failed('30秒未完成+200rpm实测平台');return
            if self.previous is not None and r['stamp']==self.previous[0]:return
            good=(r.get('telemetry_current',True) and r.get('reference_valid') and r.get('sensor_velocity_valid') and r.get('motion_kind')==3 and
                  r.get('motion_active') and not r.get('finite_speed_active') and not r.get('servo_settling') and
                  abs(r['reference_w']*30/math.pi-200)<.1 and abs(r['sensor_velocity']*30/math.pi-200)<5)
            ticks=(r['stamp']-self.previous[0])&0xffffffff if self.previous is not None else 0
            gap=now-self.previous[1] if self.previous is not None else 0
            if good and self.previous is not None and self.previous[2] and 0<ticks<=7_500_000 and 0<=gap<=.15:self.platform_ticks+=ticks
            else:self.platform_ticks=0
            self.previous=(r['stamp'],now,good)
            if self.platform_ticks>=50_000_000:
                s.emit('log',text='已确认+200rpm参考/实测连续平台1秒；请求END并等待真实退出')
                self.command('servo_end');self.stage='end';self.stage_at=now
        elif self.stage=='end':
            if s.motion is None and not s.busy and not s.queue and s.pending is None and r['mode']==4 and r.get('motion_arrived') and not r.get('motion_active') and not r.get('servo_settling'):
                self.stage='return';self.prepare('drop',1)
        elif self.stage=='return':
            if s.entry_preparation is not None and r['mode']==4 and r.get('motion_arrived') and abs(r['q'])<.015 and abs(r['motion_goal'])<.015 and not s.queue and s.pending is None and now-self.last_drop_read>=.1:
                s._put(0x12,18,0,'showcase_drop_qualification');self.last_drop_read=now
            if s.entry_preparation is None:self.stage='drop'
        elif self.stage=='drop':
            if not s.busy and s.drop_wait is None and not s.queue and s.pending is None:
                s.showcase=None;s.emit('log',text='最高速短段、真实退出、回零和落摆已完成')
