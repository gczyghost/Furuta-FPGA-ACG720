"""Bounded orchestration of accepted board motions, using live acknowledgements."""
import math
import time
from .motion import DEMO_NAME,DEMO_STEPS,upright_steps
from .protocol import ProtocolError

class MotionSession:
    def wait_motion_condition(self,predicate,seconds,text):
        from .controller import Cancelled
        self.emit('phase',text=text,seconds=-1)
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if self.stop_requested.is_set() or self.disconnect_requested.is_set():
                raise Cancelled('已取消流程 · 等待关桥确认')
            self.line()
            if not self.enabled or self.report.get('last_status',{}).get('fault'):
                raise ProtocolError('设备已停止，流程取消')
            if self.budget and time.monotonic()-self.budget_at<.4 and predicate():return
        raise ProtocolError(text+'超时；保持当前状态，流程已取消')

    def fast_ready(self):
        if not self.budget or time.monotonic()-self.budget_at>=.4:return False
        mode=self.report.get('last_status',{}).get('mode')
        flag=1 if mode=='V' else 2 if mode=='H' else 0
        return bool(self.budget['available']&flag) and not self.budget['available']&4 and self.budget['dose']<=.65 and not self.motion_active

    def upright_ready(self):
        return self.report.get('last_status',{}).get('mode')=='V' and self.fast_ready()

    def fast_step(self,degrees):
        if degrees not in (-360,-180,-90,-30,30,90,180,360):raise ProtocolError('未验收的快移步长')
        self.wait_motion_condition(self.fast_ready,25,'等待快移姿态 / 额度')
        upright=self.report['last_status']['mode']=='V';prefix='UPSTEP' if upright else 'QSTEP'
        self.ack(f'{prefix},{degrees}','COMMAND_OK,'+prefix)
        self.motion_active=True;self.emit('motion',active=True,completed=False,operation='upright' if upright else 'step')
        self.report.setdefault('fast_commands',[]).append(dict(degrees=degrees,mode='V' if upright else 'H',at=time.monotonic()))
        self.wait_motion_condition(lambda:not self.motion_active,15,f'快速移位 {degrees:+d}° · 等待到位')

    def upright_step(self,degrees):
        self.fast_step(degrees)

    def execute_turn(self,direction):
        if direction not in (-1,1):raise ProtocolError('旋转方向无效')
        if not self.enabled:self.execute_session('enable')
        self.fast_step(direction*360)

    def execute_position(self,target_degrees):
        if not math.isfinite(target_degrees) or abs(target_degrees)>180:raise ProtocolError('目标需在 ±180° 内')
        if self.motion_active:raise ProtocolError('请等待当前动作完成')
        if not self.enabled:self.execute_session('enable')
        self.wait_motion_condition(self.fast_ready,25,'等待快速定位姿态 / 额度')
        answer=self.ack(f'FAST,{math.radians(target_degrees):.7f}',lambda line:line in ('COMMAND_OK,FAST','COMMAND_OK,FAST,NOOP'))
        if answer=='COMMAND_OK,FAST,NOOP':return
        self.motion_active=True;self.emit('motion',active=True,completed=False,operation='position')
        self.report.setdefault('position_commands',[]).append(dict(degrees=target_degrees,at=time.monotonic()))
        self.wait_motion_condition(lambda:not self.motion_active,15,f'快速定位 {target_degrees:+.0f}° · 等待到位')

    def execute_demo(self):
        if self.enabled and self.report.get('last_status',{}).get('mode')=='V':
            raise ProtocolError('请先受控下落，再开始完整演示')
        self.emit('demo',active=True,text=DEMO_NAME)
        record=dict(started_at=time.monotonic(),recipe=list(DEMO_STEPS),phases=['down','upright'],completed=False)
        self.report.setdefault('demos',[]).append(record)
        try:
            self.emit('sequence',index=1,total=13);self.execute_position(0)
            for index,degrees in enumerate(DEMO_STEPS,2):
                self.emit('sequence',index=index,total=13);self.fast_step(degrees)
            self.emit('sequence',index=7,total=13);self.execute_session('continuous')
            self.wait_motion_condition(self.upright_ready,50,'演示 · 回零起摆，等待稳摆')
            for index,degrees in enumerate(DEMO_STEPS,8):
                self.emit('sequence',index=index,total=13);self.fast_step(degrees)
            self.emit('sequence',index=13,total=13);self.execute_session('drop_positive')
            if not self.motion_active:raise ProtocolError('演示下落未被接受')
            self.wait_motion_condition(lambda:not self.motion_active and self.report['last_status']['mode']=='H' and self.latest_sample.get('en0'),15,'演示 · 受控下落与抑振')
            record.update(completed=True,finished_at=time.monotonic(),enabled_standby=True)
            self.emit('phase',text='演示完成 · 回零下垂，已使能待命',seconds=0)
        finally:
            self.emit('demo',active=False,text='演示完成 · 已使能待命' if record['completed'] else '演示已中止')
            self.save()
