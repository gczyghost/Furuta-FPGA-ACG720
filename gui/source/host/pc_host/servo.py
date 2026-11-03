"""Retargetable v27 speed servo. Serial I/O stays in the owner thread."""
import math
from .protocol import ProtocolError,SERVO_IDENTITY

def servo_command(rpm):
    if isinstance(rpm,bool) or not isinstance(rpm,(float,int)) or not math.isfinite(rpm) or rpm!=int(rpm) or abs(rpm)>200:
        raise ProtocolError('速度伺服目标必须为 −200 至 +200 的整数 rpm')
    return f'SERVO,{int(rpm)}'

def execute_servo(worker,action,rpm=0):
    if worker.identity!=SERVO_IDENTITY or not worker.enabled or worker.report.get('last_status',{}).get('mode')!='V':
        raise ProtocolError('速度伺服需要 PC_LIVE v27，且已进入持续稳摆')
    if action=='servo_start':
        if worker.motion_active:raise ProtocolError('请等待当前动作完成')
        worker.wait_motion_condition(lambda:worker.upright_ready() and worker.budget['dose']<=.5,25,'等待速度伺服入口')
        worker.ack('SERVO,0','COMMAND_OK,SERVO')
        worker.servo_active=True;worker.servo_ending=False;worker.motion_active=True
        worker.emit('servo',active=True,ending=False,rpm=0)
    elif action=='servo_target':
        if not worker.servo_active or worker.servo_ending:raise ProtocolError('速度伺服未运行或正在退出')
        worker.ack(servo_command(rpm),'COMMAND_OK,SERVO')
        worker.report.setdefault('servo_targets',[]).append(dict(rpm=rpm,at=worker.latest_sample.get('t')))
        worker.emit('servo',active=True,ending=False,rpm=rpm)
    else:
        if not worker.servo_active or worker.servo_ending:raise ProtocolError('速度伺服未运行或正在退出')
        worker.ack('SERVO,END','COMMAND_OK,SERVO_END')
        worker.servo_ending=True;worker.clear_servo_target()
        worker.emit('servo',active=True,ending=True,rpm=0)
