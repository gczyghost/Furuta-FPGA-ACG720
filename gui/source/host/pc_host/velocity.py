"""Finite, frozen v24 speed recipes; importing never accesses hardware."""
import math
from .protocol import ProtocolError

SPEED_IDENTITY='IDENT,ME6025_PC_LIVE,24'
SPEED_RPMS=(1,5,10,15,20,30,40,50,60,80,100,120,150,180,200)

def speed_command(rpm):
    if isinstance(rpm,bool) or not isinstance(rpm,(int,float)) or not math.isfinite(rpm) or abs(rpm) not in SPEED_RPMS:
        raise ProtocolError('速度需选择已仿真的 1–200 rpm 档位及方向')
    return f'SPEED,{int(rpm)}'

def speed_duration(rpm):
    speed_command(rpm)
    return 1.5+2*max(1.5,1.875*abs(rpm)*math.pi/30/6)+1

def execute_speed(worker,rpm):
    command=speed_command(rpm)
    if worker.identity!=SPEED_IDENTITY:
        raise ProtocolError('速度轨迹需要 PC_LIVE v24 候选固件')
    if not worker.enabled or worker.report.get('last_status',{}).get('mode')!='V':
        raise ProtocolError('请先起摆并进入持续稳摆')
    if worker.motion_active:raise ProtocolError('请等待当前动作完成')
    worker.wait_motion_condition(lambda:worker.upright_ready() and worker.budget['dose']<=.5,25,'等待速度轨迹入口')
    worker.ack(command,'COMMAND_OK,SPEED')
    worker.motion_active=True;worker.emit('motion',active=True,completed=False,operation='speed')
    worker.report.setdefault('speed_commands',[]).append(dict(rpm=rpm,duration_s=speed_duration(rpm),cruise_s=1.0))
    worker.wait_motion_condition(lambda:not worker.motion_active,speed_duration(rpm)+7,'速度轨迹 · 加速、匀速、减速及稳摆确认')
