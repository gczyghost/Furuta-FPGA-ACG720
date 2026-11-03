"""Accepted motion recipes and resource presentation; no serial side effects."""
import math
from .protocol import ProtocolError

# Updated only with hardware acceptance, never populated from external text.
DEMO_STEPS = (90, -90, 180, -180, 360)
DEMO_NAME = '下垂 90° / 180° 往返与一周 → 起摆同组快移 → 受控下落'

def budget_sample(line):
    p=line.split(',')
    if len(p)!=8 or p[0]!='BUDGET':
        raise ProtocolError('额度帧格式不匹配')
    cap,dose,goal=map(float,p[1:4]);available=int(p[4]);w,v,work=map(float,p[5:])
    # The board reports an unwrapped goal after repeated single-turn motions.
    # Telemetry coordinates are not the bounded, user-entered motion command.
    if not all(math.isfinite(x) for x in (cap,dose,goal,w,v,work)) or not 0<=cap<=1.9 or not 0<=dose<=1.1 or not 0<=available<=7 or work<0:
        raise ProtocolError('额度帧数值无效')
    return dict(cap=cap,dose=dose,goal=goal,available=available,w=w,v=v,work=work)

def upright_steps(origin_degrees, target_degrees):
    """Exact signed path inside the physical endpoints, including explicit +/-180."""
    if not all(math.isfinite(x) and abs(x)<=180.001 for x in (origin_degrees,target_degrees)):
        raise ProtocolError('目标超出 ±180° 范围')
    delta=target_degrees-origin_degrees
    if abs(delta)<.01:return []
    units=round(abs(delta)/30)
    if abs(abs(delta)-units*30)>.01:return None
    sign=1 if delta>0 else -1;steps=[]
    while units:
        n=next(x for x in (6,3,1) if x<=units)
        steps.append(sign*n*30);units-=n
    return steps

def resource_text(budget, mode, fresh=True):
    if not fresh or budget is None:
        return '电流上限 —   ·   I²t 余量 —', '等待最新额度遥测'
    if mode=='I':return '电流上限 0 A   ·   电机未使能', '使能后显示动作额度'
    dose=budget['dose'];cap=budget['cap']
    text=f'电流上限 {cap:.2f} A  ·  I²t 余量 {max(0,1-dose):.3f} / 1.000 A²s'
    if budget['available']&4:reason='运动与到位确认中'
    elif dose>.65:reason='快移等待 I²t ≤0.650；下落需 ≤0.500 A²s'
    elif mode=='V' and dose>.5:reason='快移可用 · 下落等待 I²t ≤0.500 A²s'
    elif mode=='V' and not budget['available']&1:reason='正在稳摆，等待姿态与速度满足入口条件'
    elif mode=='V':reason='稳摆快移就绪 · 下落自动回零'
    elif not budget['available']&2:reason='正在定位 / 抑振，快移等待静稳'
    else:reason='下垂保持就绪'
    return text,reason
