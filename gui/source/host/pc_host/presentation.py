"""Display-only formatting. Raw serial evidence and control remain unchanged."""
import json
import math

STAGE_NAMES = {'P':'静止检查','D':'电流映射','A':'电角度对齐','Q':'小电流校验',
               'H':'下垂抑摆移位','V':'起摆与稳摆'}
REASONS = {'complete':'正常结束','stopped':'主动停止','LEASE_TIMEOUT':'心跳中断，已关桥',
           'PASSIVE_SETTLE_REQUIRED':'采样需复核，将保持关桥重新检查',
           'HOME_NOT_REACHED':'尚未到位，等待恢复','INITIAL_PENDULUM_MOVING':'尚未静止，等待恢复'}

def axis_ticks(low, high, angle=True):
    """Ticks are round multiples inside the viewport, independent of its edges."""
    desired=(high-low)/5
    if angle:
        steps=(.01,.02,.05,.1,.2,.5,1,2,5,10,15,30,45,90,180,360,720)
        step=min(steps,key=lambda s:abs(math.log(s/desired))) if desired<=steps[-1] else None
    else:
        step=None
    if step is None:
        base=10**math.floor(math.log10(desired))
        step=next(s*base for s in (1,2,5,10) if s*base>=desired)
    first=math.ceil((low-1e-9)/step)
    last=math.floor((high+1e-9)/step)
    return [round(i*step,8) for i in range(first,last+1)]

def tick_label(value):
    return f'{0 if abs(value)<1e-8 else value:g}'


def command_label(command):
    """Only user-visible motion/stop commands, never queries or heartbeats."""
    name, _, value = command.partition(',')
    simple = {'S': '停止/关桥', 'ENABLE': '使能', 'SWING': '回零起摆',
              'HOME': '回零', 'RECOVER': '核验恢复', 'RESET': '软复位'}
    if name in simple:
        return simple[name]
    if command=='SERVO,END':return '减速退出伺服'
    try:
        number = float(value.split(',')[0])
        if not math.isfinite(number):
            return None
        if name == 'SPEED':
            return f'速度轨迹 {number:+g} rpm'
        if name == 'SERVO':
            return f'速度伺服 {number:+g} rpm'
        if name == 'FAST':
            return f'定位 {math.degrees(number):+.0f}°'
        if name in ('UPSTEP', 'QSTEP'):
            return f'快移 {number:+g}°'
        if name == 'DROP':
            return '正向下落' if number > 0 else '反向下落'
        if name == 'V':
            return '起摆稳摆'
    except ValueError:
        pass
    return None

def port_name(text):
    return text.split(' · ',1)[0].strip()

def ordered_ports(items):
    return sorted(items,key=lambda p:(p.vid is None, p.device))

def select_port(items, previous='', remembered=''):
    devices={p.device for p in items}
    for name in (previous,remembered):
        if name in devices:return name
    return items[0].device if items else ''

def connection_error(port, error):
    detail=str(error)
    if 'configure port' in detail or '参数错误' in detail:
        return f'{port} 不接受当前串口参数。请选择 USB 串口（本机倒立摆通常为 COM3），不要选择系统通信端口；也可点击“刷新”重新查找。'
    if 'Access is denied' in detail or 'PermissionError' in detail or '拒绝访问' in detail:
        return f'{port} 被占用或无法访问。请关闭其他占用该串口的上位机，再连接。'
    if 'FileNotFoundError' in detail or '找不到' in detail:
        return f'找不到 {port}。请检查 USB 连接并点击“刷新”。'
    return f'无法连接 {port}：{detail}'

def activity_entry(text):
    """Known events become summaries; verbose diagnostics stay in serial.log."""
    if text.startswith('阶段通过：'):
        try:checks=json.loads(text.split('：',1)[1])
        except (ValueError,IndexError):return text
        parts=[f"{checks.get('duration_s',0):.1f} 秒",f"{checks.get('samples',0)} 点采样"]
        for key,name,unit in [('pendulum_rms_deg','摆角 RMS','°'),('arm_rms_deg','旋臂 RMS','°'),
                              ('center_max_error_deg','目标误差','°'),('iq_rms_error_A','电流误差',' A')]:
            if key in checks:parts.append(f'{name} {checks[key]:.3f}{unit}')
        return '通过 · '+'；'.join(parts)
    if text.startswith('→ '):return None
    if not text.startswith('← '):return text
    wire=text[2:]
    key,_,payload=wire.partition(',')
    if key=='BEGIN':
        parts=payload.split(',');mode=parts[0];ms=int(parts[2])
        duration='持续运行，手动停止' if mode=='V' and ms==0 else f'{ms/1000:g} 秒'
        return f'开始 · {STAGE_NAMES.get(mode,mode)}（{duration}）'
    if key=='RESULT':
        data=json.loads(payload);reason=data.get('reason','未知')
        bridge='双桥已关闭' if data.get('en0')==data.get('en1')==0 else '桥状态待核实'
        return f"结束 · {STAGE_NAMES.get(data.get('stage'),'运行')}：{REASONS.get(reason,reason)}；{bridge}"+('；故障已锁存' if data.get('fault') else '')
    if key=='ERROR':return '设备拒绝 · '+payload
    if key=='CENTER_SET':return f'零点 · 已保存物理零点 {payload} counts'
    if key=='PENDULUM_ZERO':return '校准 · 已更新摆杆下垂零位'
    return None
