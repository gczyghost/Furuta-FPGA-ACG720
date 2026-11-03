"""Bounded motion planning for MOT1; no transport or implicit power control."""
import math

def continuous_servo_target(rpm):
    """Validate a v27-style live target without allocating a finite recipe."""
    if type(rpm) is not int or abs(rpm)>200:
        raise ValueError('持续伺服需要整数转速 −200..200 rpm')
    return rpm

def position_request(row, target, *, multiturn=False, manual_control=False):
    if not isinstance(target,(int,float)) or not math.isfinite(target):raise ValueError('位置目标必须是有限数')
    if not multiturn:
        bank,_,goal,up=fast_request(row,math.degrees(target-row['q']),manual_control=manual_control)
        return bank,round(target*65536),goal,up
    word=round(target*65536)
    if abs(word)>205887:raise ValueError('绝对位置使用 -π..π 的物理角度')
    up=row['run'] and row['mode'] in (3,4)
    continuous_home=row.get('keep_session') and row['mode']==0 and row.get('home_settled')
    origin=round((row['motion_goal'] if up or continuous_home else row['q'])*65536)
    delta=word-origin
    if delta>205888:delta-=411775
    elif delta< -205888:delta+=411775
    bank,_,goal,up=fast_request(row,math.degrees(delta/65536),multiturn=True,manual_control=manual_control)
    return bank,word,goal,up

def demo_recipe(name, *, full_session=False):
    if name=='v27_full':
        if not full_session:raise ValueError('完整演示需要 MOT2、ENA1 和原子多圈遥测；整组未启动，未跳过步骤')
        moves=[('fast',d) for d in (90,-90,180,-180,360)]
        return [('session_home',0.),('session_swing',None)]+moves+[('peak_showcase',None)]
    if name!='bounded':raise ValueError('未知演示配方')
    return [('home',0.),('fast',30),('fast',-30),('long_run',None),
            ('fast',30),('fast',-30),('servo',(5,.5)),('position',0.),('drop',1)]

def fast_request(row, degrees, *, multiturn=False, manual_control=False):
    if not isinstance(degrees,(int,float)) or not math.isfinite(degrees) or abs(degrees)<.01:
        raise ValueError('快移角度必须是有限非零值')
    upright=row['run'] and row['mode'] in (3,4)
    down_session=multiturn and row.get('keep_session') and row['mode']==0 and row.get('home_settled') and row.get('motion_kind')==0
    if not manual_control and (row['fault'] or (row['run'] and not upright and not down_session) or (not row['run'] and not row['ready'])):
        raise ValueError('快移需要下垂待机或稳定直立')
    angle=math.radians(degrees)
    origin=row['motion_goal'] if multiturn and (upright or down_session) else row['q']
    goal=origin+angle
    if multiturn:
        if not manual_control and (abs(row['q'])>411775/65536 or abs(goal)>64):raise ValueError('等待坐标归一化和稳定到位后再启动')
    elif max(abs(row['q']),abs(goal))>2.78:raise ValueError('路径超过 MOT1 固件的 ±2.78 rad 范围')
    banks=((30,5),(90,6),(180,7),(360,8)) if upright else ((30,0),(90,1),(170,2),(180,3),(360,4))
    bank=next((b for d,b in banks if abs(degrees)<=d),None)
    if bank is None:raise ValueError('快移角度超过轨迹范围')
    return bank,round(angle*65536),goal,upright

def servo_request(row, rpm, seconds, *, multiturn=False, manual_control=False):
    if not isinstance(rpm,int) or abs(rpm)>200 or not isinstance(seconds,(int,float)) or not math.isfinite(seconds) or not .1<=seconds<=10:
        raise ValueError('有限伺服需要整数转速 ±200 rpm、时长 0.1..10 s')
    if not manual_control and (not row['run'] or row['mode'] not in (3,4) or row['fault']):
        raise ValueError('速度伺服需要稳定直立')
    # Conservative travel budget includes acceleration and jerk-limited braking.
    speed=abs(rpm)*math.pi/30
    budget=speed*(seconds+2+speed/2)+.2
    if multiturn:
        # Core speed protection remains 14 rad/s. Keep host requests below it.
        if not manual_control and abs(rpm)>120:raise ValueError('当前整机转速请求限制为 ±120 rpm')
        if not manual_control and abs(row['q'])>411775/65536:raise ValueError('等待坐标归一化后再启动伺服')
    elif abs(row['q'])+budget>2.78:raise ValueError('加减速路径超过当前摆臂范围，请降低转速或缩短时间')
    return rpm,float(seconds)
