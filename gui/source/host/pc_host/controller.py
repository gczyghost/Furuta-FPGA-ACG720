"""One serial owner, bounded actions, UI watchdog, and append-only evidence."""
import csv
from datetime import datetime
import hashlib
import json
import math
import queue
import threading
import time
from pathlib import Path

import serial
from .presentation import connection_error, command_label
from .motion import budget_sample
from .motion_session import MotionSession
from .velocity import SPEED_IDENTITY, speed_command, execute_speed
from .servo import SERVO_IDENTITY,servo_command,execute_servo

from .protocol import (ROOT, DEFAULT_OUTPUT, PROFILE_PATH, IDENTITY, LIVE_IDENTITY, LIVE_IDENTITIES, RESET_IDENTITY, profile_path, QUALIFY, STAGES, ProtocolError,
                       eligible, stopped, profile, settings, validate_stage, live_sample)


class Cancelled(RuntimeError):
    pass


class RecoverableStage(ProtocolError):
    """A bounded, bridge-off condition; never a hardware fault reset."""


class HardwareController(MotionSession, threading.Thread):
    def __init__(self, port, events, serial_factory=serial.Serial, output=None):
        super().__init__(name='ME6025 serial owner', daemon=True)
        self.port, self.events, self.serial_factory = port, events, serial_factory
        self.commands = queue.Queue(maxsize=1)
        self.stop_requested = threading.Event()
        self.disconnect_requested = threading.Event()
        self.last_ui = time.monotonic()
        self.ui_gap_max=0.
        self.last_service=0.
        self.active = False
        self.enabled = False
        self.motion_active = False
        self.servo_active = self.servo_ending = False
        self.servo_lock=threading.Lock()
        self.servo_pending=None
        self.servo_sent_at=0.
        self.stop_sent = False
        self.user_stop = False
        self.last_heartbeat = 0
        self.s = None
        self.buffer = bytearray()
        self.log = None
        self.identity = None
        self.live_writer = None
        self.live_stream = None
        self.budget=None
        self.budget_at=0
        self.latest_sample={}
        self.stage_number = 0
        self.live_last = 0
        self.last_live_at = 0
        self.live_clock_offset = None
        self.live_lag_s = 0.
        self.boot_epoch = 0
        self.expect_live = True
        self.report = dict(started_at=datetime.now().astimezone().isoformat(), port=port,
                           expected_identity=IDENTITY, data_mode='POST_STOP_CAPTURE',
                           stages=[], errors=[], stop_verified=False,
                           board_binary_hash_available=False)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.output = output or DEFAULT_OUTPUT / stamp

    def emit(self, kind, **data):
        self.events.put(dict(kind=kind, **data))

    def pulse(self):
        now=time.monotonic();self.ui_gap_max=max(self.ui_gap_max,now-self.last_ui);self.last_ui=now

    def request(self, action, target_degrees=None):
        if action=='servo_target':
            servo_command(target_degrees)
            if not self.servo_active or self.servo_ending or self.stop_requested.is_set():
                raise ProtocolError('速度伺服未运行或正在退出')
            with self.servo_lock:self.servo_pending=int(target_degrees)
            return
        if action in ('servo_start','servo_end'):
            self.commands.put_nowait(action)
            return
        if action not in ('speed', 'soft_reset', 'demo', 'turn', 'position', 'passive', 'full', 'register', 'home', 'swing', 'move', 'quick', 'continuous', 'enable', 'recover', 'drop_positive', 'drop_negative'):
            raise ValueError(action)
        if action == 'speed':
            speed_command(target_degrees);action=('speed',int(target_degrees))
        elif action == 'turn':
            if target_degrees not in (-1,1):raise ValueError('旋转方向无效')
            action=('turn',int(target_degrees))
        elif action == 'position':
            if target_degrees is None or not math.isfinite(target_degrees) or abs(target_degrees)>180:
                raise ValueError('目标角度必须在 ±180° 内')
            action=('position',float(target_degrees))
        elif action == 'move':
            if target_degrees is None or not math.isfinite(target_degrees) or abs(target_degrees)>180:
                raise ValueError('目标角度必须在 −180° 至 +180° 内')
            action = ('position', float(target_degrees))
        elif action == 'quick':
            if target_degrees not in (-170,-90,-30,30,90,170):
                raise ValueError('柔和快移步长必须为 ±30°、±90° 或 ±170°')
            action = ('quick', int(target_degrees))
        self.commands.put_nowait(action)

    def stop(self):
        self.clear_servo_target()
        self.user_stop = True
        self.stop_requested.set()

    def clear_servo_target(self):
        with self.servo_lock:self.servo_pending=None

    def take_servo_target(self):
        if not self.servo_active or self.servo_ending or self.stop_requested.is_set():return None
        now=time.monotonic()
        if now-self.servo_sent_at<.1:return None
        with self.servo_lock:
            value=self.servo_pending;self.servo_pending=None
        if value is not None:self.servo_sent_at=now;return ('servo_target',value)

    def reset_servo(self):
        self.servo_active=self.servo_ending=False
        self.clear_servo_target()
        self.emit('servo',active=False,ending=False,rpm=0)

    def disconnect(self):
        self.stop()
        self.disconnect_requested.set()

    def save(self):
        temporary = self.output / 'session.json.tmp'
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(self.output / 'session.json')

    def send(self, command):
        payload = (command + '\n').encode('ascii')
        if self.s.write(payload) != len(payload):
            raise ProtocolError('串口写入不完整')
        sent_at = time.monotonic()
        label = command_label(command)
        if label and self.latest_sample and self.last_live_at:
            self.emit('command_sent', command=command, label=label,
                      t=self.latest_sample['t'] + sent_at - self.last_live_at)
        self.log.write(f'{time.monotonic():.6f} TX {command}\n')
        self.log.flush()
        if command != 'H':
            self.emit('log', text='→ ' + command)

    def service(self):
        now = time.monotonic()
        if self.last_service:
            self.report['max_service_gap_s']=max(self.report.get('max_service_gap_s',0),now-self.last_service)
        self.last_service=now
        self.report['max_ui_pulse_gap_s']=self.ui_gap_max
        if now - self.last_ui > .5:
            if not self.disconnect_requested.is_set():
                self.report['ui_watchdog']={'gap_s':now-self.last_ui,'at':now}
                self.emit('notice',text='界面超过 0.5 秒未响应，已请求关桥并断开')
            self.disconnect_requested.set()
            self.stop_requested.set()
        if self.active:
            if self.identity in LIVE_IDENTITIES and self.expect_live and self.last_live_at and now - self.last_live_at > .5:
                self.report['errors'].append('实时遥测超过 500 ms 未更新，自动停止')
                self.stop_requested.set()
                self.last_live_at = 0
            if self.stop_requested.is_set():
                if not self.stop_sent:
                    self.send('S')
                    self.stop_sent = True
                    self.emit('phase', text='已请求停止 · 等待关桥回执和采样', seconds=0)
            elif now - self.last_heartbeat >= .15:
                self.send('H')
                self.last_heartbeat = now

    def line(self, stop_verification=False):
        self.service()
        if b'\n' not in self.buffer:
            self.buffer.extend(self.s.read(min(max(self.s.in_waiting, 1), 8192)))
        if len(self.buffer) > 262144:
            raise ProtocolError('接收缓冲区超限')
        if b'\n' not in self.buffer:
            return ''
        raw, _, remaining = self.buffer.partition(b'\n')
        self.buffer = bytearray(remaining)
        try:
            value = raw.decode('ascii', errors='strict').strip()
        except UnicodeDecodeError:
            if stop_verification:
                self.log.write('RX_STOP_FRAGMENT_HEX ' + raw.hex() + '\n')
                self.report['stop_fragments'] = self.report.get('stop_fragments', 0) + 1
                return ''
            if self.identity is not None or self.active:
                raise
            # Opening a stream can land inside an existing idle frame. Discard
            # only before the bounded identity handshake; never during control.
            self.log.write('RX_STARTUP_FRAGMENT_HEX ' + raw.hex() + '\n')
            self.log.flush()
            self.report['startup_fragments']=self.report.get('startup_fragments',0)+1
            return ''
        if value:
            self.log.write(f'{time.monotonic():.6f} RX {value}\n')
            # After STOP, only the ordered identity/status reply can establish
            # bridge state. Preserve corrupt/old telemetry in serial.log, but
            # never let it prevent receiving that reply or use it for control.
            if stop_verification and not value.startswith(('IDENT,', 'STATUS,', 'ERROR,')):
                self.report['stop_drained_lines'] = self.report.get('stop_drained_lines', 0) + 1
                return ''
            if value.startswith('BUDGET,'):
                self.budget=budget_sample(value);self.budget_at=time.monotonic()-self.live_lag_s
                self.report['last_budget']=dict(self.budget)
                self.report['budget_frames']=self.report.get('budget_frames',0)+1
                # v22 publishes retained motion fields during P/D/A/Q. Those
                # fields do not describe the qualification stage's limits.
                if self.latest_sample.get('mode') not in (None, 'I', 'H', 'V'):
                    self.budget = None
                    self.budget_at = 0
                self.emit('budget',budget=dict(self.budget) if self.budget is not None else None, received_at=self.budget_at)
                return ''
            if value.startswith('LIVE,'):
                row = live_sample(value)
                if row['sequence'] <= self.live_last:
                    raise ProtocolError('LIVE 帧序号不递增')
                self.live_last = row['sequence']
                self.last_live_at = time.monotonic()
                offset = self.last_live_at - row['t']
                self.live_clock_offset = offset if self.live_clock_offset is None else min(self.live_clock_offset, offset)
                self.live_lag_s = max(0., offset - self.live_clock_offset)
                row.update(received_at=self.last_live_at, transport_lag_s=self.live_lag_s)
                self.report['max_telemetry_lag_s'] = max(self.report.get('max_telemetry_lag_s', 0.), self.live_lag_s)
                if self.live_writer:
                    self.live_writer.writerow([0 if row['mode']=='I' else self.stage_number, time.monotonic(), *[row[k] for k in
                        ('sequence', 't', 'mode', 'theta', 'alpha', 'beta', 'iq', 'iq_ref', 'speed', 'valid', 'en0', 'en1', 'fault')], self.live_lag_s, self.boot_epoch, row.get('speed_ref','')])
                if self.live_lag_s > .5:
                    self.budget = None
                    self.stop_requested.set()
                    self.disconnect_requested.set()
                    raise ProtocolError(f'遥测积压 {self.live_lag_s:.3f} 秒，自动停止；旧数据不能作为实时反馈')
                self.latest_sample=row
                if row['mode'] not in ('I', 'H', 'V') and self.budget is not None:
                    self.budget = None
                    self.budget_at = 0
                    self.emit('budget',budget=None,received_at=0)
                self.emit('sample', sample=row)
                if self.enabled:
                    self.report['last_status'].update(mode=row['mode'],en0=int(row['en0']),en1=int(row['en1']),fault=int(row['fault']))
                    if row['mode']=='I':
                        self.reset_servo()
                        self.enabled=self.active=False
                        self.motion_active=False
                        self.emit('motion',active=False,completed=False)
                        self.emit('session', enabled=False, mode='I', reason='idle')
                if row['fault'] and self.active:
                    self.stop_requested.set()
                return ''
            if value.startswith('SESSION,'):
                info=json.loads(value.split(',',1)[1])
                self.enabled=bool(info['enabled']);self.active=self.enabled
                self.budget=None
                if not self.enabled:
                    self.reset_servo()
                    self.motion_active=False
                    self.emit('motion',active=False,completed=False)
                self.report.setdefault('session_events',[]).append(info)
                self.emit('session', **info)
            elif value=='MOTION_DONE,SERVO':
                self.motion_active=False;self.reset_servo()
                self.emit('phase',text='速度伺服已减速退出 · 继续稳摆',seconds=0)
            elif value in ('MOTION_DONE,DROP','MOTION_DONE,STEP','MOTION_DONE,UPSTEP','MOTION_DONE,SPEED'):
                self.motion_active=False
                self.emit('motion',active=False,completed=True,operation='speed' if value.endswith('SPEED') else 'upright' if value.endswith('UPSTEP') else 'step' if value.endswith('STEP') else 'drop')
            elif value.startswith('RECOVERED,'):
                self.report.setdefault('fault_recoveries',[]).append(value.split(',',1)[1])
                self.stop_requested.clear()
                self.emit('notice', text='故障条件已消失 · 已恢复空闲，等待使能指令')
            elif value.startswith('FAULT_STATE,'):
                self.report['fault_state']=json.loads(value.split(',',1)[1])
                self.emit('fault_state', **self.report['fault_state'])
            elif value.startswith('SESSION_WAITING,'):
                self.emit('session',enabled=True,mode='H',reason='waiting' if value.endswith(',1') else 'enabled')
            elif value.startswith('SESSION_PHASE,'):
                self.emit('session', enabled=True, mode=value.split(',')[1], reason='phase')
            if not value.startswith('ROW,'):
                self.log.flush()
                if not value.startswith(('WIRING_GUARD,', 'STATUS,', 'IDENT,')):
                    self.emit('log', text='← ' + value)
        return value

    def query(self, timeout=3, stop_verification=False):
        self.send('?')
        identity_seen = False
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            value = self.line(stop_verification=True) if stop_verification else self.line()
            if value.startswith('IDENT,'):
                if value not in (IDENTITY, *LIVE_IDENTITIES) or (self.identity and self.identity != value):
                    raise ProtocolError('设备固件不匹配：' + value)
                self.identity = value
                self.report['identity'] = value
                identity_seen = True
            elif identity_seen and value=='ERROR,ENABLED_STATUS_USE_LIVE':
                self.send('S');self.send('?');identity_seen=False
            elif identity_seen and value.startswith('STATUS,'):
                state = json.loads(value.split(',', 1)[1])
                self.report['last_status'] = state
                self.report['stop_verified'] = stopped(state)
                self.emit('status', state=state)
                return state
        raise ProtocolError('固件身份 / 状态应答超时')

    def ack(self, command, expected):
        if self.stop_requested.is_set():
            raise Cancelled('用户停止，后续动作已取消')
        self.send(command)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            if self.stop_requested.is_set():
                raise Cancelled('用户停止，后续动作已取消')
            value = self.line()
            matches = expected(value) if callable(expected) else value == expected
            if matches:
                return value
            if value.startswith('ERROR,'):
                raise ProtocolError(value)
        raise ProtocolError('参数确认超时：' + command)

    def stage(self, command):
        if self.stop_requested.is_set():
            raise Cancelled('用户停止，后续动作已取消')
        number = len(self.report['stages']) + 1
        self.stage_number, self.live_last = number, 0
        self.last_live_at = 0
        record = dict(command=command, result=None, metadata={}, accepted=False, csv=None)
        self.report['stages'].append(record)
        self.report['stop_verified'] = False
        columns, rows = None, []
        begun = done = False
        self.active, self.stop_sent = True, False
        self.user_stop = False
        self.last_heartbeat = 0
        duration = int(command.split(',')[2]) / 1000
        continuous = command == 'V,1.6,0'
        phase_text = '持续起摆稳摆 · 点击停止结束' if continuous else STAGES[command[0]]
        self.emit('phase', text=phase_text, seconds=-1 if continuous else duration, command=command)
        end = float('inf') if continuous else time.monotonic() + duration + 35
        stream = writer = None
        try:
            self.send(command)
            while time.monotonic() < end:
                value = self.line()
                if continuous and (self.stop_sent or not self.active) and math.isinf(end):
                    end = time.monotonic() + 35
                if not value:
                    continue
                if value.startswith('BEGIN,'):
                    if begun or value.split(',')[1] != command[0]:
                        raise ProtocolError('BEGIN 阶段不匹配')
                    begun = True
                    self.last_live_at = time.monotonic()
                    self.emit('phase', text=phase_text, seconds=-1 if continuous else duration, command=command)
                elif value.startswith('RESULT,'):
                    record['result'] = json.loads(value.split(',', 1)[1])
                    self.active = False
                    self.last_live_at = 0
                    self.emit('phase', text='关桥后回传采样 · ' + STAGES[command[0]], seconds=0)
                elif value.startswith('COLUMNS,'):
                    if columns is not None:
                        raise ProtocolError('重复采样表头')
                    columns = value.split(',')[1:]
                    path = self.output / f'{number:03d}_{command[0]}.csv'
                    record['csv'] = path.name
                    stream = path.open('w', newline='', encoding='utf-8')
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                elif value.startswith('ROW,'):
                    raw = value.split(',')[1:]
                    if writer is None or len(raw) != len(columns) or len(rows) >= 30000:
                        raise ProtocolError('采样行数量 / 列数异常')
                    writer.writerow(raw)
                    rows.append(raw)
                elif value.startswith('DONE,'):
                    done = value == 'DONE,IDLE,MOTOR_ENABLES_LOW'
                    break
                elif value.startswith('ERROR,'):
                    if value in ('ERROR,ARM_NOT_AT_CENTER', 'ERROR,HOME_WAIT_DOWN'):
                        raise RecoverableStage(value)
                    raise ProtocolError(value)
                elif ',{' in value:
                    key, data = value.split(',', 1)
                    record['metadata'][key.lower()] = json.loads(data)
            if stream:
                stream.flush()
            if record['csv'] and rows:
                self.emit('capture', path=str(self.output / record['csv']), command=command,
                          origin=record['metadata'].get('arm_origin', {}))
            if not done or not begun or record['result'] is None:
                raise ProtocolError('阶段回传不完整 / 超时')
            state = self.query()
            if not stopped(state):
                raise ProtocolError('阶段结束后双桥关闭未核实')
            if self.stop_requested.is_set() and not (continuous and self.user_stop and record['result'].get('reason') == 'stopped'):
                raise Cancelled('已停止，后续动作已取消')
            if (record['result'].get('reason') in ('HOME_NOT_REACHED', 'HOME_SETTLE_REQUIRED', 'INITIAL_PENDULUM_MOVING', 'PASSIVE_SETTLE_REQUIRED')
                    and record['result'].get('fault') == 0 and state.get('fault') == 0):
                raise RecoverableStage(record['result']['reason'])
            if self.identity in LIVE_IDENTITIES and command[0] in ('V', 'H'):
                origin = record['metadata'].get('arm_origin', {})
                if origin.get('center_valid') != 1 or abs(origin.get('offset_rad', 1)) > .0001:
                    raise ProtocolError('起摆 / 回零未使用已登记的物理零点')
            try:
                record['checks'] = validate_stage(command, record['result'], record['metadata'], columns, rows)
            except ProtocolError as exc:
                if str(exc) in ('旋臂未静止，后续动作取消', '摆杆下垂零位未通过静止 / 范围检查', '物理零点回位未通过，禁止起摆', '下垂移位未静稳，等待恢复'):
                    raise RecoverableStage(str(exc)) from exc
                raise
            record['accepted'] = True
            self.emit('log', text='阶段通过：' + json.dumps(record['checks'], ensure_ascii=False))
            if 'down_count' in record['checks']:
                zero = record['checks']['down_count']
                def zero_ack(value):
                    if not value.startswith('PENDULUM_ZERO,'):
                        return False
                    try:
                        return abs(float(value.split(',')[1]) - zero) <= .0005
                    except ValueError:
                        return False
                self.ack(f'PZERO,{zero:.5f}', zero_ack)
        except Exception as exc:
            record['error'] = str(exc)
            raise
        finally:
            if stream:
                stream.close()
            self.save()

    def execute_session(self, action):
        name,goal=(action if isinstance(action,tuple) else (action,0.))
        if self.motion_active:
            self.emit('notice',text='轨迹运动与静稳确认中；可随时停止，完成后再移位或起摆')
            return
        if name == 'quick':
            if goal not in (-170,-90,-30,30,90,170):
                raise ProtocolError('柔和快移步长无效')
            if not self.enabled or self.report.get('last_status',{}).get('mode')!='H':
                self.emit('notice',text='柔和快移需要先使能并在下垂位置静稳')
                return
            try:
                self.ack(f'QSTEP,{int(goal)}','COMMAND_OK,QSTEP')
            except ProtocolError as exc:
                if str(exc) not in ('ERROR,STEP_NOT_SETTLED','ERROR,MOTION_IN_PROGRESS'):raise
                self.emit('notice',text='尚未静稳，保持当前状态；稳定后可再次请求柔和快移')
                self.report.setdefault('step_refusals',[]).append(str(exc));self.save()
                return
            self.motion_active=True
            self.emit('motion',active=True,completed=False,operation='step')
            self.report.setdefault('step_commands',[]).append(int(goal));self.save()
            return
        if name in ('drop_positive','drop_negative'):
            if not self.enabled or self.report.get('last_status',{}).get('mode')!='V':
                self.emit('notice',text='受控下落需要先在物理零点稳摆')
                return
            self.execute_position(0)
            self.wait_motion_condition(lambda:self.upright_ready() and self.budget['dose']<=.5,25,'等待下落额度与回零稳摆')
            try:
                self.ack('DROP,'+('1' if name=='drop_positive' else '-1'),'COMMAND_OK,DROP')
            except ProtocolError as exc:
                notices={'ERROR,DROP_NOT_SETTLED':'尚未稳摆，保持当前状态；稳定后可再次请求下落',
                         'ERROR,DROP_DOSE_WAIT':'电流剂量余量不足，保持稳摆；滚动额度释放后可再次请求下落',
                         'ERROR,DROP_REQUIRES_CENTER':'请先回物理零点并稳摆，再请求受控下落',
                         'ERROR,MOTION_IN_PROGRESS':'运动执行中，请等待静稳确认'}
                if str(exc) not in notices:raise
                self.emit('notice',text=notices[str(exc)])
                self.report.setdefault('drop_refusals',[]).append(str(exc));self.save()
                return
            self.motion_active=True
            self.emit('motion',active=True,completed=False)
            self.report.setdefault('drop_commands',[]).append(name);self.save()
            return
        if name=='move' and (not math.isfinite(goal) or abs(goal)>math.pi):
            raise ProtocolError('目标角度必须在 −180° 至 +180° 内')
        state=self.report.get('last_status',{}) if self.enabled else self.query()
        if name=='recover':
            if self.enabled:
                raise ProtocolError('请先切换空闲再恢复故障')
            self.ack('RECOVER','RECOVERY_REQUESTED')
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                state=self.query()
                if not state.get('fault'):
                    self.emit('notice',text='恢复完成 · 空闲，等待指令')
                    return
                until=time.monotonic()+.1
                while time.monotonic()<until:self.line()
            raise ProtocolError('故障条件仍存在，保持空闲；请查看故障原因与传感器状态')
        if state.get('fault') or not state.get('ready'):
            raise ProtocolError('故障尚未恢复；可查看实时数据或请求故障恢复')
        if state.get('center_valid')!=1:
            raise ProtocolError('请先登记固定物理零点')
        if not self.enabled:
            cfg=profile(verify_binary=True,identity=self.identity)
            self.report['profile']=cfg
            self.report['local_firmware_sha256']=cfg['firmware_sha256']
            snapshot=self.output/'sources';snapshot.mkdir(exist_ok=True)
            for source in [profile_path(self.identity),*Path(__file__).parent.glob('*.py')]:
                (snapshot/source.name).write_bytes(source.read_bytes())
            self.report['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
            if not all(state.get(k)==1 for k in ('mapped','aligned')):
                self.emit('phase',text='首次使能 · 自动初始化',seconds=0)
                for command,expected in settings():
                    self.ack(command,expected)
                recoveries=0;qualification_started=time.monotonic()
                for command in QUALIFY:
                    pending=command
                    while True:
                        try:
                            self.stage(pending);break
                        except RecoverableStage as exc:
                            state=self.query()
                            if not eligible(state) or self.stop_requested.is_set():raise
                            if recoveries>=2 or time.monotonic()-qualification_started>120:
                                raise ProtocolError('初始化恢复预算已用尽：'+str(exc)) from exc
                            recoveries+=1
                            self.report.setdefault('recoveries',[]).append(dict(command=pending,reason=str(exc),attempt=recoveries))
                            self.emit('notice',text='初始化条件可恢复 · 自动重新核验禁用采样')
                            pending='P,0,15000'
                state=self.query()
            self.ack('LIVE,50','LIVE_HZ,50')
            self.ack('HOLD,4','HOLD_PROFILE,4')
            self.ack('HPROBE,0,1','HOLD_PROBE,0.00000,1')
            # Half-second fresh sensor/current window, independent of arm rest.
            # It is continuously accumulated by firmware while idle.
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                if self.report.get('fault_state',{}).get('healthy'):
                    break
                self.query()
                until=time.monotonic()+.1
                while time.monotonic()<until:self.line()
            self.ack('ENABLE',lambda line:line.startswith('SESSION,') and bool(json.loads(line.split(',',1)[1])['enabled']))
            self.enabled=self.active=True;self.stop_sent=False;self.last_heartbeat=0
        if name in ('home','move'):return self.execute_position(math.degrees(goal) if name=='move' else 0)
        if name in ('full','swing','continuous'):
            self.execute_position(0)
            self.ack('SWING,'+('0' if name=='continuous' else '15000'),'COMMAND_OK,SWING')
            self.emit('session',enabled=True,mode='H',reason='prepare_swing')
        self.save()

    def execute_soft_reset(self):
        if self.identity not in (RESET_IDENTITY,SPEED_IDENTITY,SERVO_IDENTITY):
            raise ProtocolError('软复位需要 PC_LIVE v23 固件')
        expected_reset_identity=self.identity
        if self.stop_requested.is_set() or self.disconnect_requested.is_set():
            raise Cancelled('软复位已取消')
        self.emit('phase',text='软复位 · 正在关桥并保存现场',seconds=0)
        self.send('S')
        self.active = self.enabled = self.motion_active = False
        state = self.query(timeout=12, stop_verification=True)
        if not stopped(state):
            raise ProtocolError('双桥关闭未核实，未发送复位')
        record = dict(requested_at=datetime.now().astimezone().isoformat(),before=dict(state),
                      before_fault_state=dict(self.report.get('fault_state',{})),completed=False)
        self.report.setdefault('soft_resets',[]).append(record)
        self.save()
        if self.live_stream:self.live_stream.flush()
        self.log.flush()
        # One explicit request, never retry it after an ambiguous response.
        self.ack('RESET,SOFT','RESETTING,SOFT')
        record['acknowledged'] = True
        self.save()
        self.boot_epoch += 1
        self.identity = None  # Boot ROM may emit a partial non-ASCII startup line.
        self.live_last = self.last_live_at = 0
        self.live_clock_offset = None
        self.live_lag_s = 0.
        self.latest_sample = {}
        self.budget = None
        self.budget_at = 0
        self.expect_live = False
        self.report['fault_state'] = {}
        self.emit('phase',text='软复位 · 等待控制板启动',seconds=0)
        deadline = time.monotonic()+12
        selftest = False
        while time.monotonic()<deadline:
            if self.stop_requested.is_set() or self.disconnect_requested.is_set():
                raise Cancelled('复位已请求，后续连接核验已取消')
            value = self.line()
            if value=='SELFTEST,FAIL':raise ProtocolError('复位后设备自检失败')
            if value=='SELFTEST,PASS':selftest = True
            if value.startswith('IDENT,') and selftest:
                if value != expected_reset_identity:raise ProtocolError('复位后固件身份不匹配')
                break
        else:
            raise ProtocolError('复位启动应答超时；未重复发送复位')
        self.identity=expected_reset_identity
        after = self.query(timeout=5)
        record['after'] = dict(after)
        if not eligible(after) or any(after.get(k) for k in ('mapped','aligned','passive_ok')):
            raise ProtocolError('复位后空闲 / 初始化状态不符合要求')
        if any(after.get(k)!=state.get(k) for k in ('center_valid','center_raw')):
            raise ProtocolError('复位前后物理零点不一致')
        self.ack('LIVE,50','LIVE_HZ,50')
        self.expect_live = True
        record.update(completed=True,boot_epoch=self.boot_epoch)
        self.save()
        self.emit('session',enabled=False,mode='I',reason='soft_reset')
        self.emit('reset_complete')

    def execute(self, action):
        name=action[0] if isinstance(action,tuple) else action
        if name in ('servo_start','servo_target','servo_end'):
            return execute_servo(self,name,action[1] if isinstance(action,tuple) else 0)
        if name=='speed':return execute_speed(self,action[1])
        if name=='soft_reset':return self.execute_soft_reset()
        if self.identity in LIVE_IDENTITIES and name=='demo':return self.execute_demo()
        if self.identity in LIVE_IDENTITIES and name=='turn':return self.execute_turn(action[1])
        if self.identity in LIVE_IDENTITIES and name=='position':return self.execute_position(action[1])
        if self.identity in LIVE_IDENTITIES and name in ('enable','recover','home','move','quick','swing','continuous','full','drop_positive','drop_negative'):
            return self.execute_session(action)
        return self.execute_legacy(action)

    def execute_legacy(self, action):
        goal = 0.
        if isinstance(action, tuple):
            action, goal = action
            if action != 'move' or not math.isfinite(goal) or abs(goal)>math.pi:
                raise ProtocolError('下垂移位目标无效')
        state = self.query()
        if not eligible(state):
            raise ProtocolError('设备未就绪、存在故障或配置不匹配')
        if action != 'passive' and self.identity not in LIVE_IDENTITIES:
            raise ProtocolError('固定物理零点功能需要 PC_LIVE 固件')
        if action in ('full', 'home', 'swing', 'move', 'continuous') and state.get('center_valid') != 1:
            raise ProtocolError('请先登记固定物理零点')
        if action in ('home', 'swing', 'move') and not all(state.get(k) == 1 for k in ('mapped', 'aligned')):
            raise ProtocolError('请先完成自动校验')
        cfg = profile(verify_binary=True, identity=self.identity)
        self.report['profile'] = cfg
        self.report['local_firmware_sha256'] = cfg['firmware_sha256']
        # Persist the exact host sources and profile used by this session.
        cfg_path = profile_path(self.identity)
        sources = [cfg_path, *sorted(Path(__file__).parent.glob('*.py'))]
        snapshot = self.output / 'sources'
        snapshot.mkdir(exist_ok=True)
        for source in sources:
            target = snapshot / source.name
            target.write_bytes(source.read_bytes())
        self.report['source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in snapshot.iterdir()}
        if self.identity in LIVE_IDENTITIES:
            self.ack('LIVE,50', 'LIVE_HZ,50')
        if action in ('passive', 'register'):
            self.ack('EXPORT,1', 'EXPORT,1')
            commands = ['P,0,5000']
        elif action == 'full' or (action == 'continuous' and not all(state.get(k)==1 for k in ('mapped','aligned'))):
            for command, expected in settings():
                self.ack(command, expected)
            commands = QUALIFY + ['H,0,18000', 'P,0,5000', 'V,1.6,0' if action=='continuous' else 'V,1.6,15000']
        else:
            for command, expected in [('EXPORT,1', 'EXPORT,1'), ('HOLD,4', 'HOLD_PROFILE,4'),
                                      ('HPROBE,0,1', 'HOLD_PROBE,0.00000,1')]:
                self.ack(command, expected)
            commands = (['P,0,5000', f'H,{goal:.7f},18000', 'P,0,5000'] if action == 'move'
                        else ['P,0,15000', 'H,0,18000', 'P,0,5000'])
            if action in ('swing', 'continuous'):
                commands += ['V,1.6,0' if action=='continuous' else 'V,1.6,15000']
        recovery_count = 0
        recovery_started = None
        for i, command in enumerate(commands):
            self.emit('sequence', index=i + 1, total=len(commands))
            pending = [command]
            while pending:
                current = pending.pop(0)
                try:
                    self.stage(current)
                except RecoverableStage as exc:
                    state = self.query()
                    if not eligible(state) or self.stop_requested.is_set():
                        raise ProtocolError('恢复前状态不允许继续') from exc
                    now = time.monotonic()
                    recovery_started = recovery_started or now
                    if recovery_count >= 2 or now - recovery_started > 120:
                        raise ProtocolError('受控恢复预算已用尽：' + str(exc)) from exc
                    recovery_count += 1
                    event = dict(at=datetime.now().astimezone().isoformat(), command=current,
                                 reason=str(exc), attempt=recovery_count, stage=len(self.report['stages']))
                    self.report.setdefault('recoveries', []).append(event)
                    self.emit('log', text=f'受控恢复 {recovery_count}/2：{exc}；关桥静置后继续')
                    self.emit('phase', text='受控恢复 · 等待下垂稳定', seconds=0)
                    if current[0] in 'HV':
                        pending = ['P,0,15000', current if current[0]=='H' else 'H,0,18000', 'P,0,5000'] + ([current] if current[0]=='V' else []) + pending
                    else:
                        pending = ['P,0,15000'] + pending
                    self.save()
        if action == 'register':
            registered = []
            def center_ack(value):
                if not value.startswith('CENTER_SET,'):
                    return False
                try:
                    raw = int(value.split(',')[1])
                except ValueError:
                    return False
                if not 0 <= raw < 16384:
                    return False
                registered.append(raw)
                return True
            self.ack('CENTER,SET', center_ack)
            state = self.query()
            if state.get('center_valid') != 1 or state.get('center_raw') != registered[0]:
                raise ProtocolError('固定零点读回不一致')
            self.report.setdefault('center_registrations', []).append(dict(raw=registered[0],
                at=datetime.now().astimezone().isoformat()))
            self.emit('log', text=f'固定物理零点已保存到板端：{registered[0]} counts')
        self.emit('phase', text='流程完成 · 双桥关闭已核实', seconds=0)

    def run(self):
        identity_verified = False
        serial_opened = False
        try:
            self.output.mkdir(parents=True, exist_ok=False)
            self.log = (self.output / 'serial.log').open('w', encoding='utf-8', buffering=1)
            self.live_stream = (self.output / 'live.csv').open('w', newline='', encoding='utf-8')
            self.live_writer = csv.writer(self.live_stream)
            self.live_writer.writerow(['stage', 'host_monotonic_s', 'sequence', 't', 'mode',
                'theta_deg', 'alpha_deg', 'beta_deg', 'iq_A', 'iq_ref_A', 'speed_rad_s',
                'valid', 'en0', 'en1', 'fault', 'transport_lag_s', 'boot_epoch', 'speed_reference_rad_s'])
            self.save()
            self.emit('output', path=str(self.output))
            self.s = self.serial_factory()
            self.s.port, self.s.baudrate = self.port, 460800
            self.s.timeout, self.s.write_timeout = .02, .1
            self.s.dtr = self.s.rts = False
            self.s.open()
            serial_opened = True
            self.report['serial_opened'] = True
            self.query()
            identity_verified = True
            self.send('S')
            state = self.query(timeout=12)
            if not stopped(state):
                raise ProtocolError('连接后双桥关闭未核实')
            self.report['data_mode'] = 'LIVE_50HZ_AND_CAPTURE' if self.identity in LIVE_IDENTITIES else 'POST_STOP_CAPTURE'
            if self.identity in LIVE_IDENTITIES:
                self.ack('LIVE,50', 'LIVE_HZ,50')
            self.emit('connected', ready=eligible(state), identity=self.identity)
            next_query = time.monotonic() + 1
            while not self.disconnect_requested.is_set():
                self.service()
                if self.disconnect_requested.is_set():
                    break
                try:
                    # Serial.read already waits when there are no bytes. An
                    # additional wait per line is ~16 ms on coarse Windows
                    # timers and cannot reliably keep up with 60 lines/sec.
                    action = self.commands.get_nowait()
                except queue.Empty:
                    action = self.take_servo_target()
                if self.stop_requested.is_set():
                    # A queued RUN is invalidated by STOP, including during setup.
                    self.send('S')
                    self.query()
                    self.enabled=self.active=False
                    self.reset_servo()
                    self.emit('session',enabled=False,mode='I',reason='stopped')
                    self.stop_requested.clear()
                    while not self.commands.empty():
                        self.commands.get_nowait()
                    if action:
                        self.emit('busy', value=False)
                    action = None
                if action:
                    retarget=isinstance(action,tuple) and action[0]=='servo_target'
                    if not retarget:self.emit('busy', value=True)
                    try:
                        self.execute(action)
                    except Cancelled as exc:
                        self.send('S');self.stop_sent=True
                        self.query(timeout=12)
                        self.enabled=self.active=False
                        self.emit('session',enabled=False,mode='I',reason='stopped')
                        self.report['errors'].append(str(exc))
                        self.emit('phase', text=str(exc), seconds=0)
                    except ProtocolError as exc:
                        self.report.setdefault('command_errors', []).append(str(exc))
                        self.emit('notice', text='指令未完成：' + str(exc))
                        if not self.enabled:
                            self.send('S')
                            self.query()
                    except Exception:
                        raise
                    finally:
                        self.active = self.enabled
                    if not self.enabled:
                        self.send('S')
                        self.query()
                    self.stop_requested.clear()
                    if not retarget:self.emit('busy', value=False)
                    next_query = time.monotonic() + 1
                    self.save()
                elif not self.enabled and time.monotonic() >= next_query:
                    self.query()
                    next_query = time.monotonic() + 1
                else:
                    self.line()  # Consume idle telemetry continuously, not once per second.
        except Exception as exc:
            self.report['errors'].append(str(exc))
            self.emit('error', text=str(exc) if serial_opened else connection_error(self.port, exc))
        finally:
            self.active = False
            self.reset_servo()
            self.report['stop_verified'] = False
            if self.s is not None and self.s.is_open:
                if identity_verified:
                    try:
                        self.send('S')
                        # STOP cancels queued commands. Query only after it;
                        # do not burst LIVE,0 and ? into the single pending slot.
                        self.report['final_status'] = self.query(timeout=15, stop_verification=True)
                    except Exception as exc:
                        self.report['errors'].append('停止核实失败：' + str(exc))
                self.s.close()
            self.report['finished_at'] = datetime.now().astimezone().isoformat()
            try:
                if self.live_stream:
                    self.live_stream.close()
                if self.log:
                    self.log.close()
                    self.save()
            except Exception as exc:
                self.emit('error', text='保存记录失败：' + str(exc))
            self.emit('disconnected', verified=self.report['stop_verified'], opened=serial_opened)
