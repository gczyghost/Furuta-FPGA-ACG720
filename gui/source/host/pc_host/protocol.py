"""Protocol and acceptance rules for BALANCE_QUIET v3 and PC_LIVE v22.

Acceptance thresholds follow tools/run_quiet_session.py; the wire protocol has
post-stop captures plus PC_LIVE telemetry. No reset, fault-clear or arbitrary RUN.
"""
import csv
from bisect import bisect_left
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(sys._MEIPASS) / 'project' if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[2]
if not (ROOT / 'profiles').exists() and (ROOT / 'reference/esp32_v27/profiles').exists():
    ROOT = ROOT / 'reference/esp32_v27'
DEFAULT_OUTPUT = (Path(os.environ.get('LOCALAPPDATA', str(Path.home()))) / 'FurutaBLDC' / 'records'
                  if getattr(sys, 'frozen', False) else ROOT / 'data/raw/pc_host')
PROFILE_PATH = ROOT / 'profiles/me6025_quiet_balance.json'
IDENTITY = 'IDENT,ME6025_BALANCE_QUIET,3'
LIVE_IDENTITY = 'IDENT,ME6025_PC_LIVE,22'
RESET_IDENTITY = 'IDENT,ME6025_PC_LIVE,23'
SPEED_IDENTITY = 'IDENT,ME6025_PC_LIVE,24'
SERVO_IDENTITY = 'IDENT,ME6025_PC_LIVE,27'
LIVE_IDENTITIES = (LIVE_IDENTITY, RESET_IDENTITY, SPEED_IDENTITY, SERVO_IDENTITY)
FIRMWARE_SHA = '1aed5f21d578aecd28ef50bbe294c4658a33ba4b37723148f7e6b389b3ec2df2'
QUALIFY = ['P,0,5000', 'D,1,1200', 'A,1.15,15000', 'Q,0.04,2500', 'P,0,15000']
STAGES = {'I':'待机遥测', 'P': '禁用采样 / 静止检查', 'D': '电流映射', 'A': '电角度对齐',
          'Q': '小电流跟踪检查', 'H': '下垂抑摆移位', 'V': '零点起摆 → 回零稳摆'}
DEFAULT_REPLAY = ROOT / 'data/raw/20261001_011_quiet_balance/20261001_175557_damping_repeat_02_V.csv'
FIELDS = ['us', 'work_us', 'q', 'v', 'ia', 'ib', 'id', 'iq', 'iq_ref', 'ud',
          'uq', 'target', 'raw', 'oid', 'diag', 'as_age_us', 'oid_age_us',
          'adc_age_us', 'adc_span_us', 'alpha', 'pend_speed']


class ProtocolError(RuntimeError):
    pass


def profile_path(identity):
    if identity == SERVO_IDENTITY:
        return ROOT / 'profiles/me6025_pc_live_v27.json'
    if identity == SPEED_IDENTITY:
        return ROOT / 'profiles/me6025_pc_live_v24.json'
    if identity == RESET_IDENTITY:
        return ROOT / 'profiles/me6025_pc_live_v23.json'
    return ROOT / 'profiles/me6025_pc_live.json' if identity == LIVE_IDENTITY else PROFILE_PATH


def profile(verify_binary=False, identity=IDENTITY):
    live = identity in LIVE_IDENTITIES
    path = profile_path(identity)
    cfg = json.loads(path.read_text(encoding='utf-8'))
    expected = dict(identity=IDENTITY, firmware='me6025_quiet_balance_v3',
                    firmware_sha256=FIRMWARE_SHA, baud=460800,
                    current_config=[7, 5867, .0001, 0], settings=['ADC,1', 'RATE,500'],
                    qualify=QUALIFY, hold=['V,1.6,15000'], hold_profile=4)
    if live:
        expected.update(identity=identity, firmware='me6025_pc_live_v27' if identity==SERVO_IDENTITY else 'me6025_pc_live_v24' if identity==SPEED_IDENTITY else 'me6025_pc_live_v23' if identity==RESET_IDENTITY else 'me6025_pc_live',
                        firmware_sha256=cfg['firmware_sha256'])
    for key, value in expected.items():
        if cfg.get(key) != value:
            raise ProtocolError(f'已验配置不匹配：{key}')
    if verify_binary:
        binary = ROOT / 'firmware' / cfg['firmware'] / '.pio/build/me6025_pwm_24v/firmware.bin'
        if hashlib.sha256(binary.read_bytes()).hexdigest() != cfg['firmware_sha256']:
            raise ProtocolError('本地已验固件 SHA-256 不匹配')
    return cfg


def stopped(state):
    return state.get('mode') == 'I' and state.get('en0') == 0 and state.get('en1') == 0


def eligible(state):
    return (stopped(state) and state.get('fault') == 0 and state.get('ready') == 1
            and state.get('wiring') == '20260930' and state.get('transport') == 'as5048_pwm'
            and state.get('bus_v') == 24 and state.get('period_us') in (500, 1000))


def live_sample(line):
    parts = line.split(',')
    if len(parts) not in (10,11) or parts[0] != 'LIVE' or parts[3] not in STAGES:
        raise ProtocolError('LIVE 帧格式不匹配')
    seq, us, flags = int(parts[1]), int(parts[2]), int(parts[9])
    if seq < 1 or us < 0 or not 0 <= flags <= 15:
        raise ProtocolError('LIVE 帧序号 / 标志异常')
    row = sample(dict(us=us, q=parts[4], alpha=parts[5], iq=parts[6],
                      iq_ref=parts[7], v=parts[8]))
    if len(parts)==11:
        reference=float(parts[10])
        if not math.isfinite(reference) or abs(reference)>22:raise ProtocolError('速度参考无效')
        row['speed_ref']=reference
    row.update(sequence=seq, mode=parts[3], valid=bool(flags & 1),
               en0=bool(flags & 2), en1=bool(flags & 4), fault=bool(flags & 8))
    return row


def settings():
    return [('C,7,5867,0.0001,0', 'CURRENT_PI,7.0000,5867.0000,0.000100,0.0000'),
            ('EXPORT,1', 'EXPORT,1'), ('ADC,1', 'ADC_BATCH,1'),
            ('RATE,500', 'PROBE_PERIOD,500'), ('HOLD,4', 'HOLD_PROFILE,4'),
            ('HPROBE,0,1', 'HOLD_PROBE,0.00000,1')]


def sample(row):
    """Firmware alpha is down-zero; beta is the wrapped upright deviation."""
    q, alpha = float(row['q']), float(row['alpha'])
    values = dict(t=float(row['us']) / 1e6, theta=math.degrees(q),
                  alpha=math.degrees(alpha), beta=math.degrees(alpha % (2 * math.pi) - math.pi),
                  iq=float(row['iq']), iq_ref=float(row['iq_ref']),
                  speed=float(row['v']), valid=True)
    if not all(math.isfinite(v) for v in values.values()):
        raise ProtocolError('采样包含非有限数值')
    return values


DISPLAY_FIELDS = dict(t='time_s', theta='theta_deg', alpha='alpha_deg', beta='beta_deg',
                      iq='iq_A', iq_ref='iq_ref_A', speed='speed_rad_s')
BUDGET_FIELDS = dict(cap='budget_current_limit_A', dose='budget_i2t_used_A2s',
                     goal='budget_target_rad', available='budget_available_flags')


def display_budget(row):
    columns = [*BUDGET_FIELDS.values(), 'budget_age_s', 'budget_valid']
    if not any(row.get(key) not in (None, '') for key in columns):
        return {}
    if any(row.get(key) in (None, '') for key in columns):
        raise ProtocolError('额度字段不完整')
    budget = {key: float(row[column]) for key, column in BUDGET_FIELDS.items()}
    age = float(row['budget_age_s'])
    flag = row['budget_valid'].strip().lower()
    if (not all(math.isfinite(v) for v in [*budget.values(), age]) or age < 0
            or not 0 <= budget['cap'] <= 1.9 or not 0 <= budget['dose'] <= 1.1
            or budget['available'] not in range(8)
            or flag not in ('true', 'false', '1', '0')):
        raise ProtocolError('额度字段数值无效')
    budget['available'] = int(budget['available'])
    valid = flag in ('true', '1')
    if valid and age >= .4:
        raise ProtocolError('额度超过 0.4 秒却标记有效')
    return dict(budget=budget, budget_age_s=age, budget_valid=valid)


def display_sample(row):
    """Display exports already contain the displayed coordinates in degrees."""
    flag = (row.get('valid') or '').strip().lower()
    if flag not in ('true', 'false', '1', '0'):
        raise ProtocolError('valid 必须为 True / False 或 1 / 0')
    values = {key: float(row[column]) for key, column in DISPLAY_FIELDS.items()}
    valid = flag in ('true', '1')
    for key, value in values.items():
        if not math.isfinite(value):
            # Old raw captures export an unobserved pendulum as invalid NaN.
            if not valid and key in ('alpha', 'beta') and math.isnan(value):
                continue
            raise ProtocolError(f'{DISPLAY_FIELDS[key]} 包含非有限数值')
    values.update(valid=valid, capture_format='display')
    if row.get('source')=='SIMULATION':values['capture_source']='SIMULATION'
    if row.get('mode'):
        if row['mode'] not in STAGES:
            raise ProtocolError('记录的设备模式无效')
        values['mode'] = row['mode']
    if row.get('speed_reference_rad_s') not in (None,''):
        reference=float(row['speed_reference_rad_s'])
        if not math.isfinite(reference) or abs(reference)>22:raise ProtocolError('记录的速度参考无效')
        values['speed_ref']=reference
    values.update(display_budget(row))
    if row.get('commands'):
        markers = json.loads(row['commands'])
        if not isinstance(markers, list) or len(markers) > 100:
            raise ProtocolError('指令标记格式无效或超过单行 100 条')
        for marker in markers:
            if (not isinstance(marker, dict) or set(marker) != {'t', 'label', 'command'}
                    or not isinstance(marker['t'], (int, float)) or not math.isfinite(marker['t'])
                    or marker['t'] < 0 or any(not isinstance(marker[k], str) or len(marker[k]) > 128
                                            for k in ('label', 'command'))):
                raise ProtocolError('指令标记字段无效')
        if markers:
            values['commands'] = markers
    return values


def write_display(path, rows, source, markers=()):
    """Keep the original nine columns, plus optional replayable command markers."""
    rows = list(rows)
    times = [row['t'] for row in rows]
    annotations = [[] for _ in rows]
    for marker in markers:
        if rows and times[0] <= marker['t']:
            index = min(bisect_left(times, marker['t']), len(rows)-1)
            annotations[index].append(marker)
    with Path(path).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source', *DISPLAY_FIELDS.values(), 'valid', 'commands', 'mode',
                         *BUDGET_FIELDS.values(), 'budget_age_s', 'budget_valid', 'speed_reference_rad_s'])
        for row, events in zip(rows, annotations):
            budget = row.get('budget')
            writer.writerow([row.get('capture_source',source), *[row[k] for k in DISPLAY_FIELDS], row['valid'],
                             json.dumps(events, ensure_ascii=False, separators=(',', ':')) if events else '',
                             row.get('mode', ''), *[budget[k] if budget else '' for k in BUDGET_FIELDS],
                             row.get('budget_age_s', '') if budget else '',
                             row.get('budget_valid', False) if budget else '', row.get('speed_ref','')])


def read_capture(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        if set(FIELDS).issubset(fields):
            parse = sample
        elif (set(DISPLAY_FIELDS.values()) | {'valid'}).issubset(fields):
            parse = display_sample
        else:
            raise ProtocolError('需要 ME6025 原始采样 CSV 或上位机导出的显示数据 CSV（time_s/theta_deg/iq_A 等字段）')
        rows = []
        previous = -1
        for row in reader:
            if len(rows) >= 100000:
                raise ProtocolError('回放文件超过 100000 点')
            try:
                value = parse(row)
            except (ValueError, TypeError, KeyError, ProtocolError) as exc:
                raise ProtocolError(f'CSV 第 {reader.line_num} 行：{exc}') from exc
            if value['t'] <= previous:
                raise ProtocolError('采样时间必须严格递增')
            rows.append(value)
            previous = value['t']
        if not rows:
            raise ProtocolError('采样文件为空')
        return rows


def capture_origin(path):
    """Recover coordinate provenance for a CSV saved by this desktop host."""
    path = Path(path)
    session = path.parent / 'session.json'
    if not session.is_file():
        return {}
    report = json.loads(session.read_text(encoding='utf-8'))
    for stage in report.get('stages', []):
        if stage.get('csv') == path.name:
            meta = stage.get('metadata', {})
            return dict(meta.get('arm_origin', {}), pendulum_observed=stage['command'][0]=='V' or 'down_move' in meta)
    return {}


def validate_stage(command, result, metadata, columns, rows):
    """Reject partial/failed captures before allowing the next powered stage."""
    if columns != FIELDS or not rows:
        raise ProtocolError('采样列不匹配或采样为空')
    continuous = command == 'V,1.6,0'
    cm = metadata.get('continuous', {})
    if continuous and not (cm.get('enabled') == 1 and cm.get('capture_tail') is True and cm.get('recovery_window_s') == 15 and cm.get('dose_window_s') == 16):
        raise ProtocolError('持续模式元数据不匹配')
    if result.get('stage') != command[0] or result.get('reason') != ('stopped' if continuous else 'complete'):
        raise ProtocolError(f'阶段未正常完成：{result.get("reason", "missing result")}')
    if any(result.get(key, 1) != 0 for key in ('fault', 'en0', 'en1')):
        raise ProtocolError('阶段故障或关桥未核实')
    timing = metadata.get('realtime', {})
    miss = result.get('miss', -1)
    if not (miss == 0 or (0 < miss and (continuous or miss <= 12) and timing.get('recovered') == miss
            and timing.get('deadline_total_max_us', 100000) <= 700
            and timing.get('phase_grid_preserved') is True)):
        raise ProtocolError('控制周期超出已验恢复预算')
    arr = np.asarray(rows, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != len(FIELDS) or not np.all(np.isfinite(arr)):
        raise ProtocolError('无效采样数据')
    col = {k: arr[:, i] for i, k in enumerate(columns)}
    t = col['us'] / 1e6
    duration = cm.get('elapsed_us', 0) / 1e6 if continuous else int(command.split(',')[2]) / 1000
    # D captures every control tick, P at 100 Hz, Q at 500 Hz, V at 200 Hz.
    max_gap = {'P': .03, 'D': .005, 'A': .03, 'Q': .006, 'V': .015, 'H': .03}[command[0]]
    if ((t[0] > .002 and not continuous) or (continuous and abs(t[0]-max(0,t[-1]-15.015))>.015) or t[-1] < duration - .03 or t[-1] > duration + .01
            or np.any(np.diff(t) <= 0) or np.any(np.diff(t) > max_gap)):
        raise ProtocolError('采样时间覆盖不完整 / 不连续')
    checks = dict(samples=len(rows), duration_s=float(t[-1]), recovered=miss)
    if continuous:
        checks.update(continuous=True, normal_user_stop=True, capture_start_s=float(t[0]), balanced=duration>=6)
        if duration < 6:
            return checks  # A valid early STOP is not evidence of settled balance.
    if command[0] == 'H':
        h = t > t[-1] - .5
        move = metadata.get('down_move', {})
        requested = float(command.split(',')[1])
        target = move.get('target_rad', float('nan'))
        if not (math.isfinite(target) and abs(math.remainder(target-requested,2*math.pi)) < .0001
                and move.get('settled_for_s', 0) >= .5 and move.get('observer_hz') == 10):
            raise ProtocolError('下垂移位未静稳，等待恢复')
        error = float(np.rad2deg(np.max(abs(col['q'][h]-target))))
        speed = float(np.sqrt(np.mean(col['v'][h] ** 2)))
        if error > 1 or speed > .15 or np.max(abs(col['iq_ref'])) > 1.00001:
            raise ProtocolError('物理零点回位未通过，禁止起摆')
        alpha = float(np.rad2deg(np.max(abs(col['alpha'][h]))))
        pend_speed = float(np.sqrt(np.mean(col['pend_speed'][h]**2)))
        if error > .6 or alpha > .5 or pend_speed > .05:
            raise ProtocolError('下垂移位未静稳，等待恢复')
        checks.update(center_max_error_deg=error, final_speed_rms_rad_s=speed,
                      target_deg=math.degrees(requested), down_max_error_deg=alpha,
                      pend_speed_rms_rad_s=pend_speed, settled_at_s=move['settled_at_s'])
    if command[0] == 'P':
        h = t >= t[-1] - 1
        rms = float(np.sqrt(np.mean(col['v'][h] ** 2)))
        span = float(np.ptp(col['q'][h]))
        drift = float(np.polyfit(t[h], col['q'][h], 1)[0])
        checks.update(velocity_rms_rad_s=rms, position_span_rad=span, drift_rad_s=drift)
        if h.sum() < 80 or rms > .15 or span > .015 or abs(drift) > .03:
            raise ProtocolError('旋臂未静止，后续动作取消')
        if command == 'P,0,15000':
            zero = float(np.mean(col['oid'][h]))
            if (np.ptp(col['oid'][h]) > 2 or abs(np.polyfit(t[h], col['oid'][h], 1)[0]) > 1
                    or abs(zero - 3398.72) > 8):
                raise ProtocolError('摆杆下垂零位未通过静止 / 范围检查')
            checks['down_count'] = zero
    if command[0] == 'Q':
        flat = ((t >= .60) & (t < .65)) | ((t >= 1.10) & (t < 1.15))
        err = float(np.sqrt(np.mean((col['iq'][flat] - col['iq_ref'][flat]) ** 2)))
        d = float(np.sqrt(np.mean(col['id'][flat] ** 2)))
        if flat.sum() < 40 or err >= .012 or d >= .015:
            raise ProtocolError('小电流跟踪不合格，后续动作取消')
        checks.update(iq_rms_error_A=err, id_rms_A=d)
    if command[0] == 'V':
        h = t > max(3, t[-1] - 3)
        beta = col['alpha'] % (2 * np.pi) - np.pi
        brms = float(np.rad2deg(np.sqrt(np.mean(beta[h] ** 2))))
        bmax = float(np.rad2deg(np.max(abs(beta[h]))))
        arm = col['q'][h]
        arms = float(np.rad2deg(np.sqrt(np.mean(arm ** 2))))
        amax = float(np.rad2deg(np.max(abs(arm))))
        boost, hold = metadata.get('swing_boost', {}), metadata.get('balance_hold', {})
        end = boost.get('exit_s', -1)
        post = t > .5 + end + .002
        holding_cap = 1.0 if hold.get('holding_cap_A') == 1 else .64
        recovery = metadata.get('performance_hold')
        if recovery is not None:
            if (not isinstance(recovery, dict) or hold.get('profile') != 4
                    or recovery.get('hold_max_cap_A') != 1.6
                    or recovery.get('recovery_hz') != 30
                    or recovery.get('hold_speed_guard_rad_s') != 14):
                raise ProtocolError('抗扰电流许可元数据不匹配')
            holding_cap = 1.6
        if not (boost.get('active_end') == 0 and 0 <= end <= .801 and np.any(post)
                and np.max(abs(col['iq_ref'][post])) <= holding_cap + .00001):
            raise ProtocolError('起摆增强退出 / 稳摆电流上限检查失败')
        if not (brms < 3 and bmax < 6 and arms < 2 and amax < 3
                and hold.get('profile') == 4 and hold.get('blend') == 1
                and abs(hold.get('integral_A', 1)) <= .120001):
            raise ProtocolError('稳摆末段 / HOLD4 参数验收失败')
        checks.update(pendulum_rms_deg=brms, arm_rms_deg=arms, boost_exit_s=end,
                      hold_start_s=float(t[h][0]), hold_end_s=float(t[-1]))
    return checks
