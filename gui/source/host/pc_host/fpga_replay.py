"""Read recordings without inventing device state for legacy display rows."""
import csv
import json
import math
from pathlib import Path

REQUIRED={'t','q','alpha','w','v','ia','ib','iq','iq_ref','ud','uq','mode','reason','run','ready',
          'calibrated','fault','hard_disabled','arm_raw','pend_raw','ia_raw','ib_raw',
          'duty_u','duty_v','duty_w','valid','sensor_valid','sensor_error'}


def read_recording(path):
    with Path(path).open(encoding='utf-8-sig',newline='') as stream:
        reader=csv.DictReader(stream)
        if not REQUIRED.issubset(reader.fieldnames or []):
            from .protocol import read_capture
            return 'esp32',read_capture(path)
        rows=[]
        for record in reader:
            if len(rows)>=100000:raise ValueError('回放文件超过100000点')
            if any(record.get(key) in ('',None) for key in REQUIRED):
                raise ValueError('FPGA记录缺少必需采样值')
            # Empty optional cells denote absent measurements/state, never zero.
            row={key:(value=='True' if value in ('True','False') else float(value)) for key,value in record.items() if key!='commands' and value not in ('',None)}
            if not all(math.isfinite(float(value)) for value in row.values()):raise ValueError('FPGA记录包含非有限值')
            for key in ('mode','reason','arm_raw','pend_raw','ia_raw','ib_raw','sensor_error'):
                if row[key]!=int(row[key]):raise ValueError('FPGA整数遥测字段无效')
                row[key]=int(row[key])
            for key in ('run','ready','calibrated','fault','hard_disabled','valid','sensor_valid'):
                if row[key] not in (0,1):raise ValueError('FPGA状态位无效')
                row[key]=bool(row[key])
            repeated_diagnostic=bool(rows and row.get('diagnostic_only') is True
                and row.get('telemetry_current') is False and row.get('control_sample_fresh') is False
                and not row['run'] and not rows[-1]['run'] and row.get('protocol_revision')==4
                and 'reset_epoch' in row and row['reset_epoch']==rows[-1].get('reset_epoch')
                and 'stamp' in row and row['stamp']==rows[-1].get('stamp'))
            if row['t']<0 or rows and (row['t']<rows[-1]['t'] or row['t']==rows[-1]['t'] and not repeated_diagnostic):
                raise ValueError('采样时间必须递增；仅允许明确标记的停止态陈旧诊断重复')
            if record.get('commands'):
                markers=json.loads(record['commands'])
                if not isinstance(markers,list) or len(markers)>100:raise ValueError('指令标记格式无效')
                for marker in markers:
                    if not isinstance(marker,dict) or set(marker)!={'t','label','command'} or not isinstance(marker['t'],(int,float)) or not math.isfinite(marker['t']) or marker['t']<0 or any(not isinstance(marker[k],str) or len(marker[k])>128 for k in ('label','command')):raise ValueError('指令标记字段无效')
                if markers:row['commands']=markers
            rows.append(row)
        if not rows:raise ValueError('回放文件为空')
        return 'fpga',rows


def legacy_plot(row):
    # NaNs are presentation gaps only, never synthesized into exported records.
    plot={key:math.nan for key in ('ia','ib','ud','uq','v','arm_raw','pend_raw','ia_raw','ib_raw')}
    plot.update(t=row['t'],q=row['theta'],alpha=row['alpha'],iq=row['iq'],
                iq_ref=row['iq_ref'],w=row['speed'],valid=row['valid'],beta=row['beta'],
                speed_rpm=row['speed']*30/math.pi,speed_ref_rpm=row.get('speed_ref',math.nan)*30/math.pi)
    return plot
