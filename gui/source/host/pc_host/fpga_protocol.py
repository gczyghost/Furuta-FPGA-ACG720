"""FPGA R1 protocol v2. Pure codec, profiles and atomic snapshot assembly."""
import binascii
import json
import math
import struct
from dataclasses import dataclass, asdict
from pathlib import Path

REVISION = 2
SUPPORTED_REVISIONS = {2:17, 3:20, 4:30}


@dataclass(frozen=True)
class BoundCalibrationProfile:
    """A read-back profile scoped to an externally verified board/bitstream identity."""
    identity: dict
    words: tuple

    def __post_init__(self):
        identity=dict(self.identity);words=tuple(self.words)
        if set(identity)!={'board_id','uart_pnp_id','bitstream_sha256','protocol_revision'}:raise ValueError('配置缺少精确板卡、UART或位流身份')
        if any(not isinstance(identity[k],str) or not identity[k] for k in ('board_id','uart_pnp_id')):raise ValueError('设备身份无效')
        digest=identity['bitstream_sha256']
        if not isinstance(digest,str) or len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest):raise ValueError('位流SHA256无效')
        if type(identity['protocol_revision']) is not int or identity['protocol_revision'] not in SUPPORTED_REVISIONS:raise ValueError('配置协议版本不支持')
        if len(words)!=8 or any(type(x) is not int or not 0<=x<=0xffffffff for x in words):raise ValueError('配置需要8个原始参数字')
        Calibration.from_words(words)
        object.__setattr__(self,'identity',identity);object.__setattr__(self,'words',words)

    def matches(self,identity,revision):
        return self.identity==identity and self.identity['protocol_revision']==revision

    def save(self,path):
        Path(path).write_text(json.dumps(dict(schema='fpga-bound-calibration-1',identity=self.identity,words=self.words),ensure_ascii=False,indent=2),encoding='utf-8')

    @classmethod
    def load(cls,path):
        data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if set(data)!={'schema','identity','words'} or data['schema']!='fpga-bound-calibration-1':raise ValueError('不是已绑定FPGA配置')
        return cls(data['identity'],tuple(data['words']))
MAGIC = {1: 0x48424154, 3: 0x434C4541, 4: 0x53594E43, 0x11: 0xC011CA1B}
CHANNELS = ['status','q','alpha','w','v','ia','ib','iq','iq_ref','ud','uq','duty_uv','duty_w','arm_raw','pend_raw','adc_raw','stamp']
FAULTS = {0:'无故障',1:'采样超时',2:'旋臂编码器错误',3:'摆杆编码器错误',4:'ADC 错误',
          0x10:'控制重入',0x11:'编码器跳变',0x12:'电流保护',0x21:'运动范围 / 运行时限',
          0x22:'捕获丢失',0x23:'模式无效',0x24:'起摆跟踪',0x31:'心跳超时',0x32:'采样过期'}
FAULTS.update({0x40:'标定 ADC 饱和',0x41:'标定过流',0x42:'标定运动 / 编码器越界',
               0x43:'对齐端点未静止',0x44:'极对数 / 正反行程不一致',0x45:'电流通道映射失败',0x46:'标定中止'})
FAULTS[0x70]='AS SPI 隔离诊断（AS计数无效，仅观察摆杆/ADC）'
FAULTS[0x25]='坐标切换未确认或超出范围'

def encode(op, addr, seq, value):
    if not all(isinstance(x,int) and 0 <= x <= 255 for x in (op,addr,seq)):
        raise ValueError('帧头字段超出范围')
    payload = struct.pack('>BBBBI',0xA5,op,addr,seq,value & 0xFFFFFFFF)
    return payload + struct.pack('>H',binascii.crc_hqx(payload,0xFFFF))

def decode(packet):
    if len(packet)!=10 or packet[0]!=0xA5 or binascii.crc_hqx(packet[:8],0xFFFF)!=int.from_bytes(packet[8:],'big'):
        raise ValueError('帧长度 / 帧头 / CRC 错误')
    _,op,addr,seq,value=struct.unpack('>BBBBI',packet[:8])
    return dict(op=op,addr=addr,seq=seq,value=value)

class StreamDecoder:
    def __init__(self): self.buffer=bytearray(); self.errors=0
    def feed(self,data):
        self.buffer.extend(data); frames=[]
        while self.buffer:
            if self.buffer[0]!=0xA5:
                del self.buffer[0]; self.errors+=1; continue
            if len(self.buffer)<10: break
            try: frame=decode(self.buffer[:10])
            except ValueError:
                del self.buffer[0]; self.errors+=1; continue
            frames.append(frame); del self.buffer[:10]
        return frames

def signed(value): return value if value<0x80000000 else value-0x100000000

class SnapshotAssembler:
    def __init__(self,stopped_diagnostics=False):
        self.stopped_diagnostics=stopped_diagnostics
        self.diagnostic_gap=False;self.last_sample_host=None;self.expected_epoch=None
        self.words=[]; self.next_seq=None; self.started=0.; self.last_stamp=None; self.elapsed=0.; self.dropped=0
        self.revision=None; self.word_count=17
        self.reset_epoch=None;self.telemetry_segment=0;self.epoch_pending=False
        self.commit_boundary=None
        self.cold_zero_only=True;self.cold_zero_at=None
    def calibration_committed(self,now):
        # Only the session's matching, value-checked COMMIT ACK arms this.
        if self.revision==4 and self.reset_epoch is not None:
            self.commit_boundary=(self.reset_epoch,now+.25)
    def feed(self,frame,now):
        if frame['op']!=0x80: return None
        addr,seq,value=(frame[k] for k in ('addr','seq','value'))
        if addr==0:
            if self.words: self.dropped+=1
            self.words=[]; self.started=now
            revision=(value>>16)&255
            if value>>24!=0xF1 or revision not in SUPPORTED_REVISIONS:
                raise ValueError('设备身份或协议版本不兼容：需要 FPGA F1 / v2、v3或v4')
            if self.revision is not None and revision!=self.revision:
                raise ValueError('连接期间协议版本改变，请重连')
            self.revision=revision;self.word_count=SUPPORTED_REVISIONS[revision]
            self.next_seq=seq
        if addr!=len(self.words) or seq!=self.next_seq or now-self.started>.1:
            self.words=[]; self.next_seq=None; self.dropped+=1; return None
        self.words.append(value); self.next_seq=(seq+1)&255
        if len(self.words)!=self.word_count: return None
        words=self.words; self.words=[]; status=words[0]; stamp=words[16]
        if self.commit_boundary is not None:
            epoch,deadline=self.commit_boundary
            if now>deadline or self.revision!=4 or words[28]!=epoch:
                self.commit_boundary=None
            elif (not words[27]&16 and status&0x87==0x80
                  and all(words[i]==0 for i in range(1,30) if i not in (27,28))):
                # B14 clears its completed snapshot on COMMIT, not its clock.
                # Discard this one non-sample, retaining raw RX in the journal.
                # Keep the last real timestamp: the next fresh sample must still
                # pass ordinary forward-time, freshness and epoch validation.
                self.commit_boundary=None;self.telemetry_segment+=1;self.dropped+=1
                return None
        # A cold completed-snapshot bank contains no device time origin.
        # Only an uninterrupted initial stream of stopped zero diagnostics may
        # establish its first real origin; never re-arm after real telemetry.
        cold_zero=(self.stopped_diagnostics and self.revision==4 and status&0x87==0
                   and not words[27]&16 and all(words[i]==0 for i in range(1,30) if i not in (27,28)))
        cold_first=(self.cold_zero_only and self.cold_zero_at is not None
                    and 0<=now-self.cold_zero_at<=.25 and self.revision==4
                    and words[28]==self.reset_epoch and status&0x83==0x80
                    and words[27]&16 and words[13]&0x80000000
                    and all(((stamp-words[i])&0xffffffff)<50_000 for i in (25,26))
                    and words[11]==words[12]==0)
        if cold_zero and self.cold_zero_only:
            if self.cold_zero_at is not None and (not 0<=now-self.cold_zero_at<=.25 or words[28]!=self.reset_epoch):self.cold_zero_only=False
            self.cold_zero_at=now
        elif not cold_zero:self.cold_zero_only=False
        duplicate=False;recovering=False
        if cold_first:
            self.last_stamp=None;recovering=True;self.telemetry_segment+=1
        expected_reset=(self.revision==4 and self.expected_epoch is not None
                        and words[28]==self.expected_epoch and words[28]!=self.reset_epoch and not status&1)
        if expected_reset:
            if (self.stopped_diagnostics and status&0x87==0x80 and not words[27]&16
                and all(words[i]==0 for i in range(1,30) if i not in (27,28))):
                # RST1 clears B68's completed bank, not its free-running clock.
                # This empty bank has no time origin. Keep the explicit reset
                # token for a real frame; do not refresh status or liveness.
                self.dropped+=1
                return None
            # Only an explicitly pending RST1 may establish a new time origin.
            # The first frame is a new segment, never continuous motion data.
            self.last_stamp=None;self.expected_epoch=None;recovering=True
        if self.last_stamp is not None:
            delta=(stamp-self.last_stamp)&0xFFFFFFFF
            if delta==0:
                if self.stopped_diagnostics and self.revision==4 and not words[27]&16 and words[28] in (self.reset_epoch,self.expected_epoch):
                    duplicate=True
                else:self.dropped+=1;return None
            if delta>100_000_000:
                if self.stopped_diagnostics and self.diagnostic_gap and self.revision==4 and not status&1 and words[28]==self.reset_epoch and delta<0x80000000:
                    # A stopped diagnostic outage has no usable time history.
                    # Start a new display segment; this first frame cannot
                    # authorize motion. A following advancing frame must pass
                    # the ordinary timestamp and freshness checks.
                    recovering=True;self.telemetry_segment+=1
                    self.elapsed+=max(0,now-(self.last_sample_host or now))
                else:raise ValueError('设备时间倒退、复位或遥测长时间中断，请重连')
            else:self.elapsed+=delta/50_000_000
        self.last_stamp=stamp
        if not duplicate:self.last_sample_host=now
        self.diagnostic_gap=duplicate or (recovering and self.revision==4 and not words[27]&16)
        row=dict(t=self.elapsed,stamp=stamp,run=bool(status&1),fault=bool(status&2),ready=bool(status&4),
                 hard_disabled=bool(status&8),mode=(status>>4)&7,calibrated=bool(status&128),reason=(status>>8)&255,
                 arm_raw=words[13]&16383,pend_raw=words[14],ia_raw=words[15]>>16,ib_raw=words[15]&65535,
                 sensor_valid=bool(words[13]&0x80000000),sensor_error=(words[13]>>16)&255,valid=bool(words[13]&0x80000000))
        row.update({CHANNELS[i]:signed(words[i])/65536 for i in range(1,11)})
        row.update(duty_u=(words[11]>>16)/65536,duty_v=(words[11]&65535)/65536,duty_w=words[12]/65536)
        # Keep legacy q in the local control frame; never silently change command semantics.
        row['protocol_revision']=self.revision
        if self.revision in (3,4):
            turns=signed(words[17]);goal=signed(words[18]);motion=words[19]
            row.update(frame_turns=turns,q_continuous=(signed(words[1])+turns*411775)/65536,
                       motion_goal=goal/65536,motion_goal_continuous=(goal+turns*411775)/65536,
                       motion_status=motion,motion_kind=motion&3,motion_active=bool(motion&8),
                       motion_finished=bool(motion&16),motion_arrived=bool(motion&32),motion_end=bool(motion&64),
                       home_settled=bool(motion&256),home_active=bool(motion&512),keep_session=bool(motion&1024))
        if self.revision==4:
            epoch=words[28]
            forward=self.reset_epoch is not None and 0<(epoch-self.reset_epoch)&0xffffffff<0x80000000
            if forward:self.telemetry_segment+=1;self.epoch_pending=True
            if self.reset_epoch is None or forward:self.reset_epoch=epoch
            row['telemetry_segment']=self.telemetry_segment
            flags=words[27]
            arm_age=((stamp-words[25])&0xffffffff)/50_000_000
            control_age=((stamp-words[26])&0xffffffff)/50_000_000
            fresh=bool(flags&16) and arm_age<.001 and control_age<.001
            if epoch==self.reset_epoch and fresh:self.epoch_pending=False
            row['telemetry_current']=epoch==self.reset_epoch and not self.epoch_pending
            row.update(current_cap=words[20]/65536,dose=words[21]/65536,
                       reference_w=signed(words[22])/65536,reference_q=signed(words[23])/65536,
                       reference_q_continuous=(signed(words[23])+signed(words[17])*411775)/65536,
                       sensor_velocity=signed(words[24])/65536,arm_sample_stamp=words[25],
                       completed_control_sample_stamp=words[26],eligibility_flags=flags,reset_epoch=words[28],
                       encoder_difference_velocity=signed(words[29])/65536,
                       arm_sample_age_s=arm_age,control_sample_age_s=control_age,control_sample_fresh=fresh,
                       reference_valid=fresh and bool(flags&1),sensor_velocity_valid=fresh and bool(flags&2),
                       current_cap_valid=fresh and bool(flags&4),dose_valid=fresh and bool(flags&8),
                       budget_valid=fresh and flags&12==12,servo_eligible=bool(flags&256),
                       speed_eligible=bool(flags&512),reset_eligible=bool(flags&1024),
                       finite_speed_active=bool(words[19]&128),servo_settling=bool(words[19]&2048))
        if duplicate:
            row['diagnostic_only']=True;row['telemetry_current']=False
        if recovering:row['telemetry_current']=False
        return row

@dataclass
class Calibration:
    offset_a: float = 2048.
    offset_b: float = 2048.
    gain_a: float = 106/65536
    gain_b: float = 106/65536
    arm_zero: int = 8192
    pend_down: int = 2048
    electrical_zero: int = 32768
    poles: int = 11
    reverse: bool = False

    def words(self):
        values=[self.offset_a,self.offset_b,self.gain_a,self.gain_b]
        if not all(math.isfinite(x) for x in values): raise ValueError('标定参数必须为有限数')
        words=[round(x*65536) for x in values]+[self.arm_zero,self.pend_down,self.electrical_zero,self.poles|(256 if self.reverse else 0)]
        if not all(512*65536<=x<=3584*65536 for x in words[:2]): raise ValueError('ADC 零偏范围为 512..3584 count')
        if not all(32<=abs(x)<=256 for x in words[2:4]): raise ValueError('电流增益绝对值范围为 32..256 / 65536 A/count')
        if not all(isinstance(x,int) and not isinstance(x,bool) for x in (self.arm_zero,self.pend_down,self.electrical_zero,self.poles)):
            raise ValueError('编码器零点和极对数必须为整数')
        if not (0<=self.arm_zero<16384 and 0<=self.pend_down<4096 and 0<=self.electrical_zero<65536 and 1<=self.poles<=32):
            raise ValueError('零点或极对数超出范围')
        if not isinstance(self.reverse,bool): raise ValueError('方向必须为布尔值')
        return [x&0xFFFFFFFF for x in words]

    @classmethod
    def from_words(cls,words):
        if len(words)!=8: raise ValueError('需要 8 个标定寄存器')
        c=cls(*(signed(x)/65536 for x in words[:4]),*words[4:7],words[7]&63,bool(words[7]&256)); c.words(); return c

    def save(self,path):
        self.words()
        Path(path).write_text(json.dumps({'schema':'furuta-fpga-r1-calibration-v2','units':'count, A/count, unsigned electrical turns/65536',
                                         'parameters':asdict(self),'hardware_verified':False},ensure_ascii=False,indent=2),encoding='utf-8')
    @classmethod
    def load(cls,path):
        obj=json.loads(Path(path).read_text(encoding='utf-8'))
        if obj.get('schema')!='furuta-fpga-r1-calibration-v2': raise ValueError('不是 FPGA R1 标定文件，不能导入 ESP32 标定')
        c=cls(**obj['parameters']); c.words(); return c

def estimate_offsets(rows):
    if len(rows)<100: raise ValueError('需至少 100 个连续静止快照（约 2 秒）')
    if any(r['run'] or r['fault'] or not r['sensor_valid'] or abs(r['w'])>.15 or abs(r['v'])>.3 for r in rows): raise ValueError('采集期间必须停止且静止、无故障、采样有效')
    if any(b['t']-a['t']<=0 or b['t']-a['t']>.1 for a,b in zip(rows,rows[1:])): raise ValueError('采样不连续')
    for key,modulus in (('arm_raw',16384),('pend_raw',4096)):
        origin=rows[0][key]
        if any(abs((r[key]-origin+modulus//2)%modulus-modulus//2)>2 for r in rows):
            raise ValueError('原始编码器发生运动，不能采集静止零偏')
    result=[]
    for key in ('ia_raw','ib_raw'):
        values=[r[key] for r in rows]
        if min(values)<512 or max(values)>3584 or max(values)-min(values)>20: raise ValueError('ADC 零偏不稳定或越界')
        result.append(sum(values)/len(values))
    return result

def fit_gain(raw_a,current_a,raw_b,current_b):
    if not all(math.isfinite(x) for x in (raw_a,current_a,raw_b,current_b)) or abs(raw_b-raw_a)<100:
        raise ValueError('两点原始读数至少相差 100 count')
    gain=(current_b-current_a)/(raw_b-raw_a); offset=raw_a-current_a/gain if gain else math.inf
    Calibration(offset_a=offset,gain_a=gain).words()
    return offset,gain
def hold_phase(angle):
    """Normalize measured Q16 position using the board's integer period."""
    return ((round(angle*65536)+205887)%411775-205887)/65536

