"""Offline protocol peer, never opens a port. Plant replay is labelled simulation."""
import csv
from pathlib import Path
import time
import math
from .fpga_protocol import encode, StreamDecoder, MAGIC, Calibration, signed

class SimSerial:
    def __init__(self,port=None,baudrate=None,timeout=.01,write_timeout=None,clock=time.monotonic,multiturn=False,live_servo=False,cold_snapshot=False,bootstrap_v4=False,postcommit_health='valid',postcommit_delay=0.):
        if bootstrap_v4 and not multiturn:raise ValueError('Bootstrap v4 requires multiturn')
        if postcommit_health not in ('valid','missing','stale','off_not_ready'):raise ValueError('Unknown health fixture')
        self.bootstrap_v4=bootstrap_v4;self.postcommit_health=postcommit_health
        if not 0<=postcommit_delay<=10:raise ValueError('Invalid postcommit sample delay')
        self.postcommit_delay=postcommit_delay;self.committed_at=None
        # Fault-injection fixture for B01's frozen v4 cold telemetry, not a model
        # of how CAL or power should recover the hardware.
        if cold_snapshot and not multiturn:raise ValueError('Cold v4 fixture requires multiturn')
        self.cold_snapshot=cold_snapshot
        if live_servo and not multiturn:raise ValueError('SV27 simulation requires MOT2')
        self.live_servo=live_servo
        self.multiturn=multiturn;self.frame_turns=0;self.keep_session=False;self.session_started=None
        self.clock=clock;self.timeout=timeout;self.started=clock();self.last_snapshot=0.;self.last_hb=0.
        self.buffer=bytearray();self.decoder=StreamDecoder();self.seq=None;self.tx_seq=0
        self.words=[0]*8;self.mask=0;self.calibrated=False;self.run=False;self.fault=False;self.reason=0
        self.mode=0;self.current=0.;self.qualified_at=clock();self.closed=False;self.bad_crc=False;self.silent=False
        self.plant=[];self.run_at=clock();self.alpha=0.;self.q=0.
        self.cal_status=0;self.cal_results=[106,106,11473,0];self.cal_failure=0
        self.long_run=False;self.drop_flags=3;self.dose=0.;self.home_capable=True;self.home_never_settles=False
        self.motion_kind=0;self.motion_arrived=False;self.motion_ending=None;self.motion_goal=0.;self.motion_rpm=0
        source=Path(__file__).resolve().parents[2]/'build/reports/closed_loop.csv'
        if source.exists():
            with source.open() as f:self.plant=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(f) if row['case']=='2']
    @property
    def in_waiting(self):return len(self.buffer)
    @property
    def ready(self):return not self.cold_snapshot and self.postcommit_health!='off_not_ready' and self.calibrated and not self.fault and self.clock()-self.last_hb<.8 and self.clock()-self.qualified_at>=.05
    def _frame(self,op,addr,seq,value):
        p=encode(op,addr,seq,value)
        if self.bad_crc:p=p[:-1]+bytes([p[-1]^1]);self.bad_crc=False
        if not self.silent:self.buffer.extend(p)
    def inject_fault(self):self.fault=True;self.run=False;self.reason=2
    def write(self,data):
        for f in self.decoder.feed(data):
            op,addr,seq,value=(f[k] for k in ('op','addr','seq','value'))
            seq_ok=self.seq is None or seq==(self.seq+1)&255
            sync=op==4 and not self.run and value==MAGIC[4]
            result=0x82;reply=2
            if not(seq_ok or op==2 or sync):reply=1
            else:
                if seq_ok or sync:self.seq=seq
                if op==2:
                    self.keep_session=False
                    self.motion_kind=0;self.motion_arrived=False;self.motion_ending=None
                    if self.run and self.mode>=5:self.cal_status=0x46000000|(self.mode<<2)
                    self.run=False;self.long_run=False;self.qualified_at=self.clock();result=0x81;reply=0
                elif op==4 and sync:result=0x81;reply=value
                elif op==1 and value==MAGIC[1]:self.last_hb=self.clock();result=0x81;reply=value
                elif op==3 and not self.run and value==MAGIC[3]:
                    self.fault=False;self.reason=0;self.cal_status=0;self.qualified_at=self.clock();result=0x81;reply=value
                elif op==0x12 and addr<8:result=0x81;reply=self.words[addr]
                elif op==0x12 and 8<=addr<14:
                    result=0x81;reply=0x43414C31 if addr==8 else self.cal_status if addr==9 else self.cal_results[addr-10]
                elif op==0x12 and addr==32:
                    result=0x81;reply=0x53563237 if self.live_servo or self.cold_snapshot or self.bootstrap_v4 else 0
                elif (self.cold_snapshot or self.bootstrap_v4) and op==0x12 and addr in (33,34,35):
                    result=0x81;reply={33:0x52535431,34:0,35:0x53504431}[addr]
                elif op==0x12 and 17<=addr<=31:
                    result=0x81
                    reply={17:0x44525031,18:self.drop_flags,19:0x4C4F4E31,20:round(self.dose*65536),21:int(self.long_run),22:0x484F4D31 if self.home_capable else 0,
                           23:(3 if not self.home_never_settles and self.clock()-self.run_at>1.5 else 1) if self.run and self.mode==7 else 0,
                           24:0x484F4D32,25:(3 if self.clock()-self.run_at>self.move_duration+1 else 1) if self.run and self.mode==0 else 0,
                           26:0x4D4F5432 if self.multiturn else 0x4D4F5431,27:(self.motion_kind|int(self.mode==0 and self.motion_kind!=0)*8|int(self.motion_arrived)*32|int(self.motion_ending is not None)*64) if self.run else 0,
                           28:round(self.motion_goal*65536),29:0x454e4131 if self.multiturn else 0,30:int(self.keep_session),31:0x44543230}[addr]
                elif op in (0x25,0x27) and addr<9 and not self.fault and (self.ready and not self.run or self.run and (self.mode in (3,4) or self.keep_session and self.mode==0 and self.motion_kind==0)):
                    target=signed(value)/65536 if op==0x27 else self.q+signed(value)/65536
                    if self.multiturn and op==0x27:
                        delta=round((target-self.q)*65536)
                        if delta>205888:delta-=411775
                        elif delta< -205888:delta+=411775
                        target=self.q+delta/65536
                    if abs(target)<=(64 if self.multiturn else 2.78) and abs(self.q)<=(411775/65536 if self.multiturn else 2.78) and (addr<5)==(not(self.run and self.mode in (3,4))):
                        self.motion_kind=1 if addr<5 else 2;self.motion_arrived=False;self.motion_ending=None
                        self.move_origin=self.q;self.move_target=target;self.motion_goal=target;self.move_duration=1.
                        self.mode=0;self.run=True;self.run_at=self.clock();result=0x81;reply=value
                elif op==0x26 and self.run and not self.fault:
                    if addr==1 and value==0 and self.mode==0 and self.motion_kind==3:
                        self.motion_ending=self.clock();result=0x81;reply=value
                    elif addr==0 and abs(signed(value))<=200 and (self.mode in (3,4) or self.mode==0 and self.motion_kind==3 and self.motion_ending is None):
                        # Preserve position up to the command timestamp before changing velocity.
                        if self.mode==0 and self.motion_kind==3:
                            self.q=self.move_origin+self.motion_rpm*3.141592653589793/30*(self.clock()-self.run_at)
                        self.motion_kind=3;self.motion_arrived=False;self.motion_ending=None;self.motion_rpm=signed(value)
                        self.move_origin=self.q;self.move_target=self.q;self.move_duration=1.;self.motion_goal=self.q
                        self.mode=0;self.run_at=self.clock();result=0x81;reply=value
                elif op==0x24 and (addr==0 or self.multiturn and addr==1) and not self.run and self.ready:
                    target=signed(value)/65536
                    shortest=self.q+(target-self.q+3.14159265)%6.2831853-3.14159265
                    if abs(target)<=(3.141592654 if self.multiturn else 2.8) and abs(shortest)<=(64 if self.multiturn else 2.8) and abs(self.alpha)<=.1:
                        self.motion_kind=0
                        self.move_origin=self.q;self.move_target=shortest;self.motion_goal=shortest;self.move_duration=3+3*abs(shortest-self.q)
                        self.run=True;self.mode=0;self.run_at=self.clock();self.keep_session=addr==1;self.long_run=addr==1
                        if self.keep_session:self.session_started=self.clock()
                        result=0x81;reply=value
                elif op==0x29 and addr==3 and value==0x53574731 and self.keep_session and self.run and self.mode==0 and abs(self.q)<.03 and abs(self.alpha)<.012 and self.dose<.65:
                    self.mode=3;self.run_at=self.clock();result=0x81;reply=value
                elif op==0x22 and addr==7 and value in (0x44525031,0x4452504E) and self.run and self.mode in (3,4) and self.drop_flags&7==3:
                    self.drop_origin=(self.q,self.alpha);self.mode=7;self.run_at=self.clock();result=0x81;reply=value
                elif op==0x23 and addr==3 and value==0x4C4F4E31 and not self.run and self.ready and abs(self.alpha)<.04 and abs(self.q)<.05:
                    self.run=True;self.mode=3;self.run_at=self.clock();self.long_run=True;result=0x81;reply=value
                elif op==0x10 and not self.run and addr<8:
                    valid=(512*65536<=value<=3584*65536 if addr<2 else 32<=abs(signed(value))<=256 if addr<4
                           else value<(16384 if addr==4 else 4096 if addr==5 else 65536) if addr<7
                           else value&0xFFFFFEC0==0 and 1<=value&63<=32)
                    if valid:self.words[addr]=value;self.mask|=1<<addr;self.calibrated=False;self.cal_status=0;result=0x81;reply=value
                elif op==0x11 and not self.run and value==MAGIC[0x11] and self.mask==255:
                    Calibration.from_words(self.words);self.calibrated=True;self.mask=0;self.qualified_at=self.clock();self.committed_at=self.clock();result=0x81;reply=value
                elif op==0x21 and not self.run and self.ready and addr in (5,6) and value==0x43414C31 and abs(self.alpha)<.15 and (addr==5 or self.words[7]&63==11):
                    self.run=True;self.mode=addr;self.run_at=self.clock();self.cal_status=(addr<<2)|1
                    self.cal_results[:2]=[abs(signed(x)) for x in self.words[2:4]];result=0x81;reply=value
                elif op==0x20 and not self.run and self.ready:
                    good=(addr==1 and abs(signed(value))<=5243 or addr==2 and abs(self.alpha)<.15 or
                          addr==3 and abs(self.alpha)<.04 and abs(self.q)<.05 or addr==4 and abs(abs(self.alpha)-3.14159265)<.15)
                    if good:self.run=True;self.mode=addr;self.current=signed(value)/65536;self.run_at=self.clock();result=0x81;reply=value
            self._frame(result,addr,seq,reply)
        return len(data)
    def read(self,size=1):
        now=self.clock()
        if self.run and now-self.last_hb>=.8:self.run=False;self.fault=True;self.reason=0x31
        if self.run and self.mode in (5,6):
            duration=1.2 if self.mode==5 else 15.
            elapsed=min(now-self.run_at,duration);self.cal_status=(self.mode<<2)|1|(int(elapsed*2000)<<8)
            if elapsed>=duration:
                self.run=False;self.qualified_at=now;self.cal_status=(self.mode<<2)|2|(int(duration*2000)<<8)
                if self.cal_failure:self.cal_status=(self.cal_failure<<24)|(self.mode<<2);self.fault=True;self.reason=self.cal_failure
        if now-self.last_snapshot>=.02:
            self.last_snapshot=now; iq=self.current if self.run and self.mode==1 else 0.;w=v=ud=uq=0.
            if self.run and self.mode==3 and self.plant:
                row=self.plant[min(int((now-self.run_at)/.01),len(self.plant)-1)]
                self.q=row['q'];self.alpha=row['alpha'];w=row['w'];v=row['v'];iq=row['iq'];ud=row['ud'];uq=row['uq']
            if self.run and self.mode==7:
                # Protocol-only presentation, not a mechanical simulation.
                fraction=max(0.,1.-(now-self.run_at))
                self.q,self.alpha=(value*fraction for value in self.drop_origin)
                w,v=(-value if fraction else 0. for value in self.drop_origin)
                if self.keep_session and now-self.run_at>1.5:
                    self.mode=0;self.motion_kind=0;self.motion_goal=0.;self.move_origin=0.;self.move_target=0.;self.move_duration=.1;self.run_at=now-1.2
            if self.run and self.mode==0:
                s=min(1.,max(0.,(now-self.run_at-.5)/self.move_duration));delta=self.move_target-self.move_origin
                self.q=self.move_origin+delta*(10*s**3-15*s**4+6*s**5)
                w=delta/self.move_duration*(30*s*s-60*s**3+30*s**4);self.alpha=3.141592653589793 if self.motion_kind>=2 else 0.
                if self.motion_kind==3:
                    elapsed=(self.motion_ending or now)-self.run_at
                    self.q=self.move_origin+self.motion_rpm*3.141592653589793/30*elapsed
                    self.motion_goal=self.q;w=0 if self.motion_ending is not None else self.motion_rpm*3.141592653589793/30
                    if self.motion_ending is not None and now-self.motion_ending>1:self.mode=4;self.motion_arrived=True
                elif self.motion_kind and s==1:
                    if self.motion_kind==1:self.motion_kind=0
                    else:self.mode=4;self.motion_arrived=True
            deadline=20.903 if self.mode==7 else (610 if self.long_run else 130) if self.mode in (3,4) else 20
            expired=(now-self.session_started>=610) if self.keep_session else (now-self.run_at>=deadline)
            if self.run and expired:self.run=False;self.fault=True;self.reason=0x21;self.keep_session=False
            home_active=self.run and self.mode==0 and self.motion_kind==0
            home_settled=home_active and now-self.run_at>self.move_duration+1
            if self.multiturn and self.run and (home_settled or self.mode==4 and self.motion_arrived) and abs(self.motion_goal)>205952/65536:
                turns=round(self.motion_goal/(411775/65536));shift=turns*411775/65536
                self.frame_turns+=turns;self.q-=shift;self.motion_goal-=shift
                self.move_origin-=shift;self.move_target-=shift
            status=(0xF1030000 if self.multiturn else 0xF1020000)|(self.reason<<8)|(int(self.calibrated)<<7)|(self.mode<<4)|8|(int(self.ready)<<2)|(int(self.fault)<<1)|int(self.run)
            raw_arm=(8192+round(self.q*16384/6.2831853))&16383;raw_pend=(2048-round(self.alpha*4096/6.2831853))&4095
            words=[status]+[round(x*65536) for x in (self.q,self.alpha,w,v,iq,-iq/2,iq,iq,ud,uq)]+[0,0,raw_arm|0x80000000,raw_pend,(2048<<16)|2048,int((now-self.started)*50_000_000)&0xFFFFFFFF]
            if self.multiturn:
                motion=self.motion_kind|int(self.run and self.mode==0 and self.motion_kind!=0)*8|int(self.motion_arrived)*32|int(self.motion_ending is not None)*64|int(home_settled)*256|int(home_active)*512|int(self.keep_session)*1024
                words.extend((self.frame_turns,round(self.motion_goal*65536),motion))
            if self.cold_snapshot:words=[0xF1040000|8|(int(self.fault)<<1)|int(self.run)]+[0]*29
            elif self.bootstrap_v4:
                words[0]=(words[0]&0xff00ffff)|0x00040000
                if not self.calibrated or self.postcommit_health=='missing' or self.committed_at is not None and now-self.committed_at<self.postcommit_delay:words=[words[0]&~4]+[0]*29
                else:
                    stamp=words[16];sample=(stamp-(100000 if self.postcommit_health=='stale' else 100))&0xffffffff
                    flags=30|(256|512 if self.run and self.mode in (3,4) else 0)
                    reference=self.motion_rpm*math.pi/30 if self.motion_kind==3 and self.mode==0 else 0.
                    if self.motion_kind==3 and self.mode==0:flags|=1
                    words.extend((65536,0,round(reference*65536),round(self.q*65536),round(w*65536),sample,sample,flags,0,round(w*65536)))
                    if self.postcommit_health=='off_not_ready':
                        # B04 OFF receipt, not an electrical model or measured current.
                        words[5]=214919;words[6]=215367;words[7]=-430220
                        words[15]=(8<<16)|4;words[20]=0;words[27]=1054
            for addr,value in enumerate(words):self._frame(0x80,addr,self.tx_seq,value);self.tx_seq=(self.tx_seq+1)&255
        if not self.buffer:time.sleep(self.timeout)
        data=bytes(self.buffer[:size]);del self.buffer[:size];return data
    def close(self):self.closed=True
