"""Bounded stopped CAL transaction for the existing sole probe owner.

No transport, power, Claim or motion is created here. The frozen caller's guard
must check its current exact grant, same lease and fresh independent DPS OFF
evidence; the public TX ledger remains authoritative. Cold frames are emitted
as bootstrap_status only, never as normal healthy samples.
"""
from collections import deque
import time
from .fpga_protocol import BoundCalibrationProfile,SnapshotAssembler,StreamDecoder,encode,MAGIC


class ColdBootstrap:
    CAPS={24:0x484f4d32,26:0x4d4f5432,29:0x454e4131,32:0x53563237,33:0x52535431,35:0x53504431}
    def __init__(self,send,emit,identity,profile,guard,clock=time.monotonic):
        if not isinstance(profile,BoundCalibrationProfile) or not profile.matches(identity,4):raise ValueError('精确新身份/profile不匹配')
        if not callable(guard):raise ValueError('需要冻结调用者的当前授权/OFF核验')
        self.send=send;self.emit=emit;self.identity=dict(identity);self.words=list(profile.words);self.guard=guard;self.clock=clock
        self.decoder=StreamDecoder();self.assembler=SnapshotAssembler();self.queue=deque();self.pending=None;self.seq=0
        self.started=clock();self.last_hb=self.started;self.phase='identify';self.failed=False;self.done=False
        self.latest=None;self.last_status=None;self.epoch=None;self.bank={};self.after={};self.health_at=None;self.health_stamp=None;self.health_advances=0
        self.verified_reset_epoch=None
        self.frame_sync=False;self.expected_addr=None;self.telemetry_seq=None;self.tx_started=False
        self.health_times=deque(maxlen=3);self.readback_started=None;self.readback_first=None
        self.queue.extend([(2,0,0,'stop'),(4,0,MAGIC[4],'sync')])
        self.queue.extend((0x12,addr,0,'cap') for addr in self.CAPS)
        self.queue.append((0x12,34,0,'epoch'))
        self.queue.extend((0x12,addr,0,'before') for addr in range(8))
    def fail(self,reason):
        if self.failed:return
        self.failed=True;self.phase='failed';self.queue.clear();self.pending=None
        try:self.send(encode(2,0,self.seq,0))
        finally:self.emit('bootstrap_failed',text=reason)
    def cancel(self):self.fail('停止态配置已取消，不重试')
    def reset_health(self):
        self.health_stamp=None;self.health_advances=0;self.health_times.clear()
    def stop(self):
        if self.done:
            self.send(encode(2,0,self.seq,0));self.seq=(self.seq+1)&255
        else:self.cancel()
    def request(self,action,data=None):
        if action=='stop':self.stop();return
        if action!='bootstrap_profile' or self.phase!='prepared' or self.failed:raise ValueError('仅允许已准备的显式停止态bootstrap_profile')
        if not self.latest or self.clock()-self.last_status>.25 or self.latest['run'] or self.latest['fault'] or self.latest['calibrated']:raise ValueError('冷态状态不新鲜或不合格')
        self.phase='configure'
        self.queue.extend((0x10,addr,word,'write') for addr,word in enumerate(self.words))
        self.queue.append((0x11,0,MAGIC[0x11],'commit'))
    def feed(self,data):
        if self.failed or self.done:return
        if self.last_status is None and self.clock()-self.started>.25:
            self.fail('初始完整遥测帧同步250ms截止');return
        for frame in self.frames(data):
            if self.failed:return
            if frame['op']==0x80:
                if not self.frame_sync:
                    if frame['addr']!=0:continue
                    self.frame_sync=True;self.expected_addr=0;self.telemetry_seq=frame['seq']
                if frame['addr']!=self.expected_addr or frame['seq']!=self.telemetry_seq:
                    self.fail('同步后遥测地址或独立TX序列不连续');return
                self.expected_addr=(self.expected_addr+1)%30;self.telemetry_seq=(self.telemetry_seq+1)&255
                if frame['addr']==0:self.assembler=SnapshotAssembler()
                try:row=self.assembler.feed(frame,self.clock())
                except ValueError as exc:self.fail(str(exc));return
                if self.assembler.dropped:self.fail('冷态完整帧序列不连续');return
                if row is None:continue
                if row['protocol_revision']!=4 or row['run'] or row['fault']:
                    self.fail('配置期间版本/停止/故障状态不符');return
                if self.epoch is not None and row['reset_epoch']!=self.epoch:self.fail('配置期间epoch改变');return
                # Invalidate every bad observation, even when a later complete
                # row arrives in the same read() before the next poll().
                if self.phase in ('health','readback') and not (row['calibrated'] and row['sensor_valid'] and row['control_sample_fresh']):
                    self.reset_health()
                self.latest=row;self.last_status=self.clock();self.emit('cold_status',row=dict(row))
                continue
            if not self.tx_started:continue  # Opening may also encounter an old reply.
            p=self.pending
            if frame['op'] not in (0x81,0x82):self.fail('配置收到未知响应');return
            if p is None or frame['addr']!=p['addr'] or frame['seq']!=p['seq']:
                self.fail('配置响应序号/地址不匹配或迟到');return
            if frame['op']!=0x81 or p['op']!=0x12 and frame['value']!=p['value']&0xffffffff:
                self.fail('配置命令拒绝或ACK值不符');return
            self.pending=None;tag=p['tag'];value=frame['value']
            self.emit('bootstrap_reply',op=p['op'],addr=p['addr'],seq=p['seq'],value=value,epoch=self.epoch)
            if p['op']==0x12:self.emit('read_reply',addr=p['addr'],seq=p['seq'],value=value,epoch=self.epoch)
            if tag=='cap' and value!=self.CAPS[p['addr']]:self.fail('配置能力不匹配');return
            if tag=='epoch':self.epoch=value
            elif tag=='before':
                self.bank[p['addr']]=value
                if p['addr']==7:
                    if not self.latest or self.clock()-self.last_status>.25 or self.latest['calibrated'] or list(self.bank.values())!=[0]*8:
                        self.fail('冷态配置要求新鲜停止状态及未配置8zero，拒绝覆盖');return
                    if self.latest['reset_epoch']!=self.epoch:self.fail('冷态epoch不一致');return
                    self.phase='prepared';self.emit('bootstrap_prepared',identity=dict(self.identity),epoch=self.epoch,words=[self.bank[i] for i in range(8)])
            elif tag=='commit':self.health_at=self.clock();self.health_stamp=None;self.phase='health'
            elif tag=='after':
                if self.readback_first is None:self.readback_first=self.clock()
                self.after[p['addr']]=value
                if p['addr']==7:
                    if [self.after[i] for i in range(8)]!=self.words:self.fail('配置逐字回读不一致');return
    def frames(self,data):
        # Serial open may start at any byte. Seek a CRC-valid addr0 within the
        # initial deadline; after it, corruption is fatal, including same-read
        # suffix errors. ACK packets echo command seq and never consume TX seq.
        for byte in data:
            before=self.decoder.errors
            frames=self.decoder.feed(bytes((byte,)))
            if self.frame_sync and self.decoder.errors!=before:
                self.fail('同步后配置通信CRC/帧头错误');return
            yield from frames
    def poll(self):
        if self.failed or self.done:return
        now=self.clock()
        if now-self.started>5:self.fail('停止态配置5秒截止');return
        if self.last_status is None:
            if now-self.started>.25:self.fail('初始完整遥测帧同步250ms截止')
            return
        if self.last_status is not None and now-self.last_status>.25:self.fail('冷态协议断流，停止配置');return
        if self.health_at is not None and now-self.health_at>2:self.fail('配置后2秒未取得真实新鲜健康采样');return
        if self.readback_started is not None and now-self.readback_started>.25:
            self.fail('配置独立回读250ms截止，不重读');return
        if self.phase in ('health','readback') and self.latest and self.last_status>=self.health_at:
            row=self.latest
            healthy=(now-self.last_status<=.25 and row['reset_epoch']==self.epoch and row['calibrated'] and row['sensor_valid'] and row['control_sample_fresh'])
            if healthy:
                if self.health_stamp!=row['stamp']:
                    if self.health_stamp is not None and 0<(row['stamp']-self.health_stamp)&0xffffffff<=12_500_000:
                        self.health_advances+=1
                    else:self.health_advances=0;self.health_times.clear()
                    self.health_times.append(self.last_status)
                    self.health_stamp=row['stamp']
                established=self.health_advances>=2 and len(self.health_times)==3 and now-self.health_times[0]<=.25
                if established:
                    if self.phase=='health':
                        self.phase='readback';self.readback_started=now
                        self.queue.extend((0x12,addr,0,'after') for addr in range(8))
                    elif len(self.after)==8 and self.pending is None:
                        if now-self.readback_first>.25:self.fail('配置回读已过期，不重读');return
                        self.done=True;self.phase='complete';self.queue.clear();self.pending=None
                        self.emit('bootstrap_complete',identity=dict(self.identity),words=list(self.words),epoch=self.epoch,row=dict(row),
                                  configured=True,sampling_complete=True,enable_ready=bool(row['ready']),
                                  text='停止态配置与真实采样已完成；'+('板端资格就绪，仍需独立使能授权' if row['ready'] else '尚不可使能：板端ready=false'));return
            else:
                self.reset_health()
                # Stopped readback may await three new healthy observations.
                # Keep the original deadlines and the existing single READ
                # transaction; no write, commit or read is retried.
        if self.pending:
            if now-self.pending['at']>.35:self.fail('配置ACK超时，禁止自动重发')
            return
        if now-self.last_hb>=.2:self.queue.appendleft((1,0,MAGIC[1],'hb'));self.last_hb=now
        if not self.queue:return
        op,addr,value,tag=self.queue.popleft()
        try:
            if self.guard(dict(op=op,addr=addr,value=value),self.phase) is not True:raise ValueError('当前配置授权/独占/OFF证据无效')
            self.pending=dict(op=op,addr=addr,value=value,tag=tag,seq=self.seq,at=now)
            self.tx_started=True
            self.seq=(self.seq+1)&255;self.send(encode(op,addr,self.pending['seq'],value))
        except Exception as exc:self.fail(str(exc))
