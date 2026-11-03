"""ESP32 P,0,15000/PZERO preparation adapted to fresh FPGA 50 Hz samples."""
import math
from .fpga_protocol import Calibration
from .fpga_reset import ResetFlow


def slope(t,y):
    mt=sum(t)/len(t);my=sum(y)/len(y);den=sum((x-mt)**2 for x in t)
    if den<=0:raise ValueError('零点采样时基未推进')
    return sum((x-mt)*(v-my) for x,v in zip(t,y))/den


def validate_zero(rows):
    if not rows or rows[-1]['elapsed_s']<15:raise ValueError('尚未完成15秒真实采样')
    last=rows[-1]['elapsed_s'];tail=[r for r in rows if r['elapsed_s']>=last-1.-1e-6]
    t=[r['elapsed_s'] for r in tail]
    if len(t)<40 or t[-1]-t[0]<.98-1e-6:raise ValueError('末1秒覆盖不足（50Hz至少40帧/0.98秒）')
    if any(not 0<b-a<=.1 for a,b in zip(t,t[1:])):raise ValueError('末1秒采样存在断档')
    arm0=tail[0]['arm_raw'];pend0=tail[0]['pend_raw']
    q=[((r['arm_raw']-arm0+8192)%16384-8192)*math.tau/16384 for r in tail]
    pend=[pend0+(r['pend_raw']-pend0+2048)%4096-2048 for r in tail]
    rms=math.sqrt(sum(r['w']**2 for r in tail)/len(t));span=max(q)-min(q);drift=slope(t,q)
    pspan=max(pend)-min(pend);pdrift=slope(t,pend);mean=sum(pend)/len(pend)%4096
    if rms>.15 or span>.015 or abs(drift)>.03:raise ValueError('旋臂未静止，未写零点')
    if pspan>2 or abs(pdrift)>1:raise ValueError('摆杆原始零点不稳定，未写零点')
    value=math.floor(mean+.5)%4096
    return dict(pend_down=value,mean_count=mean,quantization_error_count=(value-mean+2048)%4096-2048,
                samples=len(t),coverage_s=t[-1]-t[0],arm_w_rms=rms,arm_span_rad=span,arm_drift_rad_s=drift,
                pend_span_count=pspan,pend_drift_count_s=pdrift,
                source='ESP32 P,0,15000 last-1s: v maps to FPGA w; 80/100Hz -> 40/50Hz, actual time coverage required',
                physical_precondition='Confirmed freely hanging pendulum, output disabled; old 3398.72 +/-8 not transplanted')


class PendulumZero:
    def __init__(self,s):
        self.s=s;self.started=s.clock();self.phase='stop';self.reset_used=False
        self.rows=[];self.previous=None;self.elapsed=0.;self.before=None;self.epoch=None;self.progress=-1
        self.receipts=[]

    def command(self,action,data=None):
        s=self.s;s.zero_dispatch=True
        try:s.request(action,data)
        finally:s.zero_dispatch=False

    def fail(self,reason):self.s.operation_failed('辅助下垂零点标定中止：'+reason)

    def poll(self):
        s=self.s;now=s.clock();r=s.communication_status()
        if now-self.started>45:self.fail('45秒准备截止，未自动重试');return
        if not r:return
        if r['fault']:self.fail('设备故障；未自动清错');return
        if s.reset is not None:return
        if r['run']:
            if self.phase!='stop':self.fail('标定期间必须停止且输出禁用')
            return
        idle=not s.queue and s.pending is None and s.await_state is None
        fresh=r.get('protocol_revision')==4 and r.get('control_sample_fresh') and r.get('telemetry_current',True) and r['sensor_valid']
        if self.phase in ('stop','reset') and idle:
            if not fresh:
                if self.reset_used:self.fail('单次复位后仍无有效新采样');return
                if not s.capabilities.get(33) or not r.get('reset_eligible') or s.persist_reset is None:
                    self.fail('无新采样且当前不满足RST1自身条件');return
                self.reset_used=True;self.phase='reset';s.busy=True
                s.reset=ResetFlow(s,s.persist_reset,dict(r))
                s.emit('log',text='辅助标定：采样陈旧，按RST1条件单次恢复；不清错、不使能');return
            self.epoch=r['reset_epoch'];self.phase='read';s.device_words=None;s.busy=False
            self.command('read');return
        if self.phase=='read' and idle and s.device_words is not None:
            self.before=list(s.device_words);Calibration.from_words(self.before)
            self.phase='sample';s.busy=True
            s.emit('log',text='正在标定下垂零点：输出禁用，保持自由下垂；采集15秒，检查末1秒静止性')
        if self.phase in ('sample','apply','health'):
            if r.get('reset_epoch')!=self.epoch:self.fail('采样世代改变');return
            if self.phase=='sample' and not fresh:self.fail('采样失鲜/无效，旧快照不准写');return
        if self.phase=='sample':
            stamp=r['stamp'];completed=r['completed_control_sample_stamp']
            if self.previous is not None:
                old_stamp,old_completed,old_at=self.previous
                if stamp==old_stamp:
                    if now-old_at>.25:self.fail('快照时间戳未推进')
                    return
                dt=((stamp-old_stamp)&0xffffffff)/50_000_000
                dc=((completed-old_completed)&0xffffffff)/50_000_000
                if not 0<dt<=.1 or not 0<dc<=.1 or not 0<=now-old_at<=.1:self.fail('快照/完成采样时间不连续');return
                self.elapsed+=dt
            self.previous=(stamp,completed,now)
            self.rows.append(dict(elapsed_s=self.elapsed,stamp=stamp,completed_control_sample_stamp=completed,
                received_monotonic=s.status_at if s.status_at is not None else s.last_rx,epoch=self.epoch,
                arm_raw=r['arm_raw'],pend_raw=r['pend_raw'],w=r['w'],run=r['run'],fault=r['fault'],sensor_valid=r['sensor_valid'],control_sample_fresh=r['control_sample_fresh']))
            if int(self.elapsed)!=self.progress:
                self.progress=int(self.elapsed);s.emit('zero_calibration_progress',seconds=min(15,self.elapsed))
            if self.elapsed<15:return
            try:
                checks=validate_zero(self.rows);self.words=list(self.before);self.words[5]=checks['pend_down']
                self.document=dict(phase='prepared',epoch=self.epoch,before_words=self.before,words=self.words,checks=checks,rows=self.rows,
                    collection_start=self.rows[0],collection_end=self.rows[-1],stopped_row=dict(r))
                prepared=s.persist_zero(self.document)  # Durable evidence + synchronous admission hook before any CAL write.
                if not isinstance(prepared,str) or not prepared:raise ValueError('候选证据未持久保存')
                self.document['prepared_evidence_path']=prepared
                self.phase='apply';s.busy=False;self.command('apply',Calibration.from_words(self.words))
            except Exception as exc:self.fail(str(exc))
        elif self.phase=='health':
            if now-self.health_after>2:self.fail('提交回读后2秒未确认新鲜标定状态');return
            if not (idle and not s.busy and fresh and r['calibrated'] and s.status_at>=self.health_after and
                    0<(r['completed_control_sample_stamp']-self.previous[1])&0xffffffff<=100_000_000):return
            if s.device_words!=self.words:self.fail('提交后8字回读不一致');return
            try:
                doc=dict(self.document,phase='verified',readback_words=list(s.device_words),verified_row=dict(r),receipts=list(self.receipts))
                path=s.persist_zero(doc)  # Synchronous confirmation must finish before demo can enter queue.
                if not isinstance(path,str) or not path:raise ValueError('新profile未持久保存')
            except Exception as exc:self.fail(str(exc));return
            s.emit('zero_calibration_complete',words=self.words,profile_path=path,epoch=self.epoch)
            s.zero_calibration=None;s.busy=False
            s.emit('sequence_state',active=False,sequence='zero_calibration')
            s.emit('busy',value=False)
            s.emit('log',text='辅助下垂零点标定完成，新profile已保存；保持停止，未自动启动演示')
