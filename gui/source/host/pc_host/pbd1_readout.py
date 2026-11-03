"""Fixed PBD1-v1 read transaction; called only by the existing serial owner."""
CAPABILITY=0x50424431
ORDER=(36,37,38,39,40,41,42,37)

class PBD1Readout:
    def __init__(self,session):
        self.s=session;self.replies=[];self.index=0
        self.s.busy=True;self.s._put(0x12,36,0,'pbd1')

    def finish(self,status,reason,decoded=None):
        self.s.pbd1=None;self.s.busy=False
        self.s.emit('pbd1_readout',status=status,reason=reason,replies=list(self.replies),decoded=decoded)
        self.s.emit('busy',value=False)

    def reply(self,frame,now):
        self.replies.append(dict(op=frame['op'],addr=frame['addr'],seq=frame['seq'],value=frame['value'],received_monotonic=now))
        if frame['op']==0x82:
            unsupported=self.index==0 and frame['value']==2
            self.finish('UNSUPPORTED' if unsupported else 'UNKNOWN','capability unsupported' if unsupported else 'NACK')
            return unsupported  # Only capability/address unsupported bypasses generic NACK handling.
        if self.index==0 and frame['value']!=CAPABILITY:
            self.finish('UNKNOWN','incompatible capability');return True
        self.index+=1
        if self.index<len(ORDER):
            self.s._put(0x12,ORDER[self.index],0,'pbd1');return True
        cap,first,a,b,delta,stamp,pend_stamp,last=[r['value'] for r in self.replies]
        signed_delta=delta if delta<0x80000000 else delta-0x100000000
        if first!=last or first&~0x1fff or a>0x3ffff or b>0x3ffff or not -2048<=signed_delta<=2047:
            self.finish('UNKNOWN','inconsistent metadata or malformed fields');self.s.stop();return True
        decoded=dict(captured=bool(first&1),metadata_first=first,metadata_last=last,first18=a,second18=b,
                     delta_signed12=signed_delta,fault_stamp=stamp,pend_stamp=pend_stamp,
                     predicates=(first>>1)&63,run=bool(first&128),mode=(first>>8)&7,motion_kind=(first>>11)&3)
        self.finish('CAPTURED' if first&1 else 'NO_CAPTURE','coherent read-only record',decoded)
        return True
