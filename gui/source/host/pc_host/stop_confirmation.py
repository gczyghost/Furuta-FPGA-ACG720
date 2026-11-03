"""Wire STOP receipt followed by a complete live stopped-status snapshot."""
from .fpga_protocol import SnapshotAssembler

def expected_stop_epoch(session,baseline):
    verified=getattr(session,'verified_reset_epoch',None)
    if verified is not None:return verified
    acknowledged=getattr(session,'stop_reset_epoch',None)
    return acknowledged if acknowledged is not None else baseline.get('reset_epoch')

class StopConfirmation:
    def __init__(self,seq,epoch=None,revision=None):
        self.seq=seq;self.epoch=epoch;self.revision=revision
        self.ack=False;self.assembler=None

    def feed(self,frame,now):
        if frame['op']==0x81 and frame['addr']==0 and frame['seq']==self.seq and frame['value']==0:
            if not self.ack:self.ack=True;self.assembler=None
            return None
        if not self.ack or frame['op']!=0x80:return None
        # A snapshot begun before the correlated ACK cannot establish STOP.
        # Each subsequent complete snapshot carries live run/epoch even when
        # completed-control measurement stamps have been frozen for minutes.
        if frame['addr']==0:self.assembler=SnapshotAssembler()
        if self.assembler is None:return None
        row=self.assembler.feed(frame,now)
        if row is None or row['run']:return None
        if self.epoch is not None and row.get('reset_epoch')!=self.epoch:return None
        if self.revision is not None and row['protocol_revision']!=self.revision:return None
        return row
