"""Thin deployment lifecycle. No command, device access or historical grants here.

The injected release adapter must implement bounded prepare/deploy/verify/close.
prepare owns the existing lease and returns fresh, audited safety evidence;
verify returns the just-deployed identity, exact port and lease-bound serial factory.
The adapter holds ownership until close, including through the UART session.
"""
import threading


class DeploymentFlow:
    def __init__(self,adapter,target=None):
        self.adapter=adapter;self.state='idle';self.cancelled=threading.Event();self.closed=False
        # Supplied by the exact current deployment entry, never by a CAL profile.
        self.target=None if target is None else dict(identity=dict(target['identity']),port=target['port'])
    def cancel(self):self.cancelled.set()
    def check(self):
        if self.cancelled.is_set():raise ValueError('配置已取消，未自动连接')
    def close(self):
        if not self.closed:
            self.adapter.close();self.closed=True
    def run(self):
        if self.state!='idle':raise ValueError('配置流程不可重试或重复执行')
        try:
            self.state='checking';self.check()
            evidence=self.adapter.prepare(self.cancelled)
            required=('motion_stopped','uart_owner_exited','exclusive_held','dps_off','zero_voltage_current','power_handle_closed')
            if not isinstance(evidence,dict) or any(evidence.get(key) is not True for key in required):
                raise ValueError('停止、UART退出、独占或电源OFF证据缺失/未知，拒绝配置')
            self.check();self.state='deploying';self.adapter.deploy(self.cancelled)
            self.check();self.state='verifying';result=self.adapter.verify(self.cancelled)
            self.check()
            identity=result.get('identity',{}) if isinstance(result,dict) else {}
            if set(identity)!={'board_id','uart_pnp_id','bitstream_sha256','protocol_revision'}:raise ValueError('新部署身份字段不完整')
            if any(not isinstance(identity[key],str) or not identity[key] for key in ('board_id','uart_pnp_id')):raise ValueError('设备身份无效')
            digest=identity['bitstream_sha256']
            if not isinstance(digest,str) or len(digest)!=64 or any(char not in '0123456789abcdef' for char in digest):raise ValueError('新产物哈希无效')
            if type(identity['protocol_revision']) is not int or identity['protocol_revision'] not in (2,3,4):raise ValueError('新部署协议版本无效')
            if not callable(result.get('serial_factory')) or not isinstance(result.get('port'),str) or not result['port']:raise ValueError('缺少独占串口入口')
            if self.target is not None and (identity!=self.target['identity'] or result['port']!=self.target['port']):raise ValueError('部署核验结果与本次预期设备/位流/端口不符')
            self.state='ready';return dict(result,identity=dict(identity))
        except Exception:
            self.state='failed';self.close();raise
