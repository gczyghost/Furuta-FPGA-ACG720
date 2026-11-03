"""Two live budgets with explicit stale state, not inferred temperature."""
from PySide6.QtWidgets import QWidget,QHBoxLayout,QLabel,QProgressBar
from .motion import resource_text

class ResourceBars(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);layout=QHBoxLayout(self);layout.setContentsMargins(0,0,0,0);layout.setSpacing(20)
        self.current_label=QLabel();self.current=QProgressBar();self.heat_label=QLabel();self.heat=QProgressBar();self.reason=QLabel()
        self.reason.setParent(self);self.reason.hide()
        for bar in (self.current,self.heat):bar.setRange(0,1000);bar.setTextVisible(False);bar.setFixedHeight(7)
        for text,bar in ((self.current_label,self.current),(self.heat_label,self.heat)):
            group=QHBoxLayout();group.setSpacing(8);group.addWidget(text);group.addWidget(bar,1)
            bar.setMinimumWidth(50);layout.addLayout(group,1)
        self.setToolTip('电流横条：实测 |Iq| / 当前指令限流。I²t：最近 15–16 秒三相电流平方和积分；不是温度。快移入口 ≤0.65，下落入口 ≤0.50，总保护 1.00 A²s。')
        self.refresh(None,'I',None,False)

    def refresh(self,budget,mode,iq,fresh,recorded=False):
        _,reason=resource_text(budget,mode,fresh)
        if recorded and budget is None:reason='此记录未包含额度数据'
        elif recorded and not fresh:reason='记录时额度已过期'
        self.reason.setText(reason)
        self.setToolTip(('历史额度 · ' if recorded else '')+reason+'。横条为已用比例；I²t 总额度 1.000 A²s，不是温度。')
        for widget in (self.current_label,self.current,self.heat_label,self.heat):widget.setToolTip(reason)
        valid=fresh and budget is not None
        for bar in (self.current,self.heat):bar.setEnabled(valid)
        if not valid:
            self.current_label.setText('电流额度 —');self.heat_label.setText('I²t 额度 —')
            self.current.setValue(0);self.heat.setValue(0);return
        cap=budget['cap'] if mode!='I' else 0;dose=budget['dose'];iq=abs(iq) if iq is not None else 0
        self.current_label.setText(f'电流  {iq:.2f} / {cap:.2f} A' if cap else '电流  — · 未使能')
        self.current.setValue(round(min(1,iq/cap)*1000) if cap else 0)
        self.heat_label.setText(f'I²t {dose:.3f} · 余 {max(0,1-dose):.3f} A²s')
        self.heat.setToolTip(f'已用 {dose:.3f} / 总额度 1.000 A²s · '+reason)
        self.heat.setValue(round(min(1,dose)*1000))

