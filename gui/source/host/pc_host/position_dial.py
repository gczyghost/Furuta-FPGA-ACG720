"""Discrete position intent; later knob input can reuse the same signal."""
import math
from PySide6.QtCore import Qt,QPointF,Signal,QEvent,QTimer
from PySide6.QtGui import QColor,QPainter,QPen
from PySide6.QtWidgets import QWidget,QPushButton,QLabel
from .theme import PALETTES
from .ui_timing import timed_ui

class PositionDial(QWidget):
    positionRequested=Signal(float)
    turnRequested=Signal(int)

    def __init__(self,parent=None):
        super().__init__(parent)
        self.theme='dark';self.angle=None;self.goal=None
        self.geometry_scale=1.
        self.setMinimumSize(250,270)
        self.setMaximumHeight(270)
        self.buttons={}
        for degree in range(-150,181,30):
            button=QPushButton(f'{degree}°',self);button.setObjectName('dialPosition')
            button.setAccessibleName(f'快速移位到 {degree} 度')
            button.setToolTip(f'点击即快速定位至 {degree}°，保持当前摆杆姿态')
            button.clicked.connect(lambda checked=False,d=degree:self.positionRequested.emit(float(d)))
            self.buttons[degree]=button
        self.home=QPushButton('回零',self);self.home.clicked.connect(lambda:self.positionRequested.emit(0.))
        self.readout=QLabel('位置 —',self);self.readout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.turns={}
        for sign,text in [(-1,'↶ −360°'),(1,'↷ +360°')]:
            b=QPushButton(text,self);b.setToolTip('按标示方向连续旋转一周后停止，保持使能')
            b.clicked.connect(lambda checked=False,s=sign:self.turnRequested.emit(s));self.turns[sign]=b
        QTimer.singleShot(0,self.update_metrics)

    def changeEvent(self,event):
        super().changeEvent(event)
        if event.type() in (QEvent.FontChange,QEvent.StyleChange) and hasattr(self,'turns'):
            QTimer.singleShot(0,self.update_metrics)

    def update_metrics(self):
        ratios=[1.]
        for button in self.buttons.values():
            hint=button.minimumSizeHint();ratios.extend((hint.width()/44,hint.height()/28))
        hint=self.home.minimumSizeHint();ratios.extend((hint.width()/66,hint.height()/31))
        for button in self.turns.values():
            hint=button.minimumSizeHint();ratios.extend((hint.width()/110,hint.height()/32))
        self.geometry_scale=max(ratios)
        self.setMinimumSize(math.ceil(250*self.geometry_scale),math.ceil(270*self.geometry_scale))
        self.setMaximumHeight(math.ceil(270*self.geometry_scale));self.layout_buttons();self.updateGeometry();self.update()

    def resizeEvent(self,event):
        self.layout_buttons()

    def layout_buttons(self):
        s=self.geometry_scale;cx=self.width()/2;cy=111*s;radius=91*s
        for degree,b in self.buttons.items():
            a=math.radians(degree);b.setGeometry(round(cx+radius*math.sin(a)-22*s),round(cy-radius*math.cos(a)-14*s),math.ceil(44*s),math.ceil(28*s))
        self.home.setGeometry(round(cx-33*s),round(87*s),math.ceil(66*s),math.ceil(31*s))
        self.readout.setGeometry(round(cx-52*s),round(122*s),math.ceil(104*s),math.ceil(26*s))
        self.turns[-1].setGeometry(round(10*s),round(232*s),self.width()//2-round(15*s),math.ceil(32*s))
        self.turns[1].setGeometry(self.width()//2+round(5*s),round(232*s),self.width()//2-round(15*s),math.ceil(32*s))

    def set_position(self,angle,goal=None):
        self.angle=angle;self.goal=goal
        self.readout.setText('位置 —' if angle is None else f'{math.remainder(angle,360):+.1f}°')
        self.update()

    @timed_ui('paint.position')
    def paintEvent(self,event):
        c=PALETTES[self.theme];p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        s=self.geometry_scale;center=QPointF(self.width()/2,111*s);p.setPen(QPen(QColor(c['line']),2));p.drawEllipse(center,69*s,69*s)
        for angle,color,radius in [(self.goal,'muted',64*s),(self.angle,'teal',69*s)]:
            if angle is None:continue
            a=math.radians(angle);tip=center+QPointF(radius*math.sin(a),-radius*math.cos(a))
            p.setPen(QPen(QColor(c[color]),3));p.drawEllipse(tip,4,4)
        p.end()
