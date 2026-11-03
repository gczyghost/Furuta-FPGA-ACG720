"""Compact native command panels."""
from PySide6.QtWidgets import (QDoubleSpinBox,QGridLayout,QHBoxLayout,QPushButton,
                              QProgressBar,QVBoxLayout,QWidget,QSlider)
from PySide6.QtCore import Qt
from .widgets import ThemedComboBox
from .motion import DEMO_NAME
from .position_dial import PositionDial
from .velocity import SPEED_RPMS

def build_controls(w,left,label,panel):
    box,layout=panel();layout.addWidget(label('02  /  运行控制','section'))
    w.run_btn=QPushButton('演示 · 高性能控制');w.run_btn.setObjectName('primary')
    w.run_btn.setToolTip(DEMO_NAME+'；正常结束保持使能。停止 / Esc 立即关桥。')
    w.run_btn.clicked.connect(lambda:w.action('demo'));layout.addWidget(w.run_btn)
    layout.addWidget(label('下垂快移 → 起摆快移 → 受控下落','muted',True))
    row=QHBoxLayout()
    w.enable_btn=QPushButton('使能保持');w.enable_btn.clicked.connect(lambda:w.stop() if w.enabled else w.action('enable'))
    w.continuous_btn=QPushButton('起摆稳摆');w.continuous_btn.clicked.connect(lambda:w.action('continuous'))
    w.continuous_btn.setToolTip('回零后起摆并持续保持；可移位或受控下落。')
    w.swing_btn=w.continuous_btn  # One visible action; retained for existing acceptance hooks.
    row.addWidget(w.enable_btn);row.addWidget(w.continuous_btn);layout.addLayout(row)
    row=QHBoxLayout();w.drop_direction=ThemedComboBox()
    w.drop_direction.addItem('正向','drop_positive');w.drop_direction.addItem('反向','drop_negative')
    w.drop_btn=QPushButton('受控下落');w.drop_btn.clicked.connect(lambda:w.action(w.drop_direction.currentData()))
    w.drop_btn.setToolTip('先回零稳摆，再主动制动下落；结束后保持使能。')
    row.addWidget(w.drop_direction);row.addWidget(w.drop_btn,1);layout.addLayout(row)
    w.recover_btn=QPushButton('核验并恢复故障');w.recover_btn.clicked.connect(lambda:w.action('recover'));layout.addWidget(w.recover_btn)
    w.reset_btn=QPushButton('软复位控制板');w.reset_btn.clicked.connect(lambda:w.action('soft_reset'));layout.addWidget(w.reset_btn)
    w.reset_btn.setToolTip('先关桥并保存现场，再重启控制板、清除易失状态。保留物理零点；复位后空闲，重新使能时需初始化。需要 PC_LIVE v23。')
    w.phase=label('等待连接',wrap=True);layout.addWidget(w.phase)
    w.progress=QProgressBar();w.progress.setRange(0,1000);w.progress.setTextVisible(False);layout.addWidget(w.progress)
    w.elapsed=label('—','muted');layout.addWidget(w.elapsed);left.addWidget(box)

    box,layout=panel();layout.addWidget(label('03  /  快速定位','section'))
    w.position_dial=PositionDial();w.position_dial.positionRequested.connect(w.request_position)
    w.position_dial.turnRequested.connect(w.request_turn)
    layout.addWidget(w.position_dial)
    layout.addWidget(label('点选即执行 · 30° 刻度 · 绿点为实测位置','muted',True));left.addWidget(box)

    box,layout=panel();layout.addWidget(label('04  /  速度伺服','section'))
    w.servo_readout=label('目标 0 rpm · 范围 −200 至 +200');layout.addWidget(w.servo_readout)
    w.speed_slider=QSlider(Qt.Orientation.Horizontal);w.speed_slider.setRange(-200,200)
    w.speed_slider.setSingleStep(1);w.speed_slider.setPageStep(10);w.speed_slider.setValue(0)
    w.speed_slider.valueChanged.connect(w.servo_target_changed);layout.addWidget(w.speed_slider)
    row=QHBoxLayout();w.speed_btn=QPushButton('进入速度伺服');w.speed_btn.clicked.connect(w.request_speed)
    w.speed_zero=QPushButton('目标归零');w.speed_zero.clicked.connect(lambda:w.speed_slider.setValue(0))
    row.addWidget(w.speed_btn,1);row.addWidget(w.speed_zero);layout.addLayout(row)
    layout.addWidget(label('拖动即改速 · 平滑加减速 · 保持目标速度\n归零后稳摆；退出时先减速。需 v27 固件。','muted',True));left.addWidget(box)

    box,layout=panel();toggle=QPushButton('校准与检查 +');toggle.setCheckable(True);layout.addWidget(toggle)
    advanced=QWidget();content=QVBoxLayout(advanced);content.setContentsMargins(0,0,0,0)
    w.center_btn=QPushButton('登记物理零点');w.center_btn.clicked.connect(lambda:w.action('register'))
    w.center_btn.setToolTip('禁用静止检查后覆盖板端固定零点；日常使用不需要重新登记。')
    w.passive_btn=QPushButton('禁用采样检查 · 5 s');w.passive_btn.clicked.connect(lambda:w.action('passive'))
    content.addWidget(w.center_btn);content.addWidget(w.passive_btn)
    w.center_label=label('物理零点：等待读回','muted',True);content.addWidget(w.center_label)
    w.state_label=label('桥状态 —    故障 —',wrap=True);content.addWidget(w.state_label)
    layout.addWidget(advanced);advanced.hide()
    toggle.toggled.connect(advanced.setVisible)
    toggle.toggled.connect(lambda on:toggle.setText('校准与检查 −' if on else '校准与检查 +'))
    left.addWidget(box)
