"""Original ESP32 console layout with FPGA actions supplied by the host owner."""
from PySide6.QtWidgets import QWidget,QFrame,QLabel,QVBoxLayout,QHBoxLayout,QPushButton,QScrollArea,QTabWidget,QPlainTextEdit,QProgressBar,QComboBox,QStyle,QCheckBox
from PySide6.QtCore import QEvent,QTimer
from .widgets import TraceChart,ThemedComboBox


def label(text,name=None,wrap=False):
    widget=QLabel(text)
    if name:widget.setObjectName(name)
    widget.setWordWrap(wrap);return widget


def panel():
    widget=QFrame();widget.setObjectName('panel');layout=QVBoxLayout(widget)
    layout.setContentsMargins(14,12,14,12);layout.setSpacing(8);return widget,layout

class ContentWidthScrollArea(QScrollArea):
    """Reserve the actual control width; only vertical scrolling is needed."""
    def __init__(self,content):
        super().__init__();self.fit_pending=False
        self.setWidgetResizable(True);self.setWidget(content);content.installEventFilter(self)
        self.verticalScrollBar().setObjectName('leftScrollBar')
        self.schedule_fit()
    def eventFilter(self,watched,event):
        if event.type() in (QEvent.LayoutRequest,QEvent.FontChange,QEvent.StyleChange,QEvent.Show):self.schedule_fit()
        return super().eventFilter(watched,event)
    def schedule_fit(self):
        if not self.fit_pending:self.fit_pending=True;QTimer.singleShot(0,self.fit_content)
    def fit_content(self):
        self.fit_pending=False;content=self.widget()
        if content is None:return
        pixels=content.font().pixelSize()
        scale=pixels/12 if pixels>0 else content.fontMetrics().height()/16
        gutter=round(10*scale)
        margins=content.layout().contentsMargins()
        if margins.right()!=gutter:
            content.layout().setContentsMargins(margins.left(),margins.top(),gutter,margins.bottom())
        content.layout().activate()
        width=max(312,content.minimumSizeHint().width(),content.minimumWidth())
        # Style-sheet scrollbar widths can exceed the base style metric at
        # fractional scaling. Reserve the actual hint plus rounding slack.
        width+=2*self.frameWidth()+max(self.style().pixelMetric(QStyle.PM_ScrollBarExtent),self.verticalScrollBar().sizeHint().width())+4
        if self.width()!=width:self.setFixedWidth(width)


def build_layout(w):
    old=w.takeCentralWidget();root=QWidget();w.setCentralWidget(root)
    old.setParent(root);old.hide();w.retained_diagnostic_widgets=old
    w.resize(1450,980);w.setMinimumSize(1080,760)
    layout=QVBoxLayout(root);layout.setContentsMargins(22,18,22,14);layout.setSpacing(14)
    top=QHBoxLayout();titles=QVBoxLayout();titles.addWidget(label('FURUTA  /  BLDC','title'))
    titles.addWidget(label('FPGA  ·  旋转倒立摆控制台','muted'));top.addLayout(titles);top.addStretch()
    w.badge=label('未连接','badge');top.addWidget(w.badge);top.addWidget(w.theme_combo)
    w.export_btn.setText('导出显示数据');top.addWidget(w.export_btn)
    w.folder_btn=QPushButton('记录目录');w.folder_btn.clicked.connect(w.open_records);top.addWidget(w.folder_btn)
    layout.addLayout(top)
    body=QHBoxLayout();body.setSpacing(16);left_column=QVBoxLayout();left_container=QWidget()
    left=QVBoxLayout(left_container);left.setContentsMargins(0,0,0,0);left.setSpacing(12)
    box,conn=panel();conn.addWidget(label('01  /  设备连接','section'));conn.addWidget(w.transport)
    row=QHBoxLayout();w.port.setMinimumWidth(125)
    w.port.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon);w.port.setMinimumContentsLength(15)
    row.addWidget(w.port,1);row.addWidget(w.refresh);conn.addLayout(row)
    conn.addWidget(w.connect_btn);w.connect_btn.setToolTip('460800 baud · 8N1；连接不启动运动')
    w.deploy_btn=QPushButton('配置设备并连接…')
    w.deploy_btn.setToolTip('FPGA配置专用：核实停止、原串口退出、设备独占与电源OFF；普通连接不重配。')
    w.deploy_btn.clicked.connect(lambda:w.safe(w.configure_and_connect));conn.addWidget(w.deploy_btn)
    conn.addWidget(w.banner);left.addWidget(box)
    box,control=panel();control.addWidget(label('02  /  运行控制','section'))
    w.full_demo_btn.setText('演示');w.full_demo_btn.setObjectName('primary');control.addWidget(w.full_demo_btn)
    control.addWidget(label('起摆 → 稳摆快移 → 高速演示 → 受控落摆','muted',True))
    row=QHBoxLayout();w.session_home_btn.setText('使能保持');w.session_swing_btn.setText('起摆稳摆')
    row.addWidget(w.session_home_btn);row.addWidget(w.session_swing_btn);control.addLayout(row)
    row=QHBoxLayout();w.drop_direction=ThemedComboBox();w.drop_direction.addItem('正向',1);w.drop_direction.addItem('反向',-1)
    w.drop_btn=QPushButton('受控下落');w.drop_btn.clicked.connect(lambda:w.safe(lambda:w.action('drop',w.drop_direction.currentData())))
    row.addWidget(w.drop_direction);row.addWidget(w.drop_btn,1);control.addLayout(row)
    w.clear_btn.setText('核验并恢复故障');control.addWidget(w.clear_btn)
    w.reset_btn=QPushButton('软复位控制板');w.reset_btn.setEnabled(False);w.reset_btn.setToolTip('先STOP确认并保存现场；复位后SYNC核验世代及8字，不自动运动。需要RST1能力。')
    w.reset_btn.clicked.connect(lambda:w.safe(lambda:w.action('soft_reset')));control.addWidget(w.reset_btn)
    w.phase=label('等待连接',wrap=True);control.addWidget(w.phase)
    w.progress=QProgressBar();w.progress.setRange(0,0);w.progress.setTextVisible(False);control.addWidget(w.progress)
    w.elapsed=label('—','muted');control.addWidget(w.elapsed);left.addWidget(box)
    box,position=panel();position.addWidget(label('03  /  快速定位','section'));position.addWidget(w.position_dial)
    position.addWidget(label('点选即执行','muted',True));w.position_dial.setToolTip('30°刻度；绿点为实测位置');left.addWidget(box)
    box,speed=panel();speed.addWidget(label('04  /  速度伺服','section'));speed.addWidget(w.live_servo_value);speed.addWidget(w.live_servo_slider)
    w.live_servo_begin.setText('进入速度伺服');w.live_servo_zero.setText('目标归零');w.live_servo_end.setText('减速退出伺服')
    row=QHBoxLayout();row.addWidget(w.live_servo_begin,1);row.addWidget(w.live_servo_zero);speed.addLayout(row)
    w.live_servo_end.hide()  # The original single entry/exit button owns the visible action.
    speed.addWidget(label('拖动即改速 · 退出时减速','muted',True));left.addWidget(box)
    w.entry_compat=QCheckBox('B14 兼容回零（会移动）');speed.addWidget(w.entry_compat)
    w.entry_compat.setToolTip('可选过渡动作：速度进入前先回零并等待局部归帧；不等价于原地进入。默认关闭，保留直接请求。')
    w.entry_compat.toggled.connect(w.set_entry_compat)
    w.entry_compat_note=label('先回零再进入，不等价于原地进入。', 'muted',True)
    w.entry_compat_note.hide();speed.addWidget(w.entry_compat_note)
    box,cal=panel();toggle=QPushButton('校准与检查 +');toggle.setCheckable(True);cal.addWidget(toggle);w.calibration_toggle=toggle
    cal.addWidget(w.calibration_tabs);w.calibration_tabs.hide();toggle.toggled.connect(w.calibration_tabs.setVisible)
    toggle.toggled.connect(lambda active:toggle.setText('校准与检查 −' if active else '校准与检查 +'));left.addWidget(box)
    box,replay=panel();replay.addWidget(label('历史回放','section'));open_btn=QPushButton('打开采样 CSV');open_btn.clicked.connect(lambda:w.safe(w.open_replay));replay.addWidget(open_btn)
    row=QHBoxLayout();row.addWidget(w.play_btn,1);w.replay_speed=ThemedComboBox()
    for rate in (.25,.5,1,2,4):w.replay_speed.addItem(f'{rate:g}×',rate)
    w.replay_speed.setCurrentIndex(2);w.replay_speed.currentIndexChanged.connect(w.change_replay_speed)
    row.addWidget(w.replay_speed);replay.addLayout(row)
    replay.addWidget(w.replay_seek);replay.addWidget(w.plot_window);left.addWidget(box);left.addStretch()
    scroll=ContentWidthScrollArea(left_container)
    left_column.addWidget(scroll,1);w.left_scroll=scroll
    w.stop_btn.setText('■  停止 / 关桥                 Esc');w.stop_btn.setObjectName('stop');w.stop_btn.setMinimumHeight(48);left_column.addWidget(w.stop_btn);body.addLayout(left_column)
    right=QVBoxLayout();metrics=QHBoxLayout();w.metric_labels={}
    for key,name in [('theta','旋臂 θ / °'),('alpha','摆杆 α / °'),('beta','直立偏差 β / °'),('iq','实测 Iq / A')]:
        box,content=panel();content.addWidget(label(name,'section'));w.metric_labels[key]=label('—','metric');content.addWidget(w.metric_labels[key]);metrics.addWidget(box)
    right.addLayout(metrics);box,resources=panel();resources.setContentsMargins(14,8,14,8);resources.addWidget(w.resources);right.addWidget(box)
    display=QHBoxLayout();w.source_label=label('等待数据','muted',True);w.source_label.setToolTip('α下垂为0°；β直立为0°');display.addWidget(w.source_label,1)
    w.scale_combo=ThemedComboBox();w.scale_combo.addItems(['完整起摆范围','稳摆放大','下垂抑摆放大','自动偏差 · 按实测姿态'])
    w.scale_combo.currentIndexChanged.connect(w.change_scale);display.addWidget(w.scale_combo);right.addLayout(display)
    tabs=QTabWidget();dashboard=QWidget();dash=QHBoxLayout(dashboard);dash.setContentsMargins(10,10,10,10);dash.setSpacing(12)
    w.angle_chart=TraceChart('两轴角度 / °',[('q','旋臂 θ','accent'),('alpha','摆杆 α','teal')],bounds=(-200,200),periodic=True)
    w.beta_chart=TraceChart('直立偏差 / °',[('beta','β','teal')],periodic=True)
    w.current_chart=TraceChart('q轴电流 / A',[('iq','实测','accent'),('iq_ref','指令','amber')],bounds=(-1.8,1.8))
    w.speed_chart=TraceChart('旋臂速度 / rpm',[('speed_rpm','反馈','accent'),('speed_ref_rpm','平滑参考','amber')],fast=True)
    w.charts=[w.angle_chart,w.beta_chart,w.current_chart,w.speed_chart]
    w.angle_chart.marker_labels=True;w.current_chart.marker_times=True;w.speed_chart.marker_labels=True;w.speed_chart.marker_times=True
    plots=QVBoxLayout();plots.setSpacing(8)
    for chart in w.charts[:3]:plots.addWidget(chart,1)
    dash.addLayout(plots,3);details_widget=QWidget();details_widget.setMinimumWidth(210);details_widget.setMaximumWidth(290)
    details=QVBoxLayout(details_widget);details.setContentsMargins(0,0,0,0);details.setSpacing(8)
    w.pose.setMinimumWidth(0);w.pose.setMaximumWidth(16777215);details.addWidget(w.pose,2)
    log_box,logs=panel();logs.setContentsMargins(10,10,10,10);logs.addWidget(label('运行日志','section'))
    w.log.setMinimumWidth(0);w.log.setMaximumHeight(16777215);logs.addWidget(w.log,1);details.addWidget(log_box,3);dash.addWidget(details_widget,1)
    tabs.addTab(dashboard,'信号与姿态')
    speed_page=QWidget();speed_page_layout=QVBoxLayout(speed_page);speed_page_layout.addWidget(w.speed_chart,1)
    finite=QHBoxLayout();finite.addWidget(label('有限速度轨迹','muted'))
    w.finite_speed_choice=ThemedComboBox()
    for sign in (-1,1):
        for rpm in (1,5,10,15,20,30,40,50,60,80,100,120,150,180,200):w.finite_speed_choice.addItem(f'{sign*rpm:+d} rpm',sign*rpm)
    finite.addWidget(w.finite_speed_choice)
    w.finite_speed_btn=QPushButton('执行有限轨迹');w.finite_speed_btn.clicked.connect(lambda:w.safe(lambda:w.action('speed',w.finite_speed_choice.currentData())))
    finite.addWidget(w.finite_speed_btn);speed_page_layout.addLayout(finite);w.servo_page_index=tabs.addTab(speed_page,'速度伺服')
    w.parameters=QPlainTextEdit();w.parameters.setReadOnly(True);w.parameters.setPlainText('尚未连接；连接后可回读设备版本、能力和标定参数。')
    tabs.addTab(w.parameters,'运行参数与协议')
    diagnostics=QWidget();diag=QVBoxLayout(diagnostics);diag.addWidget(w.metrics);diag.addWidget(label('FPGA专用诊断操作；不属于日常使能流程','muted',True))
    for widget in (w.mode,w.current,w.arm_btn,w.home_goal,w.home_btn,w.fast_angle,w.fast_btn,w.position_goal,w.position_btn,w.servo_rpm,w.servo_seconds,w.servo_btn,w.demo_btn,w.hold_seconds,w.cycle_direction,w.long_btn):diag.addWidget(widget)
    diagnostic_scroll=QScrollArea();diagnostic_scroll.setWidgetResizable(True);diagnostic_scroll.setWidget(diagnostics);tabs.addTab(diagnostic_scroll,'设备诊断')
    right.addWidget(tabs,1);w.plot_tabs=tabs;body.addLayout(right,1);layout.addLayout(body,1)
    w.footer=label('未连接','muted',True);layout.addWidget(w.footer)
    w.scale_combo.setCurrentIndex(3);w.change_theme()
