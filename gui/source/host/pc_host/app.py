"""Native ME6025 console, inspired by the legacy V34 console."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import queue
import sys
import time

from PySide6.QtCore import QSettings, QTimer, Qt, QUrl
from PySide6.QtGui import QDesktopServices, QFont, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit, QProgressBar, QPushButton,
    QScrollArea, QSlider, QTabWidget, QVBoxLayout, QWidget)
from serial.tools import list_ports

from .controller import HardwareController
from .protocol import (ROOT, DEFAULT_OUTPUT, DEFAULT_REPLAY, LIVE_IDENTITY, LIVE_IDENTITIES, RESET_IDENTITY, eligible, profile, read_capture, capture_origin, write_display, BUDGET_FIELDS)
from .theme import stylesheet
from .widgets import TraceChart, PendulumView, ThemedComboBox as QComboBox
from .presentation import activity_entry, port_name, ordered_ports, select_port
from .angle_display import PoseSelector
from .controls import build_controls
from .velocity import SPEED_IDENTITY, speed_duration
from .servo import SERVO_IDENTITY
from .motion import resource_text
from .resource_bars import ResourceBars


def label(text, name=None, wrap=False):
    widget = QLabel(text)
    if name:
        widget.setObjectName(name)
    widget.setWordWrap(wrap)
    return widget


def panel():
    widget = QFrame()
    widget.setObjectName('panel')
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(14, 12, 14, 12)
    layout.setSpacing(8)
    return widget, layout


class MainWindow(QMainWindow):
    def __init__(self, demo=False, theme=None, persist=True):
        super().__init__()
        self.settings = QSettings('FurutaBLDC', 'ME6025Host')
        self.persist = persist
        self.events = queue.Queue()
        self._defer_telemetry_controls = False
        self.controller = None
        self.connected = self.busy = self.ready = self.closing = False
        self.is_live = False
        self.can_soft_reset = False
        self.enabled = False
        self.motion_active = False
        self.servo_active=self.servo_ending=False
        self.demo_active=False
        self.budget=None
        self.budget_at=0
        self.device_state = {}
        self.connection_failure = None
        self.last_sample_at = 0
        self.last_sample = None
        self.last_live_mode = None
        self.live_time_origin = None
        self.pose_selector = PoseSelector()
        self.samples = deque(maxlen=30000)
        self.command_markers = deque(maxlen=5000)
        self.source = 'NONE'
        self.replay = []
        self.replay_index = 0
        self.replay_running = False
        self.replay_time = 0
        self.phase_at = time.monotonic()
        self.phase_seconds = 0
        self.sequence_text = ''
        self.last_tick = self.last_render = time.monotonic()
        self.output = DEFAULT_OUTPUT
        self.setWindowTitle('Furuta BLDC 1.11.1-candidate · ME6025 倒立摆上位机')
        self.resize(1450, 980)
        self.setMinimumSize(1080, 760)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(22, 18, 22, 14)
        layout.setSpacing(14)
        top = QHBoxLayout()
        titles = QVBoxLayout()
        titles.addWidget(label('FURUTA  /  BLDC', 'title'))
        titles.addWidget(label('ME6025  ·  旋转倒立摆控制台', 'muted'))
        top.addLayout(titles)
        top.addStretch()
        self.badge = label('未连接', 'badge')
        top.addWidget(self.badge)
        self.theme_combo = QComboBox()
        self.theme_combo.addItem('深色', 'dark')
        self.theme_combo.addItem('浅色', 'light')
        top.addWidget(self.theme_combo)
        self.export_btn = QPushButton('导出显示数据')
        self.export_btn.clicked.connect(self.export_csv)
        top.addWidget(self.export_btn)
        self.folder_btn = QPushButton('记录目录')
        self.folder_btn.clicked.connect(self.open_output)
        top.addWidget(self.folder_btn)
        layout.addLayout(top)
        body = QHBoxLayout()
        body.setSpacing(16)
        left_column = QVBoxLayout()
        left_container = QWidget()
        left = QVBoxLayout(left_container)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(12)
        box, conn = panel()
        conn.addWidget(label('01  /  设备连接', 'section'))
        self.mode = QComboBox()
        self.mode.addItems(['实机 · USB 串口', '离线 · 历史实测回放'])
        self.mode.currentIndexChanged.connect(self.mode_changed)
        conn.addWidget(self.mode)
        row = QHBoxLayout()
        self.port = QComboBox()
        self.port.setEditable(True)
        self.port.currentIndexChanged.connect(lambda: self.port.lineEdit().setCursorPosition(0))
        self.port.setMinimumWidth(125)
        row.addWidget(self.port, 1)
        self.refresh_btn = QPushButton('刷新')
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row.addWidget(self.refresh_btn)
        conn.addLayout(row)
        self.connect_btn = QPushButton('连接设备')
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn.addWidget(self.connect_btn)
        conn.addWidget(label('460800 baud · 8N1 · 不自动运行', 'muted'))
        self.identity_label = label('等待设备身份', 'muted', True)
        conn.addWidget(self.identity_label)
        left.addWidget(box)
        build_controls(self,left,label,panel)
        box, replay = panel()
        replay.addWidget(label('历史回放', 'section'))
        self.file_btn = QPushButton('打开采样 CSV')
        self.file_btn.clicked.connect(self.open_capture)
        replay.addWidget(self.file_btn)
        self.replay_label = label('可回放新板原始采样', 'muted', True)
        replay.addWidget(self.replay_label)
        buttons = QHBoxLayout()
        self.play_btn = QPushButton('播放')
        self.play_btn.clicked.connect(self.toggle_play)
        buttons.addWidget(self.play_btn, 1)
        self.speed = QComboBox()
        for v in (.25, .5, 1, 2, 4):
            self.speed.addItem(f'{v:g}×', v)
        self.speed.setCurrentIndex(2)
        buttons.addWidget(self.speed)
        replay.addLayout(buttons)
        self.seek = QSlider(Qt.Orientation.Horizontal)
        self.seek.setRange(0, 1000)
        self.seek.sliderMoved.connect(self.seek_replay)
        replay.addWidget(self.seek)
        self.replay_panel=box
        left.addWidget(box)
        left.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(left_container)
        scroll.setMinimumWidth(312)
        scroll.setMaximumWidth(348)
        left_column.addWidget(scroll, 1)
        self.stop_btn = QPushButton('■  停止 / 关桥                 Esc')
        self.stop_btn.setObjectName('stop')
        self.stop_btn.setMinimumHeight(48)
        self.stop_btn.clicked.connect(self.stop)
        left_column.addWidget(self.stop_btn)
        body.addLayout(left_column)
        right = QVBoxLayout()
        metrics = QHBoxLayout()
        self.metrics = {}
        for key, name in [('theta', '旋臂 θ / °'), ('alpha', '摆杆 α / °'),
                          ('beta', '直立偏差 β / °'), ('iq', '实测 Iq / A')]:
            box, content = panel()
            content.addWidget(label(name, 'section'))
            self.metrics[key] = label('—', 'metric')
            content.addWidget(self.metrics[key])
            metrics.addWidget(box)
        right.addLayout(metrics)
        box,resources=panel()
        resources.setContentsMargins(14,8,14,8)
        self.resource_bars=ResourceBars();resources.addWidget(self.resource_bars);right.addWidget(box)
        self.budget_label=self.resource_bars.heat_label;self.budget_reason=self.resource_bars.reason
        self.source_label = label('等待数据 · α 下垂为 0°，β 直立为 0°', 'muted', True)
        display_row = QHBoxLayout()
        display_row.addWidget(self.source_label, 1)
        self.scale_combo = QComboBox()
        self.scale_combo.addItems(['完整起摆范围', '稳摆放大', '下垂抑摆放大', '自动偏差 · 按实测姿态'])
        self.scale_combo.currentIndexChanged.connect(self.change_scale)
        display_row.addWidget(self.scale_combo)
        right.addLayout(display_row)
        self.tabs = QTabWidget()
        dashboard = QWidget()
        dash = QHBoxLayout(dashboard)
        dash.setContentsMargins(10, 10, 10, 10)
        dash.setSpacing(12)
        self.angle_chart = TraceChart('两轴角度 / °', [('theta', '旋臂 θ', 'accent'), ('alpha', '摆杆 α', 'teal')],bounds=(-200,200),periodic=True)
        self.angle_chart.setToolTip('±180° 处按最短角度变化连续拼接；边缘显示相差 360° 的等价曲线。原始记录和导出不变。')
        self.beta_chart = TraceChart('直立偏差 / °', [('beta', 'β', 'teal')],periodic=True)
        self.current_chart = TraceChart('q 轴电流 / A', [('iq', '实测', 'accent'), ('iq_ref', '指令', 'amber')], bounds=(-1.8, 1.8))
        self.charts = [self.angle_chart, self.beta_chart, self.current_chart]
        for chart in self.charts:
            chart.markers = self.command_markers
        self.angle_chart.marker_labels = True
        self.current_chart.marker_times = True
        self.angle_chart.setToolTip('虚线为已发送指令，悬停查看名称和时间；标记不表示设备已接受或动作已完成。')
        self.scale_combo.setCurrentIndex(3)
        plots = QVBoxLayout();plots.setSpacing(8)
        for chart in self.charts:
            plots.addWidget(chart,1)
        dash.addLayout(plots, 3)
        self.detail_column = QWidget()
        self.detail_column.setMinimumWidth(210)
        self.detail_column.setMaximumWidth(290)
        details = QVBoxLayout(self.detail_column)
        details.setContentsMargins(0,0,0,0);details.setSpacing(8)
        self.pose = PendulumView()
        details.addWidget(self.pose, 2)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setPlaceholderText('连接、阶段确认和停止信息将在这里显示。')
        log_box, log_layout = panel()
        log_layout.setContentsMargins(10,10,10,10)
        log_layout.addWidget(label('运行日志','section'))
        self.log.setMinimumWidth(0)
        log_layout.addWidget(self.log,1)
        details.addWidget(log_box, 3)
        dash.addWidget(self.detail_column, 1)
        self.tabs.addTab(dashboard, '信号与姿态')
        self.speed_chart=TraceChart('旋臂速度 / rpm', [('speed_rpm','反馈','accent'),('speed_ref_rpm','平滑参考','amber')],fast=True)
        self.speed_chart.markers=self.command_markers;self.speed_chart.marker_labels=True;self.speed_chart.marker_times=True
        self.tabs.addTab(self.speed_chart,'速度伺服')
        self.parameters = QPlainTextEdit()
        self.parameters.setReadOnly(True)
        self.parameters.setPlainText(self.parameter_text())
        self.tabs.addTab(self.parameters, '运行参数与协议')
        self.diagnostics = QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.diagnostics.setPlaceholderText('实机状态和会话结果')
        self.tabs.addTab(self.diagnostics, '设备诊断')
        right.addWidget(self.tabs, 1)
        body.addLayout(right, 1)
        layout.addLayout(body, 1)
        self.footer = label('就绪 · 启动后不连接串口、不使能电机', 'muted', True)
        layout.addWidget(self.footer)
        self.shortcut = QShortcut(QKeySequence('Esc'), self)
        self.shortcut.activated.connect(self.stop)
        self.theme_combo.currentIndexChanged.connect(self.change_theme)
        selected_theme = theme or str(self.settings.value('theme', 'dark'))
        self.theme_combo.setCurrentIndex(1 if selected_theme == 'light' else 0)
        self.change_theme()
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.setInterval(25)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        self.update_controls()
        if demo:
            self.mode.setCurrentIndex(1)
            self.load_replay(DEFAULT_REPLAY)
            self.toggle_play()

    def parameter_text(self):
        return ('ME6025 · 24 V · PC_LIVE v22/v23/v24/v27 / 上位机 1.11.1-candidate\n\n'
                '演示：下垂 90° / 180° 往返、一周 → 回零起摆 → 同组稳摆快移 → 受控下落 → 使能待命。\n'
                '起摆稳摆：持续保持，直到受控下落或停止。\n'
                '速度伺服（v27）：进入时目标归零；滑条 −200 至 +200 rpm 在线改速，保持目标直到修改。\n'
                '目标归零平滑减速并稳摆；退出伺服先减速再交回位置保持。停止仍立即关桥。\n'
                '统一移位：目标是相对固定物理零点的绝对角；下垂 / 直立自动选择已验轨迹。\n'
                '轮盘点击即快速定位；±360° 连续旋转一周后停止。\n'
                '位置采用最短路径；整圈按钮保留明确方向。旧平滑移位已封存。\n\n'
                '额度：保持限流 1 A，起摆 / 下落 / 稳摆快移参考段瞬态最高 1.6 A。\n'
                '滚动 I²t 保护 1 A²s；快移入口 ≤0.650、下落入口 ≤0.500 A²s。\n'
                '这是三相电流平方和的最近 15–16 s 保守窗口，不代表温度或连续热额定。\n'
                '额度及姿态不满足时有限等待；指令被拒绝不自动重发。\n\n'
                'HOLD4 稳摆反馈和 30 Hz 观测不变；下垂为 10 Hz。\n'
                '电流环 2 kHz，500 μs 周期；停机、测角、硬电流与心跳保护保持。\n'
                '停止 / Esc 立即关桥并取消后续演示步骤；关窗 / 断开也关桥。\n'
                '正常演示完成保持使能，窗口继续维持心跳；失联 1 s 关桥。\n\n'
                '固定物理零点保存在板端。日常运行不得重新登记零点绕过入口检查。\n'
                'α 下垂为 0°，β=wrap(α−180°)，θ 相对固定物理零点。\n'
                '实时遥测 50 Hz，额度 10 Hz；原始串口与在线数据保存到记录目录。\n'
                '真实外力加载、任意轨迹跟踪与连续热额定未验收。')

    def change_theme(self):
        name = self.theme_combo.currentData()
        self.setStyleSheet(stylesheet(name))
        for combo in self.findChildren(QComboBox):combo.set_theme(name)
        for widget in [*self.charts,self.speed_chart, self.pose,self.position_dial]:
            widget.theme = name
            widget.update()
        if self.persist:
            self.settings.setValue('theme', name)

    def change_scale(self):
        selected = self.scale_combo.currentIndex()
        zoom = selected != 0
        automatic=selected==3
        down=selected==2 or (automatic and self.pose_selector.pose!='upright')
        self.beta_chart.title = ('下垂偏差 α / °' if down else '直立偏差 β / °')+(' · 自动' if automatic else '')
        if automatic and self.pose_selector.pose is None:self.beta_chart.title='自动偏差 · 等待有效姿态'
        self.beta_chart.series = [('alpha', 'α', 'teal')] if down else [('beta', 'β', 'teal')]
        self.beta_chart.bounds = (-5, 5) if zoom else (-200,200)
        self.current_chart.bounds = (-1.1, 1.1) if zoom else (-1.8, 1.8)
        self.beta_chart.update()
        self.current_chart.update()

    def refresh_ports(self):
        previous = port_name(self.port.currentText())
        items = ordered_ports(list_ports.comports())
        remembered = str(self.settings.value('last_successful_port', '')) if self.persist else ''
        selected = select_port(items, previous, remembered)
        self.port.clear()
        for item in items:
            description=item.description.replace('USB-SERIAL ','').replace(f' ({item.device})','')
            self.port.addItem(f'{item.device} · {description}', item.device)
            self.port.setItemData(self.port.count()-1, item.description+'\n'+item.hwid, Qt.ItemDataRole.ToolTipRole)
        if selected:
            self.port.setCurrentIndex(next(i for i,p in enumerate(items) if p.device==selected))
            self.port.lineEdit().setCursorPosition(0)
        else:
            self.port.setEditText('')
            self.port.lineEdit().setPlaceholderText('未发现串口，请插入 USB 后刷新')

    def mode_changed(self):
        if hasattr(self, 'play_btn'):
            self.update_controls()

    def update_controls(self):
        attached = self.controller is not None
        offline = self.mode.currentIndex() == 1
        self.mode.setEnabled(not attached)
        self.port.setEnabled(not attached and not offline)
        self.refresh_btn.setEnabled(not attached and not offline)
        self.connect_btn.setText('断开设备' if attached else '加载默认历史记录' if offline else '连接设备')
        self.connect_btn.setEnabled(not self.closing)
        operable = self.connected and (self.ready or self.enabled) and not self.device_state.get('fault') and not self.busy and not self.closing and not self.motion_active
        idle_ready = operable and not self.enabled
        self.enable_btn.setText('空闲 · 关桥' if self.enabled else '使能保持')
        self.enable_btn.setEnabled(self.connected and not self.busy and not self.closing and (self.enabled or self.ready))
        self.recover_btn.setEnabled(self.connected and not self.busy and not self.enabled and bool(self.device_state.get('fault')))
        self.recover_btn.setVisible(bool(self.device_state.get('fault')))
        self.reset_btn.setEnabled(self.connected and self.can_soft_reset and not self.busy and not self.motion_active and not self.closing)
        centered = self.device_state.get('center_valid') == 1
        qualified = all(self.device_state.get(k) == 1 for k in ('mapped', 'aligned'))
        self.run_btn.setEnabled(operable and self.is_live and centered and self.last_live_mode!='V')
        self.passive_btn.setEnabled(idle_ready)
        self.center_btn.setEnabled(idle_ready and self.is_live)
        self.position_dial.setEnabled(operable and self.is_live and centered)
        speed_ready=self.connected and self.enabled and not self.closing and not self.busy and not self.device_state.get('fault') and self.last_live_mode=='V' and getattr(self.controller,'identity',None)==SERVO_IDENTITY
        self.speed_btn.setEnabled(speed_ready and not self.servo_ending and (self.servo_active or not self.motion_active))
        self.speed_btn.setText('减速退出伺服' if self.servo_active else '进入速度伺服')
        self.speed_slider.setEnabled(speed_ready and self.servo_active and not self.servo_ending)
        self.speed_zero.setEnabled(self.speed_slider.isEnabled())
        self.continuous_btn.setEnabled(operable and self.is_live and centered and self.last_live_mode!='V')
        self.drop_btn.setEnabled(operable and self.is_live and self.enabled and self.last_live_mode=='V')
        self.drop_direction.setEnabled(not self.motion_active and not self.busy)
        self.replay_panel.setVisible(offline)
        self.file_btn.setEnabled(not attached)
        self.play_btn.setEnabled(bool(self.replay) and not attached)
        self.seek.setEnabled(bool(self.replay) and not attached)
        self.speed.setEnabled(not attached)
        self.export_btn.setEnabled(bool(self.samples))
        self.progress.setVisible(self.busy)
        self.elapsed.setVisible(self.busy)

    def toggle_connection(self):
        if self.controller:
            self.controller.disconnect()
            self.busy = True
            self.phase.setText('正在停止、保存并断开…')
            self.update_controls()
            return
        if self.mode.currentIndex() == 1:
            self.load_replay(DEFAULT_REPLAY)
            return
        port = port_name(self.port.currentText())
        if not port:
            self.footer.setText('请选择串口')
            return
        self.replay_running = False
        self.play_btn.setText('播放')
        self.clear_data()
        self.source = 'NONE'
        self.source_label.setText('等待设备数据')
        self.connection_failure = None
        self.live_time_origin = None
        self.budget=None;self.budget_at=0
        self.device_state = {}
        self.is_live = False
        self.identity_label.setText('等待设备身份核对')
        self.center_label.setText('物理零点：等待设备读回')
        self.state_label.setText('尚未连接设备')
        self.diagnostics.clear()
        self.connected = self.ready = False
        self.busy = True
        self.controller = HardwareController(port, self.events)
        self.controller.start()
        self.badge.setText('连接中')
        self.update_controls()

    def action(self, name):
        if not self.controller or self.busy or (name not in ('recover','soft_reset') and not (self.ready or self.enabled)):
            return
        if name=='soft_reset' and not self.reset_btn.isEnabled():return
        self.busy = True
        self.phase.setText('准备并核对参数…')
        self.controller.request(name)
        self.update_controls()

    def request_position(self,degrees):
        if not self.position_dial.isEnabled() or not self.controller:return
        self.busy=True;self.phase.setText(f'快速定位至 {degrees:+.0f}°')
        self.controller.request('position',degrees);self.update_controls()

    def request_speed(self):
        if not self.speed_btn.isEnabled() or not self.controller:return
        self.busy=True
        self.controller.request('servo_end' if self.servo_active else 'servo_start')
        self.tabs.setCurrentWidget(self.speed_chart);self.update_controls()

    def servo_target_changed(self,rpm):
        self.servo_readout.setText(f'目标 {rpm:+d} rpm · 范围 −200 至 +200')
        if self.controller and self.servo_active and not self.servo_ending:
            try:self.controller.request('servo_target',rpm)
            except (ValueError,RuntimeError) as exc:self.phase.setText(str(exc))

    def request_turn(self,direction):
        if not self.position_dial.isEnabled() or not self.controller:return
        self.busy=True;self.phase.setText(f'快速旋转 {direction*360:+d}°')
        self.controller.request('turn',direction);self.update_controls()

    def stop(self):
        if self.controller:
            self.controller.stop()
            self.phase.setText('已请求停止 · 等待设备确认')
        else:
            self.replay_running = False
            self.play_btn.setText('播放')
            self.phase.setText('回放已暂停')

    def clear_data(self):
        self.pose_selector.reset()
        if self.scale_combo.currentIndex()==3:self.change_scale()
        self.samples.clear()
        self.last_sample = None
        self.last_sample_at = 0
        for chart in [*self.charts,self.speed_chart]:
            chart.samples.clear()
            chart.update()
        for metric in self.metrics.values():
            metric.setText('—')
        self.pose.sample = None
        self.pose.update()

    def append_sample(self, row):
        self.command_markers.extend(row.get('commands', []))
        previous=self.pose_selector.pose
        self.pose_selector.update(row)
        if self.scale_combo.currentIndex()==3 and previous!=self.pose_selector.pose:self.change_scale()
        self.samples.append(row)
        self.last_sample = row
        self.last_sample_at = row.get('received_at', time.monotonic()) - row.get('transport_lag_s', 0.)
        for chart in [*self.charts,self.speed_chart]:
            plotted=dict(row,speed_rpm=row['speed']*30/3.141592653589793,speed_ref_rpm=row.get('speed_ref',float('nan'))*30/3.141592653589793) if chart is self.speed_chart else row
            chart.append(plotted, redraw=False)

    def load_replay(self, path):
        try:
            replay = read_capture(path)
            raw = replay[0].get('capture_format') != 'display'
            origin = capture_origin(path) if raw else {}
            if origin.get('center_valid'):
                import math
                for row in replay:
                    row['theta'] += math.degrees(origin['offset_rad'])
            # Older H captures did not run the pendulum observer; retain provenance.
            stem = Path(path).stem
            if raw and (stem.endswith(('_P', '_D', '_A', '_Q')) or (stem.endswith('_H') and not origin.get('pendulum_observed'))):
                for row in replay:
                    row['alpha'] = row['beta'] = float('nan')
                    row['valid'] = False
        except Exception as exc:
            self.footer.setText('无法打开记录：' + str(exc))
            return False
        self.replay = replay
        self.command_markers.clear()
        self.clear_data()
        self.replay_index = 0
        self.replay_time = self.replay[0]['t']
        self.replay_running = False
        self.source = 'REPLAY'
        self.replay_label.setText(Path(path).name)
        self.replay_label.setToolTip(str(path))
        self.source_label.setText('历史实测回放 · ' + Path(path).name)
        if all(row.get('capture_source')=='SIMULATION' for row in replay):
            self.source_label.setText('离线仿真回放 · 非实测 · '+Path(path).name)
        if origin.get('center_valid'):
            self.source_label.setText('历史采样 · 固定物理零点坐标 · ' + Path(path).name)
        self.badge.setText('离线回放')
        self.footer.setText('历史数据 · 不连接串口 · 可拖动进度回看')
        self.phase.setText('历史记录已载入')
        self.play_btn.setText('播放')
        self.seek.setValue(0)
        self.seek_replay(0)
        self.update_controls()
        return True

    def toggle_play(self):
        if not self.replay or self.controller:
            return
        if self.replay_index >= len(self.replay):
            self.seek_replay(0)
        self.replay_running = not self.replay_running
        self.play_btn.setText('暂停' if self.replay_running else '播放')

    def seek_replay(self, position):
        if not self.replay or self.controller:
            return
        start = self.replay[0]['t']
        self.replay_time = start + (self.replay[-1]['t'] - start) * position / 1000
        self.seek.setValue(position)
        self.clear_data()
        self.command_markers.clear()
        self.replay_index = 0
        while self.replay_index < len(self.replay) and self.replay[self.replay_index]['t'] <= self.replay_time:
            row = self.replay[self.replay_index]
            if row['t'] >= self.replay_time - 20:
                self.append_sample(row)
            self.replay_index += 1

    def open_capture(self):
        path, _ = QFileDialog.getOpenFileName(self, '打开 ME6025 采样', str(ROOT / 'data/raw'), 'CSV (*.csv)')
        if path:
            self.mode.setCurrentIndex(1)
            self.load_replay(path)

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出显示数据', 'me6025_display.csv', 'CSV (*.csv)')
        if not path:
            return
        try:
            write_display(path, self.samples, self.source, self.command_markers)
            self.footer.setText('已导出：' + path)
        except OSError as exc:
            self.footer.setText('导出失败：' + str(exc))

    def open_output(self):
        self.output.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.output)))

    def append_activity(self, text):
        try:
            entry = activity_entry(text)
        except (ValueError, KeyError, IndexError, TypeError):
            entry = '设备消息格式异常，详见记录目录中的 serial.log'
        if entry:
            self.log.appendPlainText(time.strftime('%H:%M:%S') + '  ' + entry)

    def handle(self, event):
        kind = event['kind']
        if kind == 'servo':
            entering=event['active'] and not self.servo_active
            self.servo_active=event['active'];self.servo_ending=event['ending']
            self.motion_active=self.servo_active
            if entering or not self.servo_active or self.servo_ending:
                self.speed_slider.blockSignals(True);self.speed_slider.setValue(0);self.speed_slider.blockSignals(False)
                self.servo_readout.setText('目标 0 rpm · 范围 −200 至 +200')
            if self.servo_active:
                self.phase.setText('速度伺服 · 减速退出中' if self.servo_ending else f'速度伺服 · 已确认 {event["rpm"]:+d} rpm，可拖动滑条改速')
        elif kind == 'command_sent':
            if self.connected and self.live_time_origin is not None:
                stamp = event['t'] - self.live_time_origin
                if stamp >= 0:
                    self.command_markers.append(dict(t=stamp, label=event['label'], command=event['command']))
        elif kind == 'budget':
            self.budget=event['budget'];self.budget_at=event.get('received_at', time.monotonic())
            title,reason=resource_text(self.budget,self.last_live_mode)
            self.resource_bars.refresh(self.budget,self.last_live_mode,(self.last_sample or {}).get("iq"),True)
        elif kind == 'demo':
            self.demo_active=event['active'];self.phase.setText(event['text'])
            self.footer.setText(event['text'])
            self.append_activity(event['text'])
        elif kind == 'session':
            self.enabled=bool(event['enabled'])
            if not self.enabled:self.motion_active=False
            mode=event.get('mode','I');reason=event.get('reason','')
            if 'fault' in event:self.device_state['fault']=event['fault']
            if mode=='V':
                text='使能 · 起摆 / 稳摆中，可直接切换指令'
            elif self.enabled:
                text='使能 · 正在回零并抑摆，满足条件后起摆' if reason=='prepare_swing' else '使能 · 位置保持与主动抑摆，等待指令'
            else:
                text='空闲 · 电机已关闭，可自由移动旋臂'
            if reason=='waiting':text='使能等待 · 电机关闭，运动进入可接管范围后保持；可更新目标或停止'
            if event.get('fault'):text='已保护停机 · '+reason+' · 可核验恢复，遥测继续'
            self.phase.setText(text)
            self.badge.setText('使能等待' if reason=='waiting' else '已使能' if self.enabled else '空闲')
            self.footer.setText(text)
            self.append_activity(text)
        elif kind == 'motion':
            self.motion_active=event['active']
            step=event.get('operation') in ('step','upright','position')
            if event['active']:
                self.phase.setText('快移 · 正在定位和确认到位，可随时停止' if step else '受控下落 · 正在制动和确认静稳，可随时停止')
            elif event.get('completed'):
                self.phase.setText('快移完成 · 在目标位置保持，可移位或回零' if step else '下落回零完成 · 在物理零点保持，可移位或再次起摆')
            self.footer.setText(self.phase.text())
        elif kind == 'notice':
            self.phase.setText(event['text']);self.append_activity(event['text'])
        elif kind == 'fault_state':
            if self.device_state.get('fault'):
                self.phase.setText('故障：'+event['reason']+(' · 正在核验传感器恢复' if event['auto_recoverable'] else ' · 可请求核验恢复'))
        elif kind == 'connected':
            # The first telemetry frame after a successful connection is t=0.
            # This is display-only; the serial recorder retains device time.
            self.live_time_origin = None
            self.clear_data()
            self.command_markers.clear()
            self.connected, self.busy = True, False
            self.ready = event['ready']
            self.is_live = event['identity'] in LIVE_IDENTITIES
            self.can_soft_reset = event['identity'] in (RESET_IDENTITY,SPEED_IDENTITY,SERVO_IDENTITY)
            self.identity_label.setText(event['identity'].replace('IDENT,', '').replace(',', ' v'))
            self.badge.setText('实机已连接')
            if self.persist and self.controller:
                self.settings.setValue('last_successful_port', self.controller.port)
            self.append_activity('连接 · 固件身份已确认，双桥关闭')
            self.phase.setText('空闲 · 双桥关闭，可自由移动旋臂，实时遥测已开启' if self.is_live and all(self.device_state.get(k)==1 for k in ('mapped','aligned')) else '待机 · 需要自动校验，等待指令')
            self.footer.setText('已连接 · 点击运行才开始动作')
            self.source_label.setText('50 Hz 在线遥测 · 初始化完成后自动更新' if self.is_live else '当前固件为停机回传模式 · 运行时显示进度')
        elif kind == 'reset_complete':
            self.clear_data()
            self.command_markers.clear()
            self.live_time_origin = None
            self.budget=None;self.budget_at=0
            self.ready=eligible(self.device_state)
            self.phase.setText('软复位完成 · 保持空闲，下一次使能将重新初始化')
            self.footer.setText('复位前记录已保存 · 物理零点保留 · 未自动运动')
            self.append_activity('软复位完成 · 控制板已重启并核实关桥')
        elif kind == 'status':
            state = event['state']
            self.device_state = state
            self.ready = eligible(state)
            self.center_label.setText(f'物理零点：已登记 · {state["center_raw"]} counts'
                if state.get('center_valid') else '物理零点：未登记 / 固件不支持')
            self.state_label.setText(f'桥 M0={state.get("en0", "?")}  M1={state.get("en1", "?")}    故障={state.get("fault", "?")}')
            self.diagnostics.setPlainText(json.dumps(state, ensure_ascii=False, indent=2))
        elif kind == 'busy':
            self.busy = event['value']
            if not self.busy:
                self.phase_seconds = 0
                self.progress.setRange(0, 1000)
                self.progress.setValue(0)
                self.elapsed.setText('等待下一条指令')
        elif kind == 'sequence':
            self.sequence_text = f'{event["index"]} / {event["total"]}'
        elif kind == 'phase':
            self.phase.setText(event['text'])
            self.phase_at, self.phase_seconds = time.monotonic(), event['seconds']
            self.progress.setRange(0, 0 if self.phase_seconds < 0 else 1000)
            self.progress.setValue(0)
            if 'command' in event:
                self.clear_data()
                self.source = 'LIVE' if self.is_live else 'NONE'
                self.source_label.setText('实时测量 · 50 Hz' if self.is_live else '运行中 · 当前固件无实时遥测，等待停机回传')
        elif kind == 'sample':
            original = event['sample']
            if self.live_time_origin is None:
                self.live_time_origin = original['t']
            row = dict(original, t=original['t'] - self.live_time_origin)
            if self.budget is not None:
                age = max(0, time.monotonic() - self.budget_at)
                row.update(budget={k:self.budget[k] for k in BUDGET_FIELDS},
                           budget_age_s=age, budget_valid=age < .4)
            mode = row.get('mode')
            if self.source != 'LIVE' or (self.last_sample and row['t'] < self.last_sample['t']):
                self.clear_data()
                self.last_live_mode = mode
            self.source = 'LIVE'
            self.append_sample(row)
            self.last_live_mode=mode
            if self.enabled and mode=='H' and not row['en0']:
                self.badge.setText('使能等待')
                self.phase.setText('使能等待 · 电机关闭，进入可接管范围后保持；可更新目标或停止')
            elif self.enabled:
                self.badge.setText('已使能')
            self.device_state.update(fault=int(row['fault']))
            self.state_label.setText(f'桥 M0={int(row["en0"])}  M1={int(row["en1"])}    故障={int(row["fault"])} · 实时')
        elif kind == 'capture':
            try:
                rows = read_capture(event['path'])
                origin = event.get('origin', {})
                if origin.get('center_valid'):
                    import math
                    for row in rows:
                        row['theta'] += math.degrees(origin['offset_rad'])
                if event['command'][0] not in ('V', 'H'):
                    for row in rows:
                        row['alpha'] = row['beta'] = float('nan')
                        row['valid'] = False
                self.clear_data()
                self.command_markers.clear()
                for row in rows:
                    self.append_sample(row)
                self.source = 'POST_STOP_CAPTURE'
                self.source_label.setText(('停机前约 15 秒采样 · ' if event['command']=='V,1.6,0'
                                           else '停机后完整采样 · ') + Path(event['path']).name)
            except Exception as exc:
                self.log.appendPlainText('采样显示失败：' + str(exc))
        elif kind == 'output':
            self.output = Path(event['path'])
        elif kind == 'log':
            self.append_activity(event['text'])
        elif kind == 'error':
            self.ready = False
            self.phase_seconds = 0
            self.connection_failure = event['text']
            self.phase.setText(('连接失败：' if not self.connected else '流程终止：') + event['text'])
            self.append_activity('错误 · ' + event['text'])
            self.footer.setText('流程终止：' + event['text'])
        elif kind == 'disconnected':
            self.enabled=False
            self.motion_active=False
            self.budget=None;self.budget_at=0
            self.connected = self.busy = self.ready = False
            self.controller = None
            self.phase_seconds = 0
            self.progress.setValue(0)
            self.elapsed.setText('—')
            self.badge.setText('已断开')
            if event.get('opened') is False:
                text = '未连接设备 · 未发送运动指令'
            else:
                text = '双桥关闭已核实' if event['verified'] else '停机未核实，请检查设备状态'
            self.state_label.setText(text)
            self.footer.setText((self.connection_failure or text) + ' · 记录：' + str(self.output))
            self.append_activity(text)
            if self.closing:
                QTimer.singleShot(0, self.close)
        if kind not in ('sample', 'budget') or not self._defer_telemetry_controls:
            self.update_controls()

    def poll(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if self.controller:
            self.controller.pulse()
        # Bound UI work even if a device sends too many events.
        deadline = time.perf_counter() + .015
        self._defer_telemetry_controls = True
        try:
            for _ in range(200):
                try:
                    self.handle(self.events.get_nowait())
                except queue.Empty:
                    break
                if time.perf_counter() >= deadline:
                    break
        finally:
            self._defer_telemetry_controls = False
        self.update_controls()
        now = time.monotonic()
        if self.replay_running and not self.controller:
            self.replay_time += min(dt, .25) * self.speed.currentData()
            while self.replay_index < len(self.replay) and self.replay[self.replay_index]['t'] <= self.replay_time:
                self.append_sample(self.replay[self.replay_index])
                self.replay_index += 1
            start = self.replay[0]['t']
            duration = self.replay[-1]['t'] - start
            fraction = (self.replay_time - start) / duration if duration > 0 else 1
            self.seek.setValue(int(min(1, max(0, fraction)) * 1000))
            if self.replay_index == len(self.replay):
                self.replay_running = False
                self.play_btn.setText('重新播放')
                self.phase.setText('历史回放结束')
        if self.phase_seconds < 0:
            elapsed = int(time.monotonic() - self.phase_at)
            prefix = f'演示 {self.sequence_text} · ' if self.demo_active else ''
            self.elapsed.setText(f'{prefix}已用 {elapsed//60:02d}:{elapsed%60:02d} · 可随时停止')
        elif self.phase_seconds > 0:
            elapsed = now - self.phase_at
            self.progress.setValue(min(1000, int(elapsed / self.phase_seconds * 1000)))
            self.elapsed.setText(f'{self.sequence_text}    {min(elapsed, self.phase_seconds):.1f} / {self.phase_seconds:g} s · 主机计时')
        if now - self.last_render < .1:
            return
        self.last_render = now
        title,reason=resource_text(self.budget,self.last_live_mode,self.connected and now-self.budget_at<.4)
        fresh=self.connected and now-self.budget_at<.4
        if self.source in ('REPLAY', 'POST_STOP_CAPTURE') and not self.controller:
            row = self.last_sample or {}
            self.resource_bars.refresh(row.get('budget'),row.get('mode'),row.get('iq'),row.get('budget_valid',False),recorded=True)
            if not row.get('budget'):
                self.budget_reason.setText('此记录未包含额度数据')
                self.resource_bars.setToolTip('此记录未包含额度数据，不补造历史额度。')
            else:
                self.resource_bars.setToolTip(f"历史额度 · 数据年龄 {row['budget_age_s']:.3f} s · "+self.budget_reason.text())
        else:
            self.resource_bars.refresh(self.budget,self.last_live_mode,(self.last_sample or {}).get("iq"),fresh)
        import math
        self.position_dial.set_position((self.last_sample or {}).get("theta") if fresh else None,math.degrees(math.remainder(self.budget["goal"],math.tau)) if fresh and self.budget else None)
        stale = self.source == 'LIVE' and now - self.last_sample_at > .3
        row = self.last_sample if not stale and self.last_sample and self.last_sample.get('valid') else None
        import math
        for key, widget in self.metrics.items():
            value = row[key] if row else float('nan')
            if key=='theta' and math.isfinite(value):
                value=math.remainder(value,360)
            widget.setText(f'{value:+.3f}' if key == 'iq' and math.isfinite(value) else
                           f'{value:+.2f}' if math.isfinite(value) else '—')
        self.pose.sample = row if row and row['valid'] else None
        self.pose.update()
        for chart in [*self.charts,self.speed_chart]:
            chart.update()
        if stale:
            self.source_label.setText('遥测已过期 · 当前数值和姿态未知')
        elif self.source == 'LIVE':
            self.source_label.setText(('待机实时测量' if self.last_live_mode=='I' else '实时测量') + ' · 50 Hz · 连接起计时 · 最近 20 秒')
        self.export_btn.setEnabled(bool(self.samples))

    def closeEvent(self, event):
        if self.controller:
            event.ignore()
            self.closing = True
            self.controller.disconnect()
            self.phase.setText('正在关桥并保存记录…')
            self.update_controls()
        else:
            self.timer.stop()
            event.accept()

