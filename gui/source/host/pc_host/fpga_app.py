"""FPGA workspace in the original Qt host; shared charts, pose and theme."""
from collections import deque
from bisect import bisect_right,bisect_left
import csv
import json
from dataclasses import asdict
import math
import os
from pathlib import Path
import queue
import threading
import time
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont,QFontDatabase
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,QFormLayout,QLabel,
    QPushButton,QComboBox,QCheckBox,QTabWidget,QPlainTextEdit,QScrollArea,QFileDialog,QGroupBox,QSlider)
from .scroll_inputs import ScrollSpinBox as QSpinBox,ScrollDoubleSpinBox as QDoubleSpinBox
from .widgets import TraceChart,PendulumView,ThemedComboBox
from .theme import stylesheet
from .fpga_controller import FPGAController
from .fpga_protocol import Calibration,BoundCalibrationProfile,hold_phase,FAULTS,estimate_offsets,fit_gain
from .fpga_simulator import SimSerial
from .position_dial import PositionDial
from .fpga_replay import read_recording,legacy_plot
from .resource_bars import ResourceBars
from .angle_display import PoseSelector
from .ui_timing import UiTiming,timed_ui

ROOT=Path(__file__).resolve().parents[2]

def current_display_confirmed(row):
    """Display applicability only; never used for connection or command admission.

    Fresh transport/control stamps do not establish analog readiness while the
    board is stopped and not ready. Preserve those values as raw diagnostics.
    """
    return bool(row.get('valid') and row.get('sensor_valid')
                and row.get('telemetry_current',True)
                and (row.get('protocol_revision')!=4 or row.get('control_sample_fresh'))
                and (row.get('run') or row.get('ready') or row.get('fault')))

class FPGAWindow(QMainWindow):
    def __init__(self,record_directory=None,serial_factory=None,events=None,device_identity=None,settings_path=None,deployment_adapter=None,deployment_target=None,manual_control=False,scan_ports_on_start=False):
        super().__init__();self.setWindowTitle('Furuta BLDC · FPGA R1 工作台');self.resize(1440,960)
        for name in ('msyh.ttc','msyhbd.ttc','segoeui.ttf'):
            font=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts'/name
            if font.exists():QFontDatabase.addApplicationFont(str(font))
        QApplication.instance().setFont(QFont('Microsoft YaHei UI',10))
        self.events=events if events is not None else queue.Queue();self.controller=None;self.connected=False;self.busy=False
        self.record_directory=record_directory
        self.settings_path=Path(settings_path) if settings_path else None
        self.preferences={};self.loading_preferences=True;self.phase_at=None;self.operation_name='操作'
        if self.settings_path and self.settings_path.exists():
            try:
                data=json.loads(self.settings_path.read_text(encoding='utf-8-sig'))
                if isinstance(data,dict):self.preferences={key:data[key] for key in ('theme','last_successful_port') if isinstance(data.get(key),str)}
            except (OSError,ValueError):pass
        self.serial_factory=serial_factory;self.closing=False
        self.deployment_adapter=deployment_adapter;self.deployment_flow=None;self.deployment_thread=None;self.deploying=False
        self.deployment_target=deployment_target
        self.deployment_owner=None
        self.reset_pending=False
        self.manual_control=bool(manual_control)
        self.device_identity=dict(device_identity or {}) if serial_factory is not None else {}
        self.auto_profile=None;self.last_device_words=None;self.diagnostic_at=None;self.diagnostic_row=None
        self.candidate=None;self.capabilities={}
        self.pending_sample_display=None
        self.sequence_active=False
        self.servo_active=False;self.servo_ending=False
        self.latest=None;self.last_rx=0.;self.samples=deque(maxlen=30000);self.offset_rows=None;self.offset_started=0.
        self.replay=[];self.replay_index=0;self.replay_running=False;self.legacy=None
        self.replay_wall=0.;self.replay_time=0.;self.replay_rate=1.
        self.replay_format='fpga'
        self.command_markers=deque(maxlen=5000)
        self.setStyleSheet(stylesheet('dark'))
        root=QWidget();self.setCentralWidget(root);layout=QVBoxLayout(root)
        title=QHBoxLayout();title.addWidget(QLabel('FURUTA / FPGA R1     双编码器 · FOC · 倒立摆'))
        self.theme_combo=ThemedComboBox();self.theme_combo.addItem('深色','dark');self.theme_combo.addItem('浅色','light');title.addWidget(self.theme_combo)
        title.addStretch();legacy=QPushButton('ESP32 原控制台');legacy.clicked.connect(self.open_legacy);title.addWidget(legacy)
        self.global_stop=QPushButton('STOP · 停止');self.global_stop.clicked.connect(lambda:self.safe(lambda:self.action('stop')))
        self.global_stop.setStyleSheet('background:#8b3434;color:white;font-weight:bold;');title.addWidget(self.global_stop)
        self.export_btn=QPushButton('导出曲线 CSV');self.export_btn.clicked.connect(self.export_dialog);title.addWidget(self.export_btn);layout.addLayout(title)
        self.banner=QLabel('离线待机 · 未连接 · 不自动运行');self.banner.setWordWrap(True);layout.addWidget(self.banner)
        body=QHBoxLayout();layout.addLayout(body,1);left=QWidget();left.setFixedWidth(400);ll=QVBoxLayout(left)
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(left);scroll.setFixedWidth(424);body.addWidget(scroll);self.left_scroll=scroll
        conn=QGroupBox('设备连接');cl=QVBoxLayout(conn);ll.addWidget(conn)
        self.transport=ThemedComboBox();self.transport.addItems(['离线协议模拟（非实机）','FPGA USB 串口','多圈协议模拟（非实机）','SV27 持续伺服模拟（非实机）']);cl.addWidget(self.transport)
        portrow=QHBoxLayout();self.port=ThemedComboBox();self.port.setEditable(True);self.refresh=QPushButton('刷新串口');self.refresh.clicked.connect(self.refresh_ports)
        portrow.addWidget(self.port,1);portrow.addWidget(self.refresh);cl.addLayout(portrow)
        self.connect_btn=QPushButton('连接');self.connect_btn.clicked.connect(self.toggle_connection);cl.addWidget(self.connect_btn)
        cl.addWidget(QLabel('460800 · 8N1 · FPGA F1 / 协议 v2–v4'))
        tabs=QTabWidget();ll.addWidget(tabs);self.calibration_tabs=tabs
        calibration=QWidget();cal=QVBoxLayout(calibration);tabs.addTab(calibration,'参数标定');self.editors={}
        fields=[('offset_a','IA 零偏 / count',512,3584,4),('offset_b','IB 零偏 / count',512,3584,4),
                ('gain_a','IA 增益 / A/count',-.00390625,.00390625,8),('gain_b','IB 增益 / A/count',-.00390625,.00390625,8),
                ('arm_zero','旋臂物理零点 / count',0,16383,0),('pend_down','摆杆下垂零点 / count',0,4095,0),
                ('electrical_zero','电角偏置 / count（65536/圈）',0,65535,0),('poles','极对数',1,32,0)]
        form=QFormLayout();defaults=asdict(Calibration())
        for key,label,low,high,dec in fields:
            edit=QDoubleSpinBox() if dec else QSpinBox();edit.setRange(low,high)
            if dec:edit.setDecimals(dec);edit.setSingleStep(1/65536 if key.startswith('gain') else .25)
            edit.setValue(defaults[key]);self.editors[key]=edit;form.addRow(label,edit)
        self.reverse=QCheckBox('旋臂 / 换相方向反转');form.addRow(self.reverse);cal.addLayout(form)
        note=QLabel('先采集零偏、确认增益幅值并写入。自动标定会请求通电：先电流方向，再电角对齐；结果需显式应用。请确认实际供电与接线。');note.setWordWrap(True);cal.addWidget(note)
        self.map_btn=self.button(cal,'电流方向标定 · 1.2 秒',lambda:self.action('commission',5))
        self.align_btn=self.button(cal,'电角自动对齐 · 15 秒 / 11 极对',lambda:self.action('commission',6))
        self.commission_status=QLabel('自动标定尚未开始');self.commission_status.setWordWrap(True);cal.addWidget(self.commission_status)
        self.candidate_btn=self.button(cal,'应用候选结果并回读核对',self.apply_candidate)
        self.offset_btn=self.button(cal,'采集静止 ADC 零偏（2 秒）',self.start_offsets)
        self.zero_arm=self.button(cal,'将当前旋臂原始值填为物理零点',lambda:self.capture_zero('arm_zero','arm_raw'))
        self.zero_pend=self.button(cal,'将当前摆杆原始值填为下垂零点',lambda:self.capture_zero('pend_down','pend_raw'))
        self.apply_btn=self.button(cal,'写入、提交并回读核对',lambda:self.action('apply',self.calibration()))
        self.read_btn=self.button(cal,'回读设备参数',lambda:self.action('read'))
        self.button(cal,'保存标定文件',self.save_profile);self.button(cal,'加载标定文件',self.load_profile)
        self.cal_status=QLabel('未写入 / 未核对');self.cal_status.setWordWrap(True);cal.addWidget(self.cal_status)
        for editor in self.editors.values():editor.valueChanged.connect(lambda:self.cal_status.setText('表单已修改，尚未写入'))
        self.reverse.toggled.connect(lambda:self.cal_status.setText('表单已修改，尚未写入'))
        gain=QWidget();gl=QVBoxLayout(gain);tabs.addTab(gain,'两点增益')
        info=QLabel('填入外部测得的两点电流及对应 ADC 原始值，计算符号、增益和零偏。计算只更新表单，需显式写入。');info.setWordWrap(True);gl.addWidget(info)
        self.gain_channel=ThemedComboBox();self.gain_channel.addItems(['IA','IB']);gl.addWidget(self.gain_channel);gf=QFormLayout();self.gain_points=[]
        for label,value,low,high in [('读数 1 / count',1400,0,4095),('电流 1 / A',-1,-5,5),('读数 2 / count',2700,0,4095),('电流 2 / A',1,-5,5)]:
            e=QDoubleSpinBox();e.setDecimals(5);e.setRange(low,high);e.setValue(value);self.gain_points.append(e);gf.addRow(label,e)
        gl.addLayout(gf);self.button(gl,'计算并填入标定表',self.compute_gain);gl.addStretch()
        control=QGroupBox('模式控制');ctl=QVBoxLayout(control);ll.addWidget(control)
        self.mode=ThemedComboBox();self.mode.addItems(['1 · 小电流测试','2 · 下垂位置保持','3 · 起摆 → 接摆 → 稳摆','4 · 直立附近稳摆']);ctl.addWidget(self.mode)
        self.current=QDoubleSpinBox();self.current.setDecimals(3);self.current.setRange(-.08,.08);self.current.setSingleStep(.01);self.current.setSuffix(' A（模式 1）');ctl.addWidget(self.current)
        self.arm_btn=self.button(ctl,'启动所选模式',lambda:self.action('arm',(self.mode.currentIndex()+1,self.current.value())))
        self.home_goal=QDoubleSpinBox();self.home_goal.setRange(-160,160);self.home_goal.setDecimals(2);self.home_goal.setSuffix(' ° 下垂位置目标');ctl.addWidget(self.home_goal)
        self.home_btn=self.button(ctl,'下垂位置移动 → 静止确认 → STOP',lambda:self.action('home',math.radians(self.home_goal.value())))
        self.session_home_btn=self.button(ctl,'使能保持',self.enable_hold)
        self.session_swing_btn=self.button(ctl,'起摆稳摆',self.start_continuous)
        motion_box=QGroupBox('位置与有限速度运动');ml=QVBoxLayout(motion_box);ctl.addWidget(motion_box)
        self.fast_angle=QDoubleSpinBox();self.fast_angle.setRange(-180,180);self.fast_angle.setValue(30);self.fast_angle.setSuffix(' ° 相对快移');ml.addWidget(self.fast_angle)
        self.fast_btn=self.button(ml,'执行快移',lambda:self.action('fast',self.fast_angle.value()))
        self.position_goal=QDoubleSpinBox();self.position_goal.setRange(-159,159);self.position_goal.setSuffix(' ° 绝对位置');ml.addWidget(self.position_goal)
        self.position_btn=self.button(ml,'快速定位',lambda:self.action('position',math.radians(self.position_goal.value())))
        self.position_dial=PositionDial();ml.addWidget(self.position_dial)
        self.position_dial.positionRequested.connect(lambda degree:self.safe(lambda:self.action('position',math.radians(degree))))
        self.position_dial.turnRequested.connect(lambda sign:self.safe(lambda:self.action('fast',sign*360)))
        self.servo_rpm=QSpinBox();self.servo_rpm.setRange(-200,200);self.servo_rpm.setValue(5);self.servo_rpm.setSuffix(' rpm');ml.addWidget(self.servo_rpm)
        self.servo_seconds=QDoubleSpinBox();self.servo_seconds.setRange(.1,10);self.servo_seconds.setValue(.5);self.servo_seconds.setSuffix(' s 后减速');ml.addWidget(self.servo_seconds)
        self.servo_btn=self.button(ml,'有限伺服 → 减速 → 位置稳摆',lambda:self.action('servo',(self.servo_rpm.value(),self.servo_seconds.value())))
        self.live_servo_begin=self.button(ml,'进入速度伺服（初始 0 rpm）',self.begin_live_servo)
        self.live_servo_value=QLabel('目标 0 rpm');ml.addWidget(self.live_servo_value)
        self.live_servo_slider=QSlider(Qt.Horizontal);self.live_servo_slider.setRange(-200,200);self.live_servo_slider.setSingleStep(1)
        self.live_servo_slider.valueChanged.connect(self.live_servo_target);ml.addWidget(self.live_servo_slider)
        self.live_servo_zero=self.button(ml,'目标归零 · 保持伺服',lambda:self.live_servo_slider.setValue(0))
        self.live_servo_end=self.button(ml,'减速退出伺服',lambda:self.action('servo_end'))
        self.motion_bounds=QLabel('连接后按固件能力设置运动范围。');self.motion_bounds.setWordWrap(True);ml.addWidget(self.motion_bounds)
        self.demo_btn=self.button(ml,'有限范围演示 · ±30° 往返与短伺服',lambda:self.action('demo','bounded'))
        self.full_demo_btn=self.button(ml,'演示',lambda:self.action('demo','v27_full'))
        self.full_demo_btn.setToolTip('起摆后依次快移 +90°、−90°、+180°、−180°、+360°；随后 +200 rpm 有效平台保持1秒，减速退出并受控落摆。使用当前标定；必要时先回零，等待真实到位及热预算。')
        self.hold_seconds=QSpinBox();self.hold_seconds.setRange(1,600);self.hold_seconds.setValue(60);self.hold_seconds.setSuffix(' 秒稳摆后自动落摆');ctl.addWidget(self.hold_seconds)
        self.cycle_direction=ThemedComboBox();self.cycle_direction.addItems(['循环结束正向落摆','循环结束反向落摆']);ctl.addWidget(self.cycle_direction)
        self.long_btn=self.button(ctl,'起摆 → 定时稳摆 → 落摆 → STOP',lambda:self.action('cycle',(self.hold_seconds.value(),1 if self.cycle_direction.currentIndex()==0 else -1)))
        self.drop_positive=self.button(ctl,'正向受控落摆 → 静止确认 → STOP',lambda:self.action('drop',1))
        self.drop_negative=self.button(ctl,'反向受控落摆 → 静止确认 → STOP',lambda:self.action('drop',-1))
        self.stop_btn=self.button(ctl,'STOP · 停止',lambda:self.action('stop'));self.stop_btn.setStyleSheet('background:#8b3434;color:white;font-weight:bold;')
        self.clear_btn=self.button(ctl,'核验并恢复故障',lambda:self.action('recover'))
        ll.insertWidget(1,control)
        self.button(ll,'打开 FPGA / ESP32 曲线 CSV 回放',self.open_replay);self.play_btn=self.button(ll,'播放 / 暂停回放',self.toggle_replay)
        self.replay_seek=QSlider(Qt.Horizontal);self.replay_seek.setRange(0,1000)
        self.replay_seek.sliderMoved.connect(self.seek_replay);ll.addWidget(self.replay_seek)
        self.plot_window=QSpinBox();self.plot_window.setRange(5,120);self.plot_window.setValue(20);self.plot_window.setSuffix(' 秒图表窗口')
        self.plot_window.valueChanged.connect(self.set_plot_window);ll.addWidget(self.plot_window);ll.addStretch()
        right=QVBoxLayout();body.addLayout(right,1)
        self.metrics=QLabel('等待完整快照');self.metrics.setWordWrap(True);right.addWidget(self.metrics)
        self.resources=ResourceBars();right.addWidget(self.resources)
        self.charts=[TraceChart('角度 / °',[('q','旋臂','teal'),('alpha','摆杆（下垂为0）','#6eacff')]),
                     TraceChart('电流 / A',[('ia','IA','teal'),('ib','IB','#6eacff'),('iq','Iq','amber'),('iq_ref','Iq目标','#ff7b7b')]),
                     TraceChart('电压 / V',[('ud','Ud','teal'),('uq','Uq','#6eacff')])]
        self.plot_tabs=QTabWidget();right.addWidget(self.plot_tabs,1)
        physical=QWidget();physical_layout=QVBoxLayout(physical)
        for chart in self.charts:physical_layout.addWidget(chart,1)
        self.plot_tabs.addTab(physical,'换算量（需有效标定）')
        raw=QWidget();raw_layout=QVBoxLayout(raw)
        raw_layout.addWidget(QLabel('原始计数未应用标定；采样有效性以上方状态为准，错误数据保留供排查。'))
        raw_charts=[TraceChart('编码器原始计数',[('arm_raw','旋臂 AS','teal'),('pend_raw','摆杆 OID','#6eacff')]),
                    TraceChart('ADC 原始计数',[('ia_raw','IN0 / IA','teal'),('ib_raw','IN1 / IB','#6eacff')])]
        for chart in raw_charts:raw_layout.addWidget(chart,1)
        self.charts.extend(raw_charts);self.plot_tabs.addTab(raw,'原始采样')
        speed_page=QWidget();speed_layout=QVBoxLayout(speed_page)
        self.speed_chart=TraceChart('观测角速度 / rad/s',[('w','旋臂观测','teal'),('v','摆杆观测','#6eacff')])
        speed_layout.addWidget(self.speed_chart);self.charts.append(self.speed_chart);self.plot_tabs.addTab(speed_page,'速度')
        bottom=QHBoxLayout();self.pose=PendulumView();self.pose.setMinimumWidth(240);self.pose.setMaximumWidth(280);bottom.addWidget(self.pose)
        self.log=QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(1000);self.log.setMaximumHeight(190);bottom.addWidget(self.log,1);right.addLayout(bottom)
        self.theme_combo.currentIndexChanged.connect(self.change_theme)
        self.ui_timing=UiTiming()
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(20);self.update_controls()
        self.pose_selector=PoseSelector()
        from .fpga_layout import build_layout
        build_layout(self);self.update_controls()
        for index,widget in enumerate([*self.charts,self.pose,self.position_dial]):
            widget.ui_timing=self.ui_timing;widget.ui_timing_name='.'+str(index)
        from .chart_raster import ChartRaster
        self.chart_raster=ChartRaster();self.last_render=-float('inf')
        for chart in self.charts:chart.frame_renderer=self.chart_raster
        self.select_port(self.preferences.get('last_successful_port',''))
        self.theme_combo.setCurrentIndex(1 if self.preferences.get('theme')=='light' else 0)
        self.loading_preferences=False
        saved_profile=self.preferences.get('calibration_profile')
        if saved_profile:
            try:self.set_calibration(Calibration.load(saved_profile));self.log.appendPlainText('已加载上次实际回读的新标定表单；未自动写入设备')
            except Exception as exc:self.log.appendPlainText('保存的标定表单未加载：'+str(exc))
        self.refresh_operation_display();self.refresh_parameters()
        for chart in self.charts:chart.markers=self.command_markers
        if scan_ports_on_start:self.safe(self.refresh_ports)
    def save_preference(self,key,value):
        self.preferences[key]=value
        if not self.settings_path:return
        try:
            self.settings_path.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.settings_path.with_suffix(self.settings_path.suffix+'.tmp')
            temporary.write_text(json.dumps(self.preferences,ensure_ascii=False,indent=2),encoding='utf-8')
            temporary.replace(self.settings_path)
        except OSError as exc:self.log.appendPlainText('界面偏好未保存：'+str(exc))
    def invalidate_device_binding(self):
        # A deployment claim belongs to one uninterrupted connection, not a port name.
        self.device_identity={};self.auto_profile=None
    def release_deployment(self):
        if self.deployment_flow is not None:
            try:self.deployment_flow.close()
            except Exception as exc:self.log.appendPlainText('部署独占清理失败：'+str(exc));return
            self.deployment_flow=None;self.serial_factory=None
    def configure_and_connect(self):
        if self.controller is not None or self.deploying:raise ValueError('请先停止并等待原串口完全退出')
        if self.deployment_flow is not None:raise ValueError('上次部署独占尚未清理完成')
        if self.deployment_adapter is None:raise ValueError('发布部署入口尚未绑定，普通连接仍可使用')
        if self.legacy is not None and self.legacy.controller is not None:raise ValueError('请先退出原串口owner')
        from .fpga_deployment import DeploymentFlow
        self.invalidate_device_binding();self.deploying=True
        self.deployment_flow=DeploymentFlow(self.deployment_adapter,self.deployment_target)
        def work():
            try:self.events.put(dict(kind='deployment_ready',result=self.deployment_flow.run()))
            except Exception as exc:self.events.put(dict(kind='deployment_failed',text=str(exc)))
        self.deployment_thread=threading.Thread(target=work,name='fpga-deployment',daemon=False)
        self.deployment_thread.start();self.update_controls()
    def refresh_operation_display(self):
        active=bool(self.busy or self.servo_active or self.sequence_active)
        if active and self.phase_at is None:self.phase_at=time.monotonic()
        if not active:self.phase_at=None
        self.progress.setVisible(active);self.elapsed.setVisible(active)
        if active:
            elapsed=max(0,int(time.monotonic()-self.phase_at))
            self.elapsed.setText(f'{self.operation_name} · 已用 {elapsed//60:02d}:{elapsed%60:02d} · 可随时停止')
        else:self.elapsed.setText('—')
    @timed_ui('parameters')
    def refresh_parameters(self):
        row=self.latest or {};fresh=bool(self.connected and time.monotonic()-self.last_rx<=.25 and row.get('control_sample_fresh',True))
        lines=['FPGA F1 · '+('当前设备读回' if fresh else '历史/过期读回，非当前状态'),
               '协议版本：'+str(row.get('protocol_revision','未知')),
               f"本次连接距最近不同戳快照：{max(0,time.monotonic()-self.last_rx):.1f}s（不代表控制采样真实年龄）" if row else '尚无控制采样']
        for address,name in ((24,'HOME'),(26,'MOT2'),(29,'ENA1'),(32,'SV27'),(33,'RST1'),(35,'SPD1')):
            lines.append(f'{name}：'+('已报告' if self.capabilities.get(address) else '未报告/未知'))
        lines.append('设备8字读回（不取代未提交表单）：')
        if self.last_device_words is None:lines.append('尚未读回')
        else:
            lines.extend(f'[{index}] 0x{word:08X}' for index,word in enumerate(self.last_device_words))
            try:
                calibration=Calibration.from_words(self.last_device_words)
                lines.extend(f'{key} = {value}' for key,value in asdict(calibration).items())
            except ValueError:lines.append('读回值尚非有效标定')
        lines.extend(('标定单位：offset=count；gain=A/count；arm_zero=16384 count/圈；pend_down=4096 count/圈；electrical_zero=65536 count/圈',
                      '复位世代：'+str(row.get('reset_epoch','未知')),
                      '设备/位流绑定：'+('已提供可信上下文' if self.device_identity else '未提供，不自动初始化')))
        text='\n'.join(lines)
        if self.parameters.toPlainText()!=text:self.parameters.setPlainText(text)
    def open_records(self):
        from PySide6.QtCore import QUrl
        from PySide6.QtGui import QDesktopServices
        path=Path(self.record_directory) if self.record_directory else ROOT/'records'
        path.mkdir(parents=True,exist_ok=True);QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))
    def change_scale(self):
        selected=self.scale_combo.currentIndex();automatic=selected==3
        down=selected==2 or automatic and self.pose_selector.pose!='upright'
        self.beta_chart.title=('下垂偏差 α / °' if down else '直立偏差 β / °')+(' · 自动' if automatic else '')
        if automatic and self.pose_selector.pose is None:self.beta_chart.title='自动偏差 · 等待有效姿态'
        self.beta_chart.series=[('alpha','α','teal')] if down else [('beta','β','teal')]
        self.beta_chart.bounds=(-5,5) if selected else (-200,200)
        self.current_chart.bounds=(-1.1,1.1) if selected else (-1.8,1.8)
        self.beta_chart.update();self.current_chart.update()
    def refresh_metrics(self,plot,update_pose=True):
        for key,value in dict(theta=math.remainder(plot['q'],360),alpha=math.remainder(plot['alpha'],360),beta=plot['beta'],iq=plot['iq']).items():
            self.metric_labels[key].setText(f'{value:.2f}' if math.isfinite(value) else '—')
        if update_pose:self.pose_selector.update(plot)
        self.change_scale()
    def change_theme(self):
        name=self.theme_combo.currentData();self.setStyleSheet(stylesheet(name))
        if not self.loading_preferences:self.save_preference('theme',name)
        for widget in [*self.charts,self.pose,self.position_dial]:widget.theme=name;widget.update()
        for combo in self.findChildren(QComboBox):
            if hasattr(combo,'set_theme'):combo.set_theme(name)
    def set_plot_window(self,seconds):
        for chart in self.charts:chart.window=seconds;chart.update()
    def keyPressEvent(self,event):
        if event.key()==Qt.Key_Escape:
            if self.controller:self.safe(lambda:self.action('stop'))
            event.accept();return
        super().keyPressEvent(event)
    def button(self,layout,text,callback):
        b=QPushButton(text);b.clicked.connect(lambda:self.safe(callback,label=b.text()));layout.addWidget(b);return b
    def trace_action(self,kind,**fields):
        trace=getattr(self.controller,'trace_ui_action',None)
        if trace is not None:trace(kind,**fields)
    def safe(self,callback,label=None):
        try:
            if label:
                self.log.appendPlainText('点击：'+label)
                self.trace_action('activated',label=label)
            return callback()
        except Exception as exc:
            reason='请求队列已满，请等待当前操作或STOP取消' if isinstance(exc,queue.Full) else str(exc) or type(exc).__name__
            text='操作被拒绝：'+reason
            self.log.appendPlainText(text);self.banner.setText(text)
            self.trace_action('rejected',text=text,error_type=type(exc).__name__)
            return None
    def refresh_ports(self):
        from serial.tools import list_ports
        from .port_labels import present_port_identities,friendly_ports,preferred_port
        old=self.selected_port();ports=list(list_ports.comports())
        try:identities=present_port_identities()
        except (OSError,ValueError):identities={}
        labelled=list(friendly_ports(ports,identities));selected=preferred_port(labelled,old)
        self.port.clear()
        for device,label,role in labelled:
            self.port.addItem(label,device);self.port.setItemData(self.port.count()-1,label,Qt.ToolTipRole)
        self.select_port(selected)
        self.port.setToolTip(self.port.currentText() if selected else '未找到唯一匹配的FPGA通信口；请选择并确认，尚未连接')
        if not selected:self.log.appendPlainText('串口扫描完成；未自动选择：设备缺失或身份不唯一，请确认通信口。')
    def selected_port(self):
        index=self.port.currentIndex()
        if index>=0 and self.port.currentText()==self.port.itemText(index):return self.port.itemData(index) or self.port.currentText()
        return self.port.currentText().strip()
    def select_port(self,port):
        index=self.port.findData(port)
        if index>=0:self.port.setCurrentIndex(index)
        else:self.port.setEditText(port)
        if self.port.lineEdit():self.port.lineEdit().setCursorPosition(0)
    def calibration(self):
        c=Calibration(**{k:e.value() for k,e in self.editors.items()},reverse=self.reverse.isChecked());c.words();return c
    def set_calibration(self,c):
        for k,value in asdict(c).items():
            if k=='reverse':self.reverse.setChecked(value)
            else:self.editors[k].setValue(value)
    def action(self,action,data=None):
        if not self.controller:raise ValueError('请先连接')
        if action!='servo_target':
            self.log.appendPlainText('请求：'+action+'（等待控制器与设备确认）')
            self.trace_action('request',action=action,data_repr=repr(data))
        if action in ('commission','apply','stop','clear'):self.candidate=None
        if action=='commission':self.commission_status.setText('请求自动标定；等待设备确认')
        self.controller.request(action,data)
        if action in ('continuous','demo') or action=='initialize_hold' and data[1]=='continuous':self.sequence_active=True
        if action not in ('servo_target',):
            self.operation_name={'soft_reset':'软复位核验','recover':'故障恢复','initialize_hold':'初始化与使能','continuous':'起摆稳摆','demo':'演示','servo_begin':'持续伺服','servo_end':'减速退出','stop':'等待停止确认','speed':'有限速度轨迹'}.get(action,'设备操作')
            self.phase_at=time.monotonic()
        if action in ('soft_reset','recover'):self.reset_pending=True
        if action=='stop':self.reset_pending=False
        if action in ('stop','servo_end'):self.servo_ending=True
        self.busy=True;self.update_controls()
        if action=='apply':self.cal_status.setText('写入 / 回读处理中，尚未核对')
    def begin_live_servo(self):
        if self.servo_active:
            self.action('servo_end');return
        self.live_servo_slider.blockSignals(True);self.live_servo_slider.setValue(0);self.live_servo_slider.blockSignals(False)
        self.live_servo_value.setText('目标 0 rpm · 等待进入')
        self.action('servo_begin')
    def set_entry_compat(self,checked):
        self.entry_compat_note.setVisible(checked)
        if self.controller:self.controller.build14_entry_compat=bool(checked)
        self.log.appendPlainText('B14过渡兼容已开启：进入速度前可能先回零，会改变实际位置。' if checked else 'B14过渡兼容已关闭：人工速度命令直接请求设备。')
        self.update_controls()
    def enable_hold(self):
        if self.latest and self.latest.get('run') and self.latest.get('keep_session'):
            self.action('stop');return
        if self.latest is None:raise ValueError('需要当前位置遥测')
        if self.auto_profile is not None and not self.manual_control:
            self.action('initialize_hold',(self.auto_profile,'hold'));return
        if not self.manual_control and not self.latest.get('calibrated'):raise ValueError('首次使能缺少已绑定的核验配置；未写入或使能')
        self.action('session_home',hold_phase(self.latest['q']))
    def start_continuous(self):
        if not self.manual_control and self.latest and not self.latest['run'] and self.auto_profile is not None:
            self.action('initialize_hold',(self.auto_profile,'continuous'));return
        self.action('continuous')
    def live_servo_target(self,rpm):
        self.live_servo_value.setText(f'目标 {rpm} rpm')
        self.safe(lambda:self.action('servo_target',rpm))
    def apply_candidate(self):
        if self.candidate is None:raise ValueError('没有有效候选结果')
        candidate=self.candidate;self.set_calibration(candidate);self.action('apply',candidate)
    def toggle_connection(self):self.safe(self._toggle_connection)
    def _toggle_connection(self):
        if self.deploying:raise ValueError('配置未完成，不能连接')
        if self.controller:self.invalidate_device_binding();self.controller.close();return
        if self.legacy is not None and self.legacy.controller is not None:raise ValueError('请先断开 ESP32 控制台')
        self.samples.clear();self.latest=None;self.replay_running=False;self.replay=[]
        self.last_device_words=None
        self.pose_selector.reset()
        self.command_markers.clear()
        for chart in self.charts:chart.clear();chart.markers=self.command_markers
        self.offset_rows=None;self.cal_status.setText('新会话：未回读核对')
        self.candidate=None;self.commission_status.setText('新会话：无候选结果')
        factory=SimSerial if self.transport.currentIndex()==0 else self.serial_factory
        if self.transport.currentIndex()==2:factory=lambda *a,**kw:SimSerial(*a,multiturn=True,**kw)
        if self.transport.currentIndex()==3:factory=lambda *a,**kw:SimSerial(*a,multiturn=True,live_servo=True,**kw)
        if self.transport.currentIndex()==1 and not self.selected_port():raise ValueError('请选择串口')
        if self.transport.currentIndex()==1 and '驱动电源（非通信口）' in self.port.currentText():raise ValueError('此端口是驱动电源，请选择FPGA通信口')
        args={'serial_factory':factory} if factory else {}
        args['manual_control']=self.manual_control
        args['build14_entry_compat']=self.entry_compat.isChecked()
        args['backend']='hardware' if self.transport.currentIndex()==1 else 'simulation'
        if self.record_directory is not None:args['output']=self.record_directory
        if self.transport.currentIndex()==1 and self.device_identity:args['profile_identity']=self.device_identity
        self.diagnostic_at=None;self.diagnostic_row=None
        self.controller=FPGAController(self.selected_port(),self.events,**args);self.controller.start();self.connect_btn.setText('断开');self.banner.setText('等待 FPGA 身份与同步');self.update_controls()
    @timed_ui('controls')
    def update_controls(self):
        # Compute the same final policy before touching Qt. Temporary disable/
        # enable clears a real mouse press even when the final state is enabled.
        enabled_states={}
        def _enable(widget,value):enabled_states[widget]=bool(value)
        def _enabled(widget):return enabled_states.get(widget,widget.isEnabled())
        if hasattr(self,'deploy_btn'):
            _enable(self.deploy_btn,self.deployment_adapter is not None and self.controller is None and not self.deploying and self.deployment_flow is None)
            self.deploy_btn.setText('正在核验与配置…' if self.deploying else '配置设备并连接…')
        _enable(self.connect_btn,not self.deploying)
        fresh=self.latest is not None and time.monotonic()-self.last_rx<.25 and (self.latest.get('protocol_revision')!=4 or self.latest.get('control_sample_fresh'))
        stopped=fresh and not self.latest['run'];idle=self.connected and stopped and not self.busy
        for b in (self.offset_btn,self.zero_arm,self.zero_pend,self.apply_btn,self.read_btn,self.clear_btn):_enable(b,idle)
        _enable(self.offset_btn,idle and self.latest['sensor_valid'] and not self.latest['fault'])
        for editor in [*self.editors.values(),self.reverse]:_enable(editor,not self.busy and (not self.connected or stopped))
        _enable(self.arm_btn,self.connected and fresh and not self.busy and self.latest['ready'] and not self.latest['run'] and not self.latest['fault'])
        _enable(self.long_btn,_enabled(self.arm_btn) and all(self.capabilities.get(addr) for addr in (17,19,22)))
        _enable(self.home_btn,_enabled(self.arm_btn) and bool(self.capabilities.get(24)))
        motion_ready=self.connected and fresh and not self.busy and not self.sequence_active and not self.latest['fault'] and bool(self.capabilities.get(26))
        multi=bool(self.latest and self.capabilities.get(26) and self.latest.get('protocol_revision') in (3,4))
        self.fast_angle.setRange(-360 if multi else -180,360 if multi else 180)
        self.position_goal.setRange(-180 if multi else -159,180 if multi else 159)
        self.home_goal.setRange(-180 if multi else -160,180 if multi else 160)
        self.servo_rpm.setRange(-120 if multi and not self.manual_control else -200,120 if multi and not self.manual_control else 200)
        self.motion_bounds.setText('MOT2：支持整圈快移；持续伺服和有限速度按设备能力提供。绝对位置表示物理角度。' if multi else 'MOT1：路径与制动行程限制在 ±2.78 rad。')
        upright=motion_ready and self.latest['run'] and self.latest['mode'] in (3,4)
        enabled_down=motion_ready and multi and self.latest.get('keep_session') and self.latest['mode']==0 and self.latest.get('home_settled')
        keeping=bool(self.latest and self.latest.get('keep_session') and self.latest['run'])
        _enable(self.session_home_btn,bool(self.connected and multi and self.capabilities.get(29) and (_enabled(self.arm_btn) or fresh and keeping and not self.busy)))
        self.session_home_btn.setText('空闲 · 关桥' if keeping else '使能保持')
        _enable(self.session_swing_btn,bool(multi and (enabled_down or _enabled(self.arm_btn)) and self.capabilities.get(29)))
        can_initialize=bool(self.connected and fresh and not self.busy and self.latest and not self.latest['run'] and not self.latest['fault'] and self.latest['sensor_valid'] and self.auto_profile and self.auto_profile.matches(self.device_identity,self.latest.get('protocol_revision')) and all(self.capabilities.get(a) for a in (24,26,29)))
        if can_initialize:_enable(self.session_home_btn,True);_enable(self.session_swing_btn,True)
        for b in (self.fast_btn,self.position_btn):_enable(b,bool(upright or enabled_down or motion_ready and _enabled(self.arm_btn)))
        _enable(self.position_dial,bool(multi and _enabled(self.position_btn)))
        self.position_dial.set_position(math.degrees(self.latest['q']) if fresh else None,
            math.degrees(self.latest['motion_goal']) if fresh and 'motion_goal' in self.latest else None)
        if self.replay and self.replay_format=='esp32' and self.samples and not self.controller:
            self.position_dial.set_position(self.samples[-1]['theta'])
        _enable(self.servo_btn,bool(upright))
        servo_qualified=bool(self.latest and (self.latest.get('protocol_revision')!=4 or self.latest.get('servo_eligible') and self.latest.get('control_sample_fresh')))
        _enable(self.live_servo_begin,bool(upright and self.capabilities.get(32) and self.latest.get('keep_session') and servo_qualified))
        live_ready=bool(self.connected and fresh and self.servo_active and not self.servo_ending and self.latest['run'] and not self.latest['fault'] and self.latest['mode']==0 and self.latest.get('motion_kind')==3)
        self.live_servo_begin.setText('正在减速退出' if self.servo_ending and self.servo_active else '退出速度伺服' if self.servo_active else '进入速度伺服')
        if self.servo_active:_enable(self.live_servo_begin,live_ready)
        for widget in (self.live_servo_slider,self.live_servo_zero,self.live_servo_end):_enable(widget,live_ready)
        _enable(self.demo_btn,bool(_enabled(self.arm_btn) and all(self.capabilities.get(x) for x in (17,19,22,24,26))))
        _enable(self.full_demo_btn,bool(multi and (_enabled(self.arm_btn) or enabled_down) and all(self.capabilities.get(x) for x in (17,19,22,24,26,29))))
        keeping=bool(self.latest and self.latest.get('keep_session'))
        self.drop_positive.setText('正向受控落摆 → 下垂使能待机' if keeping else '正向受控落摆 → 静止确认 → STOP')
        self.drop_negative.setText('反向受控落摆 → 下垂使能待机' if keeping else '反向受控落摆 → 静止确认 → STOP')
        for b in (self.drop_positive,self.drop_negative):
            _enable(b,self.connected and fresh and not self.busy and self.latest['run'] and not self.latest['fault'] and self.latest['mode'] in (3,4) and all(self.capabilities.get(addr) for addr in (17,22)))
        for b in (self.map_btn,self.align_btn):_enable(b,_enabled(self.arm_btn))
        _enable(self.candidate_btn,idle and self.candidate is not None and not self.latest['fault'])
        _enable(self.stop_btn,self.controller is not None)
        _enable(self.global_stop,self.controller is not None)
        _enable(self.transport,self.controller is None and not self.deploying);_enable(self.port,self.controller is None and not self.deploying);_enable(self.refresh,self.controller is None and not self.deploying)
        if hasattr(self,'drop_btn'):
            _enable(self.drop_btn,_enabled(self.drop_positive))
            self.badge.setText('已连接' if self.connected else '未连接')
            self.phase.setText('等待操作完成' if self.busy else '等待连接' if not self.connected else '控制采样未更新 · 不能使能' if not fresh else '未就绪' if not self.latest['run'] and not self.latest['ready'] else '等待操作')
        if hasattr(self,'finite_speed_btn'):
            _enable(self.finite_speed_btn,bool(upright and self.capabilities.get(35) and self.latest.get('protocol_revision')==4 and self.latest.get('keep_session') and self.latest.get('speed_eligible') and self.latest.get('control_sample_fresh')))
        if hasattr(self,'reset_btn'):
            communication_fresh=fresh or self.diagnostic_at is not None and time.monotonic()-self.diagnostic_at<=.25
            recovery_enabled=bool(self.connected and communication_fresh and not self.reset_pending and not self.closing)
            _enable(self.reset_btn,bool(recovery_enabled and self.capabilities.get(33)))
            _enable(self.clear_btn,recovery_enabled)
            _enable(self.read_btn,bool(recovery_enabled and not self.busy))
        # Parameter restoration uses recent stopped communication, not motion sampling.
        status=self.diagnostic_row if self.diagnostic_at is not None and time.monotonic()-self.diagnostic_at<=.25 else self.latest if self.latest is not None and time.monotonic()-self.last_rx<=.25 else None
        _enable(self.apply_btn,bool(self.connected and status is not None and not status['run'] and not self.busy and not self.reset_pending and not self.sequence_active and not self.closing))
        if self.manual_control:
            # Manual commands use the live connection and transaction owner.
            # Qualification flags remain visible data; firmware accepts/refuses.
            available=bool(self.connected and not self.busy and not self.sequence_active and not self.closing)
            for widget in (self.arm_btn,self.map_btn,self.align_btn):_enable(widget,available)
            for widget,required in ((self.home_btn,(24,)),(self.long_btn,(19,)),
                    (self.session_home_btn,(24,26,29)),(self.session_swing_btn,(24,26,29)),
                    (self.fast_btn,(26,)),(self.position_btn,(26,)),(self.position_dial,(26,)),
                    (self.servo_btn,(26,)),(self.drop_positive,(17,22)),(self.drop_negative,(17,22)),
                    (self.demo_btn,(17,19,22,24,26)),(self.full_demo_btn,(17,19,22,24,26,29))):
                _enable(widget,available and all(self.capabilities.get(a) for a in required))
            _enable(self.live_servo_begin,bool(self.connected and not self.servo_ending and self.capabilities.get(32) and (available or self.servo_active)))
            for widget in (self.live_servo_slider,self.live_servo_zero,self.live_servo_end):
                _enable(widget,bool(self.connected and self.servo_active and not self.servo_ending))
            if hasattr(self,'finite_speed_btn'):_enable(self.finite_speed_btn,available and bool(self.capabilities.get(35)))
            if hasattr(self,'drop_btn'):_enable(self.drop_btn,_enabled(self.drop_positive))
            if hasattr(self,'phase') and available:self.phase.setText('人工操作 · 命令由设备确认')
        if hasattr(self,'entry_compat'):
            _enable(self.entry_compat,self.manual_control and not self.busy and not self.sequence_active and not self.servo_active)
            if self.entry_compat.isChecked() and not self.servo_active:
                self.live_servo_begin.setText('回零后进入速度伺服')
            if hasattr(self,'finite_speed_btn'):
                self.finite_speed_btn.setText('回零后执行有限轨迹' if self.entry_compat.isChecked() else '执行有限轨迹')
        if self.sequence_active:
            for widget in (self.arm_btn,self.home_btn,self.long_btn,self.session_home_btn,self.session_swing_btn,
                           self.fast_btn,self.position_btn,self.position_dial,self.servo_btn,self.demo_btn,self.full_demo_btn,
                           self.drop_positive,self.drop_negative,self.drop_btn,self.finite_speed_btn):_enable(widget,False)
            if not self.servo_active:_enable(self.live_servo_begin,False)
            self.phase.setText('起摆/演示流程进行中 · 可STOP中止')
        for widget,value in enabled_states.items():
            widget.setEnabled(value)

    def start_offsets(self):
        self.offset_rows=[];self.offset_started=time.monotonic();self.log.appendPlainText('开始采集静止零偏；不自动写入')
    def capture_zero(self,key,raw):
        if self.latest is None or self.latest['run'] or not self.latest['sensor_valid'] or time.monotonic()-self.last_rx>.25:raise ValueError('需要新鲜、采样有效的停止状态')
        self.editors[key].setValue(self.latest[raw]);self.cal_status.setText('表单已修改，需重新写入核对')
    def compute_gain(self):
        offset,gain=fit_gain(*(e.value() for e in self.gain_points));suffix='a' if self.gain_channel.currentIndex()==0 else 'b'
        self.editors['offset_'+suffix].setValue(offset);self.editors['gain_'+suffix].setValue(gain);self.cal_status.setText('两点增益已填入，尚未写入')
    def choose_file(self,title,initial,filters,save,selected):
        # Windows native modal file dialogs do not dispatch Qt timer events.
        # A nonmodal Qt dialog keeps UI pulse, telemetry and the STOP button live.
        dialog=QFileDialog(self,title,initial,filters)
        dialog.setOption(QFileDialog.Option.DontUseNativeDialog,True)
        dialog.setWindowModality(Qt.WindowModality.NonModal)
        dialog.setAcceptMode(QFileDialog.AcceptMode.AcceptSave if save else QFileDialog.AcceptMode.AcceptOpen)
        dialog.setFileMode(QFileDialog.FileMode.AnyFile if save else QFileDialog.FileMode.ExistingFile)
        dialog.fileSelected.connect(lambda path:self.safe(lambda:selected(path)))
        dialog.finished.connect(lambda _:dialog.deleteLater())
        dialog.show()
        return dialog
    def save_profile(self):
        return self.choose_file('保存 FPGA 标定',str(ROOT/'fpga_calibration.json'),'JSON (*.json)',True,self.save_profile_path)
    def save_profile_path(self,path):
        if path:
            words=self.calibration().words()
            if self.device_identity and self.last_device_words==words and self.latest and self.latest.get('calibrated') and self.device_identity.get('protocol_revision')==self.latest.get('protocol_revision'):
                BoundCalibrationProfile(self.device_identity,tuple(words)).save(path)
                self.log.appendPlainText('已保存设备/位流绑定的读回参数；可供显式首次使能初始化')
            else:
                self.calibration().save(path);self.log.appendPlainText('已保存普通参数文件；缺少精确绑定，不能自动初始化')
    def load_profile(self):
        return self.choose_file('加载 FPGA 标定','','JSON (*.json)',False,self.load_profile_path)
    def load_profile_path(self,path):
        if path:
            self.auto_profile=None
            data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
            if data.get('schema')=='fpga-bound-calibration-1':
                profile=BoundCalibrationProfile.load(path)
                if not profile.matches(self.device_identity,self.device_identity.get('protocol_revision')):raise ValueError('配置与已核验设备/位流身份不匹配，未采用自动初始化')
                self.auto_profile=profile;self.set_calibration(Calibration.from_words(profile.words))
            else:self.set_calibration(Calibration.load(path))
            self.cal_status.setText('文件已加载，尚未写入')
    @timed_ui('sample')
    def append_sample(self,row,live=True,render=True,received_at=None):
        previous=self.latest
        if not row.get('telemetry_current',True) or previous and 'reset_epoch' in previous and 'reset_epoch' in row and (row['reset_epoch']-previous['reset_epoch'])%4294967296>=2147483648:
            self.samples.append(row);return
        if previous and 'reset_epoch' in previous and 'reset_epoch' in row and previous['reset_epoch']!=row['reset_epoch']:
            # RST1 resets the local frame/observer, not the saved physical zero.
            # Keep raw records; never draw an interpolated trajectory across it.
            for chart in self.charts:chart.clear()
            self.log.appendPlainText(f"复位世代 {previous['reset_epoch']} → {row['reset_epoch']}：开始新遥测段，保留原始记录")
        self.samples.append(row);self.latest=row
        if live:self.last_rx=time.monotonic() if received_at is None else received_at
        extended=row.get('protocol_revision')==4
        feedback=row.get('sensor_velocity',math.nan) if extended and row.get('sensor_velocity_valid') else math.nan if extended else row['w']
        plot=dict(row,q=math.degrees(row.get('q_continuous',row['q'])),alpha=math.degrees(row['alpha']),
                  beta=math.degrees(math.remainder(row['alpha']-math.pi,math.tau)),
                  speed_rpm=feedback*30/math.pi,speed_ref_rpm=(row.get('reference_w',math.nan) if row.get('reference_valid') else math.nan)*30/math.pi)
        if not current_display_confirmed(row):
            plot.update(ia=math.nan,ib=math.nan,iq=math.nan,iq_ref=math.nan)
        self.pose_selector.update(plot)
        for chart in self.charts:chart.append(plot,redraw=False)
        if self.offset_rows is not None and live:
            self.offset_rows.append(row)
            if len(self.offset_rows)>=100:
                rows=self.offset_rows;self.offset_rows=None
                try:
                    a,b=estimate_offsets(rows);self.editors['offset_a'].setValue(a);self.editors['offset_b'].setValue(b);self.cal_status.setText('静止零偏已填入，尚未写入');self.log.appendPlainText(f'静止零偏：{a:.4f} / {b:.4f} count')
                except ValueError as exc:self.log.appendPlainText('零偏采集失败：'+str(exc))
        if render:self.render_sample(row,plot,live)
        else:self.pending_sample_display=(row,plot,live)

    @timed_ui('render.sample')
    def render_sample(self,row,plot,live):
        extended=row.get('protocol_revision')==4
        self.refresh_fpga_budget(row,recorded=not live)
        self.speed_chart.series=[('speed_rpm','传感器反馈' if extended else '观测速度(v2/v3)','accent'),('speed_ref_rpm','平滑参考','amber')]
        self.refresh_metrics(plot,update_pose=False)
        self.pose.sample=dict(theta=plot['q'],alpha=plot['alpha'],valid=row['valid']);self.pose.update()
        prefix='离线协议模拟' if self.transport.currentIndex()!=1 else 'FPGA 串口'
        if not live:prefix='历史回放（非当前设备）'
        self.pose.source_caption='历史记录姿态' if not live else ('模拟姿态示意' if self.transport.currentIndex()!=1 else 'FPGA 测量姿态')
        self.banner.setText(f"{prefix} · {'运行' if row['run'] else '停止'} · {'就绪' if row['ready'] else '未就绪'}"+(f" · {FAULTS.get(row['reason'],hex(row['reason']))}" if row['fault'] else ''))
        self.source_label.setText(prefix)
        if not row['sensor_valid']:self.banner.setText(self.banner.text()+f" | 采样无效：{FAULTS.get(row['sensor_error'],row['sensor_error'])}")
        if not current_display_confirmed(row):
            self.banner.setText(self.banner.text()+' | 电流未确认（停止未就绪或采样无效/陈旧）；原值仅作诊断')
        self.metrics.setText(f"t={row['t']:.3f}s   θ={plot['q']:.2f}°   α={plot['alpha']:.2f}°   ω={row['w']:.3f} rad/s   v={row['v']:.3f} rad/s\n原始值 AS={row['arm_raw']}  OID={row['pend_raw']}  ADC IA/IB={row['ia_raw']}/{row['ib_raw']}   PWM={row['duty_u']:.3f}/{row['duty_v']:.3f}/{row['duty_w']:.3f}")
    def export_csv(self,path):
        if not self.samples:raise ValueError('没有可导出的曲线')
        if self.replay and self.controller is None and self.replay_format=='esp32':
            from .protocol import write_display
            rows=list(self.samples);markers=[marker for row in rows for marker in row.get('commands',[])]
            write_display(path,rows,'REPLAY',markers);return
        with Path(path).open('w',newline='',encoding='utf-8-sig') as f:
            rows=[dict(row) for row in self.samples];times=[row['t'] for row in rows]
            markers=list(self.command_markers) if not self.replay else [marker for row in rows for marker in row.get('commands',[])]
            annotations=[[] for row in rows]
            for marker in markers:
                if marker['t']>=times[0]:annotations[min(bisect_left(times,marker['t']),len(rows)-1)].append(marker)
            for row,events in zip(rows,annotations):row['commands']=json.dumps(events,ensure_ascii=False) if events else ''
            fields=list(dict.fromkeys(key for row in rows for key in row))
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    def refresh_fpga_budget(self,row,recorded=False,expired=False):
        if row.get('protocol_revision')!=4:
            self.resources.refresh(None,'I',None,False,recorded=recorded);return
        valid=bool(row.get('budget_valid')) and not expired
        cap,dose=row['current_cap'],row['dose'];confirmed=current_display_confirmed(row)
        iq=abs(row['iq']) if confirmed else math.nan
        suffix='' if valid else ' · 无效/过期'
        self.resources.current_label.setText((f'电流 {iq:.2f} / {cap:.2f} A' if confirmed else '电流 — · 未确认；原值仅作诊断')+suffix)
        self.resources.heat_label.setText(f'I²t {dose:.3f} · 余 {max(0,1-dose):.3f} A²s'+suffix)
        self.resources.current.setValue(round(min(1,iq/cap)*1000) if confirmed and cap>0 else 0)
        self.resources.heat.setValue(round(min(1,max(0,dose))*1000))
        for bar in (self.resources.current,self.resources.heat):bar.setEnabled(valid)
        self.resources.current.setEnabled(valid and confirmed)
        reason=('历史记录额度' if recorded else 'FPGA原子额度')+suffix
        self.resources.reason.setText(reason)
        self.resources.setToolTip(reason+f" · 完成样本年龄 {row.get('control_sample_age_s',math.nan)*1000:.3f} ms；未提供快移/下落准入位时不推算允许动作。")
    def export_dialog(self):
        return self.choose_file('导出当前显示数据 CSV','','CSV (*.csv)',True,self.export_csv)
    def load_replay(self,path):
        if self.controller:raise ValueError('回放前请断开设备')
        format_name,rows=read_recording(path)
        self.replay_format=format_name
        self.samples.clear();self.replay=rows;self.replay_index=0;self.replay_running=True;self.latest=None
        self.replay_wall=time.monotonic();self.replay_time=rows[0]['t']
        markers=[marker for row in rows for marker in row.get('commands',[])]
        for chart in self.charts:chart.clear();chart.markers=markers
        self.charts[0].marker_labels=True;self.charts[2].marker_times=True
        self.seek_replay(0)
    def append_replay(self,row):
        if self.replay_format=='fpga':self.append_sample(row,live=False);return
        self.samples.append(row);plot=legacy_plot(row)
        self.refresh_metrics(plot)
        for chart in self.charts:chart.append(plot,redraw=False)
        self.pose.sample=dict(theta=row['theta'],alpha=row['alpha'],valid=row['valid'])
        self.pose.source_caption='ESP32历史记录姿态';self.pose.update()
        self.banner.setText('ESP32 历史回放 · 非当前设备 · 未记录的FPGA状态不补造')
        self.source_label.setText('ESP32历史回放 · 非当前设备')
        self.metrics.setText(f"t={row['t']:.3f}s  θ={row['theta']:.2f}°  α={row['alpha']:.2f}°  速度={row['speed']:.3f} rad/s")
        self.position_dial.set_position(row['theta'])
        self.resources.refresh(row.get('budget'),row.get('mode','I'),row['iq'],row.get('budget_valid',False),recorded=True)
    def seek_replay(self,value):
        if not self.replay or self.controller:return
        first,last=self.replay[0]['t'],self.replay[-1]['t']
        target=first+(last-first)*value/1000
        self.replay_index=max(1,bisect_right([row['t'] for row in self.replay],target))
        self.samples.clear();self.latest=None
        for chart in self.charts:chart.clear()
        start=bisect_right([row['t'] for row in self.replay],target-self.plot_window.value())
        for row in self.replay[start:self.replay_index]:self.append_replay(row)
        self.replay_time=target;self.replay_wall=time.monotonic();self.replay_seek.setValue(value)
    def open_replay(self):
        return self.choose_file('打开采样 CSV（FPGA / ESP32）','','CSV (*.csv)',False,self.load_replay)
    def toggle_replay(self):
        if not self.replay:return
        if self.replay_running:
            self.replay_time+=(time.monotonic()-self.replay_wall)*self.replay_rate;self.replay_running=False
        else:
            if self.replay_index>=len(self.replay):
                self.replay_index=0;self.replay_time=self.replay[0]['t'];self.samples.clear();self.latest=None
                for chart in self.charts:chart.clear()
            self.replay_wall=time.monotonic();self.replay_running=True
    def change_replay_speed(self):
        now=time.monotonic()
        if self.replay_running:self.replay_time+=(now-self.replay_wall)*self.replay_rate
        self.replay_wall=now;self.replay_rate=float(self.replay_speed.currentData())
    @timed_ui('poll')
    def poll(self):
        if self.controller:
            self.controller.pulse()
            self.controller.ui_activity=self.ui_timing
        self.poll_event_count=0
        drain_started=time.perf_counter()
        for index in range(64):
            if index and time.perf_counter()-drain_started>=.008:break
            try:event=self.events.get_nowait()
            except queue.Empty:break
            self.poll_event_count+=1
            kind=event['kind']
            if kind=='deployment_ready':
                self.deploying=False
                if self.closing:self.release_deployment();continue
                result=event['result'];self.serial_factory=result['serial_factory'];self.device_identity=result['identity']
                self.transport.setCurrentIndex(1);self.select_port(result['port'])
                try:self._toggle_connection()
                except Exception as exc:
                    self.invalidate_device_binding();self.release_deployment();self.log.appendPlainText('部署后连接失败：'+str(exc))
            elif kind=='deployment_failed':
                self.deploying=False;self.invalidate_device_binding();self.release_deployment()
                self.log.appendPlainText('配置失败，未自动连接或使能：'+event['text'])
            elif kind=='bootstrap_complete':
                # A stopped configuration receipt never dispatches power/motion
                # and does not substitute for normal Session telemetry.
                text='停止态配置与真实采样已完成；'+('板端资格就绪，仍需独立使能授权' if event['row']['ready'] else '尚不可使能：板端ready=false')
                self.cal_status.setText(text);self.log.appendPlainText(text)
            elif kind=='sample':
                if event['row'].get('control_sample_fresh',True):self.diagnostic_at=None
                self.append_sample(event['row'],render=False,received_at=event.get('received_monotonic'))
            elif kind=='diagnostic_status':
                self.diagnostic_at=event.get('received_monotonic',time.monotonic())
                self.diagnostic_row=event.get('row')
                self.diagnostic_text=event.get('text','已连接 · 控制采样未更新，不能使能；请检查驱动电源与传感器')
            elif kind=='capabilities':self.capabilities=event['values']
            elif kind=='connected':
                self.connected=True;self.log.appendPlainText(event['text'])
                if self.transport.currentIndex()==1:self.save_preference('last_successful_port',self.selected_port())
            elif kind=='command_reply':
                if event['op']>=0x20 or event['op'] in (2,3,0x11):
                    self.log.appendPlainText(f"设备 {'ACK' if event['accepted'] else 'NACK'} · {event['op']:02X}:{event['addr']} · seq {event['seq']} · 返回 {event['value']}；运行状态以遥测为准")
            elif kind=='busy':self.busy=event['value']
            elif kind=='zero_calibration_progress':
                self.operation_name=f"下垂零点标定 {event['seconds']:.1f}/15秒 · 输出禁用"
            elif kind=='zero_calibration_complete':
                self.last_device_words=list(event['words']);self.auto_profile=None
                self.set_calibration(Calibration.from_words(event['words']))
                self.save_preference('calibration_profile',event['profile_path'])
                self.operation_name='辅助零点标定完成 · 保持停止'
                self.log.appendPlainText(f"下垂零点已提交并回读：{event['words'][5]} count；新profile：{event['profile_path']}；未自动启动演示")
            elif kind=='sequence_state':self.sequence_active=event['active']
            elif kind=='upright_complete':self.log.appendPlainText('起摆稳摆流程已完成；后续动作仍需实时板端资格')
            elif kind=='command_sent':self.command_markers.append(event['marker'])
            elif kind=='reset_complete':
                self.reset_pending=False;self.busy=False
                name='软复位' if event.get('operation')=='reset' else '故障恢复'
                epoch=f"世代 {event['before_epoch']} → {event['epoch']}；" if event.get('operation')=='reset' else ''
                readiness='采样就绪' if event.get('healthy',True) else '采样或运动资格未就绪'
                self.log.appendPlainText(f"{name}核验完成：{epoch}8字及物理零点保持，{readiness}。现场：{event['snapshot_path']}")
            elif kind=='servo':
                entered=event['active'] and not event['ending'] and not self.servo_active
                self.servo_active=event['active'];self.servo_ending=event['ending']
                if entered:self.plot_tabs.setCurrentIndex(self.servo_page_index)
                if event['ending']:self.live_servo_value.setText('正在减速退出 · 等待位置稳摆')
                elif not event['active']:self.live_servo_value.setText('伺服未运行')
            elif kind=='candidate_invalid':
                self.candidate=None;self.commission_status.setText('候选结果已清除；需完整完成一次标定')
            elif kind=='commission_progress':
                duration=1.2 if event['mode']==5 else 15
                self.commission_status.setText(f"{'电流方向' if event['mode']==5 else '电角对齐'}：{event['seconds']:.2f} / {duration:g} 秒；错误 0x{event['error']:02X}")
            elif kind=='commission_result':
                self.candidate=Calibration.from_words(event['words'])
                self.commission_status.setText(f'候选结果（未应用）：IA={self.candidate.gain_a:.8f}，IB={self.candidate.gain_b:.8f}，电角零点={self.candidate.electrical_zero}，反向={self.candidate.reverse}')
            elif kind=='calibration':
                self.last_device_words=list(event['words'])
                try:self.set_calibration(Calibration.from_words(event['words']));self.cal_status.setText('写入并回读核对成功' if event['verified'] else '设备参数已回读（未新写入）')
                except ValueError:self.cal_status.setText('设备尚无完整有效标定')
            elif kind=='initialization_complete':
                self.last_device_words=list(event['words'])
                self.log.appendPlainText('初始化核验完成：'+('复用已有效参数' if event['reused'] else '绑定参数已提交并逐字读回')+'；保留原物理零点')
            elif kind=='disconnected':
                self.sequence_active=False
                self.pending_sample_display=None
                self.invalidate_device_binding()
                if self.deployment_flow is not None:self.deployment_owner=self.controller
                self.reset_pending=False
                self.candidate=None;self.capabilities={}
                self.servo_active=False;self.servo_ending=False
                if not self.closing:self.controller=None
                self.connected=False;self.busy=False;self.offset_rows=None;self.connect_btn.setText('连接');self.banner.setText(event['text']);self.log.appendPlainText(event['text'])
            elif kind in ('error','rejected','log','entry_preparation'):
                self.log.appendPlainText(event['text'])
                if kind=='rejected':self.busy=False;self.reset_pending=False
                if kind=='error':self.sequence_active=False;self.pending_sample_display=None;self.invalidate_device_binding();self.connected=False;self.busy=False;self.banner.setText(event['text'])
        display=self.pending_sample_display;self.pending_sample_display=None
        render_due=time.monotonic()-self.last_render>=.1
        if display is not None and self.connected:
            if render_due:self.render_sample(*display)
            else:self.pending_sample_display=display
        if self.controller and self.latest is not None and time.monotonic()-self.last_rx>.25:self.banner.setText('遥测已过期 · 当前设备状态未知')
        if self.controller and self.diagnostic_at is not None and time.monotonic()-self.diagnostic_at<=.25:
            self.banner.setText(self.diagnostic_text+f' · 已至少{max(0,time.monotonic()-self.last_rx):.1f}s无新采样')
            for label in self.metric_labels.values():label.setText('—')
            self.pose.sample={'theta':0,'alpha':0,'valid':False};self.pose.update()
        if self.controller and self.latest is not None:
            self.refresh_fpga_budget(self.latest,expired=time.monotonic()-self.last_rx>=.4)
        if self.offset_rows is not None and time.monotonic()-self.offset_started>4:self.offset_rows=None;self.log.appendPlainText('零偏采集超时，表单未更新')
        if self.replay_running and self.controller is None and self.replay:
            target=self.replay_time+(time.monotonic()-self.replay_wall)*self.replay_rate
            while self.replay_index<len(self.replay) and self.replay[self.replay_index]['t']<=target:
                self.append_replay(self.replay[self.replay_index]);self.replay_index+=1
            span=self.replay[-1]['t']-self.replay[0]['t']
            self.replay_seek.setValue(min(1000,round((target-self.replay[0]['t'])/span*1000)) if span else 1000)
            if self.replay_index>=len(self.replay):self.replay_running=False
        if render_due:
            self.last_render=time.monotonic()
            for chart in self.charts:chart.update()
        self.update_controls()
        self.refresh_operation_display();self.refresh_parameters()
        self.footer.setText(self.banner.text())
        if self.deployment_owner is not None and not self.deployment_owner.is_alive():
            self.release_deployment();self.deployment_owner=None
    def open_legacy(self):
        if self.controller:self.log.appendPlainText('请先断开 FPGA 连接，再切换 ESP32 控制台');return
        from .app import MainWindow
        if self.legacy is None:self.legacy=MainWindow()
        self.legacy.show()
    def closeEvent(self,event):
        if self.deploying:
            self.closing=True;self.deployment_flow.cancel()
            self.banner.setText('正在取消配置并等待有界清理')
            event.ignore();QTimer.singleShot(50,self.close);return
        if self.controller:
            if not self.closing:self.closing=True;self.controller.close()
            if self.controller.is_alive():
                self.banner.setText('正在停止并核实串口关闭，请等待清理完成')
                event.ignore();QTimer.singleShot(50,self.close);return
        if self.legacy:self.legacy.close()
        self.chart_raster.close()
        self.release_deployment();self.invalidate_device_binding()
        event.accept()

def main():
    import argparse
    parser=argparse.ArgumentParser(description='Furuta FPGA graphical host; connection is always explicit')
    parser.add_argument('--manual-control',action='store_true',help='Explicit user commands: firmware decides motion eligibility')
    parser.add_argument('--records',type=Path,help='Session log directory outside a frozen application tree')
    parser.add_argument('--settings',type=Path,help='UI preferences only; never restores connection or motion')
    args=parser.parse_args()
    settings=args.settings or Path(os.environ.get('LOCALAPPDATA',str(Path.home())))/'FurutaFPGA'/'ui-preferences.json'
    app=QApplication.instance() or QApplication([]);window=FPGAWindow(record_directory=args.records,settings_path=settings,manual_control=args.manual_control,scan_ports_on_start=True);window.show();return app.exec()


