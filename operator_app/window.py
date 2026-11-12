"""Main operator console window and operator actions."""
import math
import time
import uuid
from datetime import datetime
try:
    from PySide6.QtCore import QEvent, QTimer, Qt
    from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                   QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                   QSlider, QSpinBox, QTabWidget, QVBoxLayout, QWidget, QScrollArea, QLineEdit, QAbstractSpinBox)
except ImportError:
    from PyQt5.QtCore import QEvent, QTimer, Qt
    from PyQt5.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                 QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                 QSlider, QSpinBox, QTabWidget, QVBoxLayout, QWidget, QScrollArea, QLineEdit, QAbstractSpinBox)

try:
    from .map_canvas import MapCanvas
    from .transport import SocketWorker
    from .task_state import PHASES, TASK_NAMES, blocker_text
    from .mapping_panel import MappingPanel
    from .diagnostics import DiagnosticPanel
    from .underwater import UnderwaterPanel
except ImportError:
    from map_canvas import MapCanvas
    from transport import SocketWorker
    from task_state import PHASES, TASK_NAMES, blocker_text
    from mapping_panel import MappingPanel
    from diagnostics import DiagnosticPanel
    from underwater import UnderwaterPanel


class OperatorWindow(DiagnosticPanel, MappingPanel, QMainWindow):
    def __init__(self, url, *, mock=False, under_url=None):
        super().__init__()
        self.under_url, self.under_mock = under_url, mock
        self.under_panel = None
        self.setWindowTitle("水面 / 水下机器人控制台" if under_url else "F1 水面机器人控制台")
        self.resize(1500, 920)
        self.setFocusPolicy(Qt.StrongFocus)
        self.worker = SocketWorker(url, mock=mock, parent=self)
        self.seq, self.direction, self.speed, self.mode = 0, "STOP", 25, "BOTH"
        self.brush, self.thrust = False, False
        self.brush_percent, self.thrust_percent = 25, 25
        self._pressed_keys = set()
        self._clearing_manual = False
        self.last_telemetry_at = 0.
        self.telemetry = {}
        self._processing_incoming = False
        self._gateway_session = None
        self._retired_gateway_sessions = []
        self._telemetry_sequence = None
        self._last_received_at = 0.
        self._tracks = {}
        self._map_runtime_session = None
        self._map_received_at = 0.
        self._runtime_changed_at = 0.
        self._retired_runtime_sessions = []
        self.client_id = None
        self.capabilities = set()
        self.drive_output = {}
        self._control_identity = None
        self.pending_requests = {}
        self.initial_pose = None
        self.completed_results = {}
        self.seen_completions = set()
        self.initial_task_selected = False
        self.manual_locked = False
        self._key_directions = {
            Qt.Key_W: "FORWARD", Qt.Key_A: "LEFT", Qt.Key_S: "REVERSE", Qt.Key_D: "RIGHT"
        }
        QApplication.instance().installEventFilter(self)
        self._build()
        self.worker.message.connect(self._message)
        self.worker.status.connect(self._status)
        self.worker.manual_expired.connect(self._manual_intent_expired)
        self.worker.start()
        self.send_timer = QTimer(self)
        # GUI intent/liveness only; the transport owns the 20 Hz wire cadence.
        self.send_timer.timeout.connect(self._send_input)
        self.send_timer.start(50)
        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self._refresh_controls)
        self.state_timer.start(100)

    def _build(self):
        self.setStyleSheet("""
        QWidget { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; color:#172c3d; }
        QMainWindow, QWidget { background:#f2f5f7; }
        QFrame#card { background:#ffffff; border:1px solid #dde5ea; border-radius:12px; }
        QPushButton { background:#ffffff; border:1px solid #d5e0e5; border-radius:7px; padding:8px 14px; min-height:22px; }
        QPushButton:hover { border-color:#00877e; }
        QPushButton:disabled { background:#e8edef; color:#8a989f; border-color:#e0e6e9; }
        QPushButton#primary:disabled { background:#d9e5e4; color:#8a989f; }
        QPushButton#stop { background:#c74343; color:#ffffff; font-weight:bold; min-width:140px; }
        QPushButton#primary { background:#00877e; color:#ffffff; }
        QLabel#title { font-size:22px; font-weight:600; }
        QLabel#muted { color:#687c8c; }
        QTabWidget::pane { border:0; }
        QTabBar::tab { background:#eef2f4; border:1px solid #d5e0e5; border-bottom:0; border-radius:7px 7px 0 0; padding:10px 12px; }
        QTabBar::tab:selected { background:#2f80ed; color:#ffffff; border-color:#2f80ed; font-weight:600; }
        QPushButton:checked { background:#2f80ed; color:#ffffff; border-color:#2f80ed; }
        QPushButton:checked:hover { background:#256dcc; border-color:#256dcc; }
        """)
        root = QWidget(); self.setCentralWidget(root)
        layout = QVBoxLayout(root); layout.setContentsMargins(18, 14, 18, 14)
        top = QHBoxLayout()
        title = QLabel("水面 / 水下机器人控制台" if self.under_url else "F1  水面机器人控制台"); title.setObjectName("title")
        top.addWidget(title); top.addSpacing(20)
        self.connection = QLabel("连接中…"); self.connection.setObjectName("muted"); top.addWidget(self.connection)
        top.addStretch()
        self.stop_button = QPushButton("■  两台手动停止（空格）" if self.under_url else "■  停止手动输出"); self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(lambda: self._stop_all_manual("button")); top.addWidget(self.stop_button)
        layout.addLayout(top)
        body = QHBoxLayout(); body.setSpacing(14); layout.addLayout(body, 1)
        map_card = QFrame(); map_card.setObjectName("card"); map_layout = QVBoxLayout(map_card)
        map_title = QHBoxLayout(); map_title.addWidget(QLabel("作业地图")); map_title.addStretch(); self.data_age = QLabel("等待遥测"); self.data_age.setObjectName("muted"); map_title.addWidget(self.data_age); map_layout.addLayout(map_title)
        self.canvas = MapCanvas(); map_layout.addWidget(self.canvas, 1)
        self.wall_distance = QLabel("船体中心距左墙：无有效观测")
        self.wall_distance.setToolTip("base_link 原点到雷达拟合左墙线的垂直距离；不是滚刷间隙或任意最近障碍物距离")
        map_layout.addWidget(self.wall_distance)
        self.track_status = QLabel("轨迹：等待任务数据")
        self.track_status.setWordWrap(True); map_layout.addWidget(self.track_status)
        body.addWidget(map_card, 3)
        self.canvas.initial_pose_selected.connect(self._pose_selected)
        self.canvas.map_changed.connect(self._map_changed)
        side = QFrame(); side.setObjectName("card"); side_layout = QVBoxLayout(side); body.addWidget(side, 1)
        tabs = QTabWidget(); side_layout.addWidget(tabs)
        manual_content = QWidget(); self._build_manual(manual_content)
        manual = QScrollArea(); manual.setWidgetResizable(True); manual.setFrameShape(QFrame.NoFrame)
        manual.setFocusPolicy(Qt.NoFocus); manual.viewport().setFocusPolicy(Qt.NoFocus)
        manual.setWidget(manual_content); tabs.addTab(manual, "手动控制")
        task_content = QWidget(); self._build_task(task_content)
        task = QScrollArea(); task.setWidgetResizable(True); task.setFrameShape(QFrame.NoFrame)
        task.setWidget(task_content); tabs.addTab(task, "清洗任务")
        mapping_content = QWidget(); self._build_mapping(mapping_content)
        mapping = QScrollArea(); mapping.setWidgetResizable(True); mapping.setFrameShape(QFrame.NoFrame)
        mapping.setWidget(mapping_content); tabs.addTab(mapping, "建图与选图")
        self._build_diagnostics(tabs)
        if self.under_url:
            self.under_panel = UnderwaterPanel(self.under_url, mock=self.under_mock,
                                              log=self._log, parent=self)
            body.addWidget(self.under_panel, 1)
        camera = QFrame(); camera.setObjectName("card"); camera_layout = QVBoxLayout(camera); camera_layout.addWidget(QLabel("前向摄像头")); camera_label = QLabel("摄像头画面预留\n接入 WebRTC 后显示"); camera_label.setAlignment(Qt.AlignCenter); camera_label.setMinimumHeight(125); camera_label.setStyleSheet("background:#172a39;color:#a9bdc9;border-radius:8px;"); camera_layout.addWidget(camera_label); side_layout.addWidget(camera)
        self.status_bar = QLabel("等待网关状态"); self.status_bar.setObjectName("muted"); layout.addWidget(self.status_bar)

    def _build_manual(self, parent):
        box = QVBoxLayout(parent); box.setContentsMargins(4, 8, 4, 8)
        box.addWidget(QLabel("水面机器人 · W / A / S / D"))
        self.mode_label = QLabel("手动控制 · 网关自动管理会话"); self.mode_label.setWordWrap(True); box.addWidget(self.mode_label)
        self.speed_label = QLabel("主驱动档位（旧网关）")
        box.addWidget(self.speed_label)
        self.speed_controls = QWidget()
        percent_row = QHBoxLayout(self.speed_controls)
        percent_row.setContentsMargins(0, 0, 0, 0)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(25)
        self.speed_slider.setFocusPolicy(Qt.NoFocus)
        self.speed_slider.setToolTip("主驱动输出幅度；0 仅停止主电机，滚刷和第五电推独立控制")
        self.speed_value = QSpinBox()
        self.speed_value.setRange(0, 100)
        self.speed_value.setValue(25)
        self.speed_value.setSuffix(" %")
        self.speed_value.setKeyboardTracking(False)
        self.speed_value.setToolTip("整数 0–100；编辑前停止手动输出，按 Enter 后返回驾驶焦点")
        self.speed_slider.valueChanged.connect(self._set_speed)
        self.speed_value.valueChanged.connect(self._set_speed)
        self.speed_value.editingFinished.connect(self._finish_speed_edit)
        percent_row.addWidget(self.speed_slider, 1)
        percent_row.addWidget(self.speed_value)
        box.addWidget(self.speed_controls)
        self.speed_controls.hide()
        speeds = QHBoxLayout()
        self.speed_buttons = {}
        for value in (25, 50, 75):
            button = QPushButton(f"{value} 档"); button.setCheckable(True); button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(lambda checked, v=value: self._set_speed(v)); speeds.addWidget(button)
            self.speed_buttons[value] = button
            if value == 25: button.setChecked(True)
        box.addLayout(speeds)
        box.addWidget(QLabel("驱动轮组")); self.mode_combo = QComboBox(); self.mode_combo.addItems(["前轮", "四轮", "后轮"]); self.mode_combo.setCurrentIndex(1); self.mode_combo.currentTextChanged.connect(self._set_mode); box.addWidget(self.mode_combo)
        grid = QGridLayout(); self.drive_buttons = {}
        for row, col, label, direction in ((0,1,"↑ W","FORWARD"),(1,0,"← A","LEFT"),(1,1,"停","STOP"),(1,2,"D →","RIGHT"),(2,1,"↓ S","REVERSE")):
            button = QPushButton(label); self.drive_buttons[direction] = button; grid.addWidget(button,row,col); button.pressed.connect(lambda d=direction: self._set_direction(d)); button.released.connect(lambda: self._set_direction("STOP"))
        box.addLayout(grid); box.addWidget(QLabel("按住运行，松开停止主轮"))
        self.drive_feedback = QLabel("App 请求：STOP · 25 档\nF103 回报：等待新鲜状态")
        self.drive_feedback.setWordWrap(True)
        self.drive_feedback.setToolTip("显示请求与下位机接受的输出目标，不是电机实测转速、推力或船速")
        box.addWidget(self.drive_feedback)
        if self.under_url:
            surface_stop = QPushButton("■  水面手动停止")
            surface_stop.setFocusPolicy(Qt.NoFocus)
            surface_stop.clicked.connect(lambda: self._stop("surface_button"))
            box.addWidget(surface_stop)
        box.addStretch()
        box.addWidget(QLabel("辅助执行器输出（-100% 到 100%）"))
        brush_row = QHBoxLayout()
        self.brush_button = QPushButton("滚刷：关闭")
        self.brush_button.setCheckable(True)
        self.brush_button.toggled.connect(self._toggle_brush)
        self.brush_value = self._make_aux_input()
        self.brush_value.valueChanged.connect(lambda value: self._set_aux_percent("brush", value))
        brush_row.addWidget(self.brush_button, 1); brush_row.addWidget(self.brush_value)
        box.addLayout(brush_row)
        thrust_row = QHBoxLayout()
        self.thrust_button = QPushButton("第五电推：关闭")
        self.thrust_button.setCheckable(True)
        self.thrust_button.toggled.connect(self._toggle_thrust)
        self.thrust_value = self._make_aux_input()
        self.thrust_value.valueChanged.connect(lambda value: self._set_aux_percent("thrust", value))
        thrust_row.addWidget(self.thrust_button, 1); thrust_row.addWidget(self.thrust_value)
        box.addLayout(thrust_row)

    @staticmethod
    def _make_aux_input():
        value = QSpinBox()
        value.setRange(-100, 100)
        value.setValue(25)
        value.setSuffix(" %")
        value.setToolTip("输入 -100 到 100；启用后按此值下发")
        return value

    def _toggle_brush(self, enabled):
        self.brush = enabled
        self.brush_button.setText("滚刷：开启" if enabled else "滚刷：关闭")
        self._submit_manual_intent()

    def _toggle_thrust(self, enabled):
        self.thrust = enabled
        self.thrust_button.setText("第五电推：开启" if enabled else "第五电推：关闭")
        self._submit_manual_intent()

    def _build_task(self, parent):
        box = QVBoxLayout(parent); box.setContentsMargins(4, 8, 4, 8)
        box.addWidget(QLabel("清洗任务类型"))
        self.task_combo = QComboBox()
        for kind in ("rectangle", "l"):
            self.task_combo.addItem(TASK_NAMES[kind], kind)
        self.task_combo.currentIndexChanged.connect(self._refresh_controls)
        box.addWidget(self.task_combo)
        self.forward_boost_button = QPushButton("主电机提速：状态未知")
        self.forward_boost_button.setCheckable(True)
        self.forward_boost_button.setEnabled(False)
        self.forward_boost_button.clicked.connect(self._set_forward_boost)
        box.addWidget(self.forward_boost_button)
        self.forward_boost_status = QLabel("仅在任务开始前设置；距墙端 80 cm 内关闭前进辅助")
        self.forward_boost_status.setWordWrap(True)
        box.addWidget(self.forward_boost_status)
        self.map_status = QLabel("地图：等待加载")
        self.map_status.setWordWrap(True); box.addWidget(self.map_status)
        self.pose_button = QPushButton("设置初始位置和船头方向")
        self.pose_button.setCheckable(True)
        self.pose_button.toggled.connect(self._pick_pose)
        box.addWidget(self.pose_button)
        self.pose_hint = QLabel("点击地图并拖动箭头指定船头方向")
        self.pose_hint.setWordWrap(True); box.addWidget(self.pose_hint)
        self.publish_pose_button = QPushButton("发布初始位姿")
        self.publish_pose_button.clicked.connect(self._publish_pose)
        box.addWidget(self.publish_pose_button)
        self.localization_status = QLabel("定位：等待状态")
        self.localization_status.setWordWrap(True); box.addWidget(self.localization_status)
        self.conditions = QLabel("等待启动条件")
        self.conditions.setWordWrap(True); box.addWidget(self.conditions)
        self.task_status = QLabel("待启动")
        self.task_status.setWordWrap(True); box.addWidget(self.task_status)
        self.task_reason = QLabel("")
        self.task_reason.setWordWrap(True); box.addWidget(self.task_reason)
        self.start_details = QLabel("")
        self.start_details.setWordWrap(True); box.addWidget(self.start_details)
        self.task_buttons = {}
        task_actions = QGridLayout()
        for index, (label, action) in enumerate((("开始清洗", "start"), ("暂停任务", "pause"),
                                                 ("取消任务", "cancel"))):
            button = QPushButton(label)
            if action == "start":
                button.setObjectName("primary")
            elif action == "cancel":
                button.setObjectName("stop")
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, a=action: self._task(a))
            button.setToolTip({
                "start": "启动新的清洗任务",
                "pause": "暂停关闭输出并保留断点；继续前重新检查定位和控制权",
                "stop": "关闭输出并保留记录；再次开始创建新任务",
                "cancel": "关闭输出并清除当前边关联；历史轨迹保留",
            }[action])
            task_actions.addWidget(button, index//2, index % 2)
            self.task_buttons[action] = button
        box.addLayout(task_actions)
        self.result_label = QLabel("")
        self.result_label.setWordWrap(True); box.addWidget(self.result_label)
        box.addStretch()
        note = QLabel("人工暂停后可继续原任务。\n取消结束本次任务；再次开始创建新任务。\n故障停机需取消，排除故障后重新开始。\n顶部红色按钮仅停止手动输出。")
        note.setWordWrap(True); box.addWidget(note)
        self.pose_button.setEnabled(False)
        self.publish_pose_button.setEnabled(False)

    def _pick_pose(self, enabled):
        if enabled:
            self._stop("set_initial_pose")
        self.initial_pose = None
        self.canvas.set_picking(enabled)
        self.pose_hint.setText("在地图上按住并拖动，箭头指向船头" if enabled else "点击地图并拖动箭头指定船头方向")
        self._refresh_controls()

    def _pose_selected(self, pose):
        self.initial_pose = pose
        self.pose_hint.setText(f"待发布：x={pose['x']:.2f} m，y={pose['y']:.2f} m，船头={math.degrees(pose['yaw']):.1f}°")
        self._refresh_controls()

    def _map_changed(self):
        self.initial_pose = None
        self.pose_hint.setText("地图已更新，请重新点选初始位姿")
        self._refresh_controls()

    def _publish_pose(self):
        if not self.publish_pose_button.isEnabled() or not self.initial_pose:
            return
        self._send_request("initial_pose", **self.initial_pose)
        self.pose_button.setChecked(False)

    def _set_forward_boost(self, checked):
        if not self.forward_boost_button.isEnabled():
            return
        self._clear_manual()
        self._send_request("task", action="set_forward_boost",
                           task_type=self.task_combo.currentData(), enabled=bool(checked))

    def _send_request(self, op, **fields):
        packet = self._packet(op, request_id=uuid.uuid4().hex, **fields)
        if self.worker.send_packet(packet):
            self.pending_requests[packet["request_id"]] = (time.monotonic(), packet)
            self.status_bar.setText("操作已发送，等待机器人确认")
        else:
            self.status_bar.setText("操作未发送：队列已满")
        self._refresh_controls()

    def _refresh_controls(self, *args):
        if self._processing_incoming:
            return
        if not hasattr(self, "task_buttons"):
            return
        fresh = bool(self.last_telemetry_at) and time.monotonic()-self.last_telemetry_at < .8
        packet = self.telemetry if fresh else {}
        supported = {"task_selection", "initial_pose", "start_conditions"} <= self.capabilities
        owner = packet.get("active_client")
        operator = owner is None or owner == self.client_id
        tasks = packet.get("tasks", {})
        kind = self.task_combo.currentData()
        task = tasks.get(kind, {})
        task_fresh = task.get("fresh") is True
        local_pending = bool(self.pending_requests)
        busy = packet.get("task_busy", False)
        selecting = self.pose_button.isChecked()
        runtime = packet.get("runtime") or {}
        typing = isinstance(QApplication.focusWidget(), (QLineEdit, QAbstractSpinBox))
        drive_block = self._drive_output_block_reason()
        lock_reasons = tuple(label for condition, label in (
            (busy, "自动任务占用"), (local_pending, "操作等待回复"),
            (selecting, "正在选择初始位姿"), (not fresh, "遥测未收到或超过 800 ms"),
            (not operator, "其他操作端占用"), (runtime.get("busy"), "建图/选图操作处理中"),
            (typing, "输入框持有焦点"), (bool(drive_block), drive_block)) if condition)
        locked = bool(lock_reasons)
        self._log_change("manual_lock", lock_reasons, "App",
                         "手动按钮已禁用" if locked else "手动按钮已可用",
                         {"reasons": lock_reasons,
                          "telemetry_age_ms": round((time.monotonic()-self.last_telemetry_at)*1000, 1)
                          if self.last_telemetry_at else None})
        self.mode_label.setText("手动不可用：" + "；".join(lock_reasons) if locked
                                else "手动控制 · 网关自动管理会话")
        self._refresh_drive_controls(fresh)
        if not fresh:
            self.diagnostic_summary.setText("遥测未收到或已过期；先前桥状态不能代表当前状态")
        self._refresh_output_diagnostics(self.telemetry, fresh)
        was_locked = self.manual_locked
        self.manual_locked = locked
        if locked:
            self.worker.clear_manual_intent()
        if locked and not was_locked:
            self._clear_manual()
        # Qt otherwise moves focus from a disabled button into the next spinbox,
        # turning a temporary stale-data lock into a persistent text-input lock.
        disabling = list(self.drive_buttons.values()) + [self.brush_button, self.thrust_button]
        if locked and QApplication.focusWidget() in disabling:
            self.setFocus(Qt.OtherFocusReason)
        for button in self.drive_buttons.values():
            button.setEnabled(not locked)
        self.brush_button.setEnabled(not locked)
        self.thrust_button.setEnabled(not locked and packet.get("state", {}).get("allow_fifth_thruster") is True)
        self.task_combo.setEnabled(fresh and operator and not busy and not local_pending and not selecting)
        boost_supported = "forward_boost" in self.capabilities
        boost_known = fresh and task_fresh and type(task.get("forward_boost_enabled")) is bool
        boost_enabled = boost_known and task["forward_boost_enabled"]
        self.forward_boost_button.setChecked(boost_enabled)
        self.forward_boost_button.setText("主电机提速：" + (
            ("开启" if boost_enabled else "关闭") if boost_known else "状态未知"))
        self.forward_boost_button.setEnabled(bool(boost_supported and boost_known and operator
            and not busy and not local_pending and not selecting
            and task.get("can_set_forward_boost") is True))
        self.forward_boost_button.setToolTip(task.get("forward_boost_setting_error") or
            "只在任务开始前设置；运行中和暂停期间锁定。四电推以新标度 25% 前进辅助，纠偏优先。")
        boost_text = {
            "disabled": "提速已关闭，沿墙前进依靠滚刷",
            "active": "已启用，正在输出新标度 25% 前进辅助",
            "inactive_task": "已启用，等待开始下一次任务",
            "endpoint_cutoff": "已启用，已进入近端区，本墙段保持滚刷推进",
            "heading_correction": "已启用，航向纠偏中，前进辅助关闭",
            "waiting_follow": "已启用，等待稳定沿墙条件",
        }.get(task.get("forward_boost_reason"), "等待提速状态")
        if not boost_supported:
            boost_text = "提速功能需要更新 Nano 网关和清洗控制器"
        elif not boost_known:
            boost_text = "提速状态未确认，不能修改"
        if any(p.get("action") == "set_forward_boost" for _, p in self.pending_requests.values()):
            boost_text = "提速设置尚未确认，等待机器人回复"
        self.forward_boost_status.setText(boost_text)
        for index in range(self.task_combo.count()):
            option = tasks.get(self.task_combo.itemData(index), {})
            item = self.task_combo.model().item(index)
            item.setEnabled(option.get("fresh") is True)
            item.setToolTip("; ".join(b.get("message", "") for b in option.get("start_blockers", [])))
        self.pose_button.setEnabled(fresh and supported and operator and not local_pending and packet.get("can_set_initial_pose") is True)
        self.publish_pose_button.setEnabled(self.pose_button.isEnabled() and self.initial_pose is not None)
        if not self.pose_button.isEnabled() and selecting:
            self.pose_button.setChecked(False)
        self.task_buttons["start"].setEnabled(fresh and supported and operator and not local_pending and not selecting and task.get("can_start") is True)
        active = packet.get("active_tasks", [])
        pending_start = any(p.get("action") in ("start", "resume") for _, p in self.pending_requests.values())
        stopping = any(p.get("action") in ("pause", "stop", "cancel") for _, p in self.pending_requests.values())
        paused = task_fresh and task.get("phase") == "PAUSED"
        self.task_buttons["pause"].setText("继续任务" if paused else "暂停任务")
        resumable = ("task_resume" in self.capabilities and task.get("can_resume") is True)
        self.task_buttons["pause"].setEnabled(fresh and operator and not local_pending and
            (resumable if paused else bool(active) and task.get("phase") != "ACQUIRING"))
        self.task_buttons["pause"].setToolTip(
            "; ".join(b.get("message", "") for b in task.get("resume_blockers", [])) if paused
            else "关闭输出并保留当前任务，等待继续或取消")
        cancellable = bool(active or pending_start or paused)
        self.task_buttons["cancel"].setEnabled(fresh and operator and cancellable and not stopping)
        loc = packet.get("localization", {})
        map_id = loc.get("map_id", "")
        self.map_status.setText("地图：" + (map_id.rsplit("/", 1)[-1] if map_id else "未确认"))
        self.map_status.setToolTip(map_id)
        same_map = bool(map_id and map_id == task.get("map_id"))
        # Retain last known history on disconnect, without reusing it for controls.
        overlay_task = self.telemetry.get("tasks", {}).get(kind, {})
        overlay_map = self.telemetry.get("localization", {}).get("map_id")
        overlay_matches = bool(overlay_map and overlay_map == overlay_task.get("map_id")
                               and self._map_matches_runtime())
        self.canvas.set_task_overlay(overlay_task if overlay_matches else {}, live=fresh and task_fresh)
        distance = task.get("center_wall_distance_m")
        valid_wall = (fresh and task_fresh and same_map and task.get("wall_fresh") is True
                      and task.get("wall_valid") is True and isinstance(distance, (int, float))
                      and math.isfinite(distance) and distance > 0)
        self.wall_distance.setText(f"船体中心距左墙：{distance:.3f} m（{distance*100:.1f} cm）"
                                  if valid_wall else "船体中心距左墙：无有效观测")
        track = overlay_task.get("actual_track", {}) if overlay_matches else {}
        if track.get("run_id") != overlay_task.get("run_id"):
            track = {}
        count = track.get("point_count", 0)
        suffix = "（已截断，仅显示最近轨迹）" if track.get("truncated") else ""
        self.track_status.setText(f"{TASK_NAMES[kind]} · 本次任务历史轨迹：{count} 点{suffix}"
                                  + (" · 当前轨迹数据已过期" if not fresh or not task_fresh else "")
                                  if "task_tracks" in self.capabilities else "轨迹功能需要升级网关")
        if not loc.get("fresh"):
            text = "定位：状态未收到或已过期"
        elif not loc.get("initial_pose_received"):
            text = "定位：请设置初始位姿"
        elif loc.get("ready"):
            text = "定位：已稳定，可以检查启动条件"
        elif loc.get("state") == "PAUSED":
            text = "定位：异常，请重新设置初始位姿"
        else:
            elapsed, required = loc.get("stable_elapsed_sec") or 0., loc.get("stable_sec", 1.)
            text = f"定位：等待稳定（{min(elapsed, required):.1f} / {required:.1f} 秒）"
        text += "\n雷达：" + ("正常" if loc.get("scan_fresh") else "未就绪")
        text += "　位姿：" + ("正常" if loc.get("pose_fresh") else "未就绪")
        self.localization_status.setText(text)
        self.localization_status.setToolTip(str(loc.get("reason", "")))
        blockers = task.get("resume_blockers" if paused else "start_blockers", [])
        condition = blocker_text(blockers) or ("可以继续原任务" if paused else "启动条件已满足")
        if not fresh:
            condition = "连接中断或遥测过期，禁止开始任务"
            self.canvas.set_telemetry({"pose": None})
        elif not supported:
            condition = "请升级 Nano 网关以使用清洗任务与初始位姿功能"
        elif not operator:
            condition = "其他操作端持有控制权限"
        elif local_pending:
            condition = "操作处理中，等待机器人确认"
        self.conditions.setText(condition)
        self.conditions.setToolTip("\n".join(b.get("message", "") for b in blockers))
        self.data_age.setText("实时遥测" if fresh else "遥测已过期")
        phase = task.get("phase", "")
        mode = "实际输出" if task.get("output_enabled") else "预览模式（不驱动硬件）"
        phase_label = ("故障停机（请先取消任务）" if phase == "PAUSED" and task.get("pause_kind") == "fault"
                       else PHASES.get(phase, phase or "等待任务状态"))
        self.task_status.setText((phase_label + "\n" + mode) if task_fresh else "任务状态未知")
        self.task_status.setStyleSheet("color:#00877e;font-weight:bold;" if task_fresh and phase == "COMPLETE" else "")
        self.task_reason.setText(str(task.get("reason", "")) if task_fresh else "")
        details = ""
        if task_fresh and task.get("start_edge_id"):
            details = f"起始墙：{task['start_edge_id']} · 当前墙：{task.get('current_edge_id') or '—'}"
            region = task.get("start_region") or {}
            point = region.get("reference_wall_point")
            if point and len(point) == 2:
                details += f"\n起始参考点：({point[0]:.2f}, {point[1]:.2f}) m"
            if task.get("returning_to_start"):
                details += "\n正在补齐起始边前半段"
            if task.get("completion_scope") in ("remaining_open_route", "remaining_l_route"):
                details += "\n开放墙链仅清洗起始位置之后的路线；清洗全链请从首边入口开始"
        self.start_details.setText(details)
        result = self.completed_results.get(kind)
        self.result_label.setText(result[1] if result else "")
        self._refresh_mapping(fresh, operator, local_pending)
        for request_id, (when, _) in self.pending_requests.items():
            if time.monotonic()-when > 5:
                self.status_bar.setText("操作结果尚未确认；不会自动重发。请查看任务状态或取消任务。")
                break

    def _set_speed(self, value):
        continuous = "continuous_drive_percent_v1" in self.capabilities
        if type(value) is not int or not 0 <= value <= 100 or (not continuous and value not in (25, 50, 75)):
            return
        self.speed = value
        for widget in (self.speed_slider, self.speed_value):
            blocked = widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(blocked)
        for speed, button in self.speed_buttons.items():
            button.setChecked(speed == value)
        if value == 0:
            # Do not retain a held key/button that a later increase could revive.
            # Auxiliary actuators are deliberately independent of main output.
            self.direction = "STOP"
            self._pressed_keys.clear()
            for button in self.drive_buttons.values():
                button.setDown(False)
        self._refresh_drive_feedback()
        self._submit_manual_intent()

    def _finish_speed_edit(self):
        # Enter commits the integer and returns WASD to the main window. A click
        # into another editor may finish this edit too; do not steal that focus.
        if self.speed_value.hasFocus() or self.speed_value.lineEdit().hasFocus():
            self.setFocus(Qt.OtherFocusReason)
        self._refresh_controls()

    def _drive_output_block_reason(self):
        if "continuous_drive_percent_v1" not in self.capabilities:
            return ""
        output = self.drive_output
        if (not isinstance(output, dict)
                or any(type(output.get(key)) is not int or output[key] != expected
                       for key, expected in (("min", 0), ("max", 100), ("step", 1),
                                             ("firmware_protocol_version", 3)))):
            return "主驱动输出能力未确认，需要 F103 协议 v3"
        if output.get("available") is not True:
            return str(output.get("reason") or "F103 协议 v3 未就绪，主驱动输出不可用")
        state = self.telemetry.get("state", {})
        if self.telemetry.get("control_fresh") is not True or not self._firmware_ctrl_fresh(state):
            return "F103 控制反馈未收到或已过期"
        if state.get("source") != "NANO":
            return f"控制源为 {state.get('source', '未知')}，等待 Nano 控制权"
        if (state.get("state") not in ("READY", "ACTIVE") or state.get("armed") is not True
                or state.get("result") != "OK"):
            return "F103 尚未完成控制握手：" + str(state.get("reason") or state.get("state") or "未知")
        ack_age = self._reported_age_sec(state, "ack_age_sec")
        if ack_age is None or ack_age >= .4:
            return "F103 ACK 确认未收到或已过期"
        return ""

    def _refresh_drive_controls(self, fresh):
        continuous = "continuous_drive_percent_v1" in self.capabilities
        if not continuous and self.speed not in (25, 50, 75):
            self.speed = 25
        for widget in (self.speed_slider, self.speed_value):
            # A redundant setValue also replaces uncommitted spinbox text.
            # Periodic telemetry must not interrupt an operator typing a value.
            if widget.value() != self.speed:
                blocked = widget.blockSignals(True)
                widget.setValue(self.speed)
                widget.blockSignals(blocked)
        if not continuous and (self.speed_value.hasFocus() or self.speed_value.lineEdit().hasFocus()):
            self.setFocus(Qt.OtherFocusReason)
        self.speed_controls.setVisible(continuous)
        self.speed_label.setText("主驱动输出幅度（0–100%）" if continuous else "主驱动档位（旧网关：25 / 50 / 75）")
        for value, button in self.speed_buttons.items():
            button.setText(f"{value}%" if continuous else f"{value} 档")
            button.setChecked(value == self.speed)
        self._refresh_drive_feedback(fresh)

    def _refresh_drive_feedback(self, fresh=None):
        if fresh is None:
            fresh = bool(self.last_telemetry_at) and time.monotonic()-self.last_telemetry_at < .8
        unit = "%" if "continuous_drive_percent_v1" in self.capabilities else " 档"
        text = f"App 请求：{self.direction} · {self.speed}{unit} · {self.mode}"
        state = self.telemetry.get("state", {})
        if (not fresh or self.telemetry.get("control_fresh") is not True
                or not self._firmware_ctrl_fresh(state)):
            text += "\nF103 回报：状态未收到或已过期"
        else:
            source = state.get("source", "未知")
            speed = state.get("speed_percent", "—")
            scale = (f"遥控器 {speed} 档（独立标度）" if source == "RC" else
                     f"{speed}%" if state.get("firmware_protocol_version") == 3 else f"{speed} 档")
            text += f"\nF103 回报：{source} · {state.get('direction', '—')} · {scale}"
        self.drive_feedback.setText(text)

    def _set_mode(self, text):
        self.mode = {"前轮": "FRONT", "四轮": "BOTH", "后轮": "BACK"}[text]
        self._submit_manual_intent()

    def _set_aux_percent(self, kind, value):
        setattr(self, kind + "_percent", value)
        self._submit_manual_intent()

    def _set_direction(self, direction):
        if not self.manual_locked:
            self.direction = direction if self.speed else "STOP"
            self._refresh_drive_feedback()
            self._submit_manual_intent()
    def _packet(self, op, **fields):
        self.seq += 1; return {"op": op, "seq": self.seq, **fields}
    def _send_input(self):
        now = time.monotonic()
        if self._last_send_tick is not None:
            gap_ms = (now - self._last_send_tick) * 1000
            self._log_change("send_delay", gap_ms >= 200, "App",
                             "手动意图刷新定时器延迟" if gap_ms >= 200 else "手动意图刷新定时器正常",
                             {"interval_ms": round(gap_ms, 1), "expected_ms": 50})
        self._last_send_tick = now
        self._submit_manual_intent()

    def _submit_manual_intent(self):
        # All Qt state is read on the GUI thread. The sender only sees a copy.
        if self._clearing_manual:
            return
        fresh = bool(self.last_telemetry_at) and time.monotonic() - self.last_telemetry_at < .8
        if self.manual_locked or not fresh or self._drive_output_block_reason():
            # Clear the GUI copy too: a telemetry recovery arriving before the
            # 100 ms controls timer must not restore a previously held action.
            self._clear_manual()
            return
        accepted = self.worker.update_manual_intent(dict(
            op="manual_input", direction=self.direction,
            speed_percent=self.speed, mode=self.mode,
            brush_enabled=self.brush,
            brush_percent=self.brush_percent if self.brush else 0,
            thrust_enabled=self.thrust,
            thrust_percent=self.thrust_percent if self.thrust else 0))
        if not accepted:
            if self.worker.manual_intent_expired:
                self._manual_intent_expired()
            else:
                # A reconnect must not retain a held action awaiting handshake.
                self._clear_manual()

    def _manual_intent_expired(self):
        # A queued signal from a retired connection may arrive after reconnect.
        if not self.worker.manual_intent_expired:
            return
        self._stop("gui_input_expired")
        self.worker.acknowledge_manual_expiry()

    def _clear_manual(self):
        self.worker.clear_manual_intent()
        self._clearing_manual = True
        try:
            self.direction = "STOP"
            self._pressed_keys.clear()
            self.brush_button.setChecked(False)
            self.thrust_button.setChecked(False)
            self.brush = self.thrust = False
            self._refresh_drive_feedback()
        finally:
            self._clearing_manual = False

    def _stop(self, reason):
        self._log("App", reason, {"direction": self.direction, "brush": self.brush, "thrust": self.thrust})
        self._clear_manual()
        self.worker.send_packet(self._packet("manual_stop", reason=reason))

    def _stop_all_manual(self, reason):
        self._stop(reason)
        if self.under_panel is not None:
            self.under_panel.stop(reason)

    def _task(self, action):
        if not self.task_buttons[action].isEnabled():
            return
        self._clear_manual()
        active = self.telemetry.get("active_tasks", [])
        kind = active[0] if action != "start" and len(active) == 1 else self.task_combo.currentData()
        if action == "pause" and self.telemetry.get("tasks", {}).get(kind, {}).get("phase") == "PAUSED":
            action = "resume"
        fields = {}
        if action == "resume":
            fields["run_id"] = self.telemetry.get("tasks", {}).get(kind, {}).get("run_id", "")
        self._send_request("task", action=action, task_type=kind, **fields)

    def _status(self, text):
        self._log_change("connection", text, "连接", text)
        self.connection.setText(text)
        if text.startswith("连接失败"):
            self.last_telemetry_at = 0.
            self._clear_manual()
            self.pending_requests.clear()
            self.capabilities.clear()
            self.drive_output = {}
            self._control_identity = None
        self._refresh_controls()

    def _refresh_key_direction(self):
        self.direction = "STOP"
        if self.speed == 0:
            self._pressed_keys.clear()
        for key in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D):
            if key in self._pressed_keys:
                self.direction = self._key_directions[key]
                break
        self._refresh_drive_feedback()
        self._submit_manual_intent()

    def _accept_gateway_session(self, session):
        if not session or session == self._gateway_session:
            return True
        if session in self._retired_gateway_sessions:
            return False
        if self._gateway_session:
            self._retired_gateway_sessions.append(self._gateway_session)
            self._retired_gateway_sessions = self._retired_gateway_sessions[-8:]
        self._gateway_session = session
        self._telemetry_sequence = None
        self._last_received_at = self.last_telemetry_at = 0.
        self._tracks.clear()
        self._retired_runtime_sessions.clear()
        self._runtime_changed_at = 0.
        self.telemetry = {}
        self._control_identity = None
        self.canvas.set_task_overlay({}, live=False)
        self.canvas.set_telemetry({"pose": None})
        self._clear_manual()
        return True

    def _retire_runtime_session(self, session):
        if session and session not in self._retired_runtime_sessions:
            self._retired_runtime_sessions.append(session)
            self._retired_runtime_sessions = self._retired_runtime_sessions[-16:]

    def _map_matches_runtime(self):
        return (not self._map_runtime_session or not self.runtime_session
                or self._map_runtime_session == self.runtime_session)

    def _accept_map_packet(self, packet):
        mapping = packet.get("map") or {}
        session = packet.get("runtime_session_id") or mapping.get("runtime_session_id")
        received = packet.get("_received_monotonic", time.monotonic())
        if (not isinstance(received, (int, float)) or not math.isfinite(received)
                or received <= 0):
            return False
        if (session in self._retired_runtime_sessions
                or received < self._map_received_at
                or (session and session != self.runtime_session
                    and received < self._runtime_changed_at)):
            return False
        if session and session != self._map_runtime_session:
            self._retire_runtime_session(self._map_runtime_session)
        self._map_runtime_session = session
        self._map_received_at = received
        if not self._map_matches_runtime():
            self.canvas.set_task_overlay({}, live=False)
            self.canvas.set_telemetry({"pose": None})
        return True

    def _attach_tracks(self, packet, *, record_embedded=True):
        result = dict(packet)
        tasks = {}
        for kind, value in packet.get("tasks", {}).items():
            task = dict(value)
            task.pop("expected_track", None)
            embedded = task.get("actual_track")
            if record_embedded and isinstance(embedded, dict):
                self._tracks[kind] = embedded
            track = self._tracks.get(kind)
            if (track and track.get("map_id") == task.get("map_id")
                    and track.get("run_id") == task.get("run_id")):
                task["actual_track"] = track
            elif (not isinstance(embedded, dict)
                  or embedded.get("map_id") != task.get("map_id")
                  or embedded.get("run_id") != task.get("run_id")):
                task.pop("actual_track", None)
            tasks[kind] = task
        result["tasks"] = tasks
        result.pop("scan", None)
        return result

    def eventFilter(self, obj, event):
        if event.type() == QEvent.TouchCancel and hasattr(self, "brush_button"):
            self._stop_all_manual("touch_cancelled")
        if event.type() == QEvent.FocusIn and isinstance(obj, (QLineEdit, QAbstractSpinBox)) and hasattr(self, "brush_button"):
            self._stop_all_manual("text_input")
        if event.type() == QEvent.WindowDeactivate and obj is self:
            self._stop_all_manual("window_deactivated")
        if self.isActiveWindow() and event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            if isinstance(QApplication.focusWidget(), (QLineEdit, QAbstractSpinBox)):
                return super().eventFilter(obj, event)
            key = event.key()
            if self.under_panel is not None and self.under_panel.key(
                    key, event.type() == QEvent.KeyPress, event.isAutoRepeat()):
                return True
            if key in self._key_directions:
                # Queued auto-repeat events must not revive an expired hold.
                if event.isAutoRepeat():
                    return True
                if self.manual_locked:
                    return True
                if event.type() == QEvent.KeyPress and self.speed != 0:
                    self._pressed_keys.add(key)
                else:
                    self._pressed_keys.discard(key)
                self._refresh_key_direction()
                return True
            if event.type() == QEvent.KeyPress and key == Qt.Key_Space and not event.isAutoRepeat():
                self._stop_all_manual("keyboard")
                return True
        return super().eventFilter(obj, event)
    def _message(self, packet):
        generation = packet.get("_connection_generation")
        if (packet.get("type") in ("hello", "telemetry", "tracks", "map")
                and generation is not None and generation != self.worker.connection_generation):
            return
        if packet.get("type") == "incoming_ready":
            self._processing_incoming = True
            try:
                for incoming in self.worker.take_incoming():
                    self._message(incoming)
            finally:
                self._processing_incoming = False
                self._refresh_controls()
            return
        if packet.get("type") == "diagnostics":
            self._diagnostic_history(packet.get("diagnostics") or {})
            return
        if packet.get("type") == "tracks":
            if not self._accept_gateway_session(packet.get("session_id")):
                return
            for kind, track in packet.get("tasks", {}).items():
                if kind in ("rectangle", "l") and isinstance(track, dict):
                    self._tracks[kind] = track
            self.telemetry = self._attach_tracks(self.telemetry, record_embedded=False)
            self._refresh_controls()
            return
        if packet.get("type") == "hello":
            if packet.get("session_id") in self._retired_gateway_sessions:
                return
            self._clear_manual()
            activated = self.worker.activate_manual_session(generation)
            if generation is not None and not activated:
                return
            if not self._accept_gateway_session(packet.get("session_id")):
                return
            self.last_telemetry_at = 0.
            self.client_id = packet.get("client_id")
            self.capabilities = set(packet.get("capabilities", []))
            self.drive_output = packet.get("drive_output") or {}
            self._control_identity = None
            self._log("连接", "网关握手完成", {"diagnostic_history": "diagnostic_history" in self.capabilities})
            if "diagnostic_history" not in self.capabilities:
                self._log("App", "网关未声明诊断历史支持；仅记录本地事件与收到的桥状态，完整日志需更新 Nano 网关")
            if "map_catalog" in self.capabilities:
                self._send_request("mapping", action="list")
        elif packet.get("type") == "map":
            if not self._accept_map_packet(packet):
                return
            self.canvas.set_telemetry(packet)
        elif packet.get("type") == "telemetry":
            stream = packet.get("stream") or {}
            if not self._accept_gateway_session(stream.get("session")):
                return
            now = time.monotonic()
            received = packet.get("_received_monotonic", now)
            if (not isinstance(received, (int, float)) or not math.isfinite(received)
                    or not 0 < received <= now or received < self._last_received_at):
                return
            sequence = stream.get("sequence")
            if (isinstance(sequence, int) and self._telemetry_sequence is not None
                    and sequence <= self._telemetry_sequence):
                return
            server_queue_ms = stream.get("server_queue_ms", 0.)
            if (not isinstance(server_queue_ms, (int, float))
                    or not math.isfinite(server_queue_ms) or server_queue_ms < 0):
                return
            if isinstance(sequence, int):
                self._telemetry_sequence = sequence
            self._last_received_at = received
            packet = self._attach_tracks(packet)
            if "continuous_drive_percent_v1" in self.capabilities:
                state = packet.get("state", {})
                identity = tuple(state.get(key) for key in ("connection_id", "source", "epoch"))
                if self._control_identity is not None and identity != self._control_identity:
                    # Retire held input even if RC takeover/reconnect completed
                    # between two GUI packets and the bridge is already READY.
                    self._stop("control_identity_changed")
                self._control_identity = identity
            self._diagnostic_telemetry(packet)
            self.telemetry = packet
            self.drive_output = packet.get("drive_output") or {}
            self.last_telemetry_at = received - server_queue_ms / 1000.
            self._mapping_telemetry(packet)
            self.canvas.set_telemetry(packet if self._map_matches_runtime() else {"pose": None})
            state = packet.get("state", {})
            self.status_bar.setText(f"设备：{state.get('state','—')} · 来源：{state.get('source','—')} · {state.get('reason','')}")
            active = packet.get("active_tasks", [])
            if len(active) == 1:
                index = self.task_combo.findData(active[0])
                if index >= 0:
                    self.task_combo.setCurrentIndex(index)
                    self.initial_task_selected = True
            for kind, task in packet.get("tasks", {}).items():
                if not task.get("fresh"):
                    continue
                if not self.initial_task_selected:
                    index = self.task_combo.findData(kind)
                    if index >= 0:
                        self.task_combo.setCurrentIndex(index)
                        self.initial_task_selected = True
                run_id = task.get("run_id")
                previous = self.completed_results.get(kind)
                if previous and run_id and previous[0] != run_id:
                    self.completed_results.pop(kind)
                if task.get("phase") == "COMPLETE" and run_id:
                    ns = task.get("completed_at_ns")
                    when = datetime.fromtimestamp(ns/1e9).strftime("%Y-%m-%d %H:%M:%S") if ns else "时间未提供"
                    preview = "预览任务完成" if not task.get("output_enabled") else "任务完成"
                    if task.get("completion_scope") in ("remaining_open_route", "remaining_l_route"):
                        result = "本次剩余开放墙链执行完成；不返回、不补洗"
                    elif task.get("completion_scope") == "rectangle_route":
                        result = "矩形路线及起始边补齐执行完成"
                    else:
                        result = "本次路线执行完成"
                    if task.get("coverage_complete") is True:
                        result += "\n记录的本次要求区间已覆盖"
                    elif task.get("coverage_complete") is False:
                        result += "\n覆盖记录仍有缺口，未追加补洗"
                    text = f"{TASK_NAMES.get(kind, kind)}：{preview}\n完成时间：{when}\n{result}"
                    self.completed_results[kind] = (run_id, text)
                    if (kind, run_id) not in self.seen_completions:
                        self.seen_completions.add((kind, run_id))
                        self.status_bar.setText(text.replace("\n", " · "))
        elif packet.get("type") == "reply":
            if packet.get("message") or packet.get("success") is False:
                self._log_change("reply", (packet.get("op"), packet.get("success"), packet.get("message")),
                                 "网关回复", str(packet.get("message", "操作未完成")),
                                 {"op": packet.get("op"), "success": packet.get("success")})
            if not packet.get("pending"):
                self.pending_requests.pop(packet.get("request_id"), None)
            self._mapping_reply(packet)
            if packet.get("message"):
                prefix = "操作反馈：" if packet.get("success") else "操作未完成："
                self.status_bar.setText(prefix + str(packet["message"]))
        elif packet.get("type") == "error":
            self._log_change("error", packet.get("message"), "网关错误", str(packet.get("message", "未知错误")))
            self.status_bar.setText("网关错误：" + str(packet.get("message", "未知错误")))
        self._refresh_controls()

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self)
        self._stop("window_closed")
        if self.under_panel is not None:
            self.under_panel.shutdown()
        self.worker.close(); self.worker.wait(1000); event.accept()
