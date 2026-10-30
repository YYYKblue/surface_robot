"""Main operator console window and operator actions."""
import math
import time
import uuid
from datetime import datetime
try:
    from PySide6.QtCore import QEvent, QTimer, Qt
    from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                   QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                   QSpinBox, QTabWidget, QVBoxLayout, QWidget, QScrollArea)
except ImportError:
    from PyQt5.QtCore import QEvent, QTimer, Qt
    from PyQt5.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                 QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                 QSpinBox, QTabWidget, QVBoxLayout, QWidget, QScrollArea)

try:
    from .map_canvas import MapCanvas
    from .transport import SocketWorker
    from .task_state import PHASES, TASK_NAMES, blocker_text
except ImportError:
    from map_canvas import MapCanvas
    from transport import SocketWorker
    from task_state import PHASES, TASK_NAMES, blocker_text


class OperatorWindow(QMainWindow):
    def __init__(self, url, *, mock=False):
        super().__init__()
        self.setWindowTitle("F1 水面机器人控制台")
        self.resize(1500, 920)
        self.setFocusPolicy(Qt.StrongFocus)
        self.worker = SocketWorker(url, mock=mock, parent=self)
        self.seq, self.direction, self.speed, self.mode = 0, "STOP", 25, "BOTH"
        self.brush, self.thrust = False, False
        self.brush_percent, self.thrust_percent = 25, 25
        self._pressed_keys = set()
        self.last_telemetry_at = 0.
        self.telemetry = {}
        self.client_id = None
        self.capabilities = set()
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
        self.worker.start()
        self.send_timer = QTimer(self)
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
        QTabBar::tab { background:#eef2f4; border:1px solid #d5e0e5; border-bottom:0; border-radius:7px 7px 0 0; padding:10px 22px; }
        QTabBar::tab:selected { background:#2f80ed; color:#ffffff; border-color:#2f80ed; font-weight:600; }
        QPushButton:checked { background:#2f80ed; color:#ffffff; border-color:#2f80ed; }
        QPushButton:checked:hover { background:#256dcc; border-color:#256dcc; }
        """)
        root = QWidget(); self.setCentralWidget(root)
        layout = QVBoxLayout(root); layout.setContentsMargins(18, 14, 18, 14)
        top = QHBoxLayout()
        title = QLabel("F1  水面机器人控制台"); title.setObjectName("title")
        top.addWidget(title); top.addSpacing(20)
        self.connection = QLabel("连接中…"); self.connection.setObjectName("muted"); top.addWidget(self.connection)
        top.addStretch()
        self.stop_button = QPushButton("■  停止手动输出"); self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(lambda: self._stop("button")); top.addWidget(self.stop_button)
        layout.addLayout(top)
        body = QHBoxLayout(); body.setSpacing(14); layout.addLayout(body, 1)
        map_card = QFrame(); map_card.setObjectName("card"); map_layout = QVBoxLayout(map_card)
        map_title = QHBoxLayout(); map_title.addWidget(QLabel("作业地图")); map_title.addStretch(); self.data_age = QLabel("等待遥测"); self.data_age.setObjectName("muted"); map_title.addWidget(self.data_age); map_layout.addLayout(map_title)
        self.canvas = MapCanvas(); map_layout.addWidget(self.canvas, 1); body.addWidget(map_card, 3)
        self.canvas.initial_pose_selected.connect(self._pose_selected)
        self.canvas.map_changed.connect(self._map_changed)
        side = QFrame(); side.setObjectName("card"); side_layout = QVBoxLayout(side); body.addWidget(side, 1)
        tabs = QTabWidget(); side_layout.addWidget(tabs)
        manual = QWidget(); tabs.addTab(manual, "手动控制"); self._build_manual(manual)
        task_content = QWidget(); self._build_task(task_content)
        task = QScrollArea(); task.setWidgetResizable(True); task.setFrameShape(QFrame.NoFrame)
        task.setWidget(task_content); tabs.addTab(task, "清洗任务")
        camera = QFrame(); camera.setObjectName("card"); camera_layout = QVBoxLayout(camera); camera_layout.addWidget(QLabel("前向摄像头")); camera_label = QLabel("摄像头画面预留\n接入 WebRTC 后显示"); camera_label.setAlignment(Qt.AlignCenter); camera_label.setMinimumHeight(125); camera_label.setStyleSheet("background:#172a39;color:#a9bdc9;border-radius:8px;"); camera_layout.addWidget(camera_label); side_layout.addWidget(camera)
        self.status_bar = QLabel("等待网关状态"); self.status_bar.setObjectName("muted"); layout.addWidget(self.status_bar)

    def _build_manual(self, parent):
        box = QVBoxLayout(parent); box.setContentsMargins(4, 8, 4, 8)
        self.mode_label = QLabel("手动控制 · 网关自动管理会话"); box.addWidget(self.mode_label)
        box.addWidget(QLabel("PWM 档位"))
        speeds = QHBoxLayout()
        self.speed_buttons = {}
        for value in (25, 50, 75):
            button = QPushButton(f"{value}%"); button.setCheckable(True); button.clicked.connect(lambda checked, v=value: self._set_speed(v)); speeds.addWidget(button)
            self.speed_buttons[value] = button
            if value == 25: button.setChecked(True)
        box.addLayout(speeds)
        box.addWidget(QLabel("驱动轮组")); self.mode_combo = QComboBox(); self.mode_combo.addItems(["前轮", "四轮", "后轮"]); self.mode_combo.currentTextChanged.connect(lambda x: setattr(self, "mode", {"前轮":"FRONT", "四轮":"BOTH", "后轮":"BACK"}[x])); box.addWidget(self.mode_combo)
        grid = QGridLayout(); self.drive_buttons = {}
        for row, col, label, direction in ((0,1,"↑ W","FORWARD"),(1,0,"← A","LEFT"),(1,1,"停","STOP"),(1,2,"D →","RIGHT"),(2,1,"↓ S","REVERSE")):
            button = QPushButton(label); self.drive_buttons[direction] = button; grid.addWidget(button,row,col); button.pressed.connect(lambda d=direction: self._set_direction(d)); button.released.connect(lambda: self._set_direction("STOP"))
        box.addLayout(grid); box.addWidget(QLabel("按住运行，松开停止主轮"))
        box.addStretch()
        box.addWidget(QLabel("辅助执行器输出（-100% 到 100%）"))
        brush_row = QHBoxLayout()
        self.brush_button = QPushButton("滚刷：关闭")
        self.brush_button.setCheckable(True)
        self.brush_button.toggled.connect(self._toggle_brush)
        self.brush_value = self._make_aux_input()
        self.brush_value.valueChanged.connect(lambda value: setattr(self, "brush_percent", value))
        brush_row.addWidget(self.brush_button, 1); brush_row.addWidget(self.brush_value)
        box.addLayout(brush_row)
        thrust_row = QHBoxLayout()
        self.thrust_button = QPushButton("第五电推：关闭")
        self.thrust_button.setCheckable(True)
        self.thrust_button.toggled.connect(self._toggle_thrust)
        self.thrust_value = self._make_aux_input()
        self.thrust_value.valueChanged.connect(lambda value: setattr(self, "thrust_percent", value))
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

    def _toggle_thrust(self, enabled):
        self.thrust = enabled
        self.thrust_button.setText("第五电推：开启" if enabled else "第五电推：关闭")

    def _build_task(self, parent):
        box = QVBoxLayout(parent); box.setContentsMargins(4, 8, 4, 8)
        box.addWidget(QLabel("清洗任务类型"))
        self.task_combo = QComboBox()
        for kind in ("rectangle", "l"):
            self.task_combo.addItem(TASK_NAMES[kind], kind)
        self.task_combo.currentIndexChanged.connect(self._refresh_controls)
        box.addWidget(self.task_combo)
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
        self.task_buttons = {}
        for label, action in (("开始清洗", "start"), ("暂停任务", "pause"), ("取消任务", "cancel")):
            button = QPushButton(label)
            if action == "start":
                button.setObjectName("primary")
            button.setEnabled(False)
            button.clicked.connect(lambda checked=False, a=action: self._task(a))
            box.addWidget(button); self.task_buttons[action] = button
        self.result_label = QLabel("")
        self.result_label.setWordWrap(True); box.addWidget(self.result_label)
        box.addStretch()
        note = QLabel("暂停后需取消再重新开始。\n顶部红色按钮仅停止手动输出。")
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

    def _send_request(self, op, **fields):
        packet = self._packet(op, request_id=uuid.uuid4().hex, **fields)
        if self.worker.send_packet(packet):
            self.pending_requests[packet["request_id"]] = (time.monotonic(), packet)
            self.status_bar.setText("操作已发送，等待机器人确认")
        else:
            self.status_bar.setText("操作未发送：队列已满")
        self._refresh_controls()

    def _refresh_controls(self, *args):
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
        locked = bool(busy or local_pending or selecting or not fresh or not operator)
        if locked and not self.manual_locked:
            self._clear_manual()
        self.manual_locked = locked
        for button in self.drive_buttons.values():
            button.setEnabled(not locked)
        self.brush_button.setEnabled(not locked)
        self.thrust_button.setEnabled(not locked and packet.get("state", {}).get("allow_fifth_thruster") is True)
        self.task_combo.setEnabled(fresh and operator and not busy and not local_pending and not selecting)
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
        pending_start = any(p.get("action") == "start" for _, p in self.pending_requests.values())
        stopping = any(p.get("action") in ("pause", "cancel") for _, p in self.pending_requests.values())
        stoppable = bool(active or pending_start or (task_fresh and task.get("phase") == "PAUSED"))
        self.task_buttons["pause"].setEnabled(fresh and operator and stoppable and not stopping)
        self.task_buttons["cancel"].setEnabled(fresh and operator and stoppable and not stopping)
        loc = packet.get("localization", {})
        map_id = loc.get("map_id", "")
        self.map_status.setText("地图：" + (map_id.rsplit("/", 1)[-1] if map_id else "未确认"))
        self.map_status.setToolTip(map_id)
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
        blockers = task.get("start_blockers", [])
        condition = blocker_text(blockers) or "启动条件已满足"
        if not fresh:
            condition = "连接中断或遥测过期，禁止开始任务"
            self.canvas.set_telemetry({"pose": None, "scan": None})
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
        self.task_status.setText((PHASES.get(phase, phase or "等待任务状态") + "\n" + mode) if task_fresh else "任务状态未知")
        self.task_status.setStyleSheet("color:#00877e;font-weight:bold;" if task_fresh and phase == "COMPLETE" else "")
        self.task_reason.setText(str(task.get("reason", "")) if task_fresh else "")
        result = self.completed_results.get(kind)
        self.result_label.setText(result[1] if result else "")
        for request_id, (when, _) in self.pending_requests.items():
            if time.monotonic()-when > 5:
                self.status_bar.setText("操作结果尚未确认；不会自动重发。请查看任务状态或取消任务。")
                break

    def _set_speed(self, value):
        self.speed = value
        for speed, button in self.speed_buttons.items():
            button.setChecked(speed == value)
    def _set_direction(self, direction):
        if not self.manual_locked:
            self.direction = direction
    def _packet(self, op, **fields):
        self.seq += 1; return {"op": op, "seq": self.seq, **fields}
    def _send_input(self):
        if self.manual_locked:
            return
        self.worker.send_packet(self._packet("manual_input", direction=self.direction,
                                            speed_percent=self.speed, mode=self.mode,
                                            brush_enabled=self.brush,
                                            brush_percent=self.brush_percent if self.brush else 0,
                                            thrust_enabled=self.thrust,
                                            thrust_percent=self.thrust_percent if self.thrust else 0))
    def _clear_manual(self):
        self.direction = "STOP"
        self._pressed_keys.clear()
        self.brush_button.setChecked(False)
        self.thrust_button.setChecked(False)
        self.brush = self.thrust = False

    def _stop(self, reason):
        self._clear_manual()
        self.worker.send_packet(self._packet("manual_stop", reason=reason))

    def _task(self, action):
        if not self.task_buttons[action].isEnabled():
            return
        self._clear_manual()
        active = self.telemetry.get("active_tasks", [])
        kind = active[0] if action != "start" and len(active) == 1 else self.task_combo.currentData()
        self._send_request("task", action=action, task_type=kind)

    def _status(self, text):
        self.connection.setText(text)
        if text.startswith("连接失败"):
            self.last_telemetry_at = 0.
            self._clear_manual()
            self.pending_requests.clear()
        self._refresh_controls()

    def _refresh_key_direction(self):
        for key in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D):
            if key in self._pressed_keys:
                self.direction = self._key_directions[key]
                return
        self.direction = "STOP"

    def eventFilter(self, obj, event):
        if self.isActiveWindow() and event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            key = event.key()
            if key in self._key_directions:
                if self.manual_locked:
                    return True
                if event.type() == QEvent.KeyPress:
                    self._pressed_keys.add(key)
                else:
                    self._pressed_keys.discard(key)
                self._refresh_key_direction()
                return True
            if event.type() == QEvent.KeyPress and key == Qt.Key_Space and not event.isAutoRepeat():
                self._stop("keyboard")
                return True
        return super().eventFilter(obj, event)
    def _message(self, packet):
        if packet.get("type") == "hello":
            self.client_id = packet.get("client_id")
            self.capabilities = set(packet.get("capabilities", []))
        elif packet.get("type") == "map":
            self.canvas.set_telemetry(packet)
        elif packet.get("type") == "telemetry":
            self.telemetry = packet
            self.last_telemetry_at = time.monotonic()
            self.canvas.set_telemetry(packet)
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
                    text = f"{TASK_NAMES.get(kind, kind)}：{preview}\n完成时间：{when}\n要求区间已覆盖，共 {task.get('edge_count', '—')} 条边"
                    self.completed_results[kind] = (run_id, text)
                    if (kind, run_id) not in self.seen_completions:
                        self.seen_completions.add((kind, run_id))
                        self.status_bar.setText(text.replace("\n", " · "))
        elif packet.get("type") == "reply":
            if not packet.get("pending"):
                self.pending_requests.pop(packet.get("request_id"), None)
            if packet.get("message"):
                prefix = "操作反馈：" if packet.get("success") else "操作未完成："
                self.status_bar.setText(prefix + str(packet["message"]))
        elif packet.get("type") == "error":
            self.status_bar.setText("网关错误：" + str(packet.get("message", "未知错误")))
        self._refresh_controls()

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self)
        self._stop("window_closed"); self.worker.close(); self.worker.wait(1000); event.accept()
