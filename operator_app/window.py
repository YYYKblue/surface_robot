"""Main operator console window and operator actions."""
try:
    from PySide6.QtCore import QEvent, QTimer, Qt
    from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                   QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                   QSpinBox, QTabWidget, QVBoxLayout, QWidget)
except ImportError:
    from PyQt5.QtCore import QEvent, QTimer, Qt
    from PyQt5.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout,
                                 QHBoxLayout, QLabel, QMainWindow, QPushButton,
                                 QSpinBox, QTabWidget, QVBoxLayout, QWidget)

try:
    from .map_canvas import MapCanvas
    from .transport import SocketWorker
except ImportError:
    from map_canvas import MapCanvas
    from transport import SocketWorker


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

    def _build(self):
        self.setStyleSheet("""
        QWidget { font-family: 'Microsoft YaHei', 'Noto Sans CJK SC', sans-serif; color:#172c3d; }
        QMainWindow, QWidget { background:#f2f5f7; }
        QFrame#card { background:#ffffff; border:1px solid #dde5ea; border-radius:12px; }
        QPushButton { background:#ffffff; border:1px solid #d5e0e5; border-radius:7px; padding:8px 14px; min-height:22px; }
        QPushButton:hover { border-color:#00877e; }
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
        self.stop_button = QPushButton("■  停止全部"); self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(lambda: self._stop("button")); top.addWidget(self.stop_button)
        layout.addLayout(top)
        body = QHBoxLayout(); body.setSpacing(14); layout.addLayout(body, 1)
        map_card = QFrame(); map_card.setObjectName("card"); map_layout = QVBoxLayout(map_card)
        map_title = QHBoxLayout(); map_title.addWidget(QLabel("作业地图")); map_title.addStretch(); self.data_age = QLabel("等待遥测"); self.data_age.setObjectName("muted"); map_title.addWidget(self.data_age); map_layout.addLayout(map_title)
        self.canvas = MapCanvas(); map_layout.addWidget(self.canvas, 1); body.addWidget(map_card, 3)
        side = QFrame(); side.setObjectName("card"); side_layout = QVBoxLayout(side); body.addWidget(side, 1)
        tabs = QTabWidget(); side_layout.addWidget(tabs)
        manual = QWidget(); tabs.addTab(manual, "手动控制"); self._build_manual(manual)
        task = QWidget(); tabs.addTab(task, "清洗任务"); self._build_task(task)
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
        box.addWidget(QLabel("当前任务：单边清洗")); self.task_status = QLabel("未启动"); self.task_status.setObjectName("muted"); box.addWidget(self.task_status)
        for label, action in (("开始任务", "start"), ("暂停任务", "pause"), ("取消任务", "cancel")):
            button = QPushButton(label); button.clicked.connect(lambda checked=False, a=action: self._task(a)); box.addWidget(button)
        box.addStretch(); box.addWidget(QLabel("任务运行时，手动方向输入由网关拒绝。"))

    def _set_speed(self, value):
        self.speed = value
        for speed, button in self.speed_buttons.items():
            button.setChecked(speed == value)
    def _set_direction(self, direction): self.direction = direction
    def _packet(self, op, **fields):
        self.seq += 1; return {"op": op, "seq": self.seq, **fields}
    def _send_input(self):
        self.worker.send_packet(self._packet("manual_input", direction=self.direction,
                                            speed_percent=self.speed, mode=self.mode,
                                            brush_enabled=self.brush,
                                            brush_percent=self.brush_percent if self.brush else 0,
                                            thrust_enabled=self.thrust,
                                            thrust_percent=self.thrust_percent if self.thrust else 0))
    def _stop(self, reason):
        self.direction = "STOP"; self.brush = self.thrust = False
        self.worker.send_packet(self._packet("manual_stop", reason=reason))
    def _task(self, action):
        self.direction = "STOP"
        self.brush = self.thrust = False
        self.worker.send_packet(self._packet("manual_stop", reason="task_" + action))
        self.worker.send_packet(self._packet("task", action=action))
    def _status(self, text): self.connection.setText(text)

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
        if packet.get("type") == "map":
            self.canvas.set_telemetry(packet)
        elif packet.get("type") == "telemetry":
            self.canvas.set_telemetry(packet)
            self.data_age.setText("实时遥测")
            state = packet.get("state", {})
            self.status_bar.setText(f"设备：{state.get('state','—')} · 来源：{state.get('source','—')} · {state.get('reason','')}")
            if "allow_fifth_thruster" in state:
                allowed = bool(state["allow_fifth_thruster"])
                self.thrust_button.setEnabled(allowed)
                if not allowed and self.thrust:
                    self.thrust_button.setChecked(False)
                    self.status_bar.setText("第五电推未启用：请将 Nano 的 allow_fifth_thruster 设置为 true")
            task = packet.get("task") or {}
            if task: self.task_status.setText(str(task.get("phase", task.get("status", "未知"))))
        elif packet.get("type") == "reply" and not packet.get("success", True):
            self.status_bar.setText("操作未执行：" + str(packet.get("message", "网关拒绝")))

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self)
        self._stop("window_closed"); self.worker.close(); self.worker.wait(1000); event.accept()
