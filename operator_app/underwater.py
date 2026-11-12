"""Independent underwater controls and transport. Surface task gates do not apply."""
import math
import time
from urllib.parse import urlsplit, urlunsplit

try:
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (QApplication, QAbstractSpinBox, QFrame, QGridLayout,
                                   QHBoxLayout, QLabel, QLineEdit, QPushButton, QSlider,
                                   QSpinBox, QVBoxLayout)
except ImportError:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (QApplication, QAbstractSpinBox, QFrame, QGridLayout,
                                QHBoxLayout, QLabel, QLineEdit, QPushButton, QSlider,
                                QSpinBox, QVBoxLayout)

try:
    from .transport import SocketWorker
except ImportError:
    from transport import SocketWorker

PROTOCOL = "under-operator-v1"
DEVICE = "h7_under_original_v6"
ARROWS = {Qt.Key_Up: "FORWARD", Qt.Key_Down: "REVERSE",
          Qt.Key_Left: "LEFT", Qt.Key_Right: "RIGHT"}


def underwater_url(surface_url):
    parts = urlsplit(surface_url)
    if parts.scheme not in ("ws", "wss") or not parts.hostname:
        raise ValueError("invalid surface WebSocket URL")
    host = parts.hostname
    if ":" in host:
        host = f"[{host}]"
    return urlunsplit((parts.scheme, f"{host}:8766", "", "", ""))


class UnderwaterWorker(SocketWorker):
    def _send_now(self, ws, packet):
        super()._send_now(ws, dict(packet, protocol=PROTOCOL, device=DEVICE))

    def _mock_loop(self):
        self._receive(dict(type="hello", protocol=PROTOCOL, device=DEVICE,
                           session_id="under-mock", client_id="under-mock-client",
                           capabilities=["under_manual_v1"],
                           drive_output=dict(min=0, max=100, step=1, firmware_protocol_version=6)))
        state = dict(connected=True, connection_id="under-mock-serial", control_generation=1,
                     epoch=1, state="READY", available=True, source="NANO", owner=None,
                     armed=True, result="OK", reason="模拟就绪", last_stop_reason="",
                     report_age_sec=0., control_fresh=True, motion_enabled=True,
                     firmware_protocol_version=6, direction="STOP", speed_percent=0,
                     stop_confirmed=True)
        sequence = 0
        while not self.stop_requested.wait(.05):
            for packet in self._take_outgoing():
                if packet["op"] == "manual_stop":
                    state.update(direction="STOP", speed_percent=0, owner=None,
                                 control_generation=state["control_generation"] + 1,
                                 last_stop_reason=packet.get("reason", "operator"))
                elif (packet["op"] == "manual_input" and
                      packet.get("control_generation") == state["control_generation"]):
                    direction, speed = packet["direction"], packet["speed_percent"]
                    if direction == "STOP" or speed == 0:
                        direction, speed = "STOP", 0
                    state.update(direction=direction, speed_percent=speed,
                                 owner="under-mock-client" if speed else None)
            state.update(state="ACTIVE" if state["owner"] else "READY",
                         stop_confirmed=state["direction"] == "STOP")
            sequence += 1
            self._receive(dict(type="telemetry", protocol=PROTOCOL, device=DEVICE,
                               session_id="under-mock", sequence=sequence, state=dict(state)))


class UnderwaterPanel(QFrame):
    def __init__(self, url, *, mock=False, log=None, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self.setMinimumWidth(265)
        self.log = log or (lambda *args: None)
        self.direction, self.speed = "STOP", 25
        self.pressed = set()
        self.mouse_direction = None
        self.state = {}
        self.client_id = self.session_id = self.identity = None
        self.last_telemetry_at = 0.
        self.sequence = -1
        self.handshake = False
        self.locked = True
        self.worker = UnderwaterWorker(url, mock=mock, parent=self)
        self._build(url)
        self.worker.message.connect(self.message)
        self.worker.status.connect(self.status)
        self.worker.manual_expired.connect(self.expired)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.worker.start()

    def _build(self, url):
        box = QVBoxLayout(self)
        title = QLabel("水下机器人 · 方向键"); title.setObjectName("title"); box.addWidget(title)
        self.connection = QLabel("连接中…"); self.connection.setWordWrap(True)
        self.connection.setToolTip(url); box.addWidget(self.connection)
        self.gate = QLabel("等待水下服务"); self.gate.setWordWrap(True); box.addWidget(self.gate)
        box.addWidget(QLabel("行走输出幅度（0–100%）"))
        row = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal); self.slider.setRange(0, 100); self.slider.setValue(25)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.speed_input = QSpinBox(); self.speed_input.setRange(0, 100); self.speed_input.setValue(25)
        self.speed_input.setSuffix(" %"); self.speed_input.setKeyboardTracking(False)
        self.slider.valueChanged.connect(self.set_speed)
        self.speed_input.valueChanged.connect(self.set_speed)
        self.speed_input.editingFinished.connect(self.finish_edit)
        row.addWidget(self.slider, 1); row.addWidget(self.speed_input); box.addLayout(row)
        grid = QGridLayout(); self.buttons = {}
        for r, c, label, direction in ((0, 1, "↑", "FORWARD"), (1, 0, "←", "LEFT"),
                                      (1, 1, "■", "STOP"), (1, 2, "→", "RIGHT"),
                                      (2, 1, "↓", "REVERSE")):
            button = QPushButton(label); button.setFocusPolicy(Qt.NoFocus)
            button.setMinimumSize(55, 48); self.buttons[direction] = button
            button.setToolTip(dict(FORWARD="前进（↑）", REVERSE="后退（↓）",
                                  LEFT="左转（←）", RIGHT="右转（→）", STOP="停止水下手动输出")[direction])
            if direction == "STOP":
                button.clicked.connect(lambda: self.stop("under_button"))
            else:
                button.pressed.connect(lambda d=direction: self.button_press(d))
                button.released.connect(self.button_release)
            grid.addWidget(button, r, c)
        box.addLayout(grid)
        box.addWidget(QLabel("按住箭头运行，松开停止\n← / → 为左转 / 右转"))
        self.stop_button = QPushButton("■  水下手动停止"); self.stop_button.setObjectName("stop")
        self.stop_button.setFocusPolicy(Qt.NoFocus)
        self.stop_button.clicked.connect(lambda: self.stop("under_button")); box.addWidget(self.stop_button)
        self.feedback = QLabel("H7 回报：等待新鲜状态"); self.feedback.setWordWrap(True); box.addWidget(self.feedback)
        box.addStretch()
        note = QLabel("前进时前滚刷随动；运动时顶升目标 40%。\n输出百分比不是实测速度。\n遥控接管时，App 停止不覆盖遥控输出，需遥控回中停车。")
        note.setWordWrap(True); note.setObjectName("muted"); box.addWidget(note)

    def clear(self):
        self.worker.clear_manual_intent()
        self.direction = "STOP"
        self.pressed.clear()
        self.mouse_direction = None
        for button in self.buttons.values():
            button.setDown(False)

    def stop(self, reason):
        self.clear()
        self.worker.send_packet(dict(op="manual_stop", reason=reason))
        self.log("水下", "停止手动输出", {"reason": reason})

    def expired(self):
        if self.worker.manual_intent_expired:
            self.stop("gui_input_expired")
            self.worker.acknowledge_manual_expiry()

    def status(self, text):
        self.connection.setText(text)
        if text.startswith("连接失败"):
            self.handshake = False
            self.last_telemetry_at = 0.
            self.clear()
        self.refresh()

    def block_reason(self):
        now = time.monotonic()
        if not self.handshake:
            return "等待 H7 水下服务握手"
        age = now - self.last_telemetry_at
        if not self.last_telemetry_at or age >= .8:
            return "水下遥测未收到或已过期"
        state = self.state
        report_age = state.get("report_age_sec")
        if (state.get("control_fresh") is not True or type(report_age) not in (int, float)
                or not math.isfinite(report_age) or report_age < 0 or report_age + age >= .25):
            return "H7 回报未收到或已过期"
        if state.get("source") == "RC":
            return "遥控器控制中；需遥控回中停车"
        if (state.get("available") is not True or state.get("source") != "NANO"
                or state.get("armed") is not True or state.get("result") != "OK"):
            return "水下未就绪：" + str(state.get("reason", "等待停止握手"))
        if state.get("owner") not in (None, self.client_id):
            return "其他操作端正在控制水下机器人"
        if isinstance(QApplication.focusWidget(), (QLineEdit, QAbstractSpinBox)):
            return "输入框持有焦点"
        return ""

    def refresh(self):
        reason = self.block_reason()
        locked = bool(reason)
        if locked and (not self.locked or self.direction != "STOP"):
            self.stop("under_control_unavailable")
        self.locked = locked
        self.gate.setText(reason or "水下可操作 · 与水面控制独立")
        for direction, button in self.buttons.items():
            if direction != "STOP":
                if locked and button.hasFocus():
                    self.window().setFocus(Qt.OtherFocusReason)
                button.setEnabled(not locked)
        state = self.state
        report_age = state.get("report_age_sec")
        fresh = (self.last_telemetry_at and type(report_age) in (int, float)
                 and math.isfinite(report_age) and report_age >= 0
                 and report_age + time.monotonic() - self.last_telemetry_at < .25)
        board = (f"{state.get('source', '—')} · {state.get('direction', '—')} · {state.get('speed_percent', '—')}%"
                 if fresh else "等待新鲜状态")
        self.feedback.setText(f"App 请求：{self.direction} · {self.speed}%\nH7 回报：{board}")

    def tick(self):
        self.refresh()
        if self.locked:
            self.clear()
            return
        generation = self.state.get("control_generation")
        if type(generation) is not int or generation < 1:
            self.clear()
            return
        if not self.worker.update_manual_intent(dict(op="manual_input", direction=self.direction,
                speed_percent=self.speed if self.direction != "STOP" else 0,
                control_generation=generation)):
            self.clear()
            self.expired()

    def set_speed(self, value):
        self.speed = value
        for widget in (self.slider, self.speed_input):
            if widget.value() != value:
                previous = widget.blockSignals(True); widget.setValue(value); widget.blockSignals(previous)
        if value == 0:
            self.clear()
        self.tick()

    def finish_edit(self):
        if self.speed_input.hasFocus() or self.speed_input.lineEdit().hasFocus():
            self.window().setFocus(Qt.OtherFocusReason)

    def key(self, key, pressed, repeat=False):
        if key not in ARROWS:
            return False
        if repeat:
            return True
        if self.block_reason() or self.speed == 0:
            self.clear()
            return True
        if pressed:
            self.pressed.add(key)
        else:
            self.pressed.discard(key)
        self.direction = ARROWS[next(iter(self.pressed))] if len(self.pressed) == 1 else "STOP"
        self.tick()
        return True

    def button_press(self, direction):
        if not self.block_reason() and self.speed:
            self.pressed.clear()
            self.mouse_direction = direction
            self.direction = direction
            self.tick()

    def button_release(self):
        self.mouse_direction = None
        self.clear()
        self.tick()

    def message(self, packet):
        if packet.get("type") == "incoming_ready":
            for incoming in self.worker.take_incoming():
                self.message(incoming)
            return
        if packet.get("_connection_generation") != self.worker.connection_generation:
            return
        if packet.get("protocol") != PROTOCOL or packet.get("device") != DEVICE:
            self.handshake = False
            self.clear()
            self.refresh()
            return
        if packet.get("type") == "hello":
            output = packet.get("drive_output", {})
            self.clear()
            self.handshake = ("under_manual_v1" in packet.get("capabilities", []) and
                all(type(output.get(k)) is int and output[k] == v for k, v in
                    (("min", 0), ("max", 100), ("step", 1), ("firmware_protocol_version", 6))))
            if self.handshake:
                self.handshake = self.worker.activate_manual_session(packet["_connection_generation"])
            self.client_id, self.session_id = packet.get("client_id"), packet.get("session_id")
            self.identity = None
            self.sequence, self.last_telemetry_at = -1, 0.
        elif packet.get("type") == "telemetry" and self.handshake:
            received = packet.get("_received_monotonic")
            sequence = packet.get("sequence")
            if (packet.get("session_id") != self.session_id or type(sequence) is not int
                    or sequence <= self.sequence or type(received) not in (int, float)
                    or not math.isfinite(received) or not self.last_telemetry_at <= received <= time.monotonic()):
                return
            state = packet.get("state")
            if not isinstance(state, dict):
                return
            identity = tuple(state.get(k) for k in ("connection_id", "control_generation", "epoch", "source"))
            if self.identity is not None and self.identity != identity:
                self.clear()
            self.identity = identity
            self.state, self.sequence, self.last_telemetry_at = state, sequence, received
            self.log_state(state)
        self.refresh()

    def log_state(self, state):
        key = tuple(state.get(k) for k in ("connection_id", "source", "state", "last_stop_reason"))
        if key != getattr(self, "_logged_state", None):
            self._logged_state = key
            self.log("水下", "控制状态", {k: state.get(k) for k in
                     ("state", "source", "reason", "last_stop_reason", "report_age_sec")})

    def shutdown(self):
        self.timer.stop()
        self.stop("window_closed")
        self.worker.close()
        self.worker.wait(2000)
