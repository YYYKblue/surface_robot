import time

import pytest

try:
    from PySide6.QtCore import QEvent, Qt
    from PySide6.QtGui import QKeyEvent
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent
    from PyQt5.QtWidgets import QApplication

from operator_app.transport import SocketWorker
from operator_app.underwater import DEVICE, PROTOCOL, underwater_url
from operator_app.window import OperatorWindow


@pytest.fixture
def dual(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused", under_url="unused-under")
    window.send_timer.stop(); window.state_timer.stop(); window.under_panel.timer.stop()
    monkeypatch.setattr(window, "isActiveWindow", lambda: True)
    window.last_telemetry_at = time.monotonic(); window.manual_locked = False
    surface, under = [], []
    monkeypatch.setattr(window.worker, "update_manual_intent", lambda p: surface.append(p) or True)
    panel = window.under_panel
    monkeypatch.setattr(panel.worker, "update_manual_intent", lambda p: under.append(p) or True)
    monkeypatch.setattr(panel.worker, "activate_manual_session", lambda g: True)
    panel.message(dict(type="hello", protocol=PROTOCOL, device=DEVICE, session_id="u",
        client_id="c", capabilities=["under_manual_v1"], _connection_generation=0,
        drive_output=dict(min=0, max=100, step=1, firmware_protocol_version=6)))
    telemetry(panel)
    yield window, surface, under
    window.close()
    app.processEvents()


def telemetry(panel, **changes):
    state = dict(connected=True, connection_id="serial", control_generation=1, epoch=1,
        available=True, source="NANO", owner=None, armed=True, result="OK",
        reason="ready", report_age_sec=.01, control_fresh=True, direction="STOP",
        speed_percent=0, firmware_protocol_version=6)
    state.update(changes)
    panel.message(dict(type="telemetry", protocol=PROTOCOL, device=DEVICE, session_id="u",
        sequence=panel.sequence + 1, state=state, _connection_generation=0,
        _received_monotonic=time.monotonic()))


def key(window, code, pressed=True, repeat=False):
    event = QKeyEvent(QEvent.KeyPress if pressed else QEvent.KeyRelease, code,
                      Qt.NoModifier, "", repeat, 1)
    return window.eventFilter(window, event)


def test_wasd_and_arrows_run_independently_and_release_only_one(dual):
    window, surface, under = dual
    assert key(window, Qt.Key_W)
    assert key(window, Qt.Key_Left)
    assert surface[-1]["direction"] == "FORWARD"
    assert under[-1]["direction"] == "LEFT"
    assert window.direction == "FORWARD" and window.under_panel.direction == "LEFT"
    key(window, Qt.Key_Left, False)
    assert under[-1]["direction"] == "STOP" and window.direction == "FORWARD"
    key(window, Qt.Key_W, False)
    assert surface[-1]["direction"] == "STOP"


@pytest.mark.parametrize("direction", ["FORWARD", "REVERSE", "LEFT", "RIGHT"])
def test_arrow_buttons_press_and_release(dual, direction):
    window, _, under = dual
    panel = window.under_panel
    panel.buttons[direction].pressed.emit()
    assert under[-1]["direction"] == direction
    panel.buttons[direction].released.emit()
    assert under[-1]["direction"] == "STOP"


def test_surface_task_lock_and_disconnect_do_not_stop_underwater(dual):
    window, _, under = dual
    key(window, Qt.Key_Up)
    window.manual_locked = True
    window._status("连接失败：surface")
    window.under_panel.tick()
    assert under[-1]["direction"] == "FORWARD"
    assert not window.under_panel.locked


def test_underwater_disconnect_does_not_clear_surface(dual):
    window, _, _ = dual
    key(window, Qt.Key_W); key(window, Qt.Key_Up)
    window.under_panel.status("连接失败：under")
    assert window.direction == "FORWARD" and window.under_panel.direction == "STOP"


@pytest.mark.parametrize("action", ["space", "focus", "button"])
def test_global_stop_clears_both_sessions(dual, action):
    window, _, _ = dual
    key(window, Qt.Key_W); key(window, Qt.Key_Right)
    if action == "space":
        key(window, Qt.Key_Space)
    elif action == "focus":
        window.eventFilter(window, QEvent(QEvent.WindowDeactivate))
    else:
        window.stop_button.click()
    assert window.direction == window.under_panel.direction == "STOP"
    assert any(p["op"] == "manual_stop" for p in window.worker._take_outgoing())
    assert any(p["op"] == "manual_stop" for p in window.under_panel.worker._take_outgoing())


def test_underwater_zero_then_increase_and_repeat_cannot_revive_hold(dual):
    window, _, under = dual
    key(window, Qt.Key_Up)
    panel = window.under_panel
    panel.slider.setValue(0)
    key(window, Qt.Key_Up)
    panel.slider.setValue(40)
    key(window, Qt.Key_Up, repeat=True)
    assert panel.direction == "STOP" and under[-1]["direction"] == "STOP"
    key(window, Qt.Key_Up)
    assert under[-1]["speed_percent"] == 40 and panel.direction == "FORWARD"


def test_rc_takeover_and_return_clear_old_keys_without_affecting_surface(dual):
    window, _, under = dual
    key(window, Qt.Key_W); key(window, Qt.Key_Left)
    panel = window.under_panel
    telemetry(panel, source="RC", result="RC_OK", available=False, control_generation=2)
    assert panel.locked and not panel.pressed
    assert window.direction == "FORWARD"
    telemetry(panel, control_generation=3)
    panel.tick()
    assert under[-1]["direction"] == "STOP"
    key(window, Qt.Key_Left, repeat=True)
    assert panel.direction == "STOP"


def test_h7_feedback_age_includes_gui_wait_and_recovery_needs_new_press(dual):
    window, _, under = dual
    key(window, Qt.Key_Down)
    panel = window.under_panel
    panel.last_telemetry_at = time.monotonic() - .26
    panel.tick()
    assert panel.locked and panel.direction == "STOP"
    telemetry(panel)
    panel.tick()
    assert under[-1]["direction"] == "STOP"


def test_retired_connection_wrong_protocol_and_unknown_device_cannot_enable(dual):
    window, _, _ = dual
    panel = window.under_panel
    old = panel.last_telemetry_at
    panel.worker.connection_generation = 1
    telemetry(panel)
    assert panel.last_telemetry_at == old
    panel.message(dict(type="hello", _connection_generation=1, protocol="f1-operator-v1"))
    assert not panel.handshake and panel.locked


def test_other_operator_blocks_only_underwater(dual):
    window, _, _ = dual
    telemetry(window.under_panel, owner="other")
    assert window.under_panel.locked
    key(window, Qt.Key_W)
    assert window.direction == "FORWARD"


@pytest.mark.parametrize("source,target", [("ws://192.168.1.20:8765", "ws://192.168.1.20:8766"),
    ("ws://[::1]:8765", "ws://[::1]:8766"), ("wss://nano.example/control", "wss://nano.example:8766")])
def test_default_underwater_url_uses_same_host(source, target):
    assert underwater_url(source) == target
