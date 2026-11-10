"""GUI intent lifecycle: immediate release and no revival after a stalled UI."""
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
from operator_app.window import OperatorWindow


@pytest.fixture
def manual_window(monkeypatch):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("unused")
    window.send_timer.stop()
    window.state_timer.stop()
    window.last_telemetry_at = time.monotonic()
    window.manual_locked = False
    submitted, cancelled = [], []
    monkeypatch.setattr(window.worker, "update_manual_intent",
                        lambda packet: submitted.append(dict(packet)) or True)
    monkeypatch.setattr(window.worker, "clear_manual_intent", lambda: cancelled.append(True))
    yield window, submitted, cancelled
    window.close()
    app.processEvents()


def test_button_release_updates_sender_without_waiting_for_gui_timer(manual_window):
    window, submitted, _ = manual_window
    window.drive_buttons["FORWARD"].pressed.emit()
    assert submitted[-1]["direction"] == "FORWARD"
    window.drive_buttons["FORWARD"].released.emit()
    assert submitted[-1]["direction"] == "STOP"
    window.brush_button.setChecked(True)
    assert submitted[-1]["brush_enabled"] is True
    window.brush_button.setChecked(False)
    assert submitted[-1]["brush_enabled"] is False


def test_clearing_controls_cannot_submit_half_cleared_intent(manual_window):
    window, submitted, cancelled = manual_window
    window.direction = "FORWARD"
    window.brush_button.setChecked(True)
    window.thrust_button.setChecked(True)
    submitted.clear()
    window._clear_manual()
    assert cancelled
    assert submitted == []
    assert (window.direction, window.brush, window.thrust) == ("STOP", False, False)


def test_intent_timer_clears_stale_telemetry_hold_before_controls_timer(manual_window):
    window, submitted, _ = manual_window
    window.direction = "FORWARD"
    window.last_telemetry_at = time.monotonic() - 1.
    window._send_input()
    assert window.direction == "STOP"
    assert submitted == []
    # Telemetry can recover before the slower controls timer ever saw expiry.
    window.last_telemetry_at = time.monotonic()
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"


def test_expired_intent_clears_before_ack_and_auto_repeat_cannot_revive(manual_window, monkeypatch):
    window, submitted, _ = manual_window
    expired = [True]
    monkeypatch.setattr(SocketWorker, "manual_intent_expired", property(lambda self: expired[0]))
    acknowledged = []

    def acknowledge():
        acknowledged.append((window.direction, window.brush, window.thrust, set(window._pressed_keys)))
        expired[0] = False

    monkeypatch.setattr(window.worker, "acknowledge_manual_expiry", acknowledge)
    monkeypatch.setattr(window, "isActiveWindow", lambda: True)
    window.direction = "FORWARD"
    window._pressed_keys.add(Qt.Key_W)
    window.brush_button.setChecked(True)
    window.worker.manual_expired.emit()
    assert acknowledged == [("STOP", False, False, set())]
    assert "gui_input_expired" in window.log_view.toPlainText()
    submitted.clear()
    repeat = QKeyEvent(QEvent.KeyPress, Qt.Key_W, Qt.NoModifier, "w", True, 1)
    assert window.eventFilter(window, repeat)
    assert window.direction == "STOP"
    assert submitted == []
    window.manual_locked = False
    press = QKeyEvent(QEvent.KeyPress, Qt.Key_W, Qt.NoModifier, "w", False, 1)
    assert window.eventFilter(window, press)
    assert submitted[-1]["direction"] == "FORWARD"
    release = QKeyEvent(QEvent.KeyRelease, Qt.Key_W, Qt.NoModifier, "w", False, 1)
    assert window.eventFilter(window, release)
    assert submitted[-1]["direction"] == "STOP"


def test_hello_clears_old_hold_even_when_gateway_session_is_unchanged(manual_window, monkeypatch):
    window, _, _ = manual_window
    activated = []
    monkeypatch.setattr(window.worker, "activate_manual_session",
                        lambda generation: activated.append((generation, window.direction)) or True)
    window._gateway_session = "same-gateway"
    window.worker.connection_generation = 2
    window.direction = "FORWARD"
    window._message(dict(type="hello", session_id="same-gateway", client_id="new-client",
                         capabilities=[], _connection_generation=2))
    assert activated == [(2, "STOP")]


def test_retired_connection_cannot_replace_current_handshake_or_telemetry(manual_window):
    window, _, _ = manual_window
    window.worker.connection_generation = 2
    window.client_id = "current-client"
    window.direction = "FORWARD"
    before = window.last_telemetry_at
    window._message(dict(type="hello", session_id="same-gateway", client_id="old-client",
                         capabilities=[], _connection_generation=1))
    window._message(dict(type="telemetry", state=dict(state="READY"), _connection_generation=1))
    assert window.client_id == "current-client"
    assert window.direction == "FORWARD"
    assert window.last_telemetry_at == before


def test_export_revokes_intent_before_native_dialog(manual_window, monkeypatch):
    from operator_app import diagnostics
    window, _, _ = manual_window
    window.direction = "FORWARD"
    stopped_at_dialog = []

    def dialog(*args, **kwargs):
        stopped_at_dialog.append((window.direction, window.brush, window.thrust))
        packets = window.worker._take_outgoing()
        assert any(p.get("reason") == "diagnostics_export" for p in packets)
        return "", ""

    monkeypatch.setattr(diagnostics.QFileDialog, "getSaveFileName", dialog)
    window._export_diagnostics()
    assert stopped_at_dialog == [("STOP", False, False)]
