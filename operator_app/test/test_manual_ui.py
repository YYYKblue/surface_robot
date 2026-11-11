"""GUI intent lifecycle: immediate release and no revival after a stalled UI."""
import time

import pytest

try:
    from PySide6.QtCore import QEvent, Qt
    from PySide6.QtGui import QKeyEvent
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent
    from PyQt5.QtTest import QTest
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


def ready_state(**changes):
    return dict(dict(source="NANO", direction="STOP", speed_percent=25, state="READY",
                     connection_id="serial-a", epoch=1, connected=True, armed=True, result="OK",
                     ctrl_age_sec=.01, ack_age_sec=.01,
                     firmware_protocol_version=3, allow_fifth_thruster=True), **changes)


def continuous(window, *, available=True):
    output = dict(min=0, max=100, step=1, firmware_protocol_version=3, available=available)
    window._message(dict(type="hello", client_id="current",
                         capabilities=["continuous_drive_percent_v1"], drive_output=output))
    window._message(dict(type="telemetry", drive_output=output, control_fresh=True,
                         state=ready_state()))


def test_continuous_zero_preserves_auxiliaries_and_increase_needs_new_press(manual_window, monkeypatch):
    window, submitted, _ = manual_window
    continuous(window)
    monkeypatch.setattr(window, "isActiveWindow", lambda: True)
    window.brush_button.setChecked(True)
    window.thrust_button.setChecked(True)
    press = lambda repeat=False: QKeyEvent(QEvent.KeyPress, Qt.Key_W, Qt.NoModifier, "w", repeat, 1)
    window.eventFilter(window, press())
    assert submitted[-1]["direction"] == "FORWARD"
    window.speed_slider.setValue(0)
    assert submitted[-1]["direction"] == "STOP" and submitted[-1]["speed_percent"] == 0
    assert submitted[-1]["brush_enabled"] and submitted[-1]["thrust_enabled"]
    assert not window._pressed_keys
    # Even a fresh press while at zero cannot arm a later speed increase.
    window.eventFilter(window, press())
    window.speed_slider.setValue(37)
    window.eventFilter(window, press(True))
    assert window.direction == "STOP" and not window._pressed_keys
    assert submitted[-1]["speed_percent"] == 37
    window.eventFilter(window, press())
    assert submitted[-1]["direction"] == "FORWARD"
    assert submitted[-1]["speed_percent"] == 37
    assert window.speed_value.value() == 37
    assert not any(button.isChecked() for button in window.speed_buttons.values())


def test_continuous_shortcuts_do_not_steal_driving_focus(manual_window):
    window, submitted, _ = manual_window
    continuous(window)
    assert window.mode_combo.currentText() == "四轮" and window.mode == "BOTH"
    window.show()
    window.activateWindow()
    window.setFocus()
    QApplication.processEvents()
    QTest.keyPress(window, Qt.Key_W)
    window.speed_slider.setValue(31)
    assert submitted[-1]["direction"] == "FORWARD" and submitted[-1]["speed_percent"] == 31
    QTest.mouseClick(window.speed_buttons[75], Qt.LeftButton)
    assert QApplication.focusWidget() is window
    assert submitted[-1]["direction"] == "FORWARD" and submitted[-1]["speed_percent"] == 75
    assert window.speed_slider.focusPolicy() == Qt.NoFocus
    QTest.keyRelease(window, Qt.Key_W)
    assert submitted[-1]["direction"] == "STOP"


def test_zero_during_mouse_hold_requires_a_new_direction_press(manual_window):
    window, submitted, _ = manual_window
    continuous(window)
    window.show()
    window.activateWindow()
    QApplication.processEvents()
    forward = window.drive_buttons["FORWARD"]
    QTest.mousePress(forward, Qt.LeftButton)
    assert submitted[-1]["direction"] == "FORWARD" and forward.isDown()
    window.speed_slider.setValue(0)
    assert not forward.isDown()
    window.speed_slider.setValue(100)
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"
    QTest.mouseRelease(forward, Qt.LeftButton)
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"
    QTest.mousePress(forward, Qt.LeftButton)
    assert submitted[-1]["direction"] == "FORWARD" and submitted[-1]["speed_percent"] == 100
    QTest.mouseRelease(forward, Qt.LeftButton)


@pytest.mark.parametrize("stop", ["focus", "disconnect"])
def test_continuous_speed_edit_cannot_revive_deactivated_or_disconnected_motion(manual_window, stop):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_direction("FORWARD")
    if stop == "focus":
        window.eventFilter(window, QEvent(QEvent.WindowDeactivate))
    else:
        window._status("连接失败：测试重连")
        continuous(window)
    window.speed_slider.setValue(37)
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"


def test_speed_text_edit_stops_then_enter_returns_driving_focus(manual_window):
    window, submitted, _ = manual_window
    continuous(window)
    window.show()
    window.activateWindow()
    window.setFocus()
    QApplication.processEvents()
    QTest.keyPress(window, Qt.Key_W)
    window.brush_button.setChecked(True)
    window.speed_value.setFocus()
    QApplication.processEvents()
    window._refresh_controls()
    assert window.direction == "STOP" and not window.brush and not window._pressed_keys
    assert window.manual_locked
    window.speed_value.lineEdit().selectAll()
    QTest.keyClicks(window.speed_value.lineEdit(), "3")
    window._refresh_controls()  # A live telemetry/timer update during text entry.
    QTest.keyClicks(window.speed_value.lineEdit(), "7")
    QTest.keyClick(window.speed_value, Qt.Key_Return)
    QApplication.processEvents()
    assert QApplication.focusWidget() is window
    assert window.speed == 37 and not window.manual_locked
    assert window.direction == "STOP"
    QTest.keyRelease(window, Qt.Key_W)
    QTest.keyPress(window, Qt.Key_W)
    assert submitted[-1]["direction"] == "FORWARD" and submitted[-1]["speed_percent"] == 37


@pytest.mark.parametrize("invalid", [None, {}, dict(min=0, max=100, step=1, firmware_protocol_version=2, available=True),
    dict(min=False, max=100, step=1, firmware_protocol_version=3, available=True),
    dict(min=0, max=100, step=1, firmware_protocol_version=3, available=False)])
def test_capability_loss_stops_and_recovery_does_not_restore_old_motion(manual_window, invalid):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_direction("FORWARD")
    window._message(dict(type="telemetry", drive_output=invalid, control_fresh=True, state={}))
    assert window.manual_locked and window.direction == "STOP"
    assert "F103" in window.mode_label.text() or "主驱动" in window.mode_label.text()
    submitted.clear()
    window._send_input()
    assert not submitted
    window._message(dict(type="telemetry", control_fresh=True, state=ready_state(), drive_output=
                         dict(min=0, max=100, step=1, firmware_protocol_version=3, available=True)))
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"


def test_old_gateway_falls_back_to_three_gears_after_continuous_session(manual_window):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_speed(37)
    window._set_direction("FORWARD")
    window._message(dict(type="hello", capabilities=[]))
    window._message(dict(type="telemetry", state={}))
    assert window.speed_controls.isHidden()
    assert window.speed == 25 and window.speed_value.value() == 25
    assert window.direction == "STOP"
    window._set_speed(37)
    assert window.speed == 25
    window.speed_buttons[50].click()
    assert submitted[-1]["speed_percent"] == 50


def test_drive_feedback_distinguishes_rc_scale_targets_and_stale_state(manual_window):
    window, _, _ = manual_window
    continuous(window)
    state = dict(source="RC", speed_percent=25, direction="FORWARD", firmware_protocol_version=3,
                 connected=True, ctrl_age_sec=.01,
                 firmware_protocol_compatible=True, target_pwm_bl_br_fl_fr=[1625, 1625, 1375, 1625],
                 have_sequence=True, ack_sequence=42, sent_sequence=43, result="OK",
                 rc_valid=True, rc_flags=0, rc_sequence=7, rc_age_sec=.02, rc_channels=list(range(14)))
    window._message(dict(type="telemetry", drive_output=window.drive_output, control_fresh=True, state=state))
    assert "遥控器 25 档（独立标度）" in window.drive_feedback.text()
    assert "BL=1625" in window.output_diagnostics.text() and "ACK：42" in window.output_diagnostics.text()
    assert "不是实际转速" in window.output_diagnostics.text()
    assert "只读" in window.rc_diagnostics.text() and "CH14=13" in window.rc_diagnostics.text()
    window.last_telemetry_at = time.monotonic()-1.
    window._refresh_controls()
    assert "已过期" in window.drive_feedback.text()
    assert "已过期" in window.output_diagnostics.text()
    assert "已过期" in window.rc_diagnostics.text()


@pytest.mark.parametrize("ctrl_age,app_age,connected", [(.41, 0., True), (.35, .06, True),
    (-1., 0., True), (float("nan"), 0., True), (0., 0., False)])
def test_fresh_bridge_telemetry_cannot_present_old_ctrl_targets_as_current(manual_window, ctrl_age, app_age, connected):
    window, _, _ = manual_window
    continuous(window)
    window._message(dict(type="telemetry", drive_output=window.drive_output, control_fresh=True,
        state=dict(connected=connected, ctrl_age_sec=ctrl_age, source="NANO", direction="FORWARD",
                   speed_percent=37, target_pwm_bl_br_fl_fr=[1648,1648,1352,1648],
                   firmware_protocol_version=2, firmware_protocol_compatible=False,
                   reason="firmware protocol unavailable", rc_valid=True, rc_age_sec=.01,
                   rc_channels=[1500]*14)))
    window.last_telemetry_at -= app_age
    window._refresh_controls()
    assert "已过期" in window.drive_feedback.text()
    assert "1648" not in window.output_diagnostics.text()
    assert "CTRL 未收到或已过期" in window.output_diagnostics.text()
    assert "v2（不兼容）" in window.output_diagnostics.text()
    assert "firmware protocol unavailable" in window.output_diagnostics.text()
    if connected:
        assert "CH14=1500" in window.rc_diagnostics.text()  # RC age is independent of CTRL.


def test_rc_channel_age_is_independent_and_accumulates_display_delay(manual_window):
    window, _, _ = manual_window
    continuous(window)
    window._message(dict(type="telemetry", drive_output=window.drive_output, control_fresh=True,
        state=dict(connected=True, ctrl_age_sec=.01, source="NANO", speed_percent=37, direction="FORWARD",
                   firmware_protocol_version=3, rc_valid=True, rc_age_sec=.38, rc_channels=[1500]*14)))
    window.last_telemetry_at -= .03
    window._refresh_controls()
    assert "FORWARD · 37%" in window.drive_feedback.text()
    assert "已过期" in window.rc_diagnostics.text() and "CH14=" not in window.rc_diagnostics.text()


def test_gateway_control_cache_age_counts_for_ctrl_ack_and_rc(manual_window):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_direction("FORWARD")
    window._message(dict(type="telemetry", drive_output=window.drive_output, control_fresh=True,
        stream=dict(control_age_ms=200.), state=ready_state(ctrl_age_sec=.25, ack_age_sec=.25,
            direction="FORWARD", rc_valid=True, rc_age_sec=.25, rc_channels=[1500]*14)))
    assert window.manual_locked and window.direction == "STOP"
    assert "已过期" in window.drive_feedback.text()
    assert "已过期" in window.rc_diagnostics.text() and "CH14=" not in window.rc_diagnostics.text()
    submitted.clear()
    window._send_input()
    assert not submitted


@pytest.mark.parametrize("changes", [dict(source="RC"), dict(state="HANDSHAKE"), dict(armed=False),
    dict(result="INVALID"), dict(ack_age_sec=.41), dict(ack_age_sec=-1.), dict(ack_age_sec=float("nan"))])
def test_v3_control_readiness_clears_held_intent_before_recovery(manual_window, changes):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_direction("FORWARD")
    window._pressed_keys.add(Qt.Key_W)
    window.brush_button.setChecked(True)
    window._message(dict(type="telemetry", drive_output=window.drive_output,
                         control_fresh=True, state=ready_state(**changes)))
    assert window.manual_locked and window.direction == "STOP"
    assert not window._pressed_keys and not window.brush
    submitted.clear()
    window._send_input()
    assert not submitted
    window._message(dict(type="telemetry", drive_output=window.drive_output,
                         control_fresh=True, state=ready_state()))
    window._send_input()
    assert not window.manual_locked and submitted[-1]["direction"] == "STOP"


@pytest.mark.parametrize("changes", [dict(connection_id="serial-b"), dict(epoch=2), dict(source="RC", state="RC")])
def test_control_identity_change_sends_stop_even_if_already_ready_again(manual_window, changes):
    window, submitted, _ = manual_window
    continuous(window)
    window._set_direction("FORWARD")
    window._pressed_keys.add(Qt.Key_W)
    window.brush_button.setChecked(True)
    window._message(dict(type="telemetry", drive_output=window.drive_output,
                         control_fresh=True, state=ready_state(**changes)))
    assert window.direction == "STOP" and not window._pressed_keys and not window.brush
    packets = window.worker._take_outgoing()
    assert any(packet.get("reason") == "control_identity_changed" for packet in packets)
    # The old key hold cannot resume when control is ready again.
    recovery = dict(changes, source="NANO", state="READY")
    window._message(dict(type="telemetry", drive_output=window.drive_output,
                         control_fresh=True, state=ready_state(**recovery)))
    window._send_input()
    assert submitted[-1]["direction"] == "STOP"
    window._set_direction("FORWARD")
    assert submitted[-1]["direction"] == "FORWARD"
