"""RTT boundaries, loss, stale replies and independent surface/underwater UI."""
import pytest

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from operator_app.network_latency import NetworkLatency
from operator_app.transport import SocketWorker
from operator_app.window import OperatorWindow


@pytest.fixture
def clock(monkeypatch):
    now = [0.]
    monkeypatch.setattr("operator_app.network_latency.time.monotonic", lambda: now[0])
    return now


@pytest.mark.parametrize("delay,level", [(.02, "healthy"), (.099, "healthy"),
    (.1, "acceptable"), (.249, "acceptable"), (.25, "slow"), (.4, "slow")])
def test_rtt_thresholds(clock, delay, level):
    monitor = NetworkLatency()
    monitor.connect()
    token = monitor.probe()
    clock[0] = delay
    assert monitor.pong(token, clock[0])
    assert monitor.snapshot() == (level, delay * 1000)
    assert not monitor.pong(token, clock[0])  # Duplicate cannot refresh RTT.


def test_lost_probe_stays_red_until_a_matching_reply(clock):
    monitor = NetworkLatency()
    monitor.connect()
    assert monitor.snapshot() == ("waiting", None)
    first = monitor.probe()
    clock[0] = .02
    monitor.pong(first, clock[0])
    assert monitor.snapshot()[0] == "healthy"
    assert monitor.probe() is None
    clock[0] = .5
    lost = monitor.probe()
    clock[0] = .75
    assert monitor.snapshot() == ("timeout", None)
    clock[0] = 1.
    current = monitor.probe()
    assert monitor.snapshot() == ("timeout", None)
    assert not monitor.pong(lost, clock[0])
    assert not monitor.pong(b"unrelated server pong", clock[0])
    clock[0] = 1.02
    assert monitor.pong(current, clock[0])
    assert monitor.snapshot() == ("healthy", 20.)


def test_reconnect_discards_previous_value_and_outstanding_reply(clock):
    monitor = NetworkLatency()
    monitor.connect()
    old = monitor.probe()
    monitor.disconnect()
    assert monitor.snapshot() == ("disconnected", None)
    assert monitor.probe() is None
    assert not monitor.pong(old, .01)
    clock[0] = 1.
    monitor.connect()
    new = monitor.probe()
    assert new != old
    assert not monitor.pong(old, 1.01)
    assert monitor.snapshot() == ("waiting", None)
    clock[0] = 1.03
    assert monitor.pong(new, clock[0])
    assert monitor.snapshot() == ("healthy", 30.)


def test_stalled_sender_cannot_leave_green_or_waiting_forever(clock):
    monitor = NetworkLatency()
    monitor.connect()
    clock[0] = .75
    assert monitor.snapshot() == ("stale", None)
    token = monitor.probe()
    clock[0] = .77
    monitor.pong(token, clock[0])
    clock[0] = 1.5
    assert monitor.snapshot() == ("stale", None)


def test_mock_never_reports_a_measured_rtt(clock):
    monitor = NetworkLatency(mock=True)
    monitor.connect()
    assert monitor.probe() is None
    assert monitor.snapshot() == ("mock", None)


def test_dual_ui_shows_independent_colors_and_never_unlocks_controls(monkeypatch, clock):
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(SocketWorker, "start", lambda self: None)
    window = OperatorWindow("ws://nano:8765", under_url="ws://nano:8766")
    window.send_timer.stop()
    window.state_timer.stop()
    panel = window.under_panel
    panel.timer.stop()
    try:
        surface, under = window.worker.latency, panel.worker.latency
        surface.connect()
        under.connect()
        a, b = surface.probe(), under.probe()
        clock[0] = .025
        surface.pong(a, clock[0])
        clock[0] = .150
        under.pong(b, clock[0])
        window._refresh_controls()
        panel.refresh()
        assert window.network_latency.property("latencyState") == "healthy"
        assert "25.0 ms" in window.network_latency.text()
        assert "#16803c" in window.network_latency.styleSheet()
        assert panel.network_latency.property("latencyState") == "acceptable"
        assert "150.0 ms" in panel.network_latency.text()
        assert "#a66b00" in panel.network_latency.styleSheet()
        assert window.manual_locked and panel.locked  # No telemetry/handshake.
        under.disconnect()
        panel.refresh()
        assert panel.network_latency.property("latencyState") == "disconnected"
        assert "150.0" not in panel.network_latency.text()
        assert "#c62828" in panel.network_latency.styleSheet()
        assert window.network_latency.property("latencyState") == "healthy"
    finally:
        window.close()
        app.processEvents()
