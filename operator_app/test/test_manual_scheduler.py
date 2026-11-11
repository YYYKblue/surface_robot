"""Manual intent freshness and wire ordering without sockets or hardware."""
import json

import pytest

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from operator_app.transport import SocketWorker


def manual(direction="FORWARD", seq=1):
    return dict(op="manual_input", seq=seq, direction=direction,
                speed_percent=25, mode="BOTH", brush_enabled=False,
                brush_percent=0, thrust_enabled=False, thrust_percent=0)


class RecordingSocket:
    def __init__(self):
        self.sent = []

    def send(self, raw):
        self.sent.append(json.loads(raw))

    def abort(self):
        pass


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def scheduler(app, monkeypatch):
    clock = [100.]
    monkeypatch.setattr("operator_app.transport.time.monotonic", lambda: clock[0])
    worker, socket = SocketWorker("unused"), RecordingSocket()
    generation = worker._begin_connection(socket)
    assert worker.activate_manual_session(generation)
    try:
        yield worker, socket, clock
    finally:
        worker.close()
        worker._end_connection()


def test_background_cadence_survives_sparse_gui_updates_and_allocates_wire_sequences(scheduler):
    worker, socket, clock = scheduler
    # The GUI refreshes only every 150 ms. The transport still supplies a new
    # wire sequence every 50 ms rather than repeating an old gateway sequence.
    for tick in range(10):
        clock[0] = 100. + tick * .051
        if tick % 3 == 0:
            assert worker.update_manual_intent(manual(seq=1))
        worker._flush(socket)
    inputs = [p for p in socket.sent if p["op"] == "manual_input"]
    assert len(inputs) == 10
    assert all(p["direction"] == "FORWARD" for p in inputs)
    assert all(a["seq"] < b["seq"] for a, b in zip(inputs, inputs[1:]))
    assert not worker.manual_intent_expired


def test_gui_stall_stops_once_and_cannot_refresh_old_motion_before_ack(scheduler):
    worker, socket, clock = scheduler
    assert worker.update_manual_intent(manual())
    worker._flush(socket)
    clock[0] = 100.251
    worker._flush(socket)
    assert worker.manual_intent_expired
    assert socket.sent[-1]["op"] == "manual_stop"
    assert socket.sent[-1]["reason"] == "gui_input_expired"
    stopped_count = len(socket.sent)
    clock[0] = 101.
    assert not worker.update_manual_intent(manual())
    worker._flush(socket)
    assert len(socket.sent) == stopped_count
    # This models the GUI first clearing all local buttons and held keys.
    worker.clear_manual_intent()
    worker.acknowledge_manual_expiry()
    assert worker.update_manual_intent(manual("STOP"))
    worker._flush(socket)
    assert socket.sent[-1]["direction"] == "STOP"


def test_gui_resume_cannot_beat_sender_to_revive_expired_snapshot(scheduler):
    worker, socket, clock = scheduler
    assert worker.update_manual_intent(manual())
    worker._flush(socket)
    clock[0] = 100.251
    # No sender tick occurred during the gap. The producer itself must detect
    # expiry before overwriting the old timestamp with a new heartbeat.
    assert not worker.update_manual_intent(manual())
    worker._flush(socket)
    assert worker.manual_intent_expired
    assert socket.sent[-1]["op"] == "manual_stop"
    assert len([p for p in socket.sent if p["op"] == "manual_input"]) == 1


def test_release_invalidates_movement_already_extracted_for_send(scheduler, monkeypatch):
    worker, socket, clock = scheduler
    worker.send_packet(dict(op="mapping", action="list", request_id="list"))
    assert worker.update_manual_intent(manual())
    original = worker._send_now

    def send(ws, packet):
        if packet["op"] == "mapping":
            # A release arrives while an earlier operation is being written.
            assert worker.update_manual_intent(manual("STOP"))
        original(ws, packet)

    monkeypatch.setattr(worker, "_send_now", send)
    worker._flush(socket)
    clock[0] += .051
    worker._flush(socket)
    assert not any(p.get("direction") == "FORWARD" for p in socket.sent)
    assert any(p.get("direction") == "STOP" for p in socket.sent)


@pytest.mark.parametrize("operation", [
    dict(op="manual_stop", reason="button"),
    dict(op="task", action="start", request_id="task"),
])
def test_stop_and_discrete_operation_cancel_repeating_intent(scheduler, operation):
    worker, socket, clock = scheduler
    assert worker.update_manual_intent(manual())
    worker._flush(socket)
    assert worker.send_packet(operation)
    worker._flush(socket)
    boundary = len(socket.sent)
    clock[0] += .1
    worker._flush(socket)
    assert len(socket.sent) == boundary
    assert socket.sent[-1]["op"] == operation["op"]


def test_disconnect_and_stale_hello_cannot_restore_old_intent(scheduler):
    worker, socket, clock = scheduler
    old_generation = worker.connection_generation
    assert worker.update_manual_intent(manual())
    worker._flush(socket)
    worker._end_connection()
    assert not worker.update_manual_intent(manual())
    next_socket = RecordingSocket()
    new_generation = worker._begin_connection(next_socket)
    assert new_generation != old_generation
    assert not worker.activate_manual_session(old_generation)
    assert not worker.update_manual_intent(manual())
    worker._flush(next_socket)
    assert not any(p.get("direction") == "FORWARD" for p in next_socket.sent)
    assert worker.activate_manual_session(new_generation)
    assert worker.update_manual_intent(manual("STOP"))
    worker._flush(next_socket)
    assert next_socket.sent[-1]["direction"] == "STOP"


def test_queue_full_notification_allows_gui_to_cancel_intent(scheduler):
    worker, _, _ = scheduler
    lock_was_available = []

    def on_status(message):
        # A direct GUI callback can refresh controls and cancel the snapshot.
        # Check without blocking so a regression fails rather than deadlocking.
        available = worker.queue_lock.acquire(blocking=False)
        lock_was_available.append(available)
        if available:
            worker.queue_lock.release()
            worker.clear_manual_intent()

    worker.status.connect(on_status)
    for index in range(32):
        assert worker.send_packet(dict(op="task", request_id=str(index)))
    assert not worker.send_packet(dict(op="task", request_id="overflow"))
    assert lock_was_available == [True]


def test_speed_changes_replace_extracted_intent_and_zero_keeps_auxiliaries(scheduler, monkeypatch):
    worker, socket, clock = scheduler
    worker.send_packet(dict(op="mapping", action="list", request_id="list"))
    assert worker.update_manual_intent(dict(manual(), speed_percent=100, brush_enabled=True, brush_percent=50))
    original = worker._send_now

    def send(ws, packet):
        if packet["op"] == "mapping":
            # The UI has cleared the held direction when the slider hit zero.
            worker.update_manual_intent(dict(manual("STOP"), speed_percent=0, brush_enabled=True, brush_percent=50))
            worker.update_manual_intent(dict(manual("STOP"), speed_percent=37, brush_enabled=True, brush_percent=50))
        original(ws, packet)

    monkeypatch.setattr(worker, "_send_now", send)
    worker._flush(socket)
    clock[0] += .051
    worker._flush(socket)
    packets = [packet for packet in socket.sent if packet["op"] == "manual_input"]
    assert len(packets) == 1
    assert packets[0]["speed_percent"] == 37 and packets[0]["direction"] == "STOP"
    assert packets[0]["brush_enabled"] is True


def test_mock_reports_percentage_targets_and_zero_only_stops_main_drive():
    state = dict(mode="BOTH", speed_percent=25, direction="STOP", sent_sequence=0)
    state = SocketWorker._mock_drive_state(dict(manual(), speed_percent=37, brush_enabled=True, brush_percent=50), state)
    assert state["target_pwm_bl_br_fl_fr"] == [1648, 1648, 1352, 1648]
    assert state["ack_sequence"] == state["sent_sequence"] == 1
    state = SocketWorker._mock_drive_state(dict(manual(), speed_percent=0, brush_enabled=True, brush_percent=50), state)
    assert state["direction"] == "STOP" and state["target_pwm_bl_br_fl_fr"] == [1500]*4
    assert state["brush_enabled"] is True
    state = SocketWorker._mock_drive_state(dict(op="manual_stop"), state)
    assert not state["brush_enabled"] and not state["thrust_enabled"]
