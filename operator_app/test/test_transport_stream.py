"""Bounded receive delivery and background map decoding, without network/hardware."""
import base64
import threading
import time
import zlib

import pytest

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from operator_app import map_decoder
from operator_app.map_decoder import MapDecoder, prepare_map
from operator_app.transport import SocketWorker


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def compressed_map(revision=1):
    return dict(type="map", map=dict(width=3, height=2, resolution=.1,
        origin=dict(x=-2., y=3., yaw=.5), frame_id="map", revision=revision,
        encoding="indexed8_zlib",
        data_zlib_base64=base64.b64encode(zlib.compress(bytes([0, 1, 2, 2, 1, 0]))).decode()))


def test_receive_burst_keeps_one_notification_and_latest_state_with_replies(app):
    worker = SocketWorker("unused")
    notifications = []
    worker.message.connect(notifications.append)
    for i in range(2000):
        worker._receive(dict(type="telemetry", sequence=i))
        worker._receive(dict(type="tracks", revision=i))
        if i in (100, 500, 1000):
            worker._receive(dict(type="reply", request_id=str(i)))
    assert notifications == [dict(type="incoming_ready")]
    packets = worker.take_incoming()
    assert len(packets) == 5
    assert [p["request_id"] for p in packets if p["type"] == "reply"] == ["100", "500", "1000"]
    assert next(p for p in packets if p["type"] == "telemetry")["sequence"] == 1999
    assert next(p for p in packets if p["type"] == "tracks")["revision"] == 1999
    worker._receive(dict(type="telemetry", sequence=2000))
    assert len(notifications) == 2
    worker.close()


def test_receive_timestamp_survives_gui_delay_and_reliable_overflow_is_explicit(app, monkeypatch):
    worker = SocketWorker("unused")
    clock = [100.]
    monkeypatch.setattr("operator_app.transport.time.monotonic", lambda: clock[0])
    worker._receive(dict(type="telemetry"))
    clock[0] = 102.
    packet = worker.take_incoming()[0]
    assert packet["_received_monotonic"] == 100.
    assert packet["_transport"]["gui_queue_ms"] == 2000.
    for i in range(64):
        worker._receive(dict(type="reply", request_id=str(i)))
    with pytest.raises(ConnectionError):
        worker._receive(dict(type="reply", request_id="overflow"))
    assert len(worker.take_incoming()) == 64
    worker.close()


def test_old_manual_intent_is_not_sent_after_transport_stall(app, monkeypatch):
    worker = SocketWorker("unused")
    clock = [100.]
    monkeypatch.setattr("operator_app.transport.time.monotonic", lambda: clock[0])
    worker.send_packet(dict(op="manual_input", direction="FORWARD"))
    clock[0] = 100.3
    assert worker._take_outgoing() == []
    worker.send_packet(dict(op="manual_stop", reason="button"))
    assert worker._take_outgoing() == [dict(op="manual_stop", reason="button")]
    worker.close()


def test_manual_stop_has_reserved_capacity_and_clears_old_movement(app):
    worker = SocketWorker("unused")
    for i in range(32):
        assert worker.send_packet(dict(op="task", request_id=str(i)))
    worker.send_packet(dict(op="manual_input", direction="FORWARD"))
    assert worker.send_packet(dict(op="manual_stop", reason="button"))
    packets = worker._take_outgoing()
    assert packets[0] == dict(op="manual_stop", reason="button")
    assert [p["request_id"] for p in packets[1:]] == [str(i) for i in range(32)]
    assert not any(p["op"] == "manual_input" for p in packets)
    worker.close()


def test_disconnect_keeps_received_replies_and_existing_gui_wakeup(app):
    worker = SocketWorker("unused")
    notifications = []
    worker.message.connect(notifications.append)
    worker._receive(dict(type="reply", request_id="accepted", success=True))
    worker._receive(dict(type="telemetry", sequence=1))
    for _ in range(20):
        worker._reset_incoming()
        worker._receive(dict(type="telemetry", sequence=2))
    assert len(notifications) == 1
    packets = worker.take_incoming()
    assert packets[0]["request_id"] == "accepted"
    assert packets[1]["sequence"] == 2
    worker.close()


def test_stop_during_send_preempts_batch_and_invalidates_extracted_movement(app, monkeypatch):
    worker = SocketWorker("unused")
    worker.send_packet(dict(op="mapping", action="list"))
    worker.send_packet(dict(op="task", request_id="next"))
    worker.send_packet(dict(op="manual_input", direction="FORWARD"))
    sent = []

    def send(ws, packet):
        sent.append(packet["op"])
        if packet["op"] == "mapping":
            worker.send_packet(dict(op="manual_stop", reason="button"))

    monkeypatch.setattr(worker, "_send_now", send)
    worker._flush(None)
    assert sent == ["mapping", "manual_stop", "task"]
    assert worker._take_outgoing() == []
    worker.close()


def test_manual_age_is_rechecked_after_slow_discrete_sends(app, monkeypatch):
    worker = SocketWorker("unused")
    clock, sent = [100.], []
    monkeypatch.setattr("operator_app.transport.time.monotonic", lambda: clock[0])
    worker.send_packet(dict(op="task", request_id="one"))
    worker.send_packet(dict(op="manual_input", direction="FORWARD"))

    def send(ws, packet):
        sent.append(packet["op"])
        clock[0] += .3

    monkeypatch.setattr(worker, "_send_now", send)
    worker._flush(None)
    assert sent == ["task"]
    worker.close()


def test_compressed_map_preserves_geometry_and_vertical_orientation(app):
    packet = compressed_map()
    prepared = prepare_map(packet)
    image = prepared["_map_image"]
    assert image.width() == 3 and image.height() == 2
    assert [image.pixelIndex(x, 0) for x in range(3)] == [2, 1, 0]
    assert [image.pixelIndex(x, 1) for x in range(3)] == [0, 1, 2]
    assert prepared["map"]["origin"] == packet["map"]["origin"]
    assert "data_zlib_base64" not in prepared["map"]
    assert "data_zlib_base64" in packet["map"]  # Input remains immutable.
    packet["map"]["width"] = 2
    with pytest.raises(ValueError):
        prepare_map(packet)


def test_map_decode_keeps_latest_and_reset_rejects_running_result(app, monkeypatch):
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    delivered, decoded = [], []
    original = map_decoder.prepare_map

    def slow_prepare(packet):
        decoded.append(packet["map"]["revision"])
        if packet["map"]["revision"] == 1:
            entered.set()
            assert release.wait(2)
        return original(packet)

    def deliver(packet, order):
        delivered.append((packet["map"]["revision"], order))
        done.set()

    monkeypatch.setattr(map_decoder, "prepare_map", slow_prepare)
    errors = []
    decoder = MapDecoder(deliver, errors.append)
    try:
        decoder.submit(compressed_map(1), 1)
        assert entered.wait(2)
        decoder.submit(compressed_map(2), 2)
        decoder.reset()
        decoder.submit(compressed_map(3), 3)
        release.set()
        assert done.wait(2)
        assert decoded == [1, 3]
        assert delivered == [(3, 3)]
        assert not errors
    finally:
        release.set()
        decoder.close()


def test_map_decode_does_not_block_telemetry_and_clear_invalidates_it(app, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = map_decoder.prepare_map

    def slow_prepare(packet):
        entered.set()
        assert release.wait(2)
        return original(packet)

    monkeypatch.setattr(map_decoder, "prepare_map", slow_prepare)
    worker = SocketWorker("unused")
    try:
        worker._receive(compressed_map())
        assert entered.wait(2)
        worker._receive(dict(type="telemetry", pose=dict(x=1., y=2., yaw=0.)))
        assert worker.take_incoming()[0]["type"] == "telemetry"
        worker._receive(dict(type="map", map=None))
        release.set()
        worker.map_decoder.close()
        assert [(p["type"], p.get("map")) for p in worker.take_incoming()] == [("map", None)]
    finally:
        release.set()
        worker.close()


def test_maps_arriving_faster_than_decode_still_deliver_running_then_latest(app, monkeypatch):
    first_entered, first_release = threading.Event(), threading.Event()
    latest_entered, latest_release = threading.Event(), threading.Event()
    first_delivered, latest_delivered = threading.Event(), threading.Event()
    decoded, delivered, errors = [], [], []
    original = map_decoder.prepare_map

    def slow_prepare(packet):
        revision = packet["map"]["revision"]
        decoded.append(revision)
        if revision == 1:
            first_entered.set()
            assert first_release.wait(2)
        elif revision == 50:
            latest_entered.set()
            assert latest_release.wait(2)
        return original(packet)

    def deliver(packet, order):
        delivered.append((packet["map"]["revision"], order))
        (first_delivered if order == 1 else latest_delivered).set()

    monkeypatch.setattr(map_decoder, "prepare_map", slow_prepare)
    decoder = MapDecoder(deliver, errors.append)
    try:
        decoder.submit(compressed_map(1), 1)
        assert first_entered.wait(2)
        for revision in range(2, 51):
            decoder.submit(compressed_map(revision), revision)
        first_release.set()
        assert first_delivered.wait(2)
        assert latest_entered.wait(2)
        assert delivered == [(1, 1)]
        assert decoded == [1, 50]
        latest_release.set()
        assert latest_delivered.wait(2)
        assert delivered == [(1, 1), (50, 50)]
        assert not errors
    finally:
        first_release.set()
        latest_release.set()
        decoder.close()
