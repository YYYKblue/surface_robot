"""Real WebSocket framing under slow delivery, without ROS or hardware.

The socket pair deliberately drips a single complete message. Individual reads
make progress inside the 50 ms timeout while the whole message takes >800 ms.
"""
from contextlib import contextmanager
import json
import socket
import threading
import time

import pytest

websocket = pytest.importorskip("websocket")

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

from operator_app.transport import SocketWorker


def manual(direction="FORWARD"):
    return dict(op="manual_input", direction=direction, seq=1, speed_percent=25,
                mode="BOTH", brush_enabled=False, brush_percent=0,
                thrust_enabled=False, thrust_percent=0)


def wait_until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(.005)
    assert predicate(), "condition was not satisfied before deadline"


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


class Peer:
    def __init__(self, raw_socket):
        self.socket = raw_socket
        self.ws = websocket.WebSocket(enable_multithread=True)
        self.ws.sock = raw_socket
        self.ws.connected = True
        self.ws.settimeout(.05)
        self.done = threading.Event()
        self.condition = threading.Condition()
        self.messages = []
        self.errors = []
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while not self.done.is_set():
                try:
                    raw = self.ws.recv()
                except websocket.WebSocketTimeoutException:
                    continue
                if not raw:
                    return
                packet = json.loads(raw)
                with self.condition:
                    self.messages.append((time.monotonic(), packet))
                    self.condition.notify_all()
        except Exception as error:
            if not self.done.is_set():
                self.errors.append(error)

    def snapshot(self):
        with self.condition:
            return list(self.messages)

    def send(self, packet):
        frame = websocket.ABNF.create_frame(json.dumps(packet),
                                           websocket.ABNF.OPCODE_TEXT)
        frame.mask_value = 0
        self.socket.sendall(frame.format())

    def drip(self, packet, *, chunk=128, interval=.03):
        started, finished = threading.Event(), threading.Event()
        message = websocket.ABNF.create_frame(json.dumps(packet, separators=(",", ":")),
                                             websocket.ABNF.OPCODE_TEXT)
        message.mask_value = 0
        frame = message.format()

        def run():
            try:
                for offset in range(0, len(frame), chunk):
                    if self.done.is_set():
                        return
                    self.socket.sendall(frame[offset:offset + chunk])
                    started.set()
                    if offset + chunk < len(frame) and self.done.wait(interval):
                        return
            except (OSError, websocket.WebSocketException):
                # Closing the client intentionally interrupts unfinished frames.
                pass
            finally:
                finished.set()

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        return started, finished, thread

    def close(self):
        self.done.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.reader.join(1)
        self.socket.close()


@contextmanager
def connected_worker(monkeypatch):
    client_socket, peer_socket = socket.socketpair()
    client = websocket.WebSocket(enable_multithread=True)
    client.sock = client_socket
    client.connected = True
    attempts = []

    def connect(*args, **kwargs):
        attempts.append(time.monotonic())
        if len(attempts) > 1:
            raise ConnectionError("isolated test has no reconnect server")
        return client

    monkeypatch.setattr(websocket, "create_connection", connect)
    peer = Peer(peer_socket)
    worker = SocketWorker("ws://isolated-test")
    worker.start()
    try:
        wait_until(lambda: any(p.get("op") == "hello" for _, p in peer.snapshot()))
        peer.send(dict(type="hello", session_id="test", client_id="client", capabilities=[]))
        hello = []

        def got_hello():
            hello.extend(p for p in worker.take_incoming() if p.get("type") == "hello")
            return bool(hello)

        wait_until(got_hello)
        assert worker.activate_manual_session(hello[-1]["_connection_generation"])
        yield worker, peer, client, attempts
    finally:
        worker.close()
        assert worker.wait(2000), "socket worker did not exit after close"
        peer.close()


def test_slow_complete_message_does_not_starve_manual_input_or_stop(app, monkeypatch):
    with connected_worker(monkeypatch) as (worker, peer, _, _):
        assert worker.update_manual_intent(manual())
        wait_until(lambda: any(p.get("op") == "manual_input" for _, p in peer.snapshot()))
        # Model GUI liveness updates with 150 ms jitter, while the sender must
        # retain its independent 50 ms cadence.
        producer_done = threading.Event()
        producer_errors = []

        def refresh():
            while not producer_done.wait(.15):
                if not worker.update_manual_intent(manual()):
                    producer_errors.append("fresh GUI intent was rejected")
                    return

        producer = threading.Thread(target=refresh, daemon=True)
        producer.start()
        packet = dict(type="telemetry", padding="x" * 3967)
        start_time = time.monotonic()
        started, finished, dripper = peer.drip(packet)
        try:
            assert started.wait(1)
            wait_until(lambda: len([p for t, p in peer.snapshot()
                                   if t >= start_time and p.get("op") == "manual_input"]) >= 7)
            assert not finished.is_set(), "test message completed before exercising slow receive"
            producer_done.set()
            producer.join(1)
            stop_at = time.monotonic()
            assert worker.send_packet(dict(op="manual_stop", reason="button"))
            wait_until(lambda: any(p.get("op") == "manual_stop" and p.get("reason") == "button"
                                   for _, p in peer.snapshot()), timeout=.25)
            stop_received = next(t for t, p in peer.snapshot()
                                 if p.get("op") == "manual_stop" and p.get("reason") == "button")
            assert stop_received - stop_at < .25
            assert not finished.is_set(), "stop must arrive before the downlink frame finishes"
            arrivals = [t for t, p in peer.snapshot() if p.get("op") == "manual_input"]
            assert max(b - a for a, b in zip(arrivals, arrivals[1:])) < .25
            inputs = [p for _, p in peer.snapshot() if p.get("op") == "manual_input"]
            assert all(a["seq"] < b["seq"] for a, b in zip(inputs, inputs[1:]))
            assert not producer_errors
            assert finished.wait(2)
            assert time.monotonic() - start_time > .8
            received = []

            def got_telemetry():
                received.extend(p for p in worker.take_incoming() if p.get("type") == "telemetry")
                return bool(received)

            wait_until(got_telemetry)
            assert received[-1]["padding"] == packet["padding"]
            assert not any(p.get("op") == "manual_input" for t, p in peer.snapshot() if t > stop_received)
        finally:
            producer_done.set()
            producer.join(1)
            dripper.join(2)


def test_close_interrupts_a_partial_slow_frame(app, monkeypatch):
    with connected_worker(monkeypatch) as (worker, peer, _, _):
        started, finished, dripper = peer.drip(dict(type="telemetry", padding="x" * 12000))
        assert started.wait(1)
        worker.close()
        assert worker.wait(1000), "close left the receiver waiting for a complete frame"
        # The peer may see the aborted socket immediately; it must not need to
        # deliver the remaining several seconds of data to allow client exit.
        peer.done.set()
        dripper.join(1)


def test_sender_failure_interrupts_receiver_and_retires_connection(app, monkeypatch):
    with connected_worker(monkeypatch) as (worker, peer, client, attempts):
        started, finished, dripper = peer.drip(dict(type="telemetry", padding="x" * 12000))
        assert started.wait(1)
        failed = threading.Event()
        original = client.send

        def send(raw, *args, **kwargs):
            if json.loads(raw).get("op") == "manual_input":
                failed.set()
                raise OSError("injected sender failure")
            return original(raw, *args, **kwargs)

        monkeypatch.setattr(client, "send", send)
        assert worker.update_manual_intent(manual())
        assert failed.wait(1)
        wait_until(lambda: len(attempts) >= 2, timeout=2)
        assert not worker.update_manual_intent(manual())
        peer.done.set()
        dripper.join(1)
