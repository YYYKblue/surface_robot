"""Queue/backpressure checks with deterministic sockets, without network access."""
import asyncio
import json
import threading
import time

from operator_gateway.websocket_hub import WebSocketHub


class FakeSocket:
    def __init__(self, blocked=False):
        self.incoming = asyncio.Queue()
        self.gate = asyncio.Event()
        if not blocked:
            self.gate.set()
        self.started = asyncio.Event()
        self.sent = []
        self.close_calls = []
        self.concurrent_sends = 0
        self.max_concurrent_sends = 0

    def __aiter__(self):
        return self

    async def __anext__(self):
        value = await self.incoming.get()
        if value is None:
            raise StopAsyncIteration
        return value

    async def send(self, encoded):
        self.concurrent_sends += 1
        self.max_concurrent_sends = max(self.max_concurrent_sends, self.concurrent_sends)
        self.started.set()
        try:
            await self.gate.wait()
            self.sent.append(json.loads(encoded))
        finally:
            self.concurrent_sends -= 1

    async def close(self, code=1000, reason=""):
        self.close_calls.append((code, reason))
        await self.incoming.put(None)


async def eventually(predicate):
    async def wait():
        while not predicate():
            await asyncio.sleep(0)
    await asyncio.wait_for(wait(), 1)


async def connect(hub, socket):
    hub.loop = asyncio.get_running_loop()
    handler = asyncio.create_task(hub._handler(socket))
    await eventually(lambda: hub.stats()["client_count"] == 1)
    client_id = next(iter(hub.stats()["clients"]))
    return client_id, handler


def test_slow_sender_coalesces_streams_and_cross_thread_wakeups():
    async def run():
        hub = WebSocketHub("", 0, None)
        socket = FakeSocket(blocked=True)
        client_id, handler = await connect(hub, socket)
        hub.send(client_id, {"type": "map", "seq": -1})
        await socket.started.wait()

        def produce():
            for sequence in range(2000):
                for kind in ("map", "tracks", "telemetry"):
                    assert hub.send(client_id, {"type": kind, "seq": sequence})
            for kind in ("hello", "reply", "error", "diagnostics"):
                assert hub.send(client_id, {"type": kind})

        producer = threading.Thread(target=produce)
        producer.start()
        producer.join()
        metrics = hub.stats()
        assert metrics["pending_latest"] == 3
        assert metrics["pending_reliable"] == 4
        assert metrics["in_flight"] == 1
        assert metrics["replaced"] == 3 * 1999
        # Even thousands of cross-thread sends add no callbacks while busy.
        assert metrics["wakeups"] == 1
        assert metrics["queued_wakeups"] == 0
        socket.gate.set()
        await eventually(lambda: len(socket.sent) == 8)
        assert [packet["type"] for packet in socket.sent] == [
            "map", "hello", "reply", "error", "diagnostics", "telemetry", "tracks", "map"]
        assert [packet["seq"] for packet in socket.sent[-3:]] == [1999] * 3
        assert socket.max_concurrent_sends == 1
        await socket.close()
        await handler
        assert hub.stats()["client_count"] == 0
        assert hub.stats()["pending_latest"] == 0
    asyncio.run(run())


def test_cross_thread_wakeup_is_bounded_before_loop_consumes_it():
    async def run():
        hub = WebSocketHub("", 0, None)
        socket = FakeSocket()
        client_id, handler = await connect(hub, socket)

        def produce():
            for sequence in range(3000):
                hub.send(client_id, {"type": "telemetry", "seq": sequence})

        producer = threading.Thread(target=produce)
        producer.start()
        producer.join()
        assert hub.stats()["queued_wakeups"] == 1
        assert hub.stats()["wakeups"] == 1
        assert hub.stats()["pending_latest"] == 1
        await eventually(lambda: len(socket.sent) == 1)
        assert socket.sent[0]["seq"] == 2999
        # After a completed idle transition, a new snapshot gets one new wake.
        await eventually(lambda: not next(iter(hub.clients.values())).wake_armed)
        hub.send(client_id, {"type": "telemetry", "seq": 3000})
        await eventually(lambda: len(socket.sent) == 2)
        assert hub.stats()["wakeups"] == 2
        await socket.close()
        await handler
    asyncio.run(run())


def test_reliable_overflow_disconnects_instead_of_dropping_reply_silently():
    async def run():
        hub = WebSocketHub("", 0, None, reliable_queue_size=2)
        socket = FakeSocket()
        client_id, handler = await connect(hub, socket)
        assert hub.send(client_id, {"type": "reply", "id": 1})
        assert hub.send(client_id, {"type": "reply", "id": 2})
        assert not hub.send(client_id, {"type": "reply", "id": 3})
        assert not hub.send(client_id, {"type": "telemetry"})
        assert hub.stats()["pending_reliable"] == 2
        await asyncio.wait_for(handler, 1)
        assert socket.close_calls == [(1013, "reliable outbound queue overflow")]
        assert hub.stats()["reliable_overflows"] == 1
        assert hub.stats()["pending_reliable"] == 0
        assert hub.poll() == [("connected", client_id), ("disconnected", client_id)]
    asyncio.run(run())


def test_send_timeout_closes_socket_and_releases_pending_data():
    async def run():
        hub = WebSocketHub("", 0, None, send_timeout_sec=.02)
        socket = FakeSocket(blocked=True)
        client_id, handler = await connect(hub, socket)
        hub.send(client_id, {"type": "map"})
        await socket.started.wait()
        hub.send(client_id, {"type": "reply", "id": 1})
        hub.send(client_id, {"type": "telemetry", "seq": 1})
        await asyncio.wait_for(handler, 1)
        assert socket.close_calls == [(1013, "outbound send timed out")]
        metrics = hub.stats()
        assert metrics["send_timeouts"] == 1
        assert metrics["client_count"] == 0
        assert metrics["pending_reliable"] == metrics["pending_latest"] == 0
        assert socket.concurrent_sends == 0
    asyncio.run(run())


def test_disconnect_cancels_inflight_send_and_rejects_late_packets():
    async def run():
        hub = WebSocketHub("", 0, None)
        socket = FakeSocket(blocked=True)
        client_id, handler = await connect(hub, socket)
        hub.send(client_id, {"type": "map"})
        await socket.started.wait()
        hub.send(client_id, {"type": "reply", "id": 1})
        await socket.close()
        await asyncio.wait_for(handler, 1)
        assert socket.concurrent_sends == 0
        assert hub.stats()["client_count"] == 0
        assert not hub.send(client_id, {"type": "reply"})
    asyncio.run(run())


def test_telemetry_queue_age_uses_generation_time_without_mutating_packet():
    async def run():
        hub = WebSocketHub("", 0, None)
        socket = FakeSocket()
        client_id, handler = await connect(hub, socket)
        packet = {"type": "telemetry", "stream": {"seq": 5,
                  "generated_monotonic": time.monotonic() - 1}}
        hub.send(client_id, packet)
        await eventually(lambda: len(socket.sent) == 1)
        assert socket.sent[0]["stream"]["server_queue_ms"] >= 1000
        assert "server_queue_ms" not in packet["stream"]
        assert socket.sent[0]["stream"]["seq"] == 5
        assert "clients" not in hub.stats(include_clients=False)
        assert hub.stats()["max_serialize_ms"] >= 0
        await socket.close()
        await handler
    asyncio.run(run())


def test_packet_flood_keeps_lifecycle_events():
    hub = WebSocketHub("", 0, None)
    hub._event(("connected", "a"))
    for sequence in range(200):
        hub._event(("packet", "a", str(sequence)))
    hub._event(("disconnected", "a"))
    events = hub.poll()
    assert len(events) == 128
    assert events[0] == ("connected", "a")
    assert events[-1] == ("disconnected", "a")


def test_continuous_telemetry_cannot_starve_tracks_or_map_on_slow_socket():
    class SteppedSocket(FakeSocket):
        def __init__(self):
            super().__init__()
            self.entered = asyncio.Queue()
            self.permits = asyncio.Semaphore(0)

        async def send(self, encoded):
            packet = json.loads(encoded)
            await self.entered.put(packet)
            await self.permits.acquire()
            self.sent.append(packet)

    async def run():
        hub = WebSocketHub("", 0, None)
        socket = SteppedSocket()
        client_id, handler = await connect(hub, socket)
        for kind in ("telemetry", "tracks", "map"):
            hub.send(client_id, {"type": kind, "seq": 0})
        for sequence, expected in enumerate(("telemetry", "tracks", "telemetry", "map"), 1):
            packet = await asyncio.wait_for(socket.entered.get(), 1)
            assert packet["type"] == expected
            # Both telemetry and tracks remain pending throughout the run.
            # Map still gets a turn after the second telemetry transmission.
            hub.send(client_id, {"type": "telemetry", "seq": sequence})
            hub.send(client_id, {"type": "tracks", "seq": sequence})
            socket.permits.release()
        await eventually(lambda: len(socket.sent) == 4)
        await socket.close()
        await handler
    asyncio.run(run())
