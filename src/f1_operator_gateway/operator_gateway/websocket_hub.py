"""Bounded WebSocket queues outside the ROS executor thread.

A client has one sender, a bounded FIFO for operation replies, and one pending
snapshot per replaceable stream. Cross-thread wakeups are coalesced for the
entire busy period, so a slow socket cannot accumulate asyncio callbacks either.
"""
import asyncio
from collections import deque
from contextlib import suppress
import json
import math
import queue
import threading
import time
import uuid


LATEST_TYPES = ("telemetry", "tracks", "map")


class _ClientOutbox:
    def __init__(self, websocket):
        self.websocket = websocket
        self.reliable = deque()
        self.latest = {}
        self.wake = asyncio.Event()
        self.wake_armed = False
        self.notification_pending = False
        self.closing = None
        self.closed = False
        self.in_flight = None
        self.sender = None
        self.sent = 0
        self.replaced = 0
        self.last_send_ms = 0.0
        self.last_serialize_ms = 0.0
        self.allow_bulk = False
        self.next_bulk = 0


class WebSocketHub:
    def __init__(self, bind_address, port, on_event, *, reliable_queue_size=32,
                 send_timeout_sec=2.0):
        if reliable_queue_size < 1 or not math.isfinite(send_timeout_sec) or send_timeout_sec <= 0:
            raise ValueError("outbound queue size and send timeout must be positive")
        self.bind_address, self.port, self.on_event = bind_address, port, on_event
        self.reliable_queue_size = reliable_queue_size
        self.send_timeout_sec = send_timeout_sec
        self.events = queue.Queue(maxsize=128)
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name="operator-websocket", daemon=True)
        self.loop = None
        # Both the registry and every outbox field are protected by this lock.
        self._lock = threading.Lock()
        self.clients = {}
        self._totals = dict(sent=0, replaced=0, reliable_overflows=0,
                            send_timeouts=0, send_failures=0, wakeups=0,
                            max_send_ms=0.0, max_serialize_ms=0.0)

    def start(self):
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("WebSocket server did not start")
        if self.error:
            raise self.error

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as error:
            self.error = error
            self.ready.set()

    async def _serve(self):
        try:
            import websockets
        except ImportError:
            self.error = RuntimeError("python3-websockets is required for f1_operator_gateway")
            self.ready.set()
            return
        self.loop = asyncio.get_running_loop()
        self.server = await websockets.serve(
            self._handler, self.bind_address, self.port,
            max_size=128 * 1024, max_queue=4, ping_interval=5, ping_timeout=3,
            close_timeout=.5)
        if not self.server.sockets:
            raise RuntimeError("WebSocket listener has no bound sockets; check network permissions")
        self.ready.set()
        while not self.stop_event.is_set():
            await asyncio.sleep(.1)
        self.server.close()
        await self.server.wait_closed()

    async def _handler(self, websocket, *args):
        client_id = uuid.uuid4().hex
        outbox = _ClientOutbox(websocket)
        with self._lock:
            self.clients[client_id] = outbox
        outbox.sender = asyncio.create_task(self._sender(outbox))
        self._event(("connected", client_id))
        try:
            async for raw in websocket:
                if isinstance(raw, str):
                    self._event(("packet", client_id, raw))
        except Exception:
            pass
        finally:
            with self._lock:
                self.clients.pop(client_id, None)
                outbox.closed = True
                outbox.reliable.clear()
                outbox.latest.clear()
                outbox.notification_pending = False
                outbox.wake_armed = False
            outbox.sender.cancel()
            with suppress(asyncio.CancelledError, Exception):
                await outbox.sender
            self._event(("disconnected", client_id))

    def _event(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            # Preserve lifecycle events when evicting a packet from a flood.
            # If the queue consists only of lifecycle events, retain the old
            # bounded behavior rather than introduce an unbounded side queue.
            with self.events.mutex:
                pending = self.events.queue
                # A concurrent poll may already have freed space after Full.
                if len(pending) >= self.events.maxsize:
                    packet_index = next((i for i, value in enumerate(pending)
                                         if value[0] == "packet"), None)
                    if packet_index is not None:
                        del pending[packet_index]
                    elif event[0] == "packet":
                        return
                    else:
                        pending.popleft()
                    self.events.unfinished_tasks -= 1
                pending.append(event)
                self.events.unfinished_tasks += 1
                self.events.not_empty.notify()

    def poll(self):
        events = []
        while True:
            try:
                events.append(self.events.get_nowait())
            except queue.Empty:
                break
        return events

    def send(self, client_id, packet):
        """Queue a packet; False means no client or explicit overflow disconnect.

        Only three stream types may replace an unsent predecessor. In particular,
        operation replies and diagnostic events always retain FIFO ordering.
        """
        with self._lock:
            outbox = self.clients.get(client_id)
            if self.loop is None or outbox is None or outbox.closed or outbox.closing:
                return False
            kind = packet.get("type")
            entry = (packet, time.monotonic())
            accepted = True
            if kind in LATEST_TYPES:
                if kind in outbox.latest:
                    outbox.replaced += 1
                    self._totals["replaced"] += 1
                outbox.latest[kind] = entry
            elif len(outbox.reliable) >= self.reliable_queue_size:
                # Never silently drop a reply while leaving the client connected.
                outbox.closing = (1013, "reliable outbound queue overflow")
                self._totals["reliable_overflows"] += 1
                accepted = False
            else:
                outbox.reliable.append(entry)
            if not outbox.wake_armed:
                outbox.wake_armed = True
                outbox.notification_pending = True
                try:
                    self.loop.call_soon_threadsafe(self._wake_sender, outbox)
                except RuntimeError:
                    outbox.notification_pending = False
                    outbox.wake_armed = False
                    outbox.closing = (1001, "WebSocket loop stopped")
                    return False
                self._totals["wakeups"] += 1
            return accepted

    def broadcast(self, packet):
        with self._lock:
            client_ids = tuple(self.clients)
        for client_id in client_ids:
            self.send(client_id, packet)

    def _wake_sender(self, outbox):
        with self._lock:
            outbox.notification_pending = False
            if not outbox.closed:
                outbox.wake.set()

    async def _sender(self, outbox):
        while True:
            await outbox.wake.wait()
            with self._lock:
                if outbox.closed:
                    return
                closing = outbox.closing
                entry = None
                if not closing:
                    if outbox.reliable:
                        entry = outbox.reliable.popleft()
                    else:
                        bulk_types = LATEST_TYPES[1:]
                        bulk_waiting = any(kind in outbox.latest for kind in bulk_types)
                        if "telemetry" in outbox.latest and not (outbox.allow_bulk and bulk_waiting):
                            entry = outbox.latest.pop("telemetry")
                            outbox.allow_bulk = True
                        else:
                            # At most one bulk snapshot between telemetry sends.
                            # Round robin also prevents tracks from starving map.
                            for offset in range(len(bulk_types)):
                                index = (outbox.next_bulk + offset) % len(bulk_types)
                                kind = bulk_types[index]
                                if kind in outbox.latest:
                                    entry = outbox.latest.pop(kind)
                                    outbox.next_bulk = (index + 1) % len(bulk_types)
                                    outbox.allow_bulk = False
                                    break
                    if entry is None:
                        # Enqueue and idle transition use the same lock, so no
                        # producer can lose a wakeup between clear and wait.
                        outbox.wake.clear()
                        outbox.wake_armed = False
                        continue
                    outbox.in_flight = (entry[0].get("type"), entry[1])
            if closing:
                await self._close_socket(outbox, *closing)
                return
            packet, queued_at = entry
            started = time.monotonic()
            try:
                if packet.get("type") == "telemetry":
                    # Snapshot stream metadata without modifying the ROS-owned
                    # packet (broadcast clients may consume it at different times).
                    packet = dict(packet)
                    stream = dict(packet.get("stream") or {})
                    generated = stream.get("generated_monotonic", queued_at)
                    if not isinstance(generated, (int, float)) or not math.isfinite(generated):
                        generated = queued_at
                    stream["server_queue_ms"] = round(max(0.0, started - generated) * 1000, 3)
                    packet["stream"] = stream
                encoded = json.dumps(packet, ensure_ascii=False, allow_nan=False,
                                     separators=(",", ":"))
                serialized_ms = (time.monotonic() - started) * 1000
                with self._lock:
                    outbox.last_serialize_ms = serialized_ms
                    self._totals["max_serialize_ms"] = max(self._totals["max_serialize_ms"], serialized_ms)
                await asyncio.wait_for(outbox.websocket.send(encoded), self.send_timeout_sec)
            except asyncio.CancelledError:
                raise
            except asyncio.TimeoutError:
                with self._lock:
                    self._totals["send_timeouts"] += 1
                    outbox.closing = (1013, "outbound send timed out")
            except Exception:
                with self._lock:
                    self._totals["send_failures"] += 1
                    outbox.closing = (1011, "outbound send failed")
            else:
                elapsed_ms = (time.monotonic() - started) * 1000
                with self._lock:
                    outbox.sent += 1
                    self._totals["sent"] += 1
                    outbox.last_send_ms = elapsed_ms
                    self._totals["max_send_ms"] = max(self._totals["max_send_ms"], elapsed_ms)
            finally:
                with self._lock:
                    outbox.in_flight = None

    @staticmethod
    async def _close_socket(outbox, code, reason):
        try:
            await asyncio.wait_for(outbox.websocket.close(code=code, reason=reason), .5)
        except Exception:
            transport = getattr(outbox.websocket, "transport", None)
            if transport is not None:
                transport.abort()

    def stats(self, *, include_clients=True):
        """Thread-safe metrics; aggregate-only snapshots suit telemetry packets."""
        now = time.monotonic()
        with self._lock:
            clients = {}
            for client_id, outbox in self.clients.items():
                pending = list(outbox.reliable) + list(outbox.latest.values())
                oldest = min((stamp for _, stamp in pending), default=now)
                clients[client_id] = dict(
                    pending_reliable=len(outbox.reliable), pending_latest=len(outbox.latest),
                    latest_types=sorted(outbox.latest), in_flight=outbox.in_flight is not None,
                    in_flight_type=outbox.in_flight[0] if outbox.in_flight else None,
                    oldest_pending_age_ms=round(max(0.0, now - oldest) * 1000, 3),
                    notification_pending=outbox.notification_pending,
                    closing=outbox.closing is not None, sent=outbox.sent, replaced=outbox.replaced,
                    last_send_ms=round(outbox.last_send_ms, 3),
                    last_serialize_ms=round(outbox.last_serialize_ms, 3))
            result = dict(self._totals, client_count=len(clients),
                          reliable_queue_limit=self.reliable_queue_size,
                          latest_queue_limit=len(LATEST_TYPES),
                          pending_reliable=sum(c["pending_reliable"] for c in clients.values()),
                          pending_latest=sum(c["pending_latest"] for c in clients.values()),
                          in_flight=sum(c["in_flight"] for c in clients.values()),
                          queued_wakeups=sum(c["notification_pending"] for c in clients.values()),
                          oldest_pending_age_ms=max((c["oldest_pending_age_ms"] for c in clients.values()), default=0.0))
            if include_clients:
                result["clients"] = clients
            return result

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2)
