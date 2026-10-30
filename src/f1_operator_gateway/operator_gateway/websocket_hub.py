"""A bounded asyncio WebSocket hub kept outside the ROS executor thread."""
import asyncio
import json
import queue
import threading
import uuid


class WebSocketHub:
    def __init__(self, bind_address, port, on_event):
        self.bind_address, self.port, self.on_event = bind_address, port, on_event
        self.events = queue.Queue(maxsize=128)
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name="operator-websocket", daemon=True)
        self.loop = None
        self.clients = {}

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
        except ImportError as error:
            self.error = RuntimeError("python3-websockets is required for f1_operator_gateway")
            self.ready.set()
            return
        self.loop = asyncio.get_running_loop()
        self.server = await websockets.serve(
            self._handler, self.bind_address, self.port,
            max_size=128 * 1024, max_queue=4, ping_interval=5, ping_timeout=3)
        if not self.server.sockets:
            raise RuntimeError("WebSocket listener has no bound sockets; check network permissions")
        self.ready.set()
        while not self.stop_event.is_set():
            await asyncio.sleep(.1)
        self.server.close()
        await self.server.wait_closed()

    async def _handler(self, websocket, *args):
        client_id = uuid.uuid4().hex
        self.clients[client_id] = websocket
        self._event(("connected", client_id))
        try:
            async for raw in websocket:
                if isinstance(raw, str):
                    self._event(("packet", client_id, raw))
        except Exception:
            pass
        finally:
            self.clients.pop(client_id, None)
            self._event(("disconnected", client_id))

    def _event(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            # A flooded client must not block the ROS executor. Keep lifecycle
            # events, while dropping an obsolete manual input packet first.
            try:
                self.events.get_nowait()
                self.events.put_nowait(event)
            except queue.Empty:
                pass

    def poll(self):
        events = []
        while True:
            try:
                events.append(self.events.get_nowait())
            except queue.Empty:
                break
        return events

    def send(self, client_id, packet):
        if self.loop is None:
            return
        websocket = self.clients.get(client_id)
        if websocket is None:
            return
        asyncio.run_coroutine_threadsafe(self._send(websocket, packet), self.loop)

    def broadcast(self, packet):
        for client_id in tuple(self.clients):
            self.send(client_id, packet)

    @staticmethod
    async def _send(websocket, packet):
        try:
            await websocket.send(json.dumps(packet, ensure_ascii=False, allow_nan=False,
                                             separators=(",", ":")))
        except Exception:
            pass

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2)
