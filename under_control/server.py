"""Small independent WebSocket gateway; only manual control is exposed."""
import asyncio
from contextlib import suppress
import json
import uuid

from .protocol import DEVICE, PROTOCOL, decode_request


class Server:
    def __init__(self, worker, host="127.0.0.1", port=8766):
        self.worker, self.host, self.port = worker, host, port
        self.session_id = uuid.uuid4().hex
        self.server = None

    async def start(self):
        import websockets
        self.server = await websockets.serve(
            self.handler, self.host, self.port, max_size=4096, max_queue=1,
            ping_interval=5, ping_timeout=3, close_timeout=.2)

    async def close(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()

    def hello(self, client):
        return dict(type="hello", protocol=PROTOCOL, device=DEVICE,
                    session_id=self.session_id, client_id=client,
                    capabilities=["under_manual_v1"], telemetry_hz=10,
                    drive_output=dict(min=0, max=100, step=1, firmware_protocol_version=6))

    async def handler(self, ws, *args):
        from websockets.exceptions import ConnectionClosed
        client = uuid.uuid4().hex
        telemetry = None
        try:
            first = decode_request(await asyncio.wait_for(ws.recv(), 2))
            if first["op"] != "hello":
                raise ValueError("hello required")
            await asyncio.wait_for(ws.send(json.dumps(self.hello(client))), .2)
            telemetry = asyncio.create_task(self._telemetry(ws))
            last_sequence = -1
            async for raw in ws:
                packet = decode_request(raw)
                if packet["op"] == "manual_input":
                    if packet["seq"] <= last_sequence:
                        continue  # Duplicates never renew input freshness.
                    last_sequence = packet["seq"]
                    # Worker validates the generation and input deadline under
                    # its serial lock even if this network loop was delayed.
                    self.worker.submit(client, packet)
                elif packet["op"] == "manual_stop":
                    self.worker.stop(packet.get("reason", "operator"), client)
                else:
                    raise ValueError("duplicate hello")
        except (ValueError, UnicodeError, asyncio.TimeoutError):
            await ws.close(code=1008, reason="invalid or timed out underwater session")
        except ConnectionClosed:
            pass
        finally:
            self.worker.disconnect_client(client)
            if telemetry is not None:
                telemetry.cancel()
                with suppress(asyncio.CancelledError, Exception):
                    await telemetry

    async def _telemetry(self, ws):
        sequence = 0
        try:
            while True:
                sequence += 1
                packet = dict(type="telemetry", protocol=PROTOCOL, device=DEVICE,
                              session_id=self.session_id, sequence=sequence,
                              state=self.worker.snapshot())
                # Generate at send time; no FIFO of old control feedback.
                await asyncio.wait_for(ws.send(json.dumps(packet)), .2)
                await asyncio.sleep(.1)
        finally:
            await ws.close()
