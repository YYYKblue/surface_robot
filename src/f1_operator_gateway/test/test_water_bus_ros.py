"""Four simulated Modbus slaves on one PTY, through ROS and the real gateway socket."""
import asyncio
import json
import os
import pty
import select
import struct
import threading
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
import websockets

from operator_gateway.node import OperatorGateway
from operator_gateway.water_node import WaterQualityNode
from operator_gateway.water_quality import crc_bytes


def test_shared_bus_to_websocket_and_missing_slave(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "174")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    master, slave = pty.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    responses = {11: (790, 265), 12: (10000, 265, 490, 550), 13: (0,), 14: (3374, 219)}
    requests = []
    done = threading.Event()
    silence = threading.Event()

    def emulate():
        buffer = b""
        while not done.is_set():
            if not select.select([master], [], [], .05)[0]:
                continue
            buffer += os.read(master, 256)
            while len(buffer) >= 8:
                request, buffer = buffer[:8], buffer[8:]
                address, function, start, count = struct.unpack(">BBHH", request[:6])
                requests.append((address, function, start, count, crc_bytes(request[:6]) == request[6:]))
                if address not in responses or (address == 12 and silence.is_set()):
                    continue
                words = responses[address]
                data = bytes((address, 3, 2 * len(words))) + struct.pack(f">{len(words)}H", *words)
                os.write(master, data + crc_bytes(data))

    rclpy.init(args=["--ros-args", "-p", "f1_operator_gateway:port:=0",
                     "-p", "water_quality:port:=" + path, "-p", "ph_address:=11",
                     "-p", "ec_address:=12", "-p", "chlorine_address:=13", "-p", "turbidity_address:=14",
                     "-p", "ec_k:=1", "-p", "turbidity_range_ntu:=50"])
    emulation = threading.Thread(target=emulate)
    water = WaterQualityNode()
    gateway = OperatorGateway()
    executor = SingleThreadedExecutor()
    executor.add_node(water)
    executor.add_node(gateway)
    thread = threading.Thread(target=executor.spin)
    emulation.start()
    thread.start()

    async def scenario():
        port = gateway.hub.server.sockets[0].getsockname()[1]
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await ws.send(json.dumps({"op": "hello", "client": "water-test"}))

            async def until(check):
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    readings = json.loads(await asyncio.wait_for(ws.recv(), 3)).get("water_quality", {})
                    if check(readings):
                        return readings
                raise AssertionError("water telemetry did not reach expected state")

            readings = await until(lambda data: len(data) == 4 and all(s["status"] == "ok" for s in data.values()))
            assert readings["ph"] == dict(status="ok", value=7.9, temperature_c=26.5)
            assert readings["ec"] == dict(status="ok", value=1000., temperature_c=26.5, salinity_ppm=490., tds_ppm=550.)
            assert readings["chlorine"] == dict(status="ok", value=0.)
            assert readings["turbidity"] == dict(status="ok", value=33.74, temperature_c=21.9)
            silence.set()
            readings = await until(lambda data: data.get("ec", {}).get("status") == "offline")
            assert "value" not in readings["ec"]
            assert all(readings[key]["status"] == "ok" for key in ("ph", "chlorine", "turbidity"))
            silence.clear()
            await until(lambda data: data.get("ec", {}).get("status") == "ok")
            assert set(requests) == {(11, 3, 0, 2, True), (12, 3, 0, 4, True),
                                     (13, 3, 0, 1, True), (14, 3, 0, 2, True)}

    try:
        asyncio.run(scenario())
    finally:
        executor.shutdown()
        thread.join()
        gateway.destroy_node()
        water.destroy_node()
        rclpy.shutdown()
        done.set()
        emulation.join()
        os.close(master)
        os.close(slave)
