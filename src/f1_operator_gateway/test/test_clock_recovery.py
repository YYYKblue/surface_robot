"""Supervision boundary: bridge exit retires the operator, fresh launch is idle.

The bridge's injected clock-step tests cover exit code 75 separately. Here a
SIGTERM simulates that process exit without adjusting host time or opening tty.
"""
import asyncio
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

import websockets


def test_bridge_exit_shuts_down_operator_and_restart_is_idle(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "179")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", (Path(__file__).parent / "cyclonedds_localhost.xml").as_uri())
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    command = ["ros2", "launch", "f1_operator_gateway", "operator_system.launch.py",
               f"workspace_root:={tmp_path}", "bind_address:=127.0.0.1", f"port:={port}",
               "output_enabled:=false"]

    async def idle(process, log):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            assert process.poll() is None, log.read_text()
            try:
                async with websockets.connect(f"ws://127.0.0.1:{port}", open_timeout=1) as ws:
                    while time.monotonic() < deadline:
                        packet = json.loads(await asyncio.wait_for(ws.recv(), 2))
                        if packet.get("type") == "telemetry":
                            assert packet["runtime"]["mode"] == "idle"
                            assert packet["runtime"]["current_map"] == ""
                            assert packet["active_client"] is None
                            assert not packet["active_tasks"]
                            assert all(not task["can_start"] and not task.get("run_id")
                                       for task in packet["tasks"].values())
                            return
            except (OSError, asyncio.TimeoutError):
                await asyncio.sleep(.1)
        raise AssertionError(log.read_text())

    # Relaunch with the same workspace: the first exit must release the lock,
    # listener and bridge, and no saved runtime may automatically resume.
    for attempt in range(2):
        log = tmp_path / f"launch-{attempt}.log"
        with log.open("w") as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                asyncio.run(idle(process, log))
                bridge = re.search(r"\[f1_bridge-\d+\]: process started with pid \[(\d+)\]", log.read_text())
                assert bridge, log.read_text()
                os.kill(int(bridge.group(1)), signal.SIGTERM)
                process.wait(timeout=12)
                assert "sending signal 'SIGINT'" in log.read_text(), log.read_text()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
