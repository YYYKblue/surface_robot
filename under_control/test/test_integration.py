"""Real WebSocket + pyserial PTY + compiled H7 C; never opens a physical port."""
import asyncio
import ctypes
import json
import os
import pty
import select
import threading
import time

import pytest

websockets = pytest.importorskip("websockets")

from under_control.protocol import DEVICE, PROTOCOL
from under_control.serial_worker import SerialWorker, open_serial
from under_control.server import Server


@pytest.fixture
def board(h7):
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    done = threading.Event()
    state = dict(port=port, frames=[], output=(0, 0, 0, 0), errors=[], drop_replies=False)
    started = time.monotonic()

    def run():
        buffer = bytearray()
        try:
            while not done.wait(.002):
                now = 3000 + int((time.monotonic() - started) * 1000)
                if select.select([master], [], [], 0)[0]:
                    try:
                        buffer.extend(os.read(master, 1024))
                    except OSError:
                        pass  # Board time/watchdog must advance even with no slave open.
                    while len(buffer) >= 16:
                        frame = bytes(buffer[:16]); del buffer[:16]
                        state["frames"].append((time.monotonic(), frame))
                        h7.test_feed(frame, len(frame), now)
                h7.test_tick(now)
                output = ctypes.create_string_buffer(1024)
                count = h7.test_read(output)
                if count and not state["drop_replies"]:
                    os.write(master, output.raw[:count])
                state["output"] = tuple(h7.test_output(i) for i in range(4))
        except Exception as error:
            state["errors"].append(error)

    thread = threading.Thread(target=run)
    thread.start()
    yield state
    done.set(); thread.join(2); os.close(master)
    assert not state["errors"] and not thread.is_alive()


async def wait_for(predicate, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        await asyncio.sleep(.01)
    assert predicate(), "condition timed out"


async def send(ws, op, **fields):
    await ws.send(json.dumps(dict(protocol=PROTOCOL, device=DEVICE, op=op, **fields)))


def test_network_to_real_h7_input_timeout_ownership_and_no_replay(board):
    async def scenario():
        worker = SerialWorker(board["port"], motion_enabled=True)
        server = Server(worker, "127.0.0.1", 0)
        worker.start()
        try:
            await server.start()
            port = server.server.sockets[0].getsockname()[1]
            await wait_for(lambda: worker.snapshot()["available"])
            with pytest.raises((OSError, RuntimeError)):
                open_serial(board["port"])
            async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
                await send(ws, "hello", version=1)
                hello = json.loads(await ws.recv())
                assert hello["device"] == DEVICE
                generation = worker.snapshot()["control_generation"]
                for seq in range(1, 7):
                    await send(ws, "manual_input", seq=seq, direction="FORWARD",
                               speed_percent=60, control_generation=generation)
                    await asyncio.sleep(.05)
                assert worker.snapshot()["owner"] == hello["client_id"]
                assert board["output"][0] > 0 and board["output"][3] > 0
                # Another client cannot stop or renew this owner's command.
                async with websockets.connect(f"ws://127.0.0.1:{port}") as other:
                    await send(other, "hello", version=1)
                    await other.recv()
                    await send(other, "manual_stop", reason="wrong operator")
                    await asyncio.sleep(.02)
                    assert worker.snapshot()["owner"] == hello["client_id"]
                await wait_for(lambda: board["output"] == (0, 0, 0, 0))
                assert worker.snapshot()["last_stop_reason"] == "app input timeout"
                assert worker.snapshot()["control_generation"] != generation
                await send(ws, "manual_input", seq=100, direction="FORWARD",
                           speed_percent=60, control_generation=generation)
                await asyncio.sleep(.12)
                assert board["output"] == (0, 0, 0, 0)
                # A fresh generation and new operation can run again.
                await send(ws, "manual_input", seq=101, direction="LEFT", speed_percent=40,
                           control_generation=worker.snapshot()["control_generation"])
                await wait_for(lambda: worker.snapshot()["owner"] == hello["client_id"])
            await wait_for(lambda: worker.snapshot()["owner"] is None)
            await wait_for(lambda: board["output"] == (0, 0, 0, 0))
            assert all(len(frame) == 16 for _, frame in board["frames"])
        finally:
            await server.close()
            await asyncio.to_thread(worker.close)
    asyncio.run(scenario())


def test_lost_h7_replies_stop_and_do_not_continue_renewing_motion(board):
    async def scenario():
        worker = SerialWorker(board["port"], motion_enabled=True)
        worker.start()
        try:
            await wait_for(lambda: worker.snapshot()["available"])
            generation = worker.snapshot()["control_generation"]
            for _ in range(7):
                assert worker.submit("test", dict(direction="FORWARD", speed_percent=60,
                    control_generation=generation))[0]
                await asyncio.sleep(.05)
            assert board["output"][0] > 0
            board["drop_replies"] = True
            dropped_at = time.monotonic()
            await wait_for(lambda: not worker.snapshot()["connected"])
            assert not worker.submit("test", dict(direction="FORWARD", speed_percent=60,
                control_generation=generation))[0]
            await wait_for(lambda: board["output"] == (0, 0, 0, 0))
            assert sum(1 for at, frame in board["frames"] if at > dropped_at and frame[12]) <= 1
            board["drop_replies"] = False
            await wait_for(lambda: worker.snapshot()["available"])
            assert not worker.submit("test", dict(direction="FORWARD", speed_percent=60,
                control_generation=generation))[0]
            assert board["output"] == (0, 0, 0, 0)
        finally:
            await asyncio.to_thread(worker.close)
    asyncio.run(scenario())


def test_server_without_serial_never_claims_hardware_ready():
    async def scenario():
        worker = SerialWorker()
        server = Server(worker, "127.0.0.1", 0)
        try:
            await server.start()
            port = server.server.sockets[0].getsockname()[1]
            async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
                await send(ws, "hello", version=1)
                await ws.recv()
                state = json.loads(await ws.recv())["state"]
                assert not state["available"] and not state["connected"]
                await send(ws, "manual_input", seq=1, direction="FORWARD", speed_percent=50,
                           control_generation=state["control_generation"])
                await asyncio.sleep(.02)
                assert worker.snapshot()["owner"] is None
        finally:
            await server.close()
    asyncio.run(scenario())


def test_optional_h7_missing_at_start_hotplug_and_replug_without_worker_restart(board, tmp_path):
    async def scenario():
        port = tmp_path / "optional-h7"
        worker = SerialWorker(str(port), motion_enabled=True)
        worker.start()
        thread = worker.thread
        try:
            await wait_for(lambda: "waiting for H7 serial device" in worker.snapshot()["reason"])
            generation = worker.snapshot()["control_generation"]
            await asyncio.sleep(1.1)
            assert worker.snapshot()["control_generation"] == generation
            assert worker.snapshot()["state"] == "OFFLINE" and worker.thread is thread
            port.symlink_to(board["port"])
            await wait_for(lambda: worker.snapshot()["available"], timeout=4)
            generation = worker.snapshot()["control_generation"]
            for _ in range(7):
                assert worker.submit("test", dict(direction="FORWARD", speed_percent=40,
                    control_generation=generation))[0]
                await asyncio.sleep(.05)
            assert board["output"][0] > 0
            port.unlink()
            board["drop_replies"] = True
            await wait_for(lambda: not worker.snapshot()["connected"])
            await wait_for(lambda: board["output"] == (0, 0, 0, 0))
            board["drop_replies"] = False
            port.symlink_to(board["port"])
            await wait_for(lambda: worker.snapshot()["available"], timeout=4)
            assert worker.thread is thread and thread.is_alive()
            assert not worker.submit("test", dict(direction="FORWARD", speed_percent=40,
                control_generation=generation))[0]
            assert board["output"] == (0, 0, 0, 0)
        finally:
            await asyncio.to_thread(worker.close)
    asyncio.run(scenario())


def test_qt_arrow_to_websocket_to_real_h7_while_surface_disconnected(board):
    pytest.importorskip("websocket")
    try:
        from PySide6.QtCore import Qt
        from PySide6.QtTest import QTest
        from PySide6.QtWidgets import QApplication
    except ImportError:
        from PyQt5.QtCore import Qt
        from PyQt5.QtTest import QTest
        from PyQt5.QtWidgets import QApplication
    from operator_app.window import OperatorWindow

    app = QApplication.instance() or QApplication([])
    worker = SerialWorker(board["port"], motion_enabled=True)
    server = Server(worker, "127.0.0.1", 0)
    started, done = threading.Event(), threading.Event()
    errors = []

    async def serve():
        worker.start()
        try:
            await server.start()
            started.set()
            while not done.is_set():
                await asyncio.sleep(.01)
        finally:
            await server.close()
            await asyncio.to_thread(worker.close)

    def run():
        try:
            asyncio.run(serve())
        except Exception as error:
            errors.append(error)
            started.set()

    def qt_wait(predicate):
        end = time.monotonic() + 4
        while time.monotonic() < end:
            if predicate():
                return
            QTest.qWait(10)
        assert predicate()

    thread = threading.Thread(target=run)
    thread.start()
    window = None
    try:
        assert started.wait(3) and not errors
        port = server.server.sockets[0].getsockname()[1]
        window = OperatorWindow("ws://127.0.0.1:1", under_url=f"ws://127.0.0.1:{port}")
        window.show()
        qt_wait(lambda: not window.under_panel.locked)
        QTest.keyPress(window, Qt.Key_Up)
        qt_wait(lambda: board["output"][0] > 0)
        assert window.direction == "STOP" and window.manual_locked
        assert window.under_panel.direction == "FORWARD"
        QTest.keyRelease(window, Qt.Key_Up)
        qt_wait(lambda: board["output"] == (0, 0, 0, 0))
        assert window.under_panel.direction == "STOP"
    finally:
        if window is not None:
            window.close()
        done.set(); thread.join(3)
        app.processEvents()
    assert not errors and not thread.is_alive()
