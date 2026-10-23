import ctypes as c
import os
import pty
import select
import pytest
import serial
import threading
import time

from nano_ros_bridge.session import Command
from nano_ros_bridge import transport
from nano_ros_bridge.protocol import Control, encode, channels
from nano_ros_bridge.transport import Worker, open_serial


def wait_until(predicate, timeout=3.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("timed out")


def test_default_never_opens_port():
    worker = Worker("/not/a/real/port")
    worker.start()
    assert worker.thread is None and not worker.snapshot()["connected"]
    assert not worker.acquire("task")[0]
    worker.close()


def test_real_serial_pty_deadline_stop_and_imu(firmware):
    """Real pyserial + actual C firmware, over a local PTY, no physical device."""
    lib = firmware
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    done = threading.Event()
    errors = []
    observed = []
    buffer = bytearray()
    started = time.monotonic()

    def board():
        last_report = -100
        try:
            while not done.wait(.002):
                ms = int((time.monotonic() - started) * 1000)
                if select.select([master], [], [], 0)[0]:
                    try:
                        buffer.extend(os.read(master, 8192))
                    except OSError:
                        continue  # PTY slave not open yet.
                    while len(buffer) >= 39:
                        packet = bytes(buffer[:39])
                        del buffer[:39]
                        lib.test_host(packet, len(packet), ms)
                lib.test_tick(ms)
                observed.append(tuple(lib.test_get(i) for i in range(16, 20)))
                if ms - last_report >= 100:
                    last_report = ms
                    buf = c.create_string_buffer(256)
                    n = lib.test_report(ms, buf, len(buf))
                    os.write(master, buf.raw[:n])
                    body = f"IMU,{ms},{ms},0,0,9.8,0,0,0,1,0,0,0,0,0,0,0,6".encode()
                    check = 0
                    for b in body:
                        check ^= b
                    os.write(master, body + f"*{check:02X}\r\n".encode())
        except Exception as exc:
            errors.append(exc)

    # Transport assertions use a stable injected wall clock. Wall-clock steps
    # are tested independently below and must not make a PTY test nondeterministic.
    epoch_offset = time.time_ns() - time.monotonic_ns()
    wall_clock_ns = lambda: epoch_offset + time.monotonic_ns()
    worker = Worker(port, enabled=True, motion_enabled=True, wall_clock_ns=wall_clock_ns)
    thread = threading.Thread(target=board)
    thread.start()
    try:
        worker.start()
        try:
            wait_until(lambda: worker.snapshot()["state"] == "READY")
        except AssertionError:
            raise AssertionError((worker.snapshot(), errors, observed[-5:], bytes(buffer)))
        # A second serial owner must fail before writing any bytes.
        try:
            other = open_serial(port)
        except (RuntimeError, OSError):
            pass
        else:
            other.close()
            raise AssertionError("duplicate serial owner was permitted")
        token, _ = worker.acquire("pty-test")
        now = wall_clock_ns()
        assert worker.accept(Command(token, 0, now, now + 150_000_000, 1))
        wait_until(lambda: (1625, 1625, 1375, 1375) in observed)
        wait_until(lambda: worker.snapshot()["state"] == "READY")
        assert observed[-1] == (1500,) * 4
        assert worker.take_imu()
        assert not worker.accept(Command(token, 1, wall_clock_ns(), wall_clock_ns() + 100_000_000, 1))
        worker.stop()
        wait_until(lambda: worker.snapshot()["state"] == "READY")
    finally:
        worker.close()
        done.set()
        thread.join(2.)
        os.close(master)
    assert not errors and not thread.is_alive()


def test_clock_step_revokes_and_latches():
    offset = [time.time_ns() - time.monotonic_ns()]
    worker = Worker("/unused", wall_clock_ns=lambda: time.monotonic_ns() + offset[0])
    worker.core.token = "old"
    offset[0] += 100_000_000
    worker._check_clock()
    assert not worker.core.token and "system clock changed" in worker.core.fault
    offset[0] -= 100_000_000
    worker._check_clock()
    worker.core.opened()
    assert worker.core.fault and not worker.acquire("task")[0]


def test_worker_write_survives_scheduler_delay_after_full_commit(monkeypatch):
    """A post-write deschedule must not become pyserial's false timeout."""
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    handle = open_serial(port)
    worker = Worker(port)
    worker.handle = handle
    frame = bytes(range(39))
    real_write = os.write

    def delayed_write(fd, data):
        count = real_write(fd, data)
        if fd == handle.fileno():
            time.sleep(.025)  # Simulates a thread descheduled after syscall commit.
        return count

    monkeypatch.setattr(transport.os, "write", delayed_write)
    try:
        assert handle.write_timeout == 0
        # Demonstrate the old failure with the installed pyserial, despite
        # the full frame being observable at the receiver.
        handle.write_timeout = .010
        with pytest.raises(serial.SerialTimeoutException):
            handle.write(frame)
        assert os.read(master, 128) == frame
        handle.write_timeout = 0
        worker._write(frame)
        assert worker.write_count == 1 and worker.write_failures == 0
        assert worker.core.tx_frames == 1
        assert time.monotonic() - worker.core.last_tx_at >= .02
        assert select.select([master], [], [], 0)[0]
        assert os.read(master, 128) == frame
        assert worker.transport_diagnostics()["serial_write_max_duration_sec"] >= .02
    finally:
        worker._close()
        os.close(master)


def test_nonblocking_backpressure_drops_zero_byte_attempt():
    """No-byte EAGAIN is dropped; it is not retried or counted as sent."""
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    handle = open_serial(port)
    worker = Worker(port)
    worker.handle = handle
    try:
        # Fill the PTY output queue without draining its master.
        for _ in range(128):
            try:
                os.write(handle.fileno(), b"x" * 65536)
            except BlockingIOError:
                break
        else:
            raise AssertionError("PTY output queue did not apply backpressure")
        worker._write(b"frame")
        assert worker.write_zero_bytes == 1
        assert worker.write_count == worker.core.tx_frames == 0
        assert worker.write_blocked_since is not None
        stats = worker.transport_diagnostics()
        assert stats["serial_write_zero_bytes"] == 1
        assert stats["serial_write_blocked_sec"] < worker.core.MAX_SEND_GAP
    finally:
        worker._close()
        os.close(master)


class ScriptedPort:
    def __init__(self, report=b""):
        self.report = report
        self.write_calls = 0
        self.closed = threading.Event()

    @property
    def in_waiting(self):
        return len(self.report)

    def read(self, _count):
        report, self.report = self.report, b""
        return report

    def fileno(self):
        return 123456  # Invalid positive descriptor gives EBADF to close ioctl.

    def close(self):
        self.closed.set()


@pytest.mark.parametrize("outcome,reason", [
    (7, "short write #1 (7/14 bytes)"),
    (OSError("USB device removed"), "OSError"),
    (serial.SerialTimeoutException("ambiguous commit"), "SerialTimeoutException"),
])
def test_incomplete_write_closes_and_never_retries_old_frame(monkeypatch, outcome, reason):
    handle = ScriptedPort()
    worker = Worker("fake", enabled=True)
    def authorized_frame(_now):
        worker.core.token = "old-token"
        return b"movement-frame"

    worker.core.frame = authorized_frame
    monkeypatch.setattr(transport, "open_serial", lambda _port: handle)
    real_write = os.write

    def partial_os_write(fd, frame):
        if fd != handle.fileno():
            return real_write(fd, frame)
        handle.write_calls += 1
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome

    monkeypatch.setattr(transport.os, "write", partial_os_write)
    worker.start()
    try:
        assert handle.closed.wait(1.)
        state = worker.snapshot()
        assert state["state"] == "DISCONNECTED"
        assert not state["motion_authorized"]
        assert reason in state["reason"]
        stats = worker.transport_diagnostics()
        assert stats["serial_write_short"] == int(outcome == 7)
        assert stats["serial_write_failures"] == 1
        assert worker.core.tx_frames == 0
        assert handle.write_calls == 1
    finally:
        worker.quit.set()
        worker.thread.join(1.)
    assert not worker.thread.is_alive()


def test_real_pty_disconnection_revokes_and_rejects_old_token():
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    offset = time.time_ns() - time.monotonic_ns()
    worker = Worker(port, enabled=True, motion_enabled=True,
                    wall_clock_ns=lambda: offset + time.monotonic_ns())

    def report(ms, have_sequence):
        body = (f"CTRL,2,{ms},NANO,1,{have_sequence},0,OK,BOTH,1,0,0,0,0,"
                "1500,1500,1500,1500,25,STOP").encode()
        checksum = 0
        for byte in body:
            checksum ^= byte
        os.write(master, body + f"*{checksum:02X}\r\n".encode())

    try:
        worker.start()
        wait_until(lambda: worker.snapshot()["connected"])
        report(100, 0)
        wait_until(lambda: worker.snapshot()["tx_frames"] > 0)
        report(120, 1)
        wait_until(lambda: worker.snapshot()["state"] == "READY")
        token, _ = worker.acquire("disconnect-test")
        assert token
        os.close(master)
        master = None
        wait_until(lambda: not worker.snapshot()["connected"])
        assert not worker.snapshot()["motion_authorized"]
        assert "serial failure" in worker.snapshot()["last_stop_reason"]
        now = worker.wall_clock_ns()
        assert not worker.accept(Command(token, 0, now, now + 150_000_000, 1))
    finally:
        worker.close()
        if master is not None:
            os.close(master)


def test_zero_byte_attempt_is_not_replayed_after_command_change(monkeypatch):
    """Fresh command/sequence after EAGAIN; no old direction is replayed."""
    worker = Worker("fake", motion_enabled=True)
    worker.handle = ScriptedPort()
    now = time.monotonic()
    worker.core.opened()
    report = Control(100, "NANO", 1, False, 0, "OK", "BOTH", True,
                     False, 0, False, 0, (1500,) * 4)
    worker.core.receive(report, now)
    worker.core.ready = True
    token, _ = worker.acquire("drop-test")
    unix_ns = time.time_ns()
    assert worker.accept(Command(token, 0, unix_ns, unix_ns + 150_000_000, 1))
    attempts = []
    real_write = os.write

    def once_blocked(fd, frame):
        if fd != worker.handle.fileno():
            return real_write(fd, frame)
        attempts.append(frame)
        if len(attempts) == 1:
            raise BlockingIOError()
        return len(frame)

    monkeypatch.setattr(transport.os, "write", once_blocked)
    worker._write(worker.core.frame(time.monotonic()))
    assert worker.core.token == token and worker.core.tx_frames == 0
    assert worker.accept(Command(token, 1, unix_ns, unix_ns + 150_000_000, 0))
    worker._write(worker.core.frame(time.monotonic()))
    assert worker.core.token == token and worker.core.tx_frames == 1
    assert attempts == [encode(channels(direction=1), 1, 0),
                        encode(channels(direction=0), 1, 1)]
    # A skipped sequence does not prevent the later neutral barrier handshake.
    worker.core.stop("test stop")
    worker._write(worker.core.frame(time.monotonic()))
    worker.core.receive(Control(120, "NANO", 1, True, 2, "OK", "BOTH", True,
                                False, 0, False, 0, (1500,) * 4), time.monotonic())
    assert worker.core.ready and not worker.core.token
    assert not worker.accept(Command(token, 2, unix_ns, unix_ns + 150_000_000, 1))


def test_persistent_zero_byte_backpressure_closes_at_send_gap(monkeypatch):
    handle = ScriptedPort()
    body = b"CTRL,2,100,NANO,1,1,0,OK,BOTH,1,0,0,0,0,1500,1500,1500,1500,25,STOP"
    check = 0
    for byte in body:
        check ^= byte
    handle.report = body + f"*{check:02X}\r\n".encode()
    worker = Worker("fake", enabled=True)
    monkeypatch.setattr(transport, "open_serial", lambda _port: handle)
    real_write = os.write

    def always_blocked(fd, frame):
        if fd != handle.fileno():
            return real_write(fd, frame)
        handle.write_calls += 1
        raise BlockingIOError()

    monkeypatch.setattr(transport.os, "write", always_blocked)
    worker.start()
    try:
        assert handle.closed.wait(1.)
        state = worker.snapshot()
        assert state["state"] == "DISCONNECTED"
        assert "serial backpressure exceeded 80ms" in state["reason"]
        stats = worker.transport_diagnostics()
        assert stats["serial_write_zero_bytes"] >= 2
        assert stats["serial_write_failures"] == 1
        assert stats["serial_write_last_error_type"] == "SerialBackpressureTimeout"
        assert worker.core.tx_frames == 0
        assert handle.write_calls >= 2
    finally:
        worker.quit.set()
        worker.thread.join(1.)
    assert not worker.thread.is_alive()
