import ctypes as c
import os
import pty
import select
import threading
import time

from nano_ros_bridge.session import Command
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
