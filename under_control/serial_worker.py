"""One Linux RS485 owner, one outstanding request, no ROS or network calls."""
import os
import logging
from pathlib import Path
import threading
import time

from .protocol import decode_report
from .session import BUS_QUIET, PERIOD, REPLY_TIMEOUT, Session


def open_serial(port):
    import fcntl
    import serial
    import termios

    target = os.stat(port)
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            for fd in (process / "fd").iterdir():
                try:
                    current = fd.stat()
                    if (current.st_dev, current.st_ino) == (target.st_dev, target.st_ino):
                        raise RuntimeError(f"serial already open by PID {process.name}")
                except (FileNotFoundError, PermissionError):
                    pass
        except (FileNotFoundError, PermissionError):
            pass
    handle = serial.Serial(port, 115200, timeout=0, write_timeout=0, exclusive=True)
    try:
        fcntl.ioctl(handle.fileno(), termios.TIOCEXCL)
        flags = fcntl.fcntl(handle.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(handle.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        handle.reset_input_buffer()
    except BaseException:
        handle.close()
        raise
    return handle


class SerialWorker:
    def __init__(self, port=None, *, motion_enabled=False):
        self.port = port
        self.core = Session(motion_enabled)
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.quit = threading.Event()
        self.thread = None
        self.tx_count = self.invalid_lines = self.reconnects = 0

    def start(self):
        if self.port is not None:
            self.thread = threading.Thread(target=self._run, name="under-rs485", daemon=True)
            self.thread.start()

    def submit(self, client, packet):
        with self.lock:
            result = self.core.submit(client, packet, time.monotonic())
        self.wake.set()
        return result

    def stop(self, reason, client=None):
        with self.lock:
            accepted = self.core.stop(reason, client)
        self.wake.set()
        return accepted

    def disconnect_client(self, client):
        with self.lock:
            if self.core.owner == client:
                self.core.stop("operator disconnected", client)
        self.wake.set()

    def snapshot(self):
        with self.lock:
            return dict(self.core.snapshot(time.monotonic()), tx_count=self.tx_count,
                        invalid_lines=self.invalid_lines, reconnects=self.reconnects)

    def close(self):
        self.stop("service stopping")
        # The worker attempts a STOP request/confirmation without interrupting
        # an outstanding RS485 exchange. If transport failed, H7 owns timeout.
        deadline = time.monotonic() + .2
        while self.thread and self.thread.is_alive() and time.monotonic() < deadline:
            state = self.snapshot()
            if not state["connected"] or state["stop_confirmed"]:
                break
            self.quit.wait(.005)
        self.quit.set()
        self.wake.set()
        if self.thread:
            self.thread.join(2)

    def _run(self):
        while not self.quit.is_set():
            handle = None
            retry_delay = .5
            try:
                handle = open_serial(self.port)
                with self.lock:
                    self.core.opened()
                    self.reconnects += 1
                logging.getLogger(__name__).info("H7 serial connected: %s", self.port)
                self._exchange(handle)
            except Exception as error:
                if isinstance(error, FileNotFoundError):
                    reason = "waiting for H7 serial device: " + str(self.port)
                    retry_delay = 1.
                else:
                    reason = f"{type(error).__name__}: {error}"
                with self.lock:
                    changed = self.core.connected or self.core.reason != reason
                    if changed:
                        self.core.disconnected(reason)
                if changed:
                    logging.getLogger(__name__).info("H7 offline: %s", reason)
            finally:
                if handle is not None:
                    # Release TIOCEXCL explicitly; a PTY master may outlive this
                    # descriptor, and reconnect must still be possible.
                    import fcntl
                    import termios
                    try:
                        fcntl.ioctl(handle.fileno(), termios.TIOCNXCL)
                    except OSError:
                        pass
                    handle.close()
            self.quit.wait(retry_delay)

    def _exchange(self, handle):
        buffer = bytearray()
        last_rx = time.monotonic()
        next_tx = last_rx + BUS_QUIET
        last_generation = self.core.generation
        while not self.quit.is_set():
            self.wake.clear()
            data = handle.read(256)
            now = time.monotonic()
            if data:
                last_rx = now
                buffer.extend(data)
                if len(buffer) > 256:
                    raise ValueError("H7 reply too long")
                if b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    if remainder:
                        raise ValueError("multiple or trailing H7 replies")
                    buffer.clear()
                    try:
                        report = decode_report(bytes(line) + b"\n")
                        with self.lock:
                            self.core.receive(report, now)
                    except ValueError:
                        self.invalid_lines += 1
                        raise
            with self.lock:
                now = time.monotonic()
                self.core.tick(now)
                if self.core.pending and now - self.core.pending[4] >= REPLY_TIMEOUT:
                    raise TimeoutError("H7 reply timeout")
                if self.core.generation != last_generation:
                    last_generation = self.core.generation
                    next_tx = min(next_tx, now)  # Priority STOP, after reply + bus quiet.
                if (self.core.pending is None and not buffer and now >= next_tx
                        and now - last_rx >= BUS_QUIET):
                    packet = self.core.request(now)
                    # Select and write under the same lock. A concurrent STOP
                    # cannot leave a previously extracted motion queued to send.
                    if os.write(handle.fileno(), packet) != len(packet):
                        raise OSError("partial H7 command write")
                    self.tx_count += 1
                    next_tx = now + PERIOD
            self.wake.wait(.003)
