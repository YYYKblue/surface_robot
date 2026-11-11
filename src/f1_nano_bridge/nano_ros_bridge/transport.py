"""One serial owner and independent 50 Hz worker; no ROS calls in the worker."""
from collections import deque
import fcntl
import os
from pathlib import Path
import termios
import threading
import time

import serial

from .protocol import Control, Remote, UnsupportedProtocol, Lines, decode, forward
from .session import Session


def open_serial(port):
    # pyserial's flock coordinates cooperating clients; inspect already-open
    # descriptors too, and TIOCEXCL prevents later non-cooperating tty opens.
    target = os.stat(port)
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            for fd in (process / "fd").iterdir():
                try:
                    st = fd.stat()
                    if (st.st_dev, st.st_ino) == (target.st_dev, target.st_ino):
                        raise RuntimeError(f"serial already open by PID {process.name}")
                except (FileNotFoundError, PermissionError):
                    pass
        except (FileNotFoundError, PermissionError):
            pass
    # Worker._write uses one nonblocking os.write directly: pyserial may report
    # a timeout even after a complete write if the thread was descheduled, and
    # its zero-timeout loop can busy-spin when os.write raises EAGAIN.
    handle = serial.Serial(port, 460800, timeout=0, write_timeout=0, exclusive=True)
    try:
        fcntl.ioctl(handle.fileno(), termios.TIOCEXCL)
        flags = fcntl.fcntl(handle.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(handle.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        handle.reset_input_buffer()
    except BaseException:
        handle.close()
        raise
    return handle


class Worker:
    PERIOD = .020

    def __init__(self, port, enabled=False, motion_enabled=False, allow_fifth=False,
                 wall_clock_ns=None):
        self.port = port
        self.enabled = enabled
        self.core = Session(motion_enabled, allow_fifth)
        self.lock = threading.RLock()
        self.quit = threading.Event()
        # Only clock faults request process recovery. Other faults stay latched.
        self.clock_restart = threading.Event()
        self.clock_restart_reason = ""
        self.thread = None
        self.handle = None
        self.lines = Lines()
        self.imu = deque(maxlen=128)
        self.invalid_lines = 0
        self.write_count = 0
        self.write_failures = 0
        self.write_zero_bytes = 0
        self.write_zero_streak = 0
        self.short_writes = 0
        self.last_write_duration_sec = 0.
        self.max_write_duration_sec = 0.
        self.write_blocked_since = None
        self.last_write_error_type = ""
        self.last_write_error = ""
        self.imu_at = None
        self.imu_status = 0
        self.imu_previous = None
        self.remote = None
        self.remote_at = None
        # Clock injection is only for offline tests, never a ROS parameter.
        self.wall_clock_ns = wall_clock_ns or time.time_ns
        self.clock_offset = None
        self._check_clock()

    def _check_clock(self):
        before = time.monotonic_ns()
        unix_ns = self.wall_clock_ns()
        after = time.monotonic_ns()
        # A deschedule between clock reads is not evidence of a wall-clock step.
        # The independent 80 ms send-gap guard still handles such stalls.
        if after - before > 5_000_000:
            return
        offset = unix_ns - (before + after) // 2
        if self.clock_offset is None:
            self.clock_offset = offset
        delta = offset - self.clock_offset
        if abs(delta) > 50_000_000 and not self.core.fault:
            self.latch_clock_fault(f"system clock changed {delta / 1e6:.1f} ms; restart required")

    def latch_clock_fault(self, reason):
        with self.lock:
            if self.core.fault and not self.clock_restart.is_set():
                return  # Never clear an unrelated latched fault by restarting.
            self.core.latch_fault(reason)
            self.clock_restart_reason = reason
            self.clock_restart.set()

    def start(self):
        if self.thread is not None:
            raise RuntimeError("worker already started")
        if self.enabled:
            self.thread = threading.Thread(target=self._run, name="f103-serial", daemon=True)
            self.thread.start()

    def acquire(self, owner):
        with self.lock:
            return self.core.acquire(owner, time.monotonic())

    def accept(self, command):
        with self.lock:
            self._check_clock()
            return self.core.accept(command, time.monotonic(), self.wall_clock_ns())

    def stop(self, reason="ROS stop requested"):
        with self.lock:
            self.core.stop(reason)

    def release(self, token):
        with self.lock:
            return self.core.release(token)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            result = self.core.snapshot(now)
            result.update(invalid_lines=self.invalid_lines,
                          imu_age_sec=-1. if self.imu_at is None else now - self.imu_at,
                          imu_status=self.imu_status)
            rc_age = -1. if self.remote_at is None else now - self.remote_at
            result.update(rc_valid=bool(self.remote and self.remote.valid
                                       and not self.remote.flags & 12
                                       and 0 <= rc_age < Session.CTRL_TIMEOUT),
                          rc_age_sec=rc_age,
                          rc_flags=self.remote.flags if self.remote else 0,
                          rc_sequence=self.remote.sequence if self.remote else 0,
                          rc_channels=list(self.remote.channels) if self.remote else [1500] * 14)
            return result

    def transport_diagnostics(self):
        """Serial write telemetry for ROS diagnostics (not ControlState)."""
        with self.lock:
            now = time.monotonic()
            blocked = (0. if self.write_blocked_since is None
                       else max(0., now - self.write_blocked_since))
            result = dict(serial_write_count=self.write_count,
                          serial_write_failures=self.write_failures,
                          serial_write_zero_bytes=self.write_zero_bytes,
                          serial_write_short=self.short_writes,
                          serial_write_last_duration_sec=self.last_write_duration_sec,
                          serial_write_max_duration_sec=self.max_write_duration_sec,
                          serial_write_blocked_sec=blocked,
                          serial_write_last_error_type=self.last_write_error_type,
                          serial_write_last_error=self.last_write_error)
            result.update(self.core.diagnostics(now))
            return result

    def take_imu(self):
        with self.lock:
            samples = list(self.imu)
            self.imu.clear()
            return samples

    def latch_fault(self, reason):
        with self.lock:
            self.core.latch_fault(reason)

    def _write(self, frame):
        if frame is not None:
            started = time.monotonic()
            try:
                try:
                    written = os.write(self.handle.fileno(), frame)
                except (BlockingIOError, InterruptedError):
                    # The POSIX syscall accepted no bytes. Do not spin here or
                    # retry the same command after its original deadline.
                    written = 0
            except (OSError, serial.SerialException) as error:
                self.last_write_duration_sec = time.monotonic() - started
                self.max_write_duration_sec = max(
                    self.max_write_duration_sec, self.last_write_duration_sec)
                self.write_failures += 1
                self.last_write_error_type = type(error).__name__
                self.last_write_error = (
                    f"{error}; elapsed={self.last_write_duration_sec * 1000:.2f}ms; "
                    f"failure_count={self.write_failures}")
                raise OSError(
                    f"{type(error).__name__} failure #{self.write_failures} after "
                    f"{self.last_write_duration_sec * 1000:.2f}ms: {error}; "
                    "frame delivery is unknown, closing port without retry") from error
            self.last_write_duration_sec = time.monotonic() - started
            self.max_write_duration_sec = max(
                self.max_write_duration_sec, self.last_write_duration_sec)
            if written == 0:
                self.write_zero_bytes += 1
                self.write_zero_streak += 1
                if self.write_blocked_since is None:
                    self.write_blocked_since = started
                reference = (self.core.last_tx_at if self.core.last_tx_at is not None
                             else self.write_blocked_since)
                gap = time.monotonic() - reference
                if gap >= self.core.MAX_SEND_GAP:
                    self.write_failures += 1
                    self.last_write_error_type = "SerialBackpressureTimeout"
                    self.last_write_error = (
                        f"zero bytes accepted for {gap * 1000:.1f}ms; "
                        f"zero-write attempts={self.write_zero_streak}; "
                        f"failure_count={self.write_failures}")
                    raise OSError(
                        f"serial backpressure exceeded {self.core.MAX_SEND_GAP * 1000:.0f}ms "
                        f"after {self.write_zero_streak} zero-byte writes "
                        f"({gap * 1000:.1f}ms), failure #{self.write_failures}; "
                        "closing port")
                # EAGAIN means no part of this frame entered the stream. Drop
                # this attempt; the next 50 Hz tick builds a fresh frame.
                return
            if written != len(frame):
                self.short_writes += 1
                self.write_failures += 1
                self.last_write_error_type = "ShortSerialWrite"
                self.last_write_error = (
                    f"{written}/{len(frame)} bytes accepted in "
                    f"{self.last_write_duration_sec * 1000:.2f}ms; "
                    f"failure_count={self.write_failures}")
                raise OSError(
                    f"serial short write #{self.write_failures} "
                    f"({written}/{len(frame)} bytes) after "
                    f"{self.last_write_duration_sec * 1000:.2f}ms; "
                    "frame discarded and port closed without retry")
            self.write_count += 1
            self.write_blocked_since = None
            self.write_zero_streak = 0
            # No flush(): waiting for hardware drains can block the safety loop.
            # Use the conservative syscall-entry time. A deschedule after
            # write must not make the last physical transmit appear newer.
            self.core.sent(started)

    def _read(self):
        if self.handle.in_waiting > 8192:
            raise OSError("serial receive backlog")
        data = self.handle.read(8192)
        mono_ns, unix_ns = time.monotonic_ns(), self.wall_clock_ns()
        previous_errors = self.lines.errors
        for line in self.lines.feed(data):
            try:
                sample = decode(line)
            except UnsupportedProtocol as error:
                self.core.protocol_mismatch(error.version)
                self.remote = self.remote_at = None
                continue
            except (ValueError, UnicodeError):
                self.invalid_lines += 1
                continue
            if isinstance(sample, Control):
                old = self.core.ctrl
                if (old is not None and sample.milliseconds != old.milliseconds
                        and not forward(sample.milliseconds, old.milliseconds)):
                    self.remote = self.remote_at = None
                self.core.receive(sample, mono_ns / 1e9)
            elif isinstance(sample, Remote):
                # A repeated remote sequence can be a fresh invalid/no-signal
                # report. Firmware time, rather than the RC sequence, proves
                # this telemetry is new. A CTRL clock reset clears this cache.
                if self.remote is None or forward(sample.milliseconds, self.remote.milliseconds):
                    self.remote = sample
                    self.remote_at = mono_ns / 1e9
            elif sample["kind"] == "IMU":
                self.imu_status = sample["status"]
                seq, ms = sample["sequence"], sample["stm32_ms"]
                fresh = self.imu_previous is None
                if self.imu_previous is not None:
                    old_seq, old_ms = self.imu_previous
                    fresh = forward(seq, old_seq) and (ms == old_ms or forward(ms, old_ms))
                    reboot = (seq < old_seq and ms < old_ms
                              and not forward(seq, old_seq) and not forward(ms, old_ms))
                    fresh = fresh or reboot
                if fresh:
                    self.imu_previous = (seq, ms)
                if fresh and sample["status"] & 6 == 6:
                    self.imu_at = mono_ns / 1e9
                self.imu.append((self.core.connection_id, sample, unix_ns, mono_ns))
        self.invalid_lines += self.lines.errors - previous_errors

    def _close(self):
        if self.handle is not None:
            try:
                # Also release the tty lock on PTYs whose master remains open.
                fcntl.ioctl(self.handle.fileno(), termios.TIOCNXCL)
            except (OSError, serial.SerialException):
                pass
            self.handle.close()
            self.handle = None
        self.write_blocked_since = None
        self.write_zero_streak = 0
        self.imu.clear()
        self.imu_at = None
        self.imu_status = 0
        self.imu_previous = None
        self.remote = self.remote_at = None

    def _run(self):
        retry_at = 0.
        next_tick = time.monotonic()
        try:
            while not self.quit.is_set():
                now = time.monotonic()
                if self.quit.wait(max(0., next_tick - now)):
                    break
                now = time.monotonic()
                next_tick = now + self.PERIOD  # No catch-up bursts after a stall.
                with self.lock:
                    try:
                        self._check_clock()
                        if self.clock_restart.is_set():
                            break  # finally attempts neutral and closes the tty.
                        if self.handle is None:
                            if now < retry_at:
                                continue
                            self.handle = open_serial(self.port)
                            self.lines = Lines()
                            self.core.opened()
                        self._read()
                        self._write(self.core.frame(time.monotonic()))
                    except (OSError, serial.SerialException, RuntimeError) as error:
                        self.core.closed(f"serial failure: {error}")
                        self._close()
                        retry_at = time.monotonic() + 1.
        except Exception as error:
            with self.lock:
                self.core.latch_fault(f"worker failed: {type(error).__name__}: {error}")
        finally:
            with self.lock:
                self.core.stop("bridge shutdown")
                if self.handle is not None:
                    try:
                        # Best effort neutral. Never claim it stopped RC outputs.
                        self._write(self.core.frame(time.monotonic()))
                    except (OSError, serial.SerialException):
                        pass
                self._close()
                self.core.closed("bridge shutdown")

    def close(self):
        self.stop("bridge shutdown")
        self.quit.set()
        if self.thread is not None:
            self.thread.join(timeout=2.)
            if self.thread.is_alive():
                raise RuntimeError("serial worker did not exit; firmware watchdog must stop Nano")
