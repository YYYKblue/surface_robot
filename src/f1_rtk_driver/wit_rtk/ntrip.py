"""NTRIP v1 worker. Only CRC-checked RTCM3 frames may reach the serial owner."""

import base64
from collections import deque
from dataclasses import dataclass, field
import json
import math
import os
import re
import socket
import stat
import threading
import time


@dataclass(frozen=True)
class Config:
    host: str
    port: int
    mountpoint: str
    username: str = field(repr=False)
    password: str = field(repr=False)
    gga_interval_sec: float = 1.0
    reconnect_sec: float = 5.0
    data_timeout_sec: float = 10.0

    def __post_init__(self):
        if not isinstance(self.host, str) or not re.fullmatch(r"[A-Za-z0-9.-]+", self.host):
            raise ValueError("invalid NTRIP host")
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("invalid NTRIP port")
        if not isinstance(self.mountpoint, str) or not re.fullmatch(r"[A-Za-z0-9_.-]+", self.mountpoint):
            raise ValueError("invalid NTRIP mountpoint")
        for name in ("username", "password"):
            value = getattr(self, name)
            if (not isinstance(value, str) or not value or
                    any(ord(c) < 33 or ord(c) > 126 for c in value)):
                raise ValueError("invalid NTRIP credentials")
        if ":" in self.username:
            raise ValueError("invalid NTRIP username")
        for name in ("gga_interval_sec", "reconnect_sec", "data_timeout_sec"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("invalid NTRIP timing")


def load_config(path):
    # Credentials are never ROS parameters, command-line arguments, or log data.
    with open(path, "r", encoding="utf-8") as stream:
        mode = os.fstat(stream.fileno()).st_mode
        if not stat.S_ISREG(mode) or mode & 0o077:
            raise ValueError("NTRIP file must be private (chmod 600)")
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("invalid NTRIP configuration")
    return Config(**data)


def crc24q(data):
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmFrames:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xD3:
                del self.buffer[0]
                continue
            if len(self.buffer) < 3:
                break
            if self.buffer[1] & 0xFC:
                del self.buffer[0]
                continue
            length = ((self.buffer[1] & 3) << 8 | self.buffer[2]) + 6
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                frames.append(frame)
                del self.buffer[:length]
            else:
                self.rejected += 1
                del self.buffer[0]
        return frames


def read_response(sock):
    """Accept ICY's single status line or HTTP headers; never forward headers."""
    data = bytearray()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            part = sock.recv(4096)
        except socket.timeout:
            continue
        if not part:
            raise OSError("caster closed during handshake")
        data.extend(part)
        if b"\r\n" not in data:
            if len(data) > 16384:
                raise OSError("oversized caster response")
            continue
        first, body = bytes(data).split(b"\r\n", 1)
        if first == b"ICY 200 OK":
            return body
        if not re.fullmatch(rb"HTTP/1\.[01] 200(?: .*)?", first):
            # Report only a known numeric code, never arbitrary server text.
            match = re.match(rb"HTTP/1\.[01] ([0-9]{3})", first)
            code = match.group(1).decode() if match else "invalid"
            raise OSError("caster response " + code)
        if b"\r\n\r\n" in data:
            headers, body = bytes(data).split(b"\r\n\r\n", 1)
            if len(headers) > 16384:
                raise OSError("oversized caster response")
            lower = headers.lower()
            if b"transfer-encoding:" in lower or b"sourcetable" in lower or b"text/html" in lower:
                raise OSError("unsupported caster response body")
            return body
        if len(data) > 16384:
            raise OSError("oversized caster response")
    raise OSError("caster handshake timeout")


class NtripClient:
    MAX_QUEUE_BYTES = 16384
    MAX_QUEUE_AGE = 2.0

    def __init__(self, config, gga_max_age=1.5):
        self.config = config
        self.gga_max_age = gga_max_age
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ready = False
        self.gga = None
        self.generation = 0
        self.queue = deque()
        self.queued_bytes = 0
        self.state = "waiting_receiver"
        self.error = ""
        self.last_rtcm = None
        self.rx_bytes = self.tx_bytes = self.frames = self.rejected = self.reconnects = 0
        self.gga_sent = 0
        self.sock = None
        self.thread = threading.Thread(target=self.run, name="rtk-ntrip", daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        with self.lock:
            sock = self.sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=6)

    def set_receiver(self, ready):
        with self.lock:
            if self.ready != ready:
                self.generation += 1
                self.queue.clear()
                self.queued_bytes = 0
                self.gga = None
            self.ready = ready

    def update_gga(self, line, now):
        # Called only with a checksum-validated, valid GGA by the serial owner.
        with self.lock:
            self.gga = (line + b"\r\n", now) if line is not None else None

    def observation(self):
        with self.lock:
            valid = (self.ready and self.gga is not None and
                     time.monotonic() - self.gga[1] <= self.gga_max_age)
            return self.generation, self.gga[0] if valid else None

    def take_frame(self, now):
        with self.lock:
            if self.queue:
                stamp, frame = self.queue.popleft()
                self.queued_bytes -= len(frame)
                if now - stamp <= self.MAX_QUEUE_AGE:
                    return self.generation, stamp, frame
                self.queue.clear()
                self.queued_bytes = 0
        return None

    def record_written(self, size):
        with self.lock:
            self.tx_bytes += size

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, error=self.error, generation=self.generation,
                        received_bytes=self.rx_bytes, written_bytes=self.tx_bytes,
                        received_frames=self.frames, rejected_frames=self.rejected,
                        queued_bytes=self.queued_bytes, reconnects=self.reconnects,
                        gga_sent=self.gga_sent, rtcm_age_sec=-1.0 if self.last_rtcm is None
                        else time.monotonic() - self.last_rtcm)

    def run(self):
        while not self.stop.is_set():
            generation, gga = self.observation()
            if gga is None:
                with self.lock:
                    self.state = "waiting_gga" if self.ready else "waiting_receiver"
                self.stop.wait(0.2)
                continue
            try:
                with self.lock:
                    self.state, self.error = "connecting", ""
                self.session(generation)
            except OSError as exc:
                # Socket errors may include peer data; expose only our fixed error messages.
                known = ("caster response ", "caster handshake timeout", "oversized caster response",
                         "unsupported caster response body", "caster closed", "no RTCM",
                         "receiver/GGA unavailable", "correction queue full")
                message = str(exc)
                with self.lock:
                    self.error = message if message.startswith(known) else type(exc).__name__
                    self.state = "retrying"
                    self.reconnects += 1
            finally:
                with self.lock:
                    self.queue.clear()
                    self.queued_bytes = 0
                    self.generation += 1
            self.stop.wait(self.config.reconnect_sec)

    def session(self, generation):
        cfg = self.config
        token = base64.b64encode(f"{cfg.username}:{cfg.password}".encode()).decode()
        request = (f"GET /{cfg.mountpoint} HTTP/1.0\r\nHost: {cfg.host}:{cfg.port}\r\n"
                   "User-Agent: NTRIP f1_rtk_driver/1.0\r\n"
                   f"Authorization: Basic {token}\r\nConnection: close\r\n\r\n").encode()
        with socket.create_connection((cfg.host, cfg.port), timeout=5) as sock:
            sock.settimeout(0.5)
            with self.lock:
                self.sock = sock
            try:
                sock.sendall(request)
                first = read_response(sock)
                current, gga = self.observation()
                if current != generation or gga is None:
                    raise OSError("receiver/GGA unavailable")
                sock.sendall(gga)
                with self.lock:
                    self.gga_sent += 1
                    self.state = "connected"
                next_gga = time.monotonic() + cfg.gga_interval_sec
                last_frame = time.monotonic()
                decoder = RtcmFrames()
                data = first
                while not self.stop.is_set():
                    current, gga = self.observation()
                    if current != generation or gga is None:
                        raise OSError("receiver/GGA unavailable")
                    now = time.monotonic()
                    if now >= next_gga:
                        sock.sendall(gga)
                        with self.lock:
                            self.gga_sent += 1
                        next_gga = now + cfg.gga_interval_sec
                    if data:
                        before = decoder.rejected
                        frames = decoder.feed(data)
                        with self.lock:
                            self.rejected += decoder.rejected - before
                            if self.generation != generation:
                                raise OSError("receiver/GGA unavailable")
                            for frame in frames:
                                if self.queued_bytes + len(frame) > self.MAX_QUEUE_BYTES:
                                    raise OSError("correction queue full")
                                self.queue.append((now, frame))
                                self.queued_bytes += len(frame)
                                self.rx_bytes += len(frame)
                                self.frames += 1
                                self.last_rtcm = last_frame = now
                            if frames:
                                self.state = "streaming"
                    if now - last_frame > cfg.data_timeout_sec:
                        raise OSError("no RTCM before timeout")
                    try:
                        data = sock.recv(4096)
                        if not data:
                            raise OSError("caster closed stream")
                    except socket.timeout:
                        data = b""
            finally:
                with self.lock:
                    self.sock = None
