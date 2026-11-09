"""One background map encoder, with one replaceable input and output slot."""
import base64
import hashlib
import math
import threading
import time
import zlib


_PALETTE = bytes(0 if value >= 128 else 2 if value > 50 else 1 for value in range(256))


def encode_map(message):
    """Encode the display palette only; the ROS occupancy grid stays unchanged."""
    info = message.info
    width, height = int(info.width), int(info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError("invalid occupancy grid dimensions")
    origin, q = info.origin, info.origin.orientation
    resolution = float(info.resolution)
    x, y = float(origin.position.x), float(origin.position.y)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if resolution <= 0 or not all(math.isfinite(v) for v in (resolution, x, y, yaw)):
        raise ValueError("invalid occupancy grid geometry")
    try:
        raw = memoryview(message.data).cast("B").tobytes()
    except TypeError:
        raw = bytes(int(value) & 255 for value in message.data)
    pixels = raw.translate(_PALETTE)
    geometry = (message.header.frame_id, width, height, resolution, x, y, yaw)
    signature = geometry, hashlib.sha256(pixels).digest()
    packet = dict(frame_id=geometry[0], width=width, height=height, resolution=resolution,
                  origin=dict(x=x, y=y, yaw=yaw), encoding="indexed8_zlib",
                  data_zlib_base64=base64.b64encode(zlib.compress(pixels, level=1)).decode("ascii"))
    return signature, packet


class LatestMapEncoder:
    """Keep fresh mapping work bounded and discard results from previous sessions."""
    def __init__(self, period=2., *, encoder=encode_map):
        if not math.isfinite(period) or period <= 0:
            raise ValueError("map stream period must be positive")
        self.period, self.encoder = period, encoder
        self.condition = threading.Condition()
        self.generation = 0
        self.pending = self.ready = self.signature = None
        self.frozen_claimed = False
        self.next_at = 0.
        self.closed = False
        self.error = None
        self.thread = threading.Thread(target=self._run, name="operator-map-encoder", daemon=True)
        self.thread.start()

    def offer(self, message, *, frozen=False):
        with self.condition:
            if self.closed or self.frozen_claimed:
                return False
            if frozen:
                self.frozen_claimed = True
            self.pending = self.generation, message, frozen
            self.condition.notify()
            return True

    def take(self):
        with self.condition:
            packet, self.ready = self.ready, None
            return packet

    def reset(self):
        with self.condition:
            self.generation += 1
            self.pending = self.ready = self.signature = None
            self.frozen_claimed = False
            self.next_at = 0.
            self.error = None
            self.condition.notify()

    def _run(self):
        while True:
            with self.condition:
                while not self.closed:
                    delay = self.next_at - time.monotonic()
                    if self.pending is not None and delay <= 0:
                        job, self.pending = self.pending, None
                        break
                    self.condition.wait(timeout=max(.001, delay) if self.pending is not None else None)
                if self.closed:
                    return
            generation, message, frozen = job
            try:
                signature, packet = self.encoder(message)
                error = None
            except Exception as exc:
                signature = packet = None
                error = str(exc)
            with self.condition:
                if generation != self.generation or self.closed:
                    continue
                self.next_at = time.monotonic() + self.period
                self.error = error
                if error:
                    if frozen:
                        self.frozen_claimed = False
                elif signature != self.signature:
                    self.signature, self.ready = signature, packet

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.ready = None
            self.condition.notify()
        self.thread.join(timeout=2.)
