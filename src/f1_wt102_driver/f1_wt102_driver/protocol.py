"""WT102 UART framing and yaw conversion from the manufacturer's protocol.

Reference: https://wit-motion.yuque.com/wumwnr/ltst03/ga5l6o0tenxoaqkv
"""

import math
import struct


GYRO_SCALE_DEG_S_PER_LSB = 2000.0 / 32768.0


class WitParser:
    """Incremental parser; retain partial frames and resync after corruption."""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b'\x55')
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 11:
                break
            frame = bytes(self.buffer[:11])
            if sum(frame[:10]) & 0xff != frame[10]:
                self.bad_checksums += 1
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:11]
        return frames


def decode_yaw(frame, gyro_scale_deg_s_per_lsb=GYRO_SCALE_DEG_S_PER_LSB):
    """Return (kind, SI value), or None for an unrelated WIT frame.

    The WT102 wire scale is 2000/32768, independent of its rated +/-500 deg/s range.
    Bytes 6..7 contain calibrated Z rate; bytes 4..5 contain the raw channel.
    In angle frames, bytes 8..9 contain version data, not temperature or yaw.
    Angular velocity here is body Z rate, not general 3D Euler yaw derivative.
    """
    if not math.isfinite(gyro_scale_deg_s_per_lsb) or gyro_scale_deg_s_per_lsb <= 0:
        raise ValueError('gyro_scale_deg_s_per_lsb must be finite and positive')
    if len(frame) != 11 or frame[0] != 0x55 or sum(frame[:10]) & 0xff != frame[10]:
        raise ValueError('invalid WIT frame')
    raw_z = struct.unpack_from('<h', frame, 6)[0]
    if frame[1] == 0x52:
        return 'yaw_rate', math.radians(raw_z * gyro_scale_deg_s_per_lsb)
    if frame[1] == 0x53:
        return 'yaw', raw_z * math.pi / 32768.0
    return None
