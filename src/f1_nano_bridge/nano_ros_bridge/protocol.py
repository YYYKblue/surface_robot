"""Pure codec for F1_IMU_NEW _back2 (wire v2)."""
import binascii
from dataclasses import dataclass
import math
import struct

U32 = 0xffffffff
MODES = ("FRONT", "BOTH", "BACK")
PROFILE = "f1_direction_gear_v2"
DIRECTIONS = ("STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT")
GEARS = (25, 50, 75)


def forward(new, old):
    return 0 < ((new - old) & U32) < 0x80000000


def percent(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("effort must be numeric")
    if not math.isfinite(value) or not -100 <= value <= 100:
        raise ValueError("effort must be finite and within [-100,100]")
    return value


def servo(value):
    value = percent(value)
    if value == 0:
        return 1500
    delta = math.floor(20 + 4.3 * abs(value) + 0.5)
    return 1500 + (delta if value > 0 else -delta)


def neutral():
    values = [1500] * 16
    values[5] = 1050  # Lowest main-wheel gear, including during handshake.
    values[9:11] = [1050, 1050]
    return values


def channels(direction=0, speed_percent=25, mode=1, brush_enabled=False,
             brush_percent=0, thrust_enabled=False, thrust_percent=0):
    if type(mode) is not int or mode not in (0, 1, 2):
        raise ValueError("invalid wheel group")
    if type(direction) is not int or direction not in range(5):
        raise ValueError("direction must be STOP/FORWARD/REVERSE/LEFT/RIGHT (0..4)")
    if type(speed_percent) is not int or speed_percent not in GEARS:
        raise ValueError("main-wheel gear must be 25, 50 or 75")
    if type(brush_enabled) is not bool or type(thrust_enabled) is not bool:
        raise ValueError("auxiliary enables must be boolean")
    percent(brush_percent)
    percent(thrust_percent)
    values = neutral()
    # The firmware owns wheel mixing and installation polarity in v2.
    values[2:4] = ((1500, 1500), (1950, 1500), (1050, 1500),
                   (1500, 1050), (1500, 1950))[direction]
    values[4] = (1050, 1500, 1950)[mode]
    values[5] = (1050, 1500, 1950)[GEARS.index(speed_percent)]
    values[7] = servo(brush_percent) if brush_enabled else 1500
    values[8] = servo(thrust_percent) if thrust_enabled else 1500
    values[9] = 1950 if brush_enabled else 1050
    values[10] = 1950 if thrust_enabled else 1050
    return values


def encode(values, epoch, sequence):
    if len(values) != 16 or any(type(v) is not int or not 1050 <= v <= 1950 for v in values):
        raise ValueError("expected 16 servo integers in [1050,1950]")
    if not 1 <= epoch <= U32 or not 0 <= sequence <= U32:
        raise ValueError("invalid epoch or wire sequence")
    packed = sum(((8 * v - 6990 + 2) // 5) << (11 * i) for i, v in enumerate(values))
    sbus = b"\x0f" + packed.to_bytes(22, "little") + b"\0\0"
    body = struct.pack("<BBII", 2, 25, epoch, sequence) + sbus
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


def number(text, minimum=0, maximum=U32):
    value = int(text)
    if str(value) != text or not minimum <= value <= maximum:
        raise ValueError("invalid integer")
    return value


def checked_fields(line):
    if not isinstance(line, bytes) or len(line) > 512:
        raise ValueError("invalid line size")
    body, check = line.rstrip(b"\r\n").split(b"*")
    if len(check) != 2 or any(c not in b"0123456789ABCDEFabcdef" for c in check):
        raise ValueError("invalid XOR text")
    checksum = 0
    for byte in body:
        checksum ^= byte
    if checksum != int(check, 16):
        raise ValueError("XOR mismatch")
    return body.decode("ascii").split(",")


@dataclass(frozen=True)
class Control:
    milliseconds: int
    source: str
    epoch: int
    have_sequence: bool
    ack: int
    result: str
    mode: str
    armed: bool
    brush_enabled: bool
    brush_percent: int
    thrust_enabled: bool
    thrust_percent: int
    pwm: tuple
    speed_percent: int = 25
    direction: str = "STOP"

    @property
    def neutral(self):
        return (self.direction == "STOP" and self.pwm == (1500,) * 4 and not self.brush_enabled
                and not self.thrust_enabled and self.brush_percent == self.thrust_percent == 0)


def decode(line):
    p = checked_fields(line)
    if p[0] == "CTRL":
        if len(p) != 20 or p[1] != "2" or p[3] not in ("NANO", "RC"):
            raise ValueError("invalid CTRL schema")
        if (p[7] not in ("WAIT_NANO", "WAIT_NEUTRAL", "OK", "DIR_CONFLICT", "LINK_LOST", "INVALID")
                or p[8] not in MODES or p[18] not in ("25", "50", "75") or p[19] not in DIRECTIONS):
            raise ValueError("invalid CTRL enum")
        return Control(number(p[2]), p[3], number(p[4], 1),
                       bool(number(p[5], 0, 1)), number(p[6]), p[7], p[8],
                       bool(number(p[9], 0, 1)), bool(number(p[10], 0, 1)),
                       number(p[11], -100, 100), bool(number(p[12], 0, 1)),
                       number(p[13], -100, 100), tuple(number(v, 1100, 1900) for v in p[14:18]),
                       number(p[18]), p[19])
    if p[0] == "IMU":
        if len(p) != 18:
            raise ValueError("invalid IMU schema")
        values = [float(v) for v in p[3:17]]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite IMU")
        return dict(kind="IMU", sequence=number(p[1]), stm32_ms=number(p[2]),
                    acceleration=values[:3], angular_velocity=values[3:6],
                    quaternion_wxyz=values[6:10], angles_degrees=values[10:14],
                    status=number(p[17], 0, 31))
    raise ValueError("unknown telemetry")


class Lines:
    """Bounded line framing; discard overlong fragments until the next newline."""
    def __init__(self):
        self.buffer = bytearray()
        self.dropping = False
        self.errors = 0

    def feed(self, data):
        result = []
        for byte in data:
            if byte == 10:
                if not self.dropping and self.buffer:
                    result.append(bytes(self.buffer))
                self.buffer.clear()
                self.dropping = False
            elif not self.dropping:
                self.buffer.append(byte)
                if len(self.buffer) > 512:
                    self.buffer.clear()
                    self.dropping = True
                    self.errors += 1
        return result
