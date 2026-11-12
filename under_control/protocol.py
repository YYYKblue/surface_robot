"""H7 original-motor V6 wire contract, separate from the surface F103 codec."""
import binascii
from dataclasses import dataclass
import json
import struct

PROTOCOL = "under-operator-v1"
DEVICE = "h7_under_original_v6"
DIRECTIONS = ("STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT")
U32 = 0xffffffff
RESULTS = {"WAIT_NANO", "WAIT_NEUTRAL", "OK", "LINK_LOST", "INVALID",
           "RC_OK", "RC_WAIT_NEUTRAL", "RC_LINK_LOST", "RC_INVALID", "RC_DIR_CONFLICT"}


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("integer outside permitted range")
    return value


def encode(epoch, sequence, direction="STOP", speed=0):
    integer(epoch, 0, U32)
    integer(sequence, 0, U32)
    integer(speed, 0, 100)
    if direction not in DIRECTIONS or (epoch == 0 and (direction != "STOP" or speed)):
        raise ValueError("invalid direction or discovery command")
    if direction == "STOP" or speed == 0:
        direction, speed = "STOP", 0
    body = struct.pack("<BBIIBB", 6, 2, epoch, sequence, DIRECTIONS.index(direction), speed)
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


@dataclass(frozen=True)
class Report:
    ms: int
    epoch: int
    have_seq: bool
    ack_seq: int
    result: str
    armed: bool
    direction: str
    speed_percent: int
    current: tuple
    goal: tuple
    ramp_ms: int

    @property
    def source(self):
        return "RC" if self.result.startswith("RC_") else "NANO"

    @property
    def neutral(self):
        return (self.direction == "STOP" and self.speed_percent == 0
                and self.current == self.goal == (1500, 1500, 1500))


def decode_report(line):
    if not isinstance(line, bytes) or len(line) > 256 or not line.endswith(b"\r\n"):
        raise ValueError("invalid H7 line framing")
    body, check = line[:-2].rsplit(b"*", 1)
    checksum = 0
    for byte in body:
        checksum ^= byte
    if len(check) != 2 or int(check, 16) != checksum:
        raise ValueError("H7 XOR mismatch")
    fields = body.decode("ascii").split(",")
    if len(fields) != 17 or fields[:2] != ["UNDER", "6"]:
        raise ValueError("expected UNDER V6 with 17 fields")

    def number(index, low=0, high=U32):
        text = fields[index]
        if not text.isascii() or not text.isdecimal():
            raise ValueError("invalid H7 integer")
        return integer(int(text), low, high)

    if fields[6] not in RESULTS or fields[8] not in DIRECTIONS:
        raise ValueError("invalid H7 result or direction")
    return Report(number(2), number(3, 1), bool(number(4, 0, 1)), number(5),
                  fields[6], bool(number(7, 0, 1)), fields[8], number(9, 0, 100),
                  tuple(number(i, 1000, 2000) for i in range(10, 13)),
                  tuple(number(i, 1000, 2000) for i in range(13, 16)), number(16, 300, 300))


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def decode_request(raw):
    if not isinstance(raw, (bytes, str)) or len(raw) > 4096:
        raise ValueError("invalid request size")
    packet = json.loads(raw, object_pairs_hook=_pairs)
    if not isinstance(packet, dict) or packet.get("protocol") != PROTOCOL:
        raise ValueError("expected underwater protocol")
    if packet.get("device") != DEVICE:
        raise ValueError("wrong device profile")
    op = packet.get("op")
    if op == "hello":
        if type(packet.get("version")) is not int or packet["version"] != 1:
            raise ValueError("unsupported client version")
    elif op == "manual_input":
        integer(packet.get("seq"), 0, 2**64 - 1)
        integer(packet.get("control_generation"), 1, 2**63 - 1)
        integer(packet.get("speed_percent"), 0, 100)
        if packet.get("direction") not in DIRECTIONS:
            raise ValueError("invalid direction")
    elif op == "manual_stop":
        reason = packet.get("reason", "operator")
        if not isinstance(reason, str) or not 1 <= len(reason) <= 80:
            raise ValueError("invalid stop reason")
    else:
        raise ValueError("unsupported operation")
    return packet
