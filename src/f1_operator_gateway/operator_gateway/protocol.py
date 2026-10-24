"""Small, application-owned protocol used by the new Qt operator app.

There is deliberately no acquire/release operation here. Ownership and the
watchdog are local to the Nano gateway; the Windows app sends intent only.
"""
from dataclasses import dataclass
import json
import math

PROTOCOL = "f1-operator-v1"
MAX_PAYLOAD = 128 * 1024
OPERATIONS = {"hello", "manual_input", "manual_stop", "task"}
DIRECTIONS = {"STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT"}
MODES = {"FRONT", "BOTH", "BACK"}
TASKS = {"start", "pause", "cancel"}


@dataclass(frozen=True)
class ManualInput:
    sequence: int
    direction: str
    speed_percent: int
    mode: str
    brush_enabled: bool
    brush_percent: float
    thrust_enabled: bool
    thrust_percent: float


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _number(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} outside allowed range")
    return value


def decode(raw):
    if isinstance(raw, str):
        raw = raw.encode("utf-8")
    if not isinstance(raw, bytes) or len(raw) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    packet = json.loads(raw, object_pairs_hook=_pairs)
    if type(packet) is not dict or packet.get("protocol", PROTOCOL) != PROTOCOL:
        raise ValueError("invalid protocol")
    op = packet.get("op")
    if op not in OPERATIONS:
        raise ValueError("invalid operation")
    if op == "hello":
        client = packet.get("client", "")
        if type(client) is not str or not 1 <= len(client) <= 80:
            raise ValueError("invalid client")
        return {"op": op, "client": client, "version": packet.get("version", 1)}
    if op == "manual_stop":
        reason = packet.get("reason", "operator")
        if type(reason) is not str or not 1 <= len(reason) <= 80:
            raise ValueError("invalid stop reason")
        return {"op": op, "reason": reason}
    if op == "task":
        action = packet.get("action")
        if action not in TASKS:
            raise ValueError("invalid task action")
        return {"op": op, "action": action}
    sequence = packet.get("seq")
    if type(sequence) is not int or not 0 <= sequence < 2**64:
        raise ValueError("seq must be uint64")
    direction = packet.get("direction")
    mode = packet.get("mode")
    if direction not in DIRECTIONS or mode not in MODES:
        raise ValueError("invalid direction or wheel mode")
    speed = packet.get("speed_percent")
    if speed not in (25, 50, 75):
        raise ValueError("speed_percent must be 25, 50 or 75")
    fields = {}
    for name in ("brush_enabled", "thrust_enabled"):
        if type(packet.get(name)) is not bool:
            raise ValueError(f"{name} must be bool")
        fields[name] = packet[name]
    fields["brush_percent"] = float(_number(packet.get("brush_percent", 0), "brush_percent", -100, 100))
    fields["thrust_percent"] = float(_number(packet.get("thrust_percent", 0), "thrust_percent", -100, 100))
    return {"op": op, "input": ManualInput(sequence, direction, speed, mode, **fields)}
