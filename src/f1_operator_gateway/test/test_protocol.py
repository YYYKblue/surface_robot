import json

import pytest

from operator_gateway.protocol import decode


def test_manual_input_is_a_plain_latest_intent():
    packet = decode(json.dumps({
        "op": "manual_input", "seq": 3, "direction": "FORWARD",
        "speed_percent": 25, "mode": "BOTH", "brush_enabled": False,
        "brush_percent": 0, "thrust_enabled": False, "thrust_percent": 0}).encode())
    assert packet["input"].direction == "FORWARD"
    assert packet["input"].sequence == 3


@pytest.mark.parametrize("packet", [
    {"op": "manual_input", "seq": -1, "direction": "FORWARD", "speed_percent": 25,
     "mode": "BOTH", "brush_enabled": False, "thrust_enabled": False},
    {"op": "manual_input", "seq": 1, "direction": "BAD", "speed_percent": 25,
     "mode": "BOTH", "brush_enabled": False, "thrust_enabled": False},
    {"op": "manual_input", "seq": 1, "direction": "FORWARD", "speed_percent": 10,
     "mode": "BOTH", "brush_enabled": False, "thrust_enabled": False},
    {"op": "task", "action": "acquire"},
])
def test_invalid_packet_rejected(packet):
    with pytest.raises(ValueError):
        decode(json.dumps(packet).encode())


def test_duplicate_keys_rejected():
    with pytest.raises(ValueError):
        decode(b'{"op":"hello","op":"manual_stop"}')
