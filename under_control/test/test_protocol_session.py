from dataclasses import replace
import json

import pytest

from under_control.protocol import DEVICE, PROTOCOL, Report, decode_report, decode_request, encode
from under_control.session import Session


def report(ms, epoch=1, ack=1, **changes):
    return replace(Report(ms, epoch, True, ack, "OK", True, "STOP", 0,
                          (1500,) * 3, (1500,) * 3, 300), **changes)


def ready():
    core = Session(motion_enabled=True)
    core.opened()
    core.request(0.)
    core.receive(report(100, have_seq=False, armed=False, result="WAIT_NANO"), .01)
    core.request(.05)
    core.receive(report(150, ack=core.sequence), .06)
    assert core.ready
    return core


def motion(core, direction="FORWARD", speed=50, **changes):
    return dict(dict(control_generation=core.generation, direction=direction, speed_percent=speed), **changes)


@pytest.mark.parametrize("epoch,sequence,direction,speed,expected", [
    (0, 0, "STOP", 0, "a55a060200000000000000000000742f"),
    (1, 1, "STOP", 0, "a55a0602010000000100000000009105"),
    (1, 100, "FORWARD", 50, "a55a060201000000640000000132a83c"),
    (1, 101, "STOP", 0, "a55a060201000000650000000000285c"),
])
def test_documented_wire_examples(epoch, sequence, direction, speed, expected):
    assert encode(epoch, sequence, direction, speed).hex() == expected


def test_report_checksum_schema_and_rc_source():
    body = b"UNDER,6,123,1,1,30,RC_OK,1,LEFT,50,1500,1500,1700,1250,1250,1700,300"
    check = 0
    for b in body:
        check ^= b
    wire = body + f"*{check:02X}\r\n".encode()
    state = decode_report(wire)
    assert state.source == "RC" and state.ack_seq == 30 and not state.neutral
    for bad in (wire[:-2], wire.replace(b"LEFT", b"LEFX"), wire.replace(b"UNDER", b"CTRL")):
        with pytest.raises(ValueError):
            decode_report(bad)


@pytest.mark.parametrize("fields", [dict(speed_percent=True), dict(direction="UP"),
    dict(control_generation=0), dict(speed_percent=101), dict(seq=-1),
    dict(protocol="f1-operator-v1"), dict(device="f103"), dict(op="task")])
def test_reject_wrong_device_and_invalid_commands(fields):
    packet = dict(protocol=PROTOCOL, device=DEVICE, op="manual_input", seq=1,
                  control_generation=1, direction="FORWARD", speed_percent=25)
    packet.update(fields)
    with pytest.raises(ValueError):
        decode_request(json.dumps(packet))


def test_reject_duplicate_json_key():
    with pytest.raises(ValueError):
        decode_request('{"op":"hello","op":"manual_input"}')


def test_discovery_cannot_move_and_start_requires_confirmed_neutral():
    core = Session(True)
    core.opened()
    assert not core.submit("a", motion(core), 0.)[0]
    assert core.request(0.)[12:14] == b"\0\0"
    core.receive(report(1, have_seq=False, armed=False, result="WAIT_NANO"), .01)
    assert not core.ready
    core.request(.05)
    core.receive(report(51, ack=core.sequence, current=(1600, 1400, 1700)), .06)
    assert not core.ready


def test_input_timeout_invalidates_generation_and_cannot_replay_old_hold():
    core = ready()
    packet = motion(core)
    assert core.submit("a", packet, .07)[0]
    assert core.request(.08)[12:14] == bytes((1, 50))
    core.receive(report(180, ack=core.sequence, direction="FORWARD", speed_percent=50), .09)
    core.tick(.321)
    assert core.intent == ("STOP", 0) and core.owner is None
    assert not core.submit("a", packet, .322)[0]
    assert core.request(.323)[12:14] == b"\0\0"


def test_stop_preempts_next_request_and_other_operator_cannot_override():
    core = ready()
    assert core.submit("a", motion(core), .07)[0]
    assert not core.submit("b", motion(core, "REVERSE"), .08)[0]
    assert not core.stop("other", "b")
    generation = core.generation
    assert core.stop("release", "a")
    assert core.generation != generation
    assert core.request(.09)[12:14] == b"\0\0"


def test_old_neutral_ack_cannot_confirm_or_rearm_a_new_stop():
    core = ready()
    core.request(.1)
    old_sequence = core.sequence
    core.stop("operator stop")
    core.receive(report(200, ack=old_sequence), .11)
    assert not core.ready and not core.snapshot(.12)["stop_confirmed"]
    core.request(.15)
    core.receive(report(250, ack=core.sequence), .16)
    assert core.ready and core.snapshot(.17)["stop_confirmed"]


def test_request_reply_no_overlap_and_ack_is_required():
    core = ready()
    assert core.request(.1)
    assert core.request(.11) is None
    with pytest.raises(ValueError, match="ACK"):
        core.receive(report(200, ack=core.sequence - 1), .12)


def test_rc_takeover_and_return_require_new_generation_and_neutral():
    core = ready()
    old = motion(core)
    assert core.submit("a", old, .07)[0]
    core.request(.08)
    core.receive(report(200, epoch=2, result="RC_OK", direction="LEFT", speed_percent=50), .09)
    assert not core.ready and core.intent == ("STOP", 0)
    assert core.snapshot(.1)["source"] == "RC"
    core.request(.13)
    core.receive(report(250, epoch=3, result="WAIT_NANO", armed=False), .14)
    assert not core.submit("a", old, .15)[0]
    assert core.request(.18)[12:14] == b"\0\0"
    core.receive(report(300, epoch=3, ack=core.sequence), .19)
    assert core.ready
    assert not core.submit("a", old, .2)[0]


@pytest.mark.parametrize("ms", [150, 1])
def test_repeated_uptime_or_reboot_even_with_same_epoch_invalidates_reply(ms):
    core = ready()
    core.request(.1)
    with pytest.raises(ValueError, match="uptime"):
        core.receive(report(ms, ack=core.sequence), .11)


def test_wire_sequence_and_uptime_wrap_are_valid():
    core = ready()
    core.sequence = 0xffffffff
    core.report = replace(core.report, ms=0xfffffff0)
    core.request(.1)
    assert core.sequence == 0
    core.receive(report(20, ack=0), .11)
    assert core.ready


def test_hardware_feedback_age_and_disabled_output_gate():
    core = ready()
    core.motion_enabled = False
    assert not core.submit("a", motion(core), .1)[0]
    assert not core.snapshot(.1)["available"]
    core.motion_enabled = True
    assert core.snapshot(.1)["available"]
    stale = core.snapshot(.311)
    assert not stale["available"] and stale["direction"] is None
    assert stale["source"] == "UNKNOWN" and not core.ready
