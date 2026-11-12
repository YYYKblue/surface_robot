import ctypes

import pytest

from under_control.protocol import decode_report
from under_control.session import Session


def exchange(h7, core, ms):
    frame = core.request(ms / 1000.)
    assert frame
    h7.test_feed(frame, len(frame), ms)
    h7.test_tick(ms + 3)
    buf = ctypes.create_string_buffer(1024)
    size = h7.test_read(buf)
    assert size
    reply = decode_report(buf.raw[:size])
    core.receive(reply, (ms + 3) / 1000.)
    return reply


def arm(h7):
    core = Session(True)
    core.opened()
    for ms in range(0, 3051, 50):
        exchange(h7, core, ms)
        if ms < 3000:
            assert not core.ready
    assert core.ready
    return core


@pytest.mark.parametrize("direction,fr,front", [("FORWARD", (0, 1), 60),
    ("REVERSE", (1, 0), 0), ("LEFT", (1, 1), 0), ("RIGHT", (0, 0), 0)])
def test_actual_h7_direction_lift_brush_and_release(h7, direction, fr, front):
    core = arm(h7)
    for ms in range(3100, 3501, 50):
        assert core.submit("app", dict(control_generation=core.generation,
            direction=direction, speed_percent=60), ms / 1000.)[0]
        exchange(h7, core, ms)
    assert [h7.test_output(i) for i in (0, 1, 2, 3)] == [60, 60, 40, front]
    assert tuple(h7.test_fr(i) for i in (0, 1)) == fr
    assert h7.test_top() == 1404
    core.stop("key released", "app")
    result = exchange(h7, core, 3550)
    assert result.neutral
    assert all(h7.test_output(i) == 0 for i in range(4))
    assert h7.test_top() == 1000


def test_real_h7_timeout_changes_epoch_and_recovery_does_not_replay(h7):
    core = arm(h7)
    old_generation = core.generation
    core.submit("app", dict(control_generation=old_generation, direction="FORWARD", speed_percent=60), 3.1)
    exchange(h7, core, 3100)
    h7.test_tick(3401)
    assert all(h7.test_output(i) == 0 for i in range(4))
    reply = exchange(h7, core, 3450)
    assert reply.epoch != 1 and not core.ready
    for ms in range(3500, 3901, 50):
        exchange(h7, core, ms)
    assert core.ready
    assert not core.submit("app", dict(control_generation=old_generation,
                                      direction="FORWARD", speed_percent=60), 3.91)[0]
    assert all(h7.test_output(i) == 0 for i in range(4))


def test_real_h7_remote_stop_semantics_and_return(h7):
    core = arm(h7)
    old_generation = core.generation
    h7.test_remote(1)
    reply = exchange(h7, core, 3100)
    assert reply.source == "RC" and not core.ready
    core.stop("App STOP cannot stop RC")
    for ms in range(3150, 3551, 50):
        exchange(h7, core, ms)
    assert h7.test_output(0) == 40 and h7.test_output(1) == 40
    h7.test_remote(0)
    for ms in range(3600, 4051, 50):
        exchange(h7, core, ms)
    assert core.ready
    assert not core.submit("app", dict(control_generation=old_generation,
                                      direction="FORWARD", speed_percent=60), 4.06)[0]
    assert all(h7.test_output(i) == 0 for i in range(4))
