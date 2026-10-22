from dataclasses import replace
import math

import pytest

from f1_rectangle_cleaning.config import Config
from f1_rectangle_cleaning.demo import sample, run_demo
from f1_rectangle_cleaning.mission import Mission, command_fields, Effort


NOW = 1_000_000_000


def started(config=None):
    m = Mission(config)
    assert m.start(sample(m.c, NOW), NOW)[0]
    return m


def follow(m):
    for i in range(12):
        now = NOW + i * 50_000_000
        m.update(sample(m.c, now), now)
    assert m.phase == "FOLLOW_A"
    return now


def test_two_long_segments_require_explicit_manual_transfer():
    assert run_demo() == ["APPROACH_A", "FOLLOW_A", "STOP_A", "WAIT_MANUAL_TRANSFER",
                          "APPROACH_B", "FOLLOW_B", "STOP_B", "FINISHED_SEGMENTS"]


@pytest.mark.parametrize("field", ["wall", "pose", "motion"])
def test_stale_feedback_pauses_and_never_auto_resumes(field):
    m = started()
    f = sample(m.c, NOW)
    f = replace(f, **{field: replace(getattr(f, field), stamp_ns=NOW - 300_000_000)})
    assert m.update(f, NOW) is None
    assert m.phase == "PAUSED"
    assert m.update(sample(m.c, NOW + 50_000_000), NOW + 50_000_000) is None


@pytest.mark.parametrize("changes", [dict(valid=False), dict(front_observed=False),
    dict(front_fraction=.4), dict(rigid_collision=True), dict(angle=.3),
    dict(distance=.35), dict(distance=float("nan")), dict(support=.1), dict(residual=.1)])
def test_bad_wall_feedback_stops(changes):
    m = started()
    f = sample(m.c, NOW)
    assert m.update(replace(f, wall=replace(f.wall, **changes)), NOW) is None
    assert m.phase == "PAUSED"


def test_repeated_scan_never_renews_deadline():
    m = started()
    f = sample(m.c, NOW)
    effort = m.update(f, NOW)
    assert effort.valid_until_ns == NOW + 100_000_000
    assert m.update(f, NOW + 50_000_000) is None
    assert m.last_effort == effort
    assert m.update(f, NOW + 100_000_000) is None
    assert m.phase == "PAUSED"


def test_fifth_mapping_unknown_is_inhibited_and_known_sign_combines_with_forward():
    effort = Effort(NOW, NOW + 100_000_000, NOW + 100_000_000, 1, 8.)
    assert not command_fields(effort, Config())["thrust_enabled"]
    for sign in (-1, 1):
        fields = command_fields(effort, Config(fifth_left_sign=sign))
        assert fields["direction"] == 1 and fields["thrust_percent"] == sign * 8.
        assert fields["thrust_enabled"] and not fields["brush_enabled"]


def test_side_thrust_slew_and_immediate_removal_when_too_close():
    m = started()
    a = m.update(sample(m.c, NOW + 50_000_000, distance=.55), NOW + 50_000_000)
    assert 0 < a.side_left_percent <= 1.
    b = m.update(sample(m.c, NOW + 100_000_000, distance=.55), NOW + 100_000_000)
    assert b.side_left_percent <= 2.
    e = m.update(sample(m.c, NOW + 150_000_000, distance=.42), NOW + 150_000_000)
    assert e.direction == 0 and e.side_left_percent == 0


def test_forward_pulse_has_fixed_cutoff_and_neutral_dwell_can_keep_side_thrust():
    m = started(Config(fifth_left_sign=1))
    now = follow(m)
    first = m.last_effort
    assert first.direction == 1
    now += 50_000_000
    e = m.update(sample(m.c, now, distance=.46), now)
    assert e.direction == 0 and e.side_left_percent > 0
    assert e.direction_until_ns == 0
    for i in range(1, 8):
        t = now + i * 50_000_000
        assert m.update(sample(m.c, t, distance=.46), t).direction == 0


def test_stop_waits_for_continuous_stationary_feedback():
    m = started()
    now = follow(m)
    end = m.c.length_m - m.c.end_inset_m + .001
    for i in range(1, 13):
        t = now + i * 50_000_000
        e = m.update(sample(m.c, t, along=end, speed=.05), t)
        assert e.direction == 0 and e.side_left_percent == 0
        assert m.phase == "STOP_A"
    now = t
    for i in range(1, 13):
        t = now + i * 50_000_000
        m.update(sample(m.c, t, along=end), t)
    assert m.phase == "WAIT_MANUAL_TRANSFER"
    assert m.completed_segments == ["A"]
    assert not m.confirm_second_wall(sample(m.c, t, along=end), t)[0]
    assert m.confirm_second_wall(sample(m.c, t, "B", end), t)[0]


def test_front_obstacle_before_planned_endpoint_does_not_mark_segment_complete():
    m = started()
    f = sample(m.c, NOW)
    m.update(replace(f, wall=replace(f.wall, front_clearance=.4)), NOW)
    assert m.phase == "PAUSED" and not m.completed_segments
    assert "front obstacle" in m.reason


def test_map_endpoint_with_inconsistent_front_wall_pauses():
    m = started()
    f = sample(m.c, NOW, along=m.c.length_m - m.c.end_inset_m + .001)
    m.update(replace(f, wall=replace(f.wall, front_clearance=1.5)), NOW)
    assert m.phase == "PAUSED" and "disagree" in m.reason


def test_rectangle_frame_roundtrip_and_wrong_wall_identity():
    c = Config()
    assert c.pool_xy(*c.map_xy(.8, .5)) == pytest.approx((.8, .5))
    m = Mission(c)
    f = sample(c, NOW, "B", c.length_m - c.end_inset_m)
    f = replace(f, pose=replace(f.pose, yaw=c.axis_yaw_rad))
    assert not m.start(f, NOW)[0]


@pytest.mark.parametrize("changes", [dict(fifth_left_sign=2), dict(wall_target_m=.32),
    dict(length_m=1.), dict(command_ttl_sec=.2), dict(end_inset_m=.5), dict(side_max_percent=100.),
    dict(width_m=math.nan)])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):
        replace(Config(), **changes).validate()


def test_future_feedback_and_timestamp_skew_rejected():
    m = started()
    f = sample(m.c, NOW)
    assert "future" in m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW + 1)), NOW)
    assert "skew" in m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW - 110_000_000)), NOW)


def test_pause_requires_cancel_and_new_start_at_A():
    m = started()
    m.halt("test")
    assert not m.start(sample(m.c, NOW), NOW)[0]
    m.halt("cancel", "CANCELLED")
    assert m.start(sample(m.c, NOW), NOW)[0]


def test_approach_timeout_prevents_indefinite_pushing():
    m = started(Config(phase_timeout_sec=.2))
    for i in range(1, 6):
        now = NOW + i * 50_000_000
        m.update(sample(m.c, now, distance=.55), now)
    assert m.phase == "PAUSED" and m.side_output == 0
