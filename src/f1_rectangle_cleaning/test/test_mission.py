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


def update(m, elapsed=0., **kwargs):
    now = NOW + round(elapsed * 1e9)
    return m.update(sample(m.c, now, **kwargs), now)


def test_single_side_demo_stops_without_transfer():
    history = run_demo()
    assert history[0] == "FOLLOW" and history[-1] == "STOPPED"
    assert "CORRECT_HEADING" in history and "NEUTRAL_DWELL" not in history
    assert not any("TRANSFER" in p or "_B" in p for p in history)


def test_default_parallel_brush_propulsion_keeps_main_motors_neutral():
    m = started()
    e = update(m)
    fields = command_fields(e, m.c)
    assert fields == dict(direction=0, speed_percent=25, mode=1, brush_enabled=True,
                          brush_percent=15., thrust_enabled=False, thrust_percent=0.)
    assert e.direction_until_ns == 0


def test_side_distance_correction_is_bidirectional_and_has_no_history():
    m = started()
    for i, (distance, expected) in enumerate([
        (.42, 0), (.421, 15), (.42, 0), (.40, 0), (.38, 0),
        (.379, -15), (.38, 0), (.40, 0), (.42, 0), (.421, 15)]):
        e = update(m, i * .05, distance=distance)
        assert e.side_left_percent == expected
        fields = command_fields(e, m.c)
        assert fields["thrust_percent"] == expected
        assert e.brush_percent == 15 and e.direction == 0


@pytest.mark.parametrize("angle,direction", [(0, 0), (3, 0), (-3, 0), (10, 0), (-10, 0),
                                            (10.01, 3), (-10.01, 4), (20, 3), (-20, 4)])
def test_heading_band_brush_gate_and_direction(angle, direction):
    m = started()
    e = update(m, angle=math.radians(angle))
    assert e.direction == direction
    assert e.brush_percent == (15 if direction == 0 else 0)
    assert command_fields(e, m.c)["speed_percent"] == 25


@pytest.mark.parametrize("angle,direction", [(12, 3), (-12, 4)])
def test_continuous_heading_correction_has_no_pulses_or_dwell(angle, direction):
    m = started()
    angle = math.radians(angle)
    for i in range(30):
        e = update(m, i * .05, angle=angle, distance=.5)
        assert e.direction == direction and e.direction_until_ns == 0
        assert e.brush_percent == 0 and e.side_left_percent == 15
        assert e.valid_until_ns == e.computed_ns + 100_000_000
        assert m.phase == "CORRECT_HEADING"


def test_alignment_immediately_restores_brush_and_correction_reverses():
    m = started()
    assert update(m, angle=.21).direction == 3
    e = update(m, .04, angle=math.radians(10))
    assert e.brush_percent == 15 and e.direction == 0
    e = update(m, .08, angle=-.21)
    assert e.direction == 4 and e.brush_percent == 0
    assert update(m, .12, angle=.21).direction == 3


@pytest.mark.parametrize("angle,direction", [(.21, 3), (-.21, 4)])
def test_turn_space_and_hull_flags_do_not_block_start_or_correction(angle, direction):
    m = Mission()
    f = sample(m.c, NOW, angle=angle)
    f = replace(f, wall=replace(f.wall, left_turn_clear=False, right_turn_clear=False,
                                rigid_collision=True))
    assert m.start(f, NOW)[0]
    e = m.update(f, NOW)
    assert e.direction == direction and e.brush_percent == 0
    assert m.phase == "CORRECT_HEADING"


@pytest.mark.parametrize("distance,side", [(.30, -15), (.8, 15)])
def test_old_wall_distance_envelope_does_not_block_brush(distance, side):
    m = started()
    e = update(m, distance=distance)
    assert e.brush_percent == 15 and e.side_left_percent == side


def test_map_stopping_with_blind_bow_and_no_local_velocity():
    m = started()
    assert update(m).brush_percent == 15  # front_observed=False is intentional.
    x = m.c.length_m - m.c.hull_half_length_m - m.c.front_stop_m
    assert update(m, .05, along=x - .001).brush_percent == 15
    e = update(m, .10, along=x + 1e-9)
    assert m.phase == "STOPPED"
    assert (e.direction, e.brush_percent, e.side_left_percent) == (0, 0, 0)
    assert "drift unmeasured" in m.reason
    assert update(m, .15) is None  # no automatic restart or second side


def test_map_bow_clearance_accounts_for_heading_and_stops_on_pose_only():
    m = started()
    update(m)
    now = NOW + 50_000_000
    x = m.c.length_m - m.c.hull_half_length_m - m.c.front_stop_m - .01
    f = sample(m.c, now, along=x, angle=.1)
    assert m.front_remaining(f.pose) < .8
    f = replace(f, wall=replace(f.wall, stamp_ns=NOW))
    assert m.update(f, now).brush_percent == 0
    assert m.phase == "STOPPED"


@pytest.mark.parametrize("clearance", [0., .7, .8])
def test_available_front_obstacle_stops_before_map_endpoint(clearance):
    m = started()
    f = sample(m.c, NOW, front_observed=True)
    e = m.update(replace(f, wall=replace(f.wall, front_clearance=clearance)), NOW)
    assert e.direction == 0 and e.brush_percent == 0 and m.phase == "STOPPED"


@pytest.mark.parametrize("angle", [0., .1, -.1])
def test_map_front_clearance_invariant_under_rectangle_half_turn(angle):
    m = started()
    f = sample(m.c, NOW, angle=angle)
    x, y = m.c.pool_xy(f.pose.x, f.pose.y)
    rx, ry = m.c.map_xy(m.c.length_m - x, m.c.width_m - y)
    flipped = replace(f.pose, x=rx, y=ry, yaw=f.pose.yaw + math.pi)
    assert m.front_remaining(flipped) == pytest.approx(m.front_remaining(f.pose))
    # The opposite heading is accepted without a special current-side config.
    other = Mission()
    assert other.start(replace(f, pose=flipped), NOW)[0]
    assert other.update(replace(f, pose=flipped), NOW).brush_percent == 15


def test_negative_map_direction_stops_at_short_wall_ahead():
    m = Mission()
    f = sample(m.c, NOW, along=1.5)
    f = replace(f, pose=replace(f.pose, yaw=m.c.axis_yaw_rad + math.pi))
    assert m.start(f, NOW)[0]
    x, y = m.c.map_xy(m.c.hull_half_length_m + m.c.front_stop_m - 1e-9, .5)
    f = replace(f, pose=replace(f.pose, x=x, y=y))
    assert m.update(f, NOW).brush_percent == 0 and m.phase == "STOPPED"


def test_start_inside_front_stop_zone_never_enables_outputs():
    m = Mission()
    x = m.c.length_m - m.c.hull_half_length_m - m.c.front_stop_m
    assert not m.start(sample(m.c, NOW, along=x + 1e-9), NOW)[0]
    assert m.phase == "IDLE" and m.last_effort is None


@pytest.mark.parametrize("field", ["wall", "pose"])
def test_stale_feedback_pauses_and_never_auto_resumes(field):
    m = started()
    f = sample(m.c, NOW)
    f = replace(f, **{field: replace(getattr(f, field), stamp_ns=NOW - 300_000_000)})
    assert m.update(f, NOW) is None
    assert m.phase == "PAUSED"
    assert update(m, .05) is None


@pytest.mark.parametrize("changes", [dict(valid=False), dict(distance=0.),
    dict(distance=float("nan")), dict(angle=float("inf"))])
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
    assert m.update(f, NOW + 50_000_000) is None and m.last_effort == effort
    assert m.update(f, NOW + 100_000_000) is None and m.phase == "PAUSED"


def test_unknown_fifth_mapping_inhibited_in_preview():
    effort = Effort(NOW, NOW + 100_000_000, side_left_percent=15., brush_percent=15.)
    assert not command_fields(effort, Config(fifth_left_sign=0))["thrust_enabled"]
    for sign in (-1, 1):
        fields = command_fields(effort, Config(fifth_left_sign=sign))
        assert fields["direction"] == 0 and fields["thrust_percent"] == sign * 15
        assert fields["thrust_enabled"] and fields["brush_enabled"]


def test_map_wall_identity_and_second_quality_gate_removed():
    m = started()
    f = sample(m.c, NOW)
    x, y = m.c.map_xy(.8, .4)
    f = replace(f, pose=replace(f.pose, x=x, y=y),
                wall=replace(f.wall, support=.1, residual=.1, rigid_collision=True))
    assert m.error(f, NOW) == ""
    assert m.update(f, NOW).brush_percent == 15


@pytest.mark.parametrize("changes", [dict(fifth_left_sign=2), dict(wall_target_m=.32),
    dict(length_m=.5), dict(command_ttl_sec=.2), dict(side_percent=100.), dict(width_m=math.nan),
    dict(side_on_distance_m=.38), dict(brush_percent=101.), dict(turn_speed_percent=50),
    dict(heading_deadband_rad=math.pi), dict(run_timeout_sec=-1.)])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):
        replace(Config(), **changes).validate()


def test_future_feedback_rejected_but_no_feedback_skew_gate():
    m = started()
    f = sample(m.c, NOW)
    assert "future" in m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW + 1)), NOW)
    assert m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW - 110_000_000)), NOW) == ""


def test_pause_requires_cancel_and_new_start():
    m = started()
    m.halt("test")
    assert not m.start(sample(m.c, NOW), NOW)[0]
    m.halt("cancel", "CANCELLED")
    assert m.start(sample(m.c, NOW), NOW)[0]


def test_timeout_stops_brush_and_side_request():
    m = started(Config(run_timeout_sec=.2))
    for i in range(5):
        update(m, i * .05, distance=.55)
    assert m.phase == "PAUSED" and m.last_effort is None


def test_default_run_has_no_arbitrary_timeout():
    m = started()
    assert update(m, elapsed=180.).brush_percent == 15 and m.phase == "FOLLOW"
