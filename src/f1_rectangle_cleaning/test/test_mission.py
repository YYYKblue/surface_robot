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
    assert "CORRECT_HEADING" in history and "NEUTRAL_DWELL" in history
    assert not any("TRANSFER" in p or "_B" in p for p in history)


def test_default_parallel_brush_propulsion_keeps_main_motors_neutral():
    m = started()
    e = update(m)
    fields = command_fields(e, m.c)
    assert fields == dict(direction=0, speed_percent=25, mode=1, brush_enabled=True,
                          brush_percent=15., thrust_enabled=False, thrust_percent=0.)
    assert e.direction_until_ns == 0


def test_hysteresis_strict_boundaries_and_history():
    m = started()
    for i, (distance, expected) in enumerate([
        (.45, 0), (.451, 15), (.45, 15), (.40, 15), (.38, 15),
        (.379, 0), (.38, 0), (.40, 0), (.45, 0), (.451, 15)]):
        e = update(m, i * .05, distance=distance)
        assert e.side_left_percent == expected
        fields = command_fields(e, m.c)
        assert fields["thrust_percent"] == expected
        assert e.brush_percent == 15 and e.direction == 0


@pytest.mark.parametrize("angle,direction", [(3, 0), (-3, 0), (3.01, 3), (-3.01, 4)])
def test_heading_band_brush_gate_and_direction(angle, direction):
    m = started()
    e = update(m, angle=math.radians(angle))
    assert e.direction == direction
    assert e.brush_percent == (15 if direction == 0 else 0)
    assert command_fields(e, m.c)["speed_percent"] == 25


@pytest.mark.parametrize("angle,direction", [(5, 3), (-5, 4)])
def test_fixed_pulse_deadline_and_full_neutral_dwell(angle, direction):
    m = started()
    angle = math.radians(angle)
    e = update(m, angle=angle)
    assert e.direction_until_ns == NOW + 100_000_000
    e2 = update(m, .05, angle=angle)
    assert e2.direction == direction and e2.direction_until_ns == e.direction_until_ns
    for t in (.10, .15, .20, .25, .30, .35, .40, .45, .50, .55, .599):
        e = update(m, t, angle=angle)
        assert e.direction == 0 and e.brush_percent == 0
    assert update(m, .60, angle=angle).direction == direction


def test_early_alignment_ends_pulse_brush_on_and_reversal_waits():
    m = started()
    assert update(m, angle=.1).direction == 3
    assert update(m, .04, angle=0.).brush_percent == 15
    for t in (.08, .13, .18, .23, .28, .33, .38, .43, .48, .53):
        assert update(m, t, angle=-.1).direction == 0
    assert update(m, .54, angle=-.1).direction == 4


def test_sign_change_cannot_reverse_inside_pulse():
    m = started()
    update(m, angle=.1)
    e = update(m, .03, angle=-.1)
    assert e.direction == 0 and e.brush_percent == 0
    assert m.next_pulse == NOW + 530_000_000


@pytest.mark.parametrize("angle,field", [(.1, "left_turn_clear"), (-.1, "right_turn_clear")])
def test_turn_space_unknown_or_blocked_latches_pause(angle, field):
    m = started()
    f = sample(m.c, NOW, angle=angle)
    assert m.update(replace(f, wall=replace(f.wall, **{field: False})), NOW) is None
    assert m.phase == "PAUSED" and m.side_output == 0


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


def test_available_front_obstacle_stops_before_map_endpoint():
    m = started()
    f = sample(m.c, NOW, front_observed=True)
    e = m.update(replace(f, wall=replace(f.wall, front_clearance=.7)), NOW)
    assert e.direction == 0 and e.brush_percent == 0 and m.phase == "STOPPED"


@pytest.mark.parametrize("field", ["wall", "pose"])
def test_stale_feedback_pauses_and_never_auto_resumes(field):
    m = started()
    f = sample(m.c, NOW)
    f = replace(f, **{field: replace(getattr(f, field), stamp_ns=NOW - 300_000_000)})
    assert m.update(f, NOW) is None
    assert m.phase == "PAUSED"
    assert update(m, .05) is None


@pytest.mark.parametrize("changes", [dict(valid=False), dict(rigid_collision=True),
    dict(angle=.3), dict(distance=.35), dict(distance=float("nan")), dict(support=.1), dict(residual=.1)])
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


def test_wrong_map_wall_or_heading_rejected():
    m = started()
    f = sample(m.c, NOW)
    x, y = m.c.map_xy(.8, .4)
    assert "disagree" in m.error(replace(f, pose=replace(f.pose, x=x, y=y)), NOW)
    assert "heading" in m.error(replace(f, pose=replace(f.pose, yaw=m.c.axis_yaw_rad + math.pi)), NOW)


@pytest.mark.parametrize("changes", [dict(fifth_left_sign=2), dict(wall_target_m=.32),
    dict(length_m=1.), dict(command_ttl_sec=.2), dict(side_percent=100.), dict(width_m=math.nan),
    dict(side_on_distance_m=.38), dict(brush_percent=101.), dict(turn_speed_percent=50),
    dict(turn_pulse_sec=.11), dict(neutral_dwell_sec=.49)])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):
        replace(Config(), **changes).validate()


def test_future_feedback_and_skew_rejected():
    m = started()
    f = sample(m.c, NOW)
    assert "future" in m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW + 1)), NOW)
    assert "skew" in m.error(replace(f, pose=replace(f.pose, stamp_ns=NOW - 110_000_000)), NOW)


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
    assert m.phase == "PAUSED" and m.side_output == 0 and m.last_effort is None
