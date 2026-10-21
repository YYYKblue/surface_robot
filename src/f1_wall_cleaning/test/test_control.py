from dataclasses import replace
import math

import pytest

from f1_wall_cleaning.config import Config
from f1_wall_cleaning.control import Follower
from f1_wall_cleaning.perception import Observation


def wall(stamp=1_000_000_000, **kwargs):
    return replace(Observation(stamp, True, "wall", .48, 0., .002, 2., 100, .98,
                               (-.5, .48), (1.5, .48), True, .8, 1., False, True, True), **kwargs)


def test_missing_geometry_and_calibration_block_live_start(calibrated):
    assert not Follower().start(wall(), 1_000_000_000, live=True)
    c = replace(calibrated, pulse_response_calibrated=False)
    assert Follower(c).start(wall(), 1_000_000_000, live=False)
    assert not Follower(c).start(wall(), 1_000_000_000, live=True)
    for changed in (dict(hull_width_m=0.), dict(stopping_distance_m=.9),
                    dict(brush_compression_m=.05), dict(forward_pulse_sec=math.nan),
                    dict(command_ttl_sec=.2)):
        with pytest.raises(ValueError):
            replace(calibrated, **changed).validate()


@pytest.mark.parametrize("distance,sign", [(.8, 1), (.465, -1)])
def test_left_wall_far_turns_left_near_turns_right(calibrated, distance, sign):
    follower = Follower(calibrated)
    o = wall(distance=distance, angle=-.04 if sign < 0 else 0.)
    assert follower.start(o, o.stamp_ns)
    effort = follower.update(o, o.stamp_ns)
    assert effort.direction == (3 if sign > 0 else 4)
    assert effort.speed_percent == 25
    assert not effort.brush_enabled


def test_stable_contact_gates_brush_and_loss_disables_immediately(calibrated):
    f = Follower(calibrated)
    assert f.start(wall(), 1_000_000_000)
    for i in range(12):
        ns = 1_000_000_000 + i * 50_000_000
        effort = f.update(wall(ns), ns)
        assert effort.brush_enabled == (i >= 10 and effort.direction == 1)
    assert f.phase == "CLEAN_WALL" and f.brush_active_sec > 0
    effort = f.update(wall(1_600_000_000, distance=.55), 1_600_000_000)
    assert not effort.brush_enabled and f.phase == "APPROACH_WALL"


@pytest.mark.parametrize("change", [
    dict(valid=False, reason="lost"), dict(front_observed=False), dict(front_clearance=.2),
    dict(rigid_collision=True), dict(distance=.44), dict(end=(.5, .48)),
    dict(angle=.5), dict(residual=.1), dict(distance=math.nan), dict(ratio=.1)])
def test_unsafe_observation_stops_and_never_auto_resumes(calibrated, change):
    f = Follower(calibrated)
    assert f.start(wall(), 1_000_000_000)
    assert f.update(wall(), 1_000_000_000)
    assert f.update(wall(1_050_000_000, **change), 1_050_000_000) is None
    assert f.phase == "PAUSED_SAFETY"
    assert f.update(wall(1_100_000_000), 1_100_000_000) is None


def test_original_scan_bounds_deadline_and_duplicate_cannot_renew(calibrated):
    f = Follower(calibrated)
    assert f.start(wall(), 1_080_000_000)
    effort = f.update(wall(), 1_080_000_000)
    assert effort.deadline_ns == 1_150_000_000
    assert f.update(wall(), 1_120_000_000) is None
    assert f.last_effort is effort
    assert f.update(wall(1_150_000_000), 1_151_000_000) is None
    assert not f.active


def test_cancellation_and_run_limit_are_not_completion(calibrated):
    f = Follower(replace(calibrated, max_run_sec=.2, max_approach_sec=.2))
    assert f.start(wall(), 1_000_000_000)
    for i in range(5):
        ns = 1_000_000_000 + i * 50_000_000
        f.update(wall(ns), ns)
    assert f.phase == "PAUSED_SAFETY"
    assert "completion" in f.reason
    f.halt("CANCELLED", "cancel")
    assert f.update(wall(1_250_000_000), 1_250_000_000) is None


def test_pulses_are_bounded_and_separated_by_neutral_dwell(calibrated):
    f = Follower(calibrated)
    assert f.start(wall(), 1_000_000_000)
    outputs = []
    for i in range(15):
        ns = 1_000_000_000 + i * 50_000_000
        outputs.append(f.update(wall(ns), ns))
    assert all(e.direction == 1 for e in outputs[:4])
    assert all(e.direction == 0 and not e.brush_enabled for e in outputs[4:10])
    assert outputs[10].direction == 1
    assert outputs[0].direction_until_ns == outputs[3].direction_until_ns == 1_200_000_000
    assert outputs[10].direction_until_ns == 1_700_000_000


def test_unknown_turn_sweep_pauses_instead_of_pivoting(calibrated):
    f = Follower(calibrated)
    o = wall(distance=.8, left_turn_clear=False)
    assert f.start(o, o.stamp_ns)
    assert f.update(o, o.stamp_ns) is None
    assert f.phase == "PAUSED_SAFETY" and "sweep" in f.reason


def test_front_slowdown_shortens_forward_pulse_without_fictitious_lower_gear(calibrated):
    outputs = []
    for clearance in (.8, .30):
        follower = Follower(calibrated)
        observation = wall(front_clearance=clearance)
        assert follower.start(observation, observation.stamp_ns)
        outputs.append(follower.update(observation, observation.stamp_ns))
    assert all(e.direction == 1 and e.speed_percent == 25 for e in outputs)
    assert outputs[1].direction_until_ns < outputs[0].direction_until_ns


def test_direction_reversal_requires_stop_and_dwell(calibrated):
    f = Follower(calibrated)
    assert f.start(wall(distance=.8), 1_000_000_000)
    assert f.update(wall(distance=.8), 1_000_000_000).direction == 3
    for i in range(1, 8):
        ns = 1_000_000_000 + i * 50_000_000
        effort = f.update(wall(ns, distance=.8, angle=-.2), ns)
        assert effort.direction == (0 if i < 7 else 4)
        assert not effort.brush_enabled


@pytest.mark.parametrize('gear', [0, 12, 50, 75, True, 25.])
def test_wall_control_cannot_raise_or_invent_a_gear(calibrated, gear):
    with pytest.raises(ValueError):
        replace(calibrated, speed_percent=gear).validate()
