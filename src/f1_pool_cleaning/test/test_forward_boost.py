"""Optional forward assistance uses the centre's remaining along-wall travel."""
from dataclasses import replace
import pytest

from f1_pool_cleaning.config import Config
from f1_pool_cleaning.geometry import PoolModel
from f1_pool_cleaning.mission import Mission, Pose, Feedback, command_fields
from f1_pool_cleaning.perception import Observation
from f1_pool_cleaning.autonomous import AutonomousMission

NOW = 1_000_000_000


def feedback(m, remaining, now=NOW, angle=0., distance=.33):
    edge = m.edge
    if edge is None:
        x, y, yaw = m.c.length_m-remaining, m.c.width_m-distance, -angle
    else:
        along = edge.length_m-remaining
        x, y = (edge.start[i]+along*edge.tangent[i]-distance*edge.left_normal[i]
                for i in (0, 1))
        yaw = edge.yaw_rad-angle
    return Feedback(Observation(now, valid=True, distance=distance, angle=angle), Pose(now, x, y, yaw))


@pytest.fixture(params=["legacy", "rect0", "rect1", "rect2", "rect3", "open0", "open1", "open2"])
def mission(request):
    c = Config(origin_x_m=0., origin_y_m=0., axis_yaw_rad=0., length_m=4., width_m=3.)
    m = Mission(c)
    if request.param != "legacy":
        pool = (PoolModel.rectangle(c) if request.param.startswith("rect") else
                PoolModel.from_polyline([(0., 0.), (4., 0.), (4., -3.), (7., -3.)]))
        assert m.configure_edge(pool, pool.edges[int(request.param[-1])])[0]
    assert m.set_forward_boost(True)[0]
    assert m.start(feedback(m, 1.5), NOW)[0]
    return m


def test_boost_cutoff_latches_but_heading_correction_remains(mission):
    m = mission
    for i, (remaining, angle, expected) in enumerate([
            (1.5, 0., 1), (1.3, .07, 3), (1.1, 0., 1), (.801, 0., 1),
            (.8, 0., 0), (.82, 0., 0), (.7, -.07, 4), (.6, 0., 0)]):
        now = NOW+i*50_000_000
        e = m.update(feedback(m, remaining, now, angle), now)
        assert e.direction == expected
        assert e.brush_percent == 50.
        assert e.forward_boost is (expected == 1)
        assert command_fields(e, m.c)["speed_percent"] == 25
        assert command_fields(e, m.c)["mode"] == 1
    assert m.forward_boost_cutoff_reached


def test_cutoff_revokes_on_pose_without_new_scan_and_never_extends_deadline(mission):
    m = mission
    original = feedback(m, .82)
    first = m.update(original, NOW)
    assert first.forward_boost
    now = NOW+20_000_000
    updated = replace(feedback(m, .8, now), wall=original.wall)
    revoked = m.update(updated, now)
    assert revoked.direction == 0 and revoked.brush_percent == 50.
    assert not revoked.forward_boost
    assert revoked.valid_until_ns == first.valid_until_ns
    assert revoked.computed_ns == first.computed_ns
    assert m.update(replace(updated, pose=replace(updated.pose, stamp_ns=now+1)), now+1) is None
    expired = first.valid_until_ns
    m.update(replace(updated, pose=replace(updated.pose, stamp_ns=expired)), expired)
    assert m.phase == "PAUSED" and m.last_effort is None


@pytest.mark.parametrize("distance,angle", [(.30, 0.), (.36, 0.), (.33, .20)])
def test_unstable_wall_or_heading_never_boosts(mission, distance, angle):
    e = mission.update(feedback(mission, 1.5, distance=distance, angle=angle), NOW)
    assert not e.forward_boost and e.direction != 1


def test_invalid_feedback_and_stop_do_not_keep_boost(mission):
    m = mission
    assert m.update(feedback(m, 1.5), NOW).forward_boost
    now = NOW+50_000_000
    f = feedback(m, 1.4, now)
    e = m.update(replace(f, wall=replace(f.wall, valid=False)), now)
    assert e.direction == 0 and not e.forward_boost and e.brush_percent == 0
    m.halt("stop", "STOPPED")
    assert not m.forward_boost_status(now)["forward_boost_active"]


@pytest.mark.parametrize("phase", ["FOLLOW", "CORRECT_HEADING", "ACQUIRING", "PAUSED",
                                   "APPROACH_START", "WAIT_WALL", "LEFT_EXIT_WALL"])
def test_setting_locked_after_start_including_pause(phase):
    m = Mission(Config(0., 0., 0., 4., 3.))
    assert m.forward_boost_enabled is False
    assert m.set_forward_boost(True)[0]
    m.phase = phase
    assert not m.set_forward_boost(False)[0]
    assert m.forward_boost_enabled
    m.halt("cancel", "CANCELLED")
    assert m.set_forward_boost(False)[0]


def test_new_wall_resets_cutoff_and_short_wall_never_boosts():
    c = Config(0., 0., 0., 4., 3.)
    pool = PoolModel.from_polyline([(0., 0.), (4., 0.), (4., -3.), (4.75, -3.)])
    m = Mission(c)
    m.configure_edge(pool, pool.edges[0])
    m.set_forward_boost(True)
    m.start(feedback(m, .8), NOW)
    assert not m.update(feedback(m, .8), NOW).forward_boost
    assert m.forward_boost_cutoff_reached
    for index, remaining, expected in [(1, 1.2, True), (2, .4, False)]:
        now = NOW+index*50_000_000
        m.transfer_next_edge = pool.edges[index]
        m._finish_transfer_to_next_edge(Pose(now, 0., 0., 0.), now)
        m.last_edge_s = None
        assert not m.forward_boost_cutoff_reached
        assert m.update(feedback(m, remaining, now), now).forward_boost is expected


def test_rectangle_prefix_uses_actual_completion_gate_instead_of_wall_end():
    c = Config(0., 0., 0., 4., 3.)
    m = AutonomousMission(c, PoolModel.rectangle(c))
    m.edge = m.pool.edges[0]
    m.phase = "FOLLOW"
    m.forward_boost_enabled = True
    m.returning_to_start, m.prefix_end = True, 2.2
    # Prefix completion is triggered by brush contact >= 2.2 - .025.
    # At 0 heading the centre therefore stops at s = 2.075, far before the wall end.
    stop_s = m.prefix_end-m.a.coverage_tolerance_m/2-c.brush_x_m
    for i, (distance, expected) in enumerate([(.81, True), (.80, False), (.82, False)]):
        now = NOW+i*20_000_000
        f = feedback(m, m.edge.length_m-(stop_s-distance), now)
        assert m.boost_remaining(f) == pytest.approx(distance)
        e = m._update_edge(f, now)
        assert e.forward_boost is expected
        assert m.edge.length_m-m.edge.project(f.pose).s_m > 2.
