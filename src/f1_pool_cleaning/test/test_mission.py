from dataclasses import replace
import math
from pathlib import Path

import pytest
import yaml

from f1_pool_cleaning.perception_config import PerceptionConfig
from f1_pool_cleaning.perception import Perception, Observation
from f1_pool_cleaning.synthetic import pool_scan
from f1_pool_cleaning.geometry import PoolModel
from f1_pool_cleaning.demo import run_demo, sample, synthetic_config
from f1_pool_cleaning.mission import Mission, Feedback, Pose, command_fields, Effort

NOW = 1_000_000_000


def started(config=None):
    m = Mission(config or synthetic_config().validate())
    assert m.start(sample(m.c, NOW), NOW)[0]
    return m


def update(m, elapsed=0., **kwargs):
    now = NOW + round(elapsed * 1e9)
    return m.update(sample(m.c, now, **kwargs), now)


def edge_feedback(config, edge, stamp_ns, along, yaw=None):
    """Synthetic left-wall feedback for any canonical rectangle edge."""
    tangent_x, tangent_y = edge.tangent
    normal_x, normal_y = edge.left_normal
    x = edge.start[0] + along * tangent_x - config.wall_target_m * normal_x
    y = edge.start[1] + along * tangent_y - config.wall_target_m * normal_y
    yaw = edge.yaw_rad if yaw is None else yaw
    wall = Observation(stamp_ns, valid=True, distance=config.wall_target_m,
                       angle=0., support=1., inliers=40, ratio=.95)
    return Feedback(wall, Pose(stamp_ns, x, y, yaw))


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
                          brush_percent=30., thrust_enabled=False, thrust_percent=0.)
    assert e.direction_until_ns == 0


def test_side_distance_correction_is_bidirectional_and_has_no_history():
    m = started()
    for i, (distance, expected) in enumerate([
        (.36, 0), (.361, 30), (.36, 0), (.34, 0), (.32, 0),
        (.319, -30), (.32, 0), (.34, 0), (.36, 0), (.361, 30)]):
        e = update(m, i * .05, distance=distance)
        assert e.side_left_percent == expected
        fields = command_fields(e, m.c)
        assert fields["thrust_percent"] == expected
        assert e.brush_percent == 30 and e.direction == 0


@pytest.mark.parametrize("angle,direction", [(0, 0), (3, 0), (-3, 0), (4, 3), (-4, 4), (10, 3), (-10, 4),
                                            (10.01, 3), (-10.01, 4), (20, 3), (-20, 4)])
def test_heading_band_brush_gate_and_direction(angle, direction):
    m = started()
    e = update(m, angle=math.radians(angle))
    assert e.direction == direction
    assert e.brush_percent == (30 if abs(angle) <= 10 else 0)
    assert command_fields(e, m.c)["speed_percent"] == 25


@pytest.mark.parametrize("angle,direction", [(12, 3), (-12, 4)])
def test_continuous_heading_correction_has_no_pulses_or_dwell(angle, direction):
    m = started()
    angle = math.radians(angle)
    for i in range(30):
        e = update(m, i * .05, angle=angle, distance=.5)
        assert e.direction == direction and e.direction_until_ns == 0
        assert e.brush_percent == 0 and e.side_left_percent == 30
        assert e.valid_until_ns == e.computed_ns + 100_000_000
        assert m.phase == "CORRECT_HEADING"


def test_alignment_immediately_restores_brush_and_correction_reverses():
    m = started()
    assert update(m, angle=.21).direction == 3
    e = update(m, .04, angle=math.radians(10))
    assert e.brush_percent == 30 and e.direction == 3
    e = update(m, .08, angle=-.21)
    assert e.direction == 4 and e.brush_percent == 0
    assert update(m, .12, angle=.21).direction == 3


@pytest.mark.parametrize("angle,direction", [(.21, 3), (-.21, 4)])
def test_turn_space_and_hull_flags_do_not_block_start_or_correction(angle, direction):
    m = Mission(synthetic_config())
    f = sample(m.c, NOW, angle=angle)
    f = replace(f, wall=replace(f.wall, left_turn_clear=False, right_turn_clear=False,
                                rigid_collision=True))
    assert m.start(f, NOW)[0]
    e = m.update(f, NOW)
    assert e.direction == direction and e.brush_percent == 0
    assert m.phase == "CORRECT_HEADING"


@pytest.mark.parametrize("distance,side", [(.30, -30), (.8, 30)])
def test_old_wall_distance_envelope_does_not_block_brush(distance, side):
    m = started()
    e = update(m, distance=distance)
    assert e.brush_percent == 30 and e.side_left_percent == side


def test_map_stopping_with_blind_bow_and_no_local_velocity():
    m = started()
    assert update(m).brush_percent == 30  # front_observed=False is intentional.
    old_stop_zone = m.c.length_m - .70
    assert update(m, .03, along=old_stop_zone).brush_percent == 30
    assert m.phase == "FOLLOW"  # 70 cm centre distance is outside the stop zone.
    x = m.c.length_m - m.c.front_stop_m
    assert update(m, .05, along=x - .001).brush_percent == 30
    e = update(m, .10, along=x + 1e-9)
    assert m.phase == "STOPPED"
    assert (e.direction, e.brush_percent, e.side_left_percent) == (0, 0, 0)
    assert "drift unmeasured" in m.reason
    assert update(m, .15) is None  # no automatic restart or second side


def test_map_bow_clearance_accounts_for_heading_and_stops_on_pose_only():
    m = started()
    update(m)
    now = NOW + 50_000_000
    x = m.c.length_m - m.c.front_stop_m + .01
    f = sample(m.c, now, along=x, angle=.1)
    assert m.front_remaining(f.pose) < m.c.front_stop_m
    f = replace(f, wall=replace(f.wall, stamp_ns=NOW))
    assert m.update(f, now).brush_percent == 0
    assert m.phase == "STOPPED"


def test_rectangle_transfer_stops_at_thirty_cm_and_combines_reverse_right_push():
    c = synthetic_config().validate()
    pool = PoolModel.rectangle(c)
    m = Mission(c)
    assert m.configure_edge(pool, pool.edges[0])[0]
    now = NOW
    assert m.start(sample(c, now, along=.8), now)[0]

    now += 100_000_000
    endpoint = sample(c, now, along=pool.edges[0].length_m - c.front_stop_m + .01)
    effort = m.update(endpoint, now)
    assert m.phase == "EDGE_STOP"
    assert m.completed_edges == ["edge_0"]
    assert effort.direction == 0 and effort.brush_percent == 0 and effort.side_left_percent == 0

    now += 250_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                       wall=replace(endpoint.wall, stamp_ns=now))
    assert m.update(endpoint, now).direction == 0
    assert m.phase == "MOVE_INWARD"

    now += 100_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                       wall=replace(endpoint.wall, stamp_ns=now))
    effort = m.update(endpoint, now)
    assert m.phase == "MOVE_INWARD"
    assert effort.direction == 2
    assert effort.side_left_percent == -c.side_percent
    assert effort.brush_percent == 0
    fields = command_fields(effort, c)
    assert fields["direction"] == 2
    assert fields["thrust_enabled"] and fields["thrust_percent"] == -c.side_percent
    assert not fields["brush_enabled"]

    now += round((c.transfer_move_duration_sec + .05) * 1e9)
    tx, ty = pool.edges[0].tangent
    nx, ny = pool.edges[0].left_normal
    moved_pose = replace(endpoint.pose,
                         stamp_ns=now,
                         x=endpoint.pose.x - .20 * tx - .20 * nx,
                         y=endpoint.pose.y - .20 * ty - .20 * ny)
    endpoint = replace(endpoint, pose=moved_pose,
                       wall=replace(endpoint.wall, stamp_ns=now))
    assert m.update(endpoint, now).direction == 0
    assert m.phase == "TURN_TO_NEXT_EDGE"

    now += 100_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now,
                                               # A rough turn inside the new
                                               # +/-20 degree transfer band is
                                               # sufficient; wall reacquisition
                                               # handles the final alignment.
                                               yaw=pool.edges[1].yaw_rad + math.radians(15)),
                       wall=replace(endpoint.wall, stamp_ns=now))
    assert m.update(endpoint, now).direction == 0
    assert m.phase == "APPROACH_NEXT_EDGE"
    assert m.edge.id == "edge_1"

    now += 100_000_000
    next_feedback = edge_feedback(c, pool.edges[1], now, .5)
    effort = m.update(next_feedback, now)
    assert m.phase == "FOLLOW"
    assert effort.brush_percent == c.brush_percent


def test_transfer_does_not_pause_on_static_boundary_clearance():
    c = synthetic_config().validate()
    pool = PoolModel.rectangle(c)
    m = Mission(c)
    assert m.configure_edge(pool, pool.edges[0])[0]
    now = NOW
    assert m.start(sample(c, now, along=.8, angle=math.radians(40)), now)[0]

    now += 100_000_000
    endpoint = sample(c, now, along=pool.edges[0].length_m - c.front_stop_m + .01,
                      angle=math.radians(40))
    # This pose is intentionally outside the old static envelope check.  The
    # transfer sequence still proceeds so the vehicle can push inward first.
    assert m.pool.pose_boundary_clearance(
        endpoint.pose, c.hull_half_length_m, c.hull_half_width_m,
        c.brush_x_m, c.brush_tip_y_m) < 0.
    assert m.update(endpoint, now).direction == 0
    assert m.phase == "EDGE_STOP"

    now += 100_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                       wall=replace(endpoint.wall, stamp_ns=now))
    assert m.update(endpoint, now).direction == 0
    assert m.phase == "MOVE_INWARD"


def test_rectangle_transfer_turn_timeout_pauses_without_auto_resume():
    c = synthetic_config(transfer_turn_timeout_sec=.2).validate()
    pool = PoolModel.rectangle(c)
    m = Mission(c)
    m.configure_edge(pool, pool.edges[0])
    now = NOW
    m.start(sample(c, now, along=.8), now)
    endpoint = sample(c, now, along=pool.edges[0].length_m - c.front_stop_m + .01)
    m.update(endpoint, now)
    # Reach TURN_TO_NEXT_EDGE without waiting for a physical stop.
    for delta in (.10, .10, .55):
        now += round(delta * 1e9)
        endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                           wall=replace(endpoint.wall, stamp_ns=now))
        m.update(endpoint, now)
    assert m.phase == "TURN_TO_NEXT_EDGE"
    now += 250_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                       wall=replace(endpoint.wall, stamp_ns=now))
    assert m.update(endpoint, now) is None
    assert m.phase == "PAUSED"


def test_rectangle_transfer_default_has_no_overall_time_timeout():
    c = synthetic_config().validate()
    pool = PoolModel.rectangle(c)
    m = Mission(c)
    m.configure_edge(pool, pool.edges[0])
    now = NOW
    m.start(sample(c, now, along=.8), now)
    endpoint = sample(c, now, along=pool.edges[0].length_m - c.front_stop_m + .01)
    m.update(endpoint, now)
    for delta in (.10, .55):
        now += round(delta * 1e9)
        endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                           wall=replace(endpoint.wall, stamp_ns=now))
        m.update(endpoint, now)
    assert m.phase == "TURN_TO_NEXT_EDGE"

    # A zero transfer_timeout_sec keeps the turn feedback loop alive even
    # after the former eight-second overall watchdog would have fired.
    now += 8_500_000_000
    endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                       wall=replace(endpoint.wall, stamp_ns=now))
    effort = m.update(endpoint, now)
    assert m.phase == "TURN_TO_NEXT_EDGE"
    assert effort.direction == 4


def test_rectangle_route_completes_all_four_edges_in_clockwise_order():
    c = synthetic_config().validate()
    pool = PoolModel.rectangle(c)
    m = Mission(c)
    now = NOW
    assert m.configure_edge(pool, pool.edges[0])[0]
    assert m.start(edge_feedback(c, pool.edges[0], now, .8), now)[0]
    m.update(edge_feedback(c, pool.edges[0], now, .8), now)

    for index in range(4):
        edge = m.edge
        now += 100_000_000
        endpoint = edge_feedback(c, edge, now, edge.length_m - c.front_stop_m + .01)
        m.update(endpoint, now)
        for delta in (.25, .25, .10):
            now += round(delta * 1e9)
            endpoint = edge_feedback(c, edge, now, edge.length_m - c.front_stop_m + .01)
            m.update(endpoint, now)
        now += 550_000_000
        tx, ty = edge.tangent
        nx, ny = edge.left_normal
        moved = replace(endpoint.pose, stamp_ns=now,
                        x=endpoint.pose.x - .20 * tx - .20 * nx,
                        y=endpoint.pose.y - .20 * ty - .20 * ny)
        endpoint = replace(endpoint, pose=moved,
                           wall=replace(endpoint.wall, stamp_ns=now))
        m.update(endpoint, now)
        now += 250_000_000
        endpoint = replace(endpoint, pose=replace(endpoint.pose, stamp_ns=now),
                           wall=replace(endpoint.wall, stamp_ns=now))
        m.update(endpoint, now)
        if index < 3:
            now += 100_000_000
            turned = replace(endpoint,
                             pose=replace(endpoint.pose, stamp_ns=now,
                                           yaw=pool.edges[index + 1].yaw_rad),
                             wall=replace(endpoint.wall, stamp_ns=now))
            m.update(turned, now)
            now += 100_000_000
            m.update(edge_feedback(c, m.edge, now, .5), now)

    assert m.phase == "COMPLETE"
    assert m.completed_edges == ["edge_0", "edge_1", "edge_2", "edge_3"]


@pytest.mark.parametrize("clearance,stopped", [(0., True), (.049, True), (.05, True),
                                              (.051, False), (.2, False), (.8, False)])
def test_available_front_observation_uses_thirty_cm_stop_threshold(clearance, stopped):
    m = started()
    f = sample(m.c, NOW, front_observed=True)
    e = m.update(replace(f, wall=replace(f.wall, front_clearance=clearance)), NOW)
    assert e.direction == 0
    assert e.brush_percent == (0 if stopped else 30)
    assert m.phase == ("STOPPED" if stopped else "FOLLOW")


@pytest.mark.parametrize("angle", [0., .1, -.1])
def test_map_front_clearance_invariant_under_pool_half_turn(angle):
    m = started()
    f = sample(m.c, NOW, angle=angle)
    x, y = m.c.pool_xy(f.pose.x, f.pose.y)
    rx, ry = m.c.map_xy(m.c.length_m - x, m.c.width_m - y)
    flipped = replace(f.pose, x=rx, y=ry, yaw=f.pose.yaw + math.pi)
    assert m.front_remaining(flipped) == pytest.approx(m.front_remaining(f.pose))
    # The opposite heading is accepted without a special current-side config.
    other = Mission(synthetic_config())
    assert other.start(replace(f, pose=flipped), NOW)[0]
    assert other.update(replace(f, pose=flipped), NOW).brush_percent == 30


def test_negative_map_direction_stops_at_short_wall_ahead():
    m = Mission(synthetic_config())
    f = sample(m.c, NOW, along=1.5)
    f = replace(f, pose=replace(f.pose, yaw=m.c.axis_yaw_rad + math.pi))
    assert m.start(f, NOW)[0]
    x, y = m.c.map_xy(m.c.front_stop_m - 1e-9, .5)
    f = replace(f, pose=replace(f.pose, x=x, y=y))
    assert m.update(f, NOW).brush_percent == 0 and m.phase == "STOPPED"


def test_start_inside_front_stop_zone_never_enables_outputs():
    m = Mission(synthetic_config())
    x = m.c.length_m - m.c.front_stop_m
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


@pytest.mark.parametrize("changes", [dict(distance=0.),
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
    effort = Effort(NOW, NOW + 100_000_000, side_left_percent=30., brush_percent=30.)
    assert not command_fields(effort, synthetic_config(fifth_left_sign=0))["thrust_enabled"]
    for sign in (-1, 1):
        fields = command_fields(effort, synthetic_config(fifth_left_sign=sign))
        assert fields["direction"] == 0 and fields["thrust_percent"] == sign * 30
        assert fields["thrust_enabled"] and fields["brush_enabled"]


def test_map_wall_identity_and_second_quality_gate_removed():
    m = started()
    f = sample(m.c, NOW)
    x, y = m.c.map_xy(.8, .4)
    f = replace(f, pose=replace(f.pose, x=x, y=y),
                wall=replace(f.wall, support=.1, residual=.1, rigid_collision=True))
    assert m.error(f, NOW) == ""
    assert m.update(f, NOW).brush_percent == 30


@pytest.mark.parametrize("changes", [dict(fifth_left_sign=2), dict(wall_target_m=.32),
    dict(length_m=.5), dict(command_ttl_sec=.2), dict(side_percent=31.), dict(width_m=math.nan),
    dict(side_on_distance_m=.34), dict(brush_percent=101.), dict(turn_speed_percent=50),
    dict(heading_deadband_rad=math.pi), dict(heading_correction_deadband_rad=math.radians(11)),
    dict(run_timeout_sec=-1.), dict(wall_recovery_sec=0.)])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):
        replace(synthetic_config(), **changes).validate()


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
    m = started(synthetic_config(run_timeout_sec=.2))
    for i in range(5):
        update(m, i * .05, distance=.55)
    assert m.phase == "PAUSED" and m.last_effort is None


def test_default_run_has_no_arbitrary_timeout():
    m = started()
    assert update(m, elapsed=180.).brush_percent == 30 and m.phase == "FOLLOW"


def lost_wall(m, now, **kwargs):
    f = sample(m.c, now, **kwargs)
    return replace(f, wall=replace(f.wall, valid=False, distance=0., angle=0.,
                                  reason="left wall association lost"))


def test_invalid_wall_rejected_at_start_with_perception_reason():
    m = Mission(synthetic_config())
    ok, reason = m.start(lost_wall(m, NOW), NOW)
    assert not ok and "association lost" in reason and m.phase == "IDLE"


def test_post_acquisition_start_can_wait_for_fresh_wall_recovery():
    m = Mission(synthetic_config())
    f = lost_wall(m, NOW)
    assert m.start(f, NOW, allow_wall_recovery=True)[0]
    assert m.phase == "WAIT_WALL" and m.wall_lost_since_ns == NOW
    e = m.update(f, NOW)
    assert (e.direction, e.brush_percent, e.side_left_percent) == (0, 0, 0)
    assert update(m, .05, angle=math.radians(40)).direction == 3


def test_brief_wall_loss_neutralizes_all_outputs_then_recovers_heading_and_brush():
    m = started()
    assert update(m, angle=.3, distance=.5).side_left_percent == 30
    e = m.update(lost_wall(m, NOW + 50_000_000), NOW + 50_000_000)
    assert m.phase == "WAIT_WALL" and m.active
    assert (e.direction, e.brush_percent, e.side_left_percent) == (0, 0, 0)
    assert "association lost" in m.reason
    e = update(m, .10, angle=math.radians(40))
    assert m.phase == "CORRECT_HEADING" and e.direction == 3 and e.brush_percent == 0
    assert m.wall_lost_since_ns is None
    assert update(m, .15, angle=math.radians(8)).brush_percent == 30
    m.update(lost_wall(m, NOW + 200_000_000), NOW + 200_000_000)
    assert m.wall_lost_since_ns == NOW + 200_000_000


def test_persistent_invalid_fresh_scans_pause_after_bounded_recovery():
    m = started(synthetic_config(wall_recovery_sec=.2))
    for i in range(4):
        now = NOW + i * 50_000_000
        e = m.update(lost_wall(m, now), now)
        assert m.phase == "WAIT_WALL" and e.direction == 0 and e.brush_percent == 0
    now = NOW + 200_000_000
    assert m.update(lost_wall(m, now), now) is None and m.phase == "PAUSED"
    assert "lost for 0.20s" in m.reason and "association lost" in m.reason
    assert update(m, .25) is None


def test_repeated_invalid_scan_cannot_renew_neutral_command_or_lease():
    m = started()
    f = lost_wall(m, NOW)
    e = m.update(f, NOW)
    assert m.update(f, NOW + 50_000_000) is None and m.last_effort == e
    assert m.update(f, NOW + 100_000_000) is None and m.phase == "PAUSED"
    assert "command deadline" in m.reason


def test_front_endpoint_still_stops_while_waiting_for_wall_on_pose_only():
    m = started()
    m.update(lost_wall(m, NOW), NOW)
    now = NOW + 50_000_000
    x = m.c.length_m - m.c.front_stop_m + 1e-9
    f = lost_wall(m, now, along=x)
    f = replace(f, wall=replace(f.wall, stamp_ns=NOW))
    e = m.update(f, now)
    assert m.phase == "STOPPED" and (e.direction, e.brush_percent, e.side_left_percent) == (0, 0, 0)


@pytest.mark.parametrize("phase", ["PAUSED", "CANCELLED"])
def test_manual_stop_during_wall_recovery_never_auto_resumes(phase):
    m = started()
    m.update(lost_wall(m, NOW), NOW)
    m.halt("user stop", phase)
    assert update(m, .05) is None and m.phase == phase


@pytest.mark.parametrize("field", ["wall", "pose"])
def test_wall_recovery_does_not_bypass_stale_feedback(field):
    m = started()
    m.update(lost_wall(m, NOW), NOW)
    now = NOW + 50_000_000
    f = lost_wall(m, now)
    f = replace(f, **{field: replace(getattr(f, field), stamp_ns=now - 300_000_000)})
    assert m.update(f, now) is None and m.phase == "PAUSED"


@pytest.mark.parametrize("sign", [-1, 1])
def test_actual_perception_profile_tracks_large_yaw_and_return_to_brush_band(sign):
    parameters = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/perception.yaml').read_text())
    c = PerceptionConfig(**parameters['pool_perception']['ros__parameters']).validate()
    estimator, m = Perception(c), Mission(synthetic_config())
    # Includes >11.5 degree changes and angles beyond the previous 34.4 degree cap.
    for i, degrees in enumerate([0, 20, 40, 60, 68, 40, 12, 8]):
        now = NOW + i * 50_000_000
        angle = math.radians(degrees * sign)
        full = pool_scan(now, wall_distance=.4, wall_angle=angle, config=c)
        scan = replace(full, angle_min=math.radians(-140), ranges=full.ranges[160:-160])
        wall = estimator.process(scan, now)
        assert wall.valid, (degrees * sign, wall.reason)
        assert wall.angle == pytest.approx(angle, abs=.01)
        f = replace(sample(m.c, now, angle=angle), wall=wall)
        if i == 0:
            assert m.start(f, now)[0]
        e = m.update(f, now)
        assert e.direction == (0 if degrees <= 3 else 3 if sign > 0 else 4)
        assert e.brush_percent == (30 if degrees <= 10 else 0)
