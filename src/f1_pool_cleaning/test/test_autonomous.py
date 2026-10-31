"""Pose-controlled acquisition and legacy wall control, without ROS/hardware."""
from dataclasses import replace
import math

import pytest

from f1_pool_cleaning.autonomous import AutonomousMission, ApproachConfig
from f1_pool_cleaning.config import Config
from f1_pool_cleaning.geometry import PoolModel, wrap
from f1_pool_cleaning.mission import Feedback, Pose
from f1_pool_cleaning.perception import Observation


def system(closed=True):
    c = Config(origin_x_m=0., origin_y_m=0., axis_yaw_rad=0., length_m=4., width_m=2.)
    pool = (PoolModel.from_boundary([(0., 2.), (4., 2.), (4., 0.), (0., 0.)]) if closed else
            PoolModel.from_polyline([(4., 0.), (0., 0.), (0., 4.)]))
    return AutonomousMission(c, pool)


def feedback(mission, p, now, valid=True):
    pose = replace(p, stamp_ns=now)
    if mission.edge:
        projection = mission.edge.project(pose)
        wall = Observation(now, valid=valid and mission.target_wall_enabled,
                           distance=projection.lateral_distance_m, angle=projection.heading_error_rad)
    else:
        wall = None
    return Feedback(wall, pose)


def run_boat(mission, initial, maximum=8000):
    """Independent kinematics execute actual outputs, not target teleportation."""
    pose, now, dt = initial, 1_000_000_000, .05
    assert mission.start(feedback(mission, pose, now), now)[0]
    phases, visits, motions = set(), [], []
    for _ in range(maximum):
        now += round(dt*1e9)
        f = feedback(mission, pose, now)
        before = mission.phase
        effort = mission.update(f, now)
        phases.add(before)
        if mission.edge and (not visits or mission.edge.id != visits[-1]):
            visits.append(mission.edge.id)
        if effort:
            motions.append((before, effort.direction, effort.side_left_percent, effort.brush_percent))
            if before in mission.APPROACH_PHASES + ('ACQUIRE_WALL',):
                assert effort.brush_percent == 0
            forward = .25 if effort.direction == 1 else -.25 if effort.direction == 2 else 0.
            forward += .20 if effort.brush_percent else 0.
            side = .20*effort.side_left_percent/mission.c.side_percent
            turn = .40 if effort.direction == 3 else -.40 if effort.direction == 4 else 0.
            pose = Pose(now, pose.x+dt*(math.cos(pose.yaw)*forward-math.sin(pose.yaw)*side),
                        pose.y+dt*(math.sin(pose.yaw)*forward+math.cos(pose.yaw)*side),
                        wrap(pose.yaw+dt*turn))
        if not mission.active:
            break
    assert mission.phase == 'COMPLETE', (mission.phase, mission.reason, pose, mission.status())
    assert mission.last_effort.direction == 0 and mission.last_effort.brush_percent == 0
    return phases, visits, motions, pose


@pytest.mark.parametrize('yaw, expected', [(0., 'edge_0'), (math.pi/2, 'edge_3'),
                                           (math.pi, 'edge_2'), (-math.pi/2, 'edge_1')])
def test_ray_selects_first_wall_in_body_left_direction(yaw, expected):
    m = system(); p = Pose(1, 2., 1., yaw)
    assert m.pool.select_left_ray(p).id == expected


def test_ray_selects_hit_wall_not_closest_wall_or_any_left_wall():
    m = system()
    # Right wall is much closer, but the +y ray hits the top wall.
    assert m.pool.select_left_ray(Pose(1, 3.8, 1., 0.)).id == 'edge_0'
    # Both top and west are left; ray hits west even though top is closer.
    assert m.pool.select_left_ray(Pose(1, .3, 1.8, math.pi/3)).id == 'edge_3'


@pytest.mark.parametrize('near_corner', [0., .005])
def test_corner_chooses_directed_wall_with_smaller_heading_angle(near_corner):
    m = system()
    p = Pose(1, 2., 1., math.atan2(2.-near_corner, 1.))
    assert m.pool.select_left_ray(p).id == 'edge_3'


def test_equal_angle_corner_tie_is_deterministic():
    pool = PoolModel.from_boundary([(0., 4.), (4., 4.), (4., 0.), (0., 0.)])
    p = Pose(1, 2., 2., math.pi/4)
    assert pool.select_left_ray(p).id == 'edge_0'


def test_open_l_no_hit_does_not_extend_walls_or_fall_back():
    m = system(False); now = 1_000_000_000
    ok, reason = m.start(Feedback(None, Pose(now, 2., 2., 0.)), now)
    assert not ok and 'left ray' in reason and not m.active


def test_nearest_point_is_center_projection_not_ray_intersection():
    m = system(); p = Pose(1_000_000_000, 2., 1., .3)
    assert m.start(Feedback(None, p), p.stamp_ns)[0]
    assert m.start_edge_id == 'edge_0'
    assert m.nearest_wall_point == pytest.approx([2., 2.])
    assert m.start_s == pytest.approx(2.)
    old = m.start_region()
    m.update(feedback(m, replace(p, x=2.2, yaw=0.), p.stamp_ns+50_000_000), p.stamp_ns+50_000_000)
    assert m.start_region() == old


def test_end_projection_is_clamped_before_legacy_front_stop():
    m = system(); m.requested_edge_id = 'edge_0'
    p = Pose(1_000_000_000, 4., 1.5, 0.)
    assert m.start(Feedback(None, p), p.stamp_ns)[0]
    assert m.nearest_wall_point == [4., 2.]
    assert m.start_s+m.a.along_tolerance_m < 4.-m.c.front_stop_m


def test_already_near_wall_hands_off_without_leaving_or_requiring_raw_scan():
    m = system(); now = 1_000_000_000; p = Pose(now, 2., 1.651, 0.)
    assert m.start(Feedback(None, p), now)[0]
    for _ in range(10):
        now += 50_000_000
        e = m.update(feedback(m, p, now), now)
        if m.phase == 'APPROACH_WALL':
            break
        assert e is not None and (e.direction, e.side_left_percent, e.brush_percent) == (0, 0., 0.)
    assert m.phase == 'APPROACH_WALL'
    now += 50_000_000
    assert m.update(feedback(m, p, now), now).brush_percent == 30.


def test_direct_motion_ignores_raw_laser_obstacles_and_has_no_planner():
    m = system(); now = 1_000_000_000; p = Pose(now, 2., .5, 0.)
    assert m.start(Feedback(None, p, object()), now)[0]
    for _ in range(2):
        now += 50_000_000
        e = m.update(Feedback(None, replace(p, stamp_ns=now), object()), now)
    assert e.side_left_percent == 30. and not e.brush_percent
    assert not hasattr(m, 'navigator') and not hasattr(m, 'executor')


def test_approach_reversal_inserts_stop_before_opposite_motion():
    m = system(); now = 1_000_000_000; p = Pose(now, 2., 1., 0.)
    assert m.start(Feedback(None, p), now)[0]
    for _ in range(2):
        now += 50_000_000
        e = m.update(feedback(m, replace(p, x=1.8), now), now)
    assert e.direction == 1
    now += 50_000_000
    e = m.update(feedback(m, replace(p, x=2.11), now), now)
    assert e.direction == 0 and e.side_left_percent == 0
    now += 50_000_000
    e = m.update(feedback(m, replace(p, x=2.11), now), now)
    assert e.direction == 2


@pytest.mark.parametrize('start', [Pose(1, 2., .5, 0.), Pose(1, 3.8, 1.5, -math.pi/2),
                                   Pose(1, 1.2, 1.1, .6)])
def test_rectangle_finishes_one_loop_and_only_the_starting_prefix(start):
    m = system()
    phases, visits, motions, pose = run_boat(m, start)
    first = int(m.start_edge_id.split('_')[1])
    assert visits == [f'edge_{(first+i)%4}' for i in range(5)]
    assert {'ACQUIRE_WALL', 'EDGE_STOP', 'MOVE_INWARD', 'TURN_TO_NEXT_EDGE', 'FOLLOW'} <= phases
    assert not m.remaining_edges and m.returning_to_start
    assert m.first_contact[0] == m.start_edge_id
    assert m.prefix_end == pytest.approx(min(m.required[m.start_edge_id][1], m.first_contact[1]+.10))
    assert m.status()['completion_scope'] == 'rectangle_route'
    assert len(m.finished_edges) == 5


@pytest.mark.parametrize('p, expected', [(Pose(1, 2., 1., math.pi), ['edge_0', 'edge_1']),
                                        (Pose(1, 1., 2., math.pi/2), ['edge_1'])])
def test_l_only_cleans_remaining_route_and_stops_at_c(p, expected):
    m = system(False)
    _, visits, _, end = run_boat(m, p)
    assert visits == expected
    assert set(m.required) == set(expected)
    assert not m.remaining_edges and m.prefix_end is None
    assert not m.returning_to_start
    assert end.y >= 4.-m.c.front_stop_m
    assert m.status()['completion_scope'] == 'remaining_l_route'


def test_l_does_not_claim_unvisited_wall_coverage():
    m = system(False)
    run_boat(m, Pose(1, 1., 2., math.pi/2))
    assert 'edge_0' not in m.required and 'edge_0' not in m.completed_edges
    assert m.required['edge_1'][0] >= 2.


def test_rectangle_started_at_effective_beginning_does_not_add_prefix_lap():
    m = system()
    _, visits, _, _ = run_boat(m, Pose(1, .21, 1.66, 0.))
    assert visits == ['edge_0', 'edge_1', 'edge_2', 'edge_3']
    assert m.prefix_end is None and not m.returning_to_start


@pytest.mark.parametrize('phase', ['ALIGN_START', 'MOVE_START_ALONG', 'MOVE_START_SIDE', 'ACQUIRE_WALL'])
def test_pause_cancel_and_stale_pose_never_continue_motion(phase):
    m = system(); now = 1_000_000_000; p = Pose(now, 2., .5, .5)
    assert m.start(Feedback(None, p), now)[0]
    m.phase = phase
    assert m.update(Feedback(None, p), now+300_000_000) is None
    assert m.phase == 'PAUSED' and m.last_effort is None
    assert not m.start(feedback(m, p, now+400_000_000), now+400_000_000)[0]
    m.halt('cancelled', 'CANCELLED')
    assert m.update(feedback(m, p, now+450_000_000), now+450_000_000) is None
    assert m.start(feedback(m, p, now+500_000_000), now+500_000_000)[0]


def test_wall_acquisition_rejects_cached_wall_and_times_out_without_motion():
    m = system(); now = 1_000_000_000; p = Pose(now, 2., 1.66, 0.)
    assert m.start(Feedback(None, p), now)[0]
    for _ in range(6):
        now += 50_000_000; m.update(feedback(m, p, now), now)
        if m.phase == 'ACQUIRE_WALL':
            break
    assert m.phase == 'ACQUIRE_WALL'
    old = Observation(m.wall_after_ns, valid=True, distance=.34, angle=0.)
    for _ in range(32):
        now += 50_000_000
        e = m.update(Feedback(old, replace(p, stamp_ns=now)), now)
        assert e is None or not (e.direction or e.side_left_percent or e.brush_percent)
    assert m.phase == 'PAUSED' and 'acquisition' in m.reason


def test_explicit_selection_and_lease_resume_keep_same_wall_and_reference():
    m = system(); m.requested_edge_id = 'edge_2'; now = 1_000_000_000
    assert m.start(Feedback(None, Pose(now, 2., 1., 0.)), now)[0]
    reference = m.start_region()
    m.halt('waiting for lease', 'ACQUIRING')
    m.halt('lease acquired', 'IDLE')
    assert m.resume_after_acquire(Feedback(None, Pose(now+10, 2.2, 1., math.pi/2)), now+10)[0]
    assert m.start_edge_id == 'edge_2' and m.start_region() == reference


def test_coverage_gaps_are_reported_without_triggering_repair_navigation():
    m = system(); run_boat(m, Pose(1, 2., .5, 0.))
    m.cleaned_by_edge = {}
    status = m.status()
    assert not status['coverage_complete'] and any(status['remaining_contact_intervals'].values())
    assert m.phase == 'COMPLETE' and not m.remaining_edges


def test_invalid_approach_parameters_are_rejected():
    with pytest.raises(ValueError):
        AutonomousMission(system().c, system().pool, ApproachConfig(along_tolerance_m=float('nan')))
