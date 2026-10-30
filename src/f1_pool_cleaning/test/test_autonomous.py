"""Map-guided acquisition and coverage, with no ROS or hardware dependency."""
from dataclasses import replace
import math

import numpy as np
import pytest

from f1_pool_cleaning.autonomous import AutonomousMission
from f1_pool_cleaning.config import Config
from f1_pool_cleaning.coverage import initial_legs, merge_intervals, missing_intervals
from f1_pool_cleaning.geometry import PoolModel, wrap
from f1_pool_cleaning.map_profile import load_occupancy, require
from f1_pool_cleaning.mission import Feedback, Pose, Effort
from f1_pool_cleaning.navigation import NavigationConfig, NavigationScan, Navigator, PlanningError, Waypoint
from f1_pool_cleaning.perception import Observation
from f1_pool_cleaning.synthetic_map import rectangle_map, l_map


def system(path, kind):
    values = require(path, kind).overrides()
    boundary = values.pop("boundary_xy", None)
    route = values.pop("route_xy", None)
    config = Config(**values).validate()
    if boundary:
        pool = PoolModel.from_boundary(list(zip(boundary[::2], boundary[1::2])))
    else:
        pool = PoolModel.from_polyline(list(zip(route[::2], route[1::2])))
    return AutonomousMission(config, pool, Navigator(load_occupancy(path), config))


@pytest.fixture
def rectangle(tmp_path):
    mission = system(rectangle_map(tmp_path), "rectangle")
    yield mission
    mission.close()


@pytest.fixture
def open_l(tmp_path):
    mission = system(l_map(tmp_path), "route")
    yield mission
    mission.close()


def feedback(mission, pose, stamp, *, valid=True, scan_points=()):
    pose = Pose(stamp, pose.x, pose.y, pose.yaw)
    if mission.edge is not None:
        p = mission.edge.project(pose)
        wall = Observation(stamp, valid=valid, distance=p.lateral_distance_m,
                           angle=p.heading_error_rad)
    else:
        wall = Observation(stamp, valid=False, reason="no left wall yet")
    return Feedback(wall, pose, NavigationScan(stamp, scan_points))


def run_boat(mission, initial, maximum=16000):
    """Execute discrete outputs through a simple independent kinematic plant.

    This exercises state/geometry integration, not hydrodynamics or stopping
    distances. The map is still checked by the production swept-footprint code.
    """
    pose, now, dt = initial, 1_000_000_000, .05
    assert mission.start(feedback(mission, pose, now), now)[0]
    phases, visited, brush = set(), [], []
    for _ in range(maximum):
        now += round(dt*1e9)
        f = feedback(mission, pose, now, valid=mission.target_wall_enabled)
        phase = mission.phase
        effort = mission.update(f, now)
        phases.add(phase)
        if mission.future is not None:
            mission.future.result(timeout=35)
        if mission.leg and (not visited or mission.leg != visited[-1]):
            visited.append(mission.leg)
        if effort is not None:
            if phase in mission.NAV_PHASES or mission.phase in mission.NAV_PHASES:
                assert effort.brush_percent == 0
            forward = (.25 if effort.direction == 1 else -.25 if effort.direction == 2 else 0.)
            forward += .20 if effort.brush_percent else 0.
            side = .20*effort.side_left_percent/mission.c.side_percent
            turn = .40 if effort.direction == 3 else -.40 if effort.direction == 4 else 0.
            if effort.brush_percent:
                brush.append((mission.edge.id, mission.edge.project(f.pose).contact_s_m))
            pose = Waypoint(pose.x+dt*(math.cos(pose.yaw)*forward-math.sin(pose.yaw)*side),
                            pose.y+dt*(math.sin(pose.yaw)*forward+math.cos(pose.yaw)*side),
                            wrap(pose.yaw+dt*turn))
        if not mission.active:
            break
    assert mission.phase == "COMPLETE", (mission.phase, mission.reason, mission.status(), pose)
    assert all(not gaps for gaps in mission.status()["remaining_contact_intervals"].values())
    assert set(mission.completed_edges) == set(mission.pool.by_id)
    return phases, visited, brush


def test_interval_union_does_not_hide_a_real_gap():
    assert merge_intervals([(1,2),(.3,.8),(.5,1.5)]) == [(.3,2)]
    assert missing_intervals((.3,3),[(.3,1),(1.2,3)],.05) == [(1,1.2)]


def test_rectangle_schedule_wraps_and_returns_to_starting_prefix(rectangle):
    required = {e.id:(.4,e.length_m-.3) for e in rectangle.pool.edges}
    legs = initial_legs(rectangle.pool.edges,required,"edge_2",1.2,.1)
    assert [leg.edge_id for leg in legs] == ["edge_2","edge_3","edge_0","edge_1","edge_2"]
    assert legs[0].start == 1.2 and legs[-1].start == .4
    assert legs[-1].end == pytest.approx(1.3)


@pytest.mark.parametrize("yaw, expected", [
    (0., "edge_0"), (math.pi/2, "edge_3"),
    (math.pi, "edge_2"), (-math.pi/2, "edge_1"),
])
def test_start_prefers_current_left_wall_over_nearer_non_left_walls(rectangle, yaw, expected):
    pose = Waypoint(2.8,1.45,yaw)
    required, legs, path = rectangle._initial_plan(pose,None)
    assert legs[0].edge_id == expected
    assert path[-1].yaw == pytest.approx(rectangle.pool.by_id[expected].yaw_rad)
    previous = pose
    for target in path:
        assert rectangle.navigator.sweep(previous,target,target.contact,
                                          rectangle.pool.by_id.get(target.contact_edge_id))
        previous = target


@pytest.mark.parametrize("edge_id", ["edge_0", "edge_2"])
def test_explicit_edge_overrides_distance_and_left_preference(rectangle, edge_id):
    _, legs, _ = rectangle._initial_plan(Waypoint(2.8,1.45,0.), edge_id)
    assert legs[0].edge_id == edge_id


def test_unreachable_left_wall_falls_back_to_nearest_other_wall(rectangle, monkeypatch):
    plan = rectangle.navigator.plan
    attempted = []
    def block_left(start, goal):
        attempted.append(goal.yaw)
        if goal.yaw == rectangle.pool.by_id["edge_0"].yaw_rad:
            raise PlanningError("left approach blocked")
        return plan(start, goal)
    monkeypatch.setattr(rectangle.navigator, "plan", block_left)
    _, legs, _ = rectangle._initial_plan(Waypoint(2.8,1.45,0.), None)
    assert legs[0].edge_id == "edge_2"
    assert attempted == [rectangle.pool.by_id[e].yaw_rad for e in ("edge_0", "edge_2")]


@pytest.mark.parametrize("blocked, expected_order", [
    ((), ["edge_0"]),
    (("edge_0",), ["edge_0", "edge_3"]),
    (("edge_0", "edge_3"), ["edge_0", "edge_3", "edge_2"]),
])
def test_all_left_candidates_are_tried_by_distance_before_non_left(
        rectangle, monkeypatch, blocked, expected_order):
    attempted = []
    def plan(start, goal):
        edge_id = next(e.id for e in rectangle.pool.edges if e.yaw_rad == goal.yaw)
        attempted.append(edge_id)
        if edge_id in blocked:
            raise PlanningError("blocked approach")
        return [goal]
    monkeypatch.setattr(rectangle.navigator, "plan", plan)
    _, legs, _ = rectangle._initial_plan(Waypoint(2.8,1.45,math.pi/4), None)
    assert attempted == expected_order
    assert legs[0].edge_id == expected_order[-1]


def test_equal_distance_left_walls_prefer_smaller_heading_change(rectangle, monkeypatch):
    rectangle.pool = PoolModel.from_boundary([(0.,4.), (4.,4.), (4.,0.), (0.,0.)])
    monkeypatch.setattr(rectangle.navigator, "usable_interval", lambda e: (.3, e.length_m-.3))
    monkeypatch.setattr(rectangle.navigator, "plan", lambda start, goal: [goal])
    # Top and west walls are equally distant and both left; west is aligned
    # more closely despite its larger edge id.
    _, legs, _ = rectangle._initial_plan(Waypoint(2.,2.,math.pi/3), None)
    assert legs[0].edge_id == "edge_3"


def test_no_left_wall_uses_nearest_remaining_wall(open_l):
    pose = Waypoint(-11.15,7.,0.)
    assert all(e.project(pose).left_offset_m <= 1e-6 for e in open_l.pool.edges)
    _, legs, _ = open_l._initial_plan(pose, None)
    assert legs[0].edge_id == "edge_1"


def test_unreachable_explicit_edge_does_not_fall_back(rectangle, monkeypatch):
    attempted = []
    def fail(start, goal):
        attempted.append(goal.yaw)
        raise PlanningError("specified approach blocked")
    monkeypatch.setattr(rectangle.navigator, "plan", fail)
    with pytest.raises(PlanningError, match="no reachable task wall"):
        rectangle._initial_plan(Waypoint(2.8,1.45,0.), "edge_2")
    assert attempted == [rectangle.pool.by_id["edge_2"].yaw_rad]


def test_unknown_and_hull_collision_are_not_accepted_as_free_water(rectangle):
    nav = rectangle.navigator
    assert not nav.free(Waypoint(.5,.5,0.))
    assert not nav.free(Waypoint(1.05,1.5,0.))
    with pytest.raises(PlanningError):
        nav.plan(Waypoint(.5,.5,0.),Waypoint(2.8,1.7,0.))


def test_live_obstacle_stops_a_map_clear_motion(rectangle):
    pose = Waypoint(2.8,1.7,0.)
    target = Waypoint(2.95,1.7,0.)
    nav = rectangle.navigator
    assert nav.sweep(pose,target)
    assert not nav.motion_clear(pose,target,NavigationScan(1,((.35,0.),)))
    assert nav.motion_clear(pose,target,NavigationScan(1,((1.,0.),)))


def test_l_return_path_stays_in_water_and_never_closes_a_wall(open_l):
    nav = open_l.navigator
    start = Waypoint(-11.2,10.5,math.pi/2)
    goal = Waypoint(-1.2,.85,math.pi)
    path = nav.plan(start,goal)
    assert not nav.sweep(start,goal)
    for a,b in zip([start]+path,path):
        assert nav.sweep(a,b)
    assert len(open_l.pool.edges) == 2


def test_rectangle_completes_all_edges_and_starting_prefix(rectangle):
    phases, legs, brush = run_boat(rectangle,Waypoint(2.8,1.45,0.))
    assert [leg.edge_id for leg in legs[:5]] == ["edge_0","edge_1","edge_2","edge_3","edge_0"]
    assert {"PLANNING","NAVIGATING","SETTLING","ACQUIRE_WALL","FOLLOW"} <= phases
    assert {"EDGE_STOP", "MOVE_INWARD", "TURN_TO_NEXT_EDGE", "APPROACH_NEXT_EDGE"} <= phases
    assert brush[0][0] == "edge_0"


def test_l_can_start_on_second_leg_and_complete_both(open_l):
    phases, legs, brush = run_boat(open_l,Waypoint(-11.15,7.,0.))
    assert [leg.edge_id for leg in legs[:3]] == ["edge_1","edge_0","edge_1"]
    assert brush[0][0] == "edge_1"


def test_stale_raw_scan_cannot_start_without_a_wall(rectangle):
    now=1_000_000_000
    f=feedback(rectangle,Waypoint(2.8,1.7,0.),now)
    assert not rectangle.start(replace(f,scan=None),now)[0]
    assert rectangle.start(f,now)[0]
    assert rectangle.update(replace(f,pose=replace(f.pose,stamp_ns=now+400_000_000)),now+400_000_000) is None
    assert rectangle.phase == "PAUSED"


def test_cancelled_search_result_never_restarts_motion(rectangle):
    now=1_000_000_000
    f=feedback(rectangle,Waypoint(2.8,1.7,0.),now)
    assert rectangle.start(f,now)[0]
    rectangle.update(f,now)
    future=rectangle.future
    rectangle.halt("cancelled","CANCELLED")
    if not future.cancelled():
        future.result(timeout=30)
    assert rectangle.update(f,now+1) is None
    assert rectangle.phase == "CANCELLED" and rectangle.future is None


def test_turns_keep_existing_commands_without_new_collision_gate(rectangle, monkeypatch):
    now = 1_000_000_000
    f = feedback(rectangle, Waypoint(2.8, 1.7, 0.), now, scan_points=((0., 0.),))
    def forbidden(*args, **kwargs):
        raise AssertionError("turn must not call the navigation collision checker")
    monkeypatch.setattr(rectangle.navigator, "motion_clear", forbidden)
    for direction in (3, 4):
        effort = rectangle._guarded(f, now, direction=direction)
        assert effort.direction == direction and effort.brush_percent == 0
    assert rectangle.navigator.sweep(Waypoint(.5, .5, 0.), Waypoint(.5, .5, math.pi/2))


def test_short_undocking_waypoint_is_executed_before_parallel_travel(rectangle):
    now = 1_000_000_000
    rectangle.edge = rectangle.pool.edges[0]
    rectangle.path = [Waypoint(2.8, 1.73, 0.), Waypoint(3.5, 1.73, 0.)]
    rectangle.path_index = 0
    rectangle.progress_since = now
    f = feedback(rectangle, Waypoint(2.8, 1.7, 0.), now)
    effort = rectangle._navigate(f, now)
    assert rectangle.path_index == 0 and effort.side_left_percent > 0


def test_unknown_target_wall_raster_transition_allows_only_brush(rectangle):
    nav, edge = rectangle.navigator, rectangle.pool.edges[0]
    pose = nav.edge_pose(edge, 1., rectangle.c.wall_target_m)
    # Contact raster uncertainty is restricted to the known wall's narrow band.
    assert nav.free(pose, contact=True, edge=edge)
    collision = nav.edge_pose(edge, 1., .10)
    assert not nav.free(collision, contact=True, edge=edge)


def test_planning_failure_pauses_with_no_motion(rectangle, monkeypatch):
    now = 1_000_000_000
    f = feedback(rectangle, Waypoint(2.8, 1.7, 0.), now)
    def fail(*args):
        raise PlanningError("blocked task")
    monkeypatch.setattr(rectangle, "_initial_plan", fail)
    assert rectangle.start(f, now)[0]
    rectangle.update(f, now)
    pending = rectangle.future
    if pending is not None:
        with pytest.raises(PlanningError):
            pending.result(timeout=1)
        rectangle.update(f, now+1)
    assert rectangle.phase == "PAUSED" and rectangle.last_effort is None
