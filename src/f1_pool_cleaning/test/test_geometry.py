import math

import pytest

from f1_pool_cleaning.config import Config
from f1_pool_cleaning.geometry import PoolModel
from f1_pool_cleaning.mission import Mission, Pose
from f1_pool_cleaning.demo import sample


NOW = 1_000_000_000


def test_rectangle_edges_are_clockwise_and_keep_wall_on_left():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    assert [edge.id for edge in pool.edges] == ["edge_0", "edge_1", "edge_2", "edge_3"]
    assert [round(math.degrees(edge.yaw_rad)) for edge in pool.edges] == [
        round(math.degrees(c.axis_yaw_rad)),
        round(math.degrees(c.axis_yaw_rad - math.pi / 2)),
        round(math.degrees(c.axis_yaw_rad + math.pi)),
        round(math.degrees(c.axis_yaw_rad + math.pi / 2)),
    ]
    assert all(edge.entry_s_m < edge.stop_s_m for edge in pool.edges)


def test_pose_selects_nearest_edge_and_projects_left_wall_distance():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    pose = Pose(NOW, *c.map_xy(.8, c.width_m - .34), c.axis_yaw_rad)
    edge, projection, reason = pool.select(pose)
    assert reason == "edge selected"
    assert edge.id == "edge_0"
    assert projection.s_m == pytest.approx(.8)
    assert projection.lateral_distance_m == pytest.approx(.34)
    assert projection.heading_error_rad == pytest.approx(0.)


def test_left_edge_selection_does_not_require_heading_alignment_before_approach():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    pose = Pose(NOW, *c.map_xy(.8, c.width_m - .34), c.axis_yaw_rad + math.radians(30.))
    edge, projection, reason = pool.select(pose)
    assert reason == "edge selected"
    assert edge.id == "edge_0"
    assert projection.left_offset_m > 0.
    assert abs(math.degrees(projection.heading_error_rad)) == pytest.approx(30.)


def test_outside_pose_cannot_select_a_right_wall_as_a_left_edge():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    pose = Pose(NOW, *c.map_xy(.8, c.width_m + .10), c.axis_yaw_rad)
    edge, projection, reason = pool.select(pose)
    assert edge is None and projection is None
    assert "no edge" in reason


def test_edge_mission_uses_new_wall_band_and_stops_at_edge_endpoint():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    mission = Mission(c)
    ok, _ = mission.configure_edge(pool, pool.edges[0])
    assert ok
    assert mission.start(sample(c, NOW, along=.8, distance=.34), NOW)[0]
    effort = mission.update(sample(c, NOW, along=.8, distance=.34), NOW)
    assert mission.phase == "FOLLOW"
    assert effort.brush_percent == c.brush_percent and effort.side_left_percent == 0.

    effort = mission.update(sample(c, NOW + 50_000_000, along=.8, distance=.31),
                            NOW + 50_000_000)
    assert mission.phase == "APPROACH_WALL"
    assert effort.side_left_percent == -c.side_percent and effort.brush_percent == 0.

    stop_now = NOW + 100_000_000
    effort = mission.update(sample(c, stop_now, along=pool.edges[0].stop_s_m + .01,
                                   distance=.34), stop_now)
    assert mission.phase == "EDGE_STOP"
    assert effort.direction == 0 and effort.brush_percent == 0 and effort.side_left_percent == 0


def test_edge_coverage_is_left_brush_contact_centerline_only():
    c = Config().validate()
    pool = PoolModel.rectangle(c)
    mission = Mission(c)
    mission.configure_edge(pool, pool.edges[0])
    assert mission.start(sample(c, NOW, along=.8, distance=.34), NOW)[0]
    mission.update(sample(c, NOW, along=.8, distance=.34), NOW)
    mission.update(sample(c, NOW + 50_000_000, along=1.1, distance=.34), NOW + 50_000_000)
    assert mission.cleaned_intervals
    low, high = mission.cleaned_intervals[0]
    assert low >= pool.edges[0].clean_start_s_m
    assert high <= pool.edges[0].clean_end_s_m
    assert low == pytest.approx(.9, abs=.02)
    assert high == pytest.approx(1.2, abs=.02)
