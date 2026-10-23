from dataclasses import replace
import math

import numpy as np
import pytest

from f1_pool_cleaning.perception import Perception
from f1_pool_cleaning.synthetic import pool_scan


@pytest.mark.parametrize("changes", [dict(hull_width_m=0.), dict(stopping_distance_m=.9),
    dict(brush_compression_m=.05), dict(lidar_x_m=math.nan), dict(scan_max_age_sec=.2),
    dict(turn_sweep_rad=.3)])
def test_invalid_perception_geometry_and_feedback_limits(calibrated, changes):
    # Keep geometry/freshness coverage formerly mixed into the deleted pulse tests.
    with pytest.raises(ValueError):
        replace(calibrated, **changes).validate()


@pytest.mark.parametrize("distance,angle", [(.48, 0.), (.8, .12), (.8, -.12), (1.2, .25)])
def test_wall_geometry_and_planar_lidar_transform(calibrated, distance, angle):
    estimator = Perception(calibrated)
    scan = pool_scan(1_000_000_000, distance, angle, config=calibrated, noise=.002)
    o = estimator.process(scan, 1_010_000_000)
    assert o.valid, o
    assert o.distance == pytest.approx(distance, abs=.006)
    assert o.angle == pytest.approx(angle, abs=.006)
    assert o.support >= calibrated.min_support_m
    assert o.front_observed and o.front_clearance > calibrated.stopping_distance_m


def test_unknown_front_is_not_free_space(calibrated):
    estimator = Perception(calibrated)
    o = estimator.process(pool_scan(1_000_000_000, config=calibrated, missing_front=True), 1_000_000_000)
    assert o.valid and not o.front_observed


def test_front_obstacle_and_rigid_envelope(calibrated):
    for front, clearance, collision in [(0.55, .15, False), (.41, .01, True)]:
        estimator = Perception(calibrated)
        o = estimator.process(pool_scan(1_000_000_000, front_x=front, config=calibrated), 1_000_000_000)
        assert o.front_clearance == pytest.approx(clearance, abs=.02)
        assert o.rigid_collision == collision


def test_duplicates_stale_frames_and_invalid_metadata(calibrated):
    scan = pool_scan(1_000_000_000, config=calibrated)
    estimator = Perception(calibrated)
    assert estimator.process(scan, 1_000_000_000).valid
    assert not estimator.process(scan, 1_010_000_000).valid
    for altered in (replace(scan, frame="other"), replace(scan, angle_increment=0.),
                    replace(scan, time_increment=.001), replace(scan, range_max=float("nan")),
                    replace(scan, ranges=(math.inf,) * len(scan.ranges))):
        assert not Perception(calibrated).process(altered, 1_000_000_000).valid
    assert not Perception(calibrated).process(scan, 1_151_000_000).valid
    assert not Perception(calibrated).process(scan, 990_000_000).valid


def test_wall_jump_rejected_until_explicit_reset(calibrated):
    estimator = Perception(calibrated)
    assert estimator.process(pool_scan(1_000_000_000, config=calibrated), 1_000_000_000).valid
    o = estimator.process(pool_scan(1_020_000_000, .9, config=calibrated), 1_020_000_000)
    assert not o.valid
    estimator.reset()
    assert estimator.process(pool_scan(1_040_000_000, .9, config=calibrated), 1_040_000_000).valid


def test_short_object_does_not_replace_supported_wall(calibrated):
    estimator = Perception(calibrated)
    scan = pool_scan(1_000_000_000, config=calibrated)
    ranges = np.array(scan.ranges)
    # A small corrupt range group breaks continuity, rather than becoming a wall.
    ranges[300:310] = .20
    o = estimator.process(replace(scan, ranges=tuple(ranges)), 1_000_000_000)
    assert o.valid and o.distance == pytest.approx(.48, abs=.01)


def test_unmeasured_geometry_still_reports_wall_but_no_front_clearance():
    estimator = Perception()
    o = estimator.process(pool_scan(1_000_000_000), 1_000_000_000)
    assert o.valid and not o.front_observed


def test_280_degree_view_does_not_imply_front_visibility(calibrated):
    full = pool_scan(1_000_000_000, config=calibrated)
    # Sensor yaw is pi: a scan centred on sensor zero may leave the bow blind.
    rear_centred = replace(full, angle_min=math.radians(-140), ranges=full.ranges[160:-160])
    o = Perception(calibrated).process(rear_centred, 1_000_000_000)
    assert o.valid and not o.front_observed
    # Same 280 degree width, this time including sensor pi (body forward).
    ranges = full.ranges[880:1440] + full.ranges[:561]
    front_centred = replace(full, angle_min=math.radians(40), ranges=ranges)
    o = Perception(calibrated).process(front_centred, 1_000_000_000)
    assert o.valid and o.front_observed


@pytest.mark.parametrize("body_angle", [-math.pi / 2, math.pi / 2, math.pi])
def test_blind_front_does_not_hide_visible_rigid_obstacle(calibrated, body_angle):
    full = pool_scan(1_000_000_000, config=calibrated)
    partial = replace(full, angle_min=math.radians(-140), ranges=full.ranges[160:-160])
    angles = (partial.angle_min + np.arange(len(partial.ranges)) * partial.angle_increment
              + calibrated.lidar_yaw_rad)
    delta = (angles - body_angle + math.pi) % (2 * math.pi) - math.pi
    ranges = list(partial.ranges)
    # Valid side/stern return inside the enclosing box; remaining wall points
    # still suffice for fitting. Bow rays are structurally absent, not inf.
    ranges[int(np.argmin(np.abs(delta)))] = .12
    observation = Perception(calibrated).process(replace(partial, ranges=tuple(ranges)), full.stamp_ns)
    assert observation.valid
    assert not observation.front_observed
    assert observation.front_clearance == 0. and observation.front_fraction == 0.
    assert observation.rigid_collision


def test_future_last_ray_and_impossible_angular_span_rejected(calibrated):
    full = pool_scan(1_000_000_000, config=calibrated)
    assert not Perception(calibrated).process(replace(full, time_increment=.00002), 1_000_000_000).valid
    assert not Perception(calibrated).process(replace(full, angle_increment=.1), 1_000_000_000).valid


def test_turn_sweep_rejects_unknown_sectors_and_nearby_hull_obstacles(calibrated):
    full = pool_scan(1_000_000_000, wall_distance=.8, config=calibrated)
    o = Perception(calibrated).process(full, full.stamp_ns)
    assert o.valid and o.left_turn_clear and o.right_turn_clear
    # Front and left wall remain visible; blind stern still vetoes a pivot.
    front_ranges = full.ranges[880:1440] + full.ranges[:561]
    partial = replace(full, angle_min=math.radians(40), ranges=front_ranges)
    o = Perception(calibrated).process(partial, partial.stamp_ns)
    assert o.front_observed and not o.left_turn_clear and not o.right_turn_clear
    # A wall just outside the static envelope can intersect the turning bow.
    near = pool_scan(1_000_000_000, wall_distance=.35, config=calibrated)
    o = Perception(calibrated).process(near, near.stamp_ns)
    assert not o.rigid_collision and not o.left_turn_clear and not o.right_turn_clear


def test_blind_sector_blocks_only_the_affected_turn_direction(calibrated):
    full = pool_scan(1_000_000_000, wall_distance=.8, config=calibrated)
    angles = (full.angle_min + np.arange(len(full.ranges)) * full.angle_increment
              + calibrated.lidar_yaw_rad + math.pi) % (2 * math.pi) - math.pi
    ranges = np.array(full.ranges)
    ranges[np.abs((angles + 2.8 + math.pi) % (2 * math.pi) - math.pi) < .08] = math.inf
    o = Perception(calibrated).process(replace(full, ranges=tuple(ranges)), full.stamp_ns)
    assert o.valid and o.front_observed
    assert o.left_turn_clear and not o.right_turn_clear


def test_intermediate_rotated_corner_cannot_be_declared_free(calibrated):
    # Obstacles lie outside the current expanded box but inside a rotated pose.
    # This checks intermediate sweep positions independently of wall fitting.
    c = replace(calibrated, turn_sweep_rad=.2)
    angles = np.linspace(-math.pi, math.pi, 20000, endpoint=False)
    for sign in (-1, 1):
        for theta in (.03, .09, .15):
            theta *= sign
            x, y = c.hull_length_m / 2 + c.rigid_margin_m - .001, c.hull_width_m / 2 + c.rigid_margin_m - .001
            if sign < 0:
                y = -y
            px, py = math.cos(theta) * x - math.sin(theta) * y, math.sin(theta) * x + math.cos(theta) * y
            dx, dy = px - c.lidar_x_m, py - c.lidar_y_m
            index = np.argmin(np.abs((angles - math.atan2(dy, dx) + math.pi) % (2 * math.pi) - math.pi))
            ranges = np.full(len(angles), 5.)
            ranges[index] = math.hypot(dx, dy)
            left, right = Perception(c)._turn_clearance(angles, np.ones(len(angles), dtype=bool), ranges)
            assert not (left if sign > 0 else right)


def test_cached_sweep_rechecks_changed_returns_blind_sectors_and_ray_count(calibrated):
    estimator = Perception(calibrated)
    angles = np.linspace(-math.pi, math.pi, 1440, endpoint=False)
    valid = np.ones(len(angles), dtype=bool)
    ranges = np.full(len(angles), 5.)
    assert estimator._turn_clearance(angles, valid, ranges) == (True, True)
    blind = valid.copy()
    blind[np.abs(angles + 2.8) < .08] = False
    assert estimator._turn_clearance(angles, blind, ranges) == (True, False)
    blocked = ranges.copy()
    blocked[len(angles) // 2] = .1
    assert estimator._turn_clearance(angles, valid, blocked) == (False, False)
    estimator.reset()
    assert estimator._turn_clearance(angles[::2], valid[::2], ranges[::2]) == (True, True)


@pytest.mark.parametrize("angle", [0., .17, -.12])
def test_connected_rear_corner_does_not_hide_left_wall(calibrated, angle):
    # Dense short-wall returns are connected to the left wall without a range
    # gap, as in the 2026-10-23 Nano capture near the back corner of the pool.
    c = replace(calibrated, lidar_x_m=-.2143195, hull_length_m=.56,
                hull_width_m=.56, brush_tip_y_m=.33, brush_compression_m=.01)
    theta = np.linspace(math.radians(-140), math.radians(140), 3500)
    rays = np.c_[np.cos(theta + c.lidar_yaw_rad), np.sin(theta + c.lidar_yaw_rad)]
    origin = np.array([c.lidar_x_m, c.lidar_y_m])
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-math.sin(angle), math.cos(angle)])
    walls = [(normal, .40), (-normal, 1.12), (-tangent, .40), (tangent, 2.48)]
    ranges = np.full(len(theta), np.inf)
    for n, d in walls:
        denom = rays @ n
        hit = denom > 1e-8
        ranges[hit] = np.minimum(ranges[hit], (d - origin @ n) / denom[hit])
    from f1_pool_cleaning.perception import Scan
    scan = Scan(10**9, c.scan_frame, theta[0], theta[1] - theta[0], tuple(ranges), .03, 20.)
    observation = Perception(c).process(scan, scan.stamp_ns)
    assert observation.valid
    assert observation.distance == pytest.approx(.4, abs=.006)
    assert observation.angle == pytest.approx(angle, abs=.006)
    assert not observation.front_observed
    assert not observation.left_turn_clear and not observation.right_turn_clear
