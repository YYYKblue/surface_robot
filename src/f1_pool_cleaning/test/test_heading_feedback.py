"""Reproduce inward-yaw wall switching with the actual 280-degree lidar view."""
from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from f1_pool_cleaning.perception_config import PerceptionConfig
from f1_pool_cleaning.perception import Perception, Scan
from f1_pool_cleaning.config import Config
from f1_pool_cleaning.demo import sample
from f1_pool_cleaning.mission import Mission


def corner_scan(c, now, bow_left_deg, hide_left=False):
    theta = np.linspace(math.radians(-140), math.radians(140), 3500)
    rays = np.c_[np.cos(theta + c.lidar_yaw_rad), np.sin(theta + c.lidar_yaw_rad)]
    origin = np.array([c.lidar_x_m, c.lidar_y_m])
    angle = math.radians(-bow_left_deg)
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-math.sin(angle), math.cos(angle)])
    # Centre-left 36.5 cm; centre-rear 40.6 cm. Values model the logged
    # distance change, rather than asserting a measured rear-wall distance.
    walls = [(normal, .365), (-normal, 1.155), (-tangent, .406), (tangent, 2.477)]
    hits = np.full((len(walls), len(theta)), np.inf)
    for index, (n, d) in enumerate(walls):
        denom = rays @ n
        hit = denom > 1e-8
        hits[index, hit] = (d - origin @ n) / denom[hit]
    ranges = hits.min(axis=0)
    if hide_left:
        ranges[np.argmin(hits, axis=0) == 0] = np.inf
    return Scan(now, c.scan_frame, float(theta[0]), float(theta[1] - theta[0]),
                tuple(ranges), .03, 20.)


def replay(legacy=False, hide_left=False):
    p = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/perception.yaml').read_text())
    values = p['pool_perception']['ros__parameters']
    if legacy:
        values.update(min_support_m=.5, association_timeout_sec=1.)
    c = PerceptionConfig(**values).validate()
    control = Config(heading_correction_deadband_rad=math.radians(10 if legacy else 3))
    estimator, mission = Perception(c), Mission(control)
    history = []
    for i, bow_left in enumerate([0, 8, 12, 16, 20] + [25] * 45):
        now = 1_000_000_000 + i * 50_000_000
        wall = estimator.process(corner_scan(c, now, bow_left, hide_left and i >= 5), now)
        f = replace(sample(mission.c, now, along=.406, distance=.365,
                           angle=math.radians(-bow_left)), wall=wall)
        if i == 0:
            assert mission.start(f, now)[0]
        effort = mission.update(f, now)
        history.append((now, wall, mission.phase, effort))
    return history


def test_legacy_profile_can_replace_left_wall_with_rear_wall_and_turn_inward():
    history = replay(legacy=True)
    assert any(phase == 'WAIT_WALL' for _, _, phase, _ in history)
    wrong = [(wall, e) for _, wall, _, e in history if e and e.direction == 3]
    assert wrong
    wall, effort = wrong[0]
    assert math.degrees(wall.angle) == pytest.approx(65., abs=.01)
    assert wall.distance == pytest.approx(.406, abs=.01)
    assert effort.brush_percent == 0


def test_current_profile_keeps_left_wall_and_corrects_while_brush_is_running():
    history = replay()
    for _, wall, phase, e in history:
        assert wall.valid and wall.distance == pytest.approx(.365, abs=.01)
        assert phase != 'WAIT_WALL' and e.direction in (0, 4)
    _, wall, _, e = history[1]  # Bow is 8 degrees into the wall.
    assert math.degrees(wall.angle) == pytest.approx(-8., abs=.01)
    assert e.direction == 4 and e.brush_percent == 30


def test_lost_left_wall_cannot_reassociate_to_rear_wall_during_active_recovery():
    history = replay(hide_left=True)
    assert any(phase == 'WAIT_WALL' for _, _, phase, _ in history)
    assert any(phase == 'PAUSED' for _, _, phase, _ in history)
    assert all(e is None or e.direction in (0, 4) for _, _, _, e in history)
