import math

import numpy as np
import pytest

from f1_localization.map_io import Occupancy, MapProfileError
from f1_area_patrol.mission import Mission, Settings
from f1_area_patrol.planner import WaterGrid, plan


def water(free):
    return WaterGrid(Occupancy(.05, -2., -1., free.shape[1], free.shape[0], free, 1-free, free))


def test_island_concavity_smaller_component_and_coverage():
    free = np.ones((110, 210), np.uint8)
    free[:, 80:91] = 0
    free[18:46, 20:48] = 0
    free[74:, :45] = 0
    g = water(free)
    start = g.xy((52, 15))
    route = plan(g, start, .35)
    assert route.disconnected_m2 > route.region.sum()*g.res**2
    assert route.skipped_m2 > 0 and all(l.kind != "repair" for l in route.legs)
    assert all(g.clear(l.start, l.end) for l in route.legs)
    assert route.legs[0].start == start
    assert all(a.end == b.start for a, b in zip(route.legs, route.legs[1:]))
    with pytest.raises(MapProfileError):
        plan(g, g.xy((35, 84)), .35)


def test_corner_supercover_and_rejoin():
    free = np.ones((35, 60), np.uint8)
    g = water(free)
    g.free[16, 21] = g.labels[16, 21] = 0
    a, b = g.xy((16, 20)), g.xy((17, 21))
    assert not g.clear(a, b)
    assert all(g.clear(p, q) for p, q in zip(g.connect(a, b), g.connect(a, b)[1:]))
    mission = Mission(g, plan(g, a, .4), Settings())
    mission.rejoin(b)
    assert g.clear(b, mission._target(b))
    assert mission.finished_lanes == mission.observed_cells == 0


def test_drive_sequence_with_pose_feedback():
    g = water(np.ones((48, 96), np.uint8))
    x, y = g.xy((24, 48))
    m = Mission(g, plan(g, (x, y), .4), Settings())
    yaw = 0.
    commands = set()
    for i in range(25000):
        out = m.update((x, y, yaw), i*.05)
        commands.add(out.direction)
        if m.index == len(m.legs):
            break
        if out.direction in ("LEFT", "RIGHT"):
            yaw += .04 if out.direction == "LEFT" else -.04
        speed = .012 if out.direction == "FORWARD" else 0.
        lateral = .009*np.sign(out.side)
        x += speed*math.cos(yaw)-lateral*math.sin(yaw)
        y += speed*math.sin(yaw)+lateral*math.cos(yaw)
    assert m.index == len(m.legs) and m.finished_lanes == m.total_lanes
    assert {"FORWARD", "STOP"} <= commands and commands & {"LEFT", "RIGHT"}
    assert 0 < m.observed_cells <= m.region_cells


@pytest.mark.parametrize("start_row, direction", [(12, 1), (46, -1), (30, 1)])
def test_sweeps_from_boat_toward_larger_half(start_row, direction):
    g = water(np.ones((61, 121), np.uint8))
    start = g.xy((start_row, 45))
    route = plan(g, start, .4)
    selected = np.argwhere(route.region)
    assert np.all(direction * (selected[:, 0] - start_row) >= 0)
    assert route.region.sum()*g.res**2 > route.skipped_m2
    lanes = [l for l in route.legs if l.kind == "lane"]
    assert len(lanes) >= 2 and lanes[0].start == start
    headings = [np.sign(l.end[0]-l.start[0]) for l in lanes]
    assert all(a == -b for a,b in zip(headings,headings[1:]))
    assert all(route.region[g.cell(*l.end)] and g.clear(l.start,l.end) for l in route.legs)


def test_gradual_drift_larger_than_old_cross_track_limit():
    g = water(np.ones((75, 121), np.uint8))
    start = g.xy((30, 55))
    mission = Mission(g, plan(g, start, .4), Settings())
    for i in range(11):
        mission.update((start[0]+i*.01, start[1]-i*.03, 0.), i*.05)
    assert mission.index == 0
    with pytest.raises(ValueError, match="跳变"):
        mission.update((start[0]+1., start[1]-.30, 0.), .6)


def test_plan_from_free_cell_next_to_wall():
    free = np.ones((30, 51), np.uint8)
    free[:, 0] = 0
    g = water(free)
    start = g.xy((15, 1))
    route = plan(g, start, .4)
    assert route.legs[0].start == start
    assert np.array_equal(g.labels > 0, free)
    assert not g.label(g.xy((15, 0)))
    assert all(g.clear(leg.start, leg.end) for leg in route.legs)


def test_connector_at_map_border_does_not_wrap_or_leave_image():
    free = np.ones((10, 12), np.uint8)
    free[:8, 5] = 0
    g = water(free)
    for row in (0, 5):
        a, b = g.xy((row, 0)), g.xy((row, 11))
        points = g.connect(a, b)
        assert len(points) > 2 and all(g.clear(p, q) for p, q in zip(points, points[1:]))
