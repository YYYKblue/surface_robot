"""Derivation of task geometry from a saved map.

The fixtures write small synthetic maps (see ``synthetic_map``) instead of
using the deployed ``maps/`` directory, which is git-ignored and therefore
absent on CI and on a fresh clone.
"""
import math

import pytest

from f1_pool_cleaning import map_profile
from f1_pool_cleaning.geometry import PoolModel
from f1_pool_cleaning.synthetic_map import RESOLUTION, l_map, rectangle_map


@pytest.fixture
def rectangle(tmp_path):
    return rectangle_map(tmp_path)


@pytest.fixture
def open_l(tmp_path):
    return l_map(tmp_path)


def test_rectangle_map_derives_wall_centre_geometry(rectangle):
    derived = map_profile.profile(rectangle).rectangle
    assert derived is not None
    # Free water is 3.60 x 1.40 m between x=1.0..4.6 and y=1.0..2.4.  A
    # two-cell wall band puts the wall centre line half a band outside each
    # edge, so the measured span is the free span plus half a band per side.
    assert derived.length_m == pytest.approx(3.70, abs=RESOLUTION)
    assert derived.width_m == pytest.approx(1.50, abs=RESOLUTION)
    assert derived.axis_yaw_rad == pytest.approx(0., abs=math.radians(1.))
    assert derived.fill_fraction > .95
    # The boundary route follows the controller's pool frame exactly.
    corners = [derived.map_xy(0., derived.width_m),
               derived.map_xy(derived.length_m, derived.width_m),
               derived.map_xy(derived.length_m, 0.),
               derived.map_xy(0., 0.)]
    assert list(derived.boundary_xy) == pytest.approx(
        [value for point in corners for value in point], abs=1e-9)
    # And it must survive the controller's own validation as four edges.
    pool = PoolModel.from_boundary(
        list(zip(*[iter(derived.boundary_xy)] * 2)), boundary_order="clockwise",
        wall_target_m=.34, corner_exclusion_m=.30, endpoint_margin_m=.30)
    assert [edge.id for edge in pool.edges] == ["edge_0", "edge_1", "edge_2", "edge_3"]


def test_rectangle_map_origin_is_the_wall_centre_corner(rectangle):
    derived = map_profile.profile(rectangle).rectangle
    # Free water starts at (1.0, 1.0); the wall centre line is 0.05 m outside.
    assert derived.origin_x_m == pytest.approx(.95, abs=RESOLUTION)
    assert derived.origin_y_m == pytest.approx(.95, abs=RESOLUTION)


def test_open_l_map_derives_the_single_clockwise_corner(open_l):
    derived = map_profile.profile(open_l).route
    assert derived is not None
    # Wall-on-left convention: west along the south wall, then a right turn
    # north along the west wall.
    a, b, c = [(derived.route_xy[i], derived.route_xy[i + 1]) for i in (0, 2, 4)]
    assert a[1] == pytest.approx(b[1], abs=.25)          # same south wall
    assert b[0] == pytest.approx(c[0], abs=.25)          # same west wall
    assert a[0] > b[0] and c[1] > b[1]                   # west, then north
    assert math.degrees(derived.turn_rad) == pytest.approx(-90., abs=3.)
    assert derived.quality == pytest.approx(1., abs=.05)


def test_rectangle_profile_rejects_an_open_l_map(open_l):
    with pytest.raises(map_profile.MapProfileError, match="not a single closed pool|wall band"):
        map_profile.require(open_l, "rectangle")


def test_route_profile_rejects_a_closed_pool_map(rectangle):
    with pytest.raises(map_profile.MapProfileError, match="no open wall route"):
        map_profile.require(rectangle, "route")


def test_missing_raster_beside_pbstream_names_the_export_step(tmp_path):
    pbstream = tmp_path / "frozen.pbstream"
    pbstream.write_bytes(b"not a real pbstream")
    with pytest.raises(map_profile.MapProfileError, match="pbstream_to_ros_map"):
        map_profile.resolve_map_files(pbstream)


def test_pbstream_and_yaml_name_the_same_map(rectangle):
    pbstream = rectangle.with_suffix(".pbstream")
    pbstream.write_bytes(b"placeholder")
    assert map_profile.resolve_pbstream(pbstream) == pbstream.resolve()
    assert map_profile.resolve_pbstream(rectangle) == pbstream.resolve()
    assert map_profile.require(rectangle, "rectangle") == map_profile.require(pbstream, "rectangle")


def test_bounds_cover_the_free_water(rectangle):
    bounds = map_profile.bounds_overrides(rectangle)
    assert bounds["origin_x_m"] == pytest.approx(1.0, abs=RESOLUTION)
    assert bounds["origin_y_m"] == pytest.approx(1.0, abs=RESOLUTION)
    assert bounds["length_m"] == pytest.approx(3.6, abs=2 * RESOLUTION)
    assert bounds["width_m"] == pytest.approx(1.4, abs=2 * RESOLUTION)


def test_describe_reports_what_the_map_supports(open_l, rectangle):
    summary = map_profile.describe(open_l)
    assert "route" in summary and "rectangle" in summary["errors"]
    assert summary["route"]["quality"] > .75
    summary = map_profile.describe(rectangle)
    assert "rectangle" in summary and "route" in summary["errors"]


def chain_map(tmp_path, yaw=0., resolution=.05, gap=False, points=None, closing=None):
    """Rasterise an independently specified five-wall stepped bank.

    Only the listed walls are occupied; the polygon's closing sides are
    unknown frontiers. This also exercises short walls below the old 2 m cut.
    """
    import cv2
    import numpy as np
    from f1_pool_cleaning.synthetic_map import write_map, wall_band

    points = np.array(points if points is not None else
                      [(8., 2.), (8., 1.), (5., 1.), (5., 2.), (1., 2.), (1., 9.)])
    polygon = np.vstack((points, closing if closing is not None else [(10., 9.), (10., 2.)]))
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    points, polygon = points @ rotation.T, polygon @ rotation.T
    lower, upper = polygon.min(axis=0) - .5, polygon.max(axis=0) + .5
    width, height = np.ceil((upper - lower) / resolution).astype(int)
    def pixels(xy):
        return np.column_stack(((xy[:, 0]-lower[0])/resolution,
                                height-(xy[:, 1]-lower[1])/resolution)).astype(np.int32)
    free = np.zeros((height, width), np.uint8)
    cv2.fillPoly(free, [pixels(polygon)], 1)
    rows, cols = np.indices(free.shape)
    xy = np.stack((lower[0]+(cols+.5)*resolution,
                   lower[1]+(height-rows-.5)*resolution), axis=-1)
    near = np.zeros(free.shape, bool)
    for start, end in zip(points, points[1:]):
        delta = end-start
        t = np.clip(((xy-start) @ delta)/(delta @ delta), 0., 1.)
        near |= np.linalg.norm(xy-start-t[..., None]*delta, axis=-1) <= 3*resolution
    occupied = wall_band(free.astype(bool)) & near
    if gap:
        # A one-metre missing stretch in the long vertical wall must break
        # the route, even though its infinite supporting line is unchanged.
        local = xy @ rotation
        occupied[(local[..., 1] > 5.) & (local[..., 1] < 6.) & (local[..., 0] < 1.2)] = False
    return write_map(tmp_path, 'chain', free.astype(bool), occupied, tuple(lower), resolution), points


@pytest.mark.parametrize('yaw,resolution', [(0., .05), (.37, .05), (-1.2, .05), (0., .1)])
def test_map_extracts_whole_mixed_turn_chain_and_short_walls(tmp_path, yaw, resolution):
    import numpy as np
    path, points = chain_map(tmp_path, yaw, resolution)
    route = map_profile.require(path, 'route')
    actual = np.asarray(route.route_xy).reshape(-1, 2)
    assert actual.shape == points.shape
    assert actual == pytest.approx(points, abs=3*resolution)
    assert route.leg_lengths_m == pytest.approx([1., 3., 1., 4., 7.], abs=4*resolution)
    assert [v > 0 for v in route.turns_rad] == [False, False, True, False]
    assert len(route.chain_lengths_m) == 1


def test_unknown_gap_is_not_bridged_by_a_cleaning_line(tmp_path):
    path, _ = chain_map(tmp_path, gap=True)
    # Missing occupied support cannot be hidden by a high average score on
    # one long line. The two supported wall stretches remain separate.
    route = map_profile.require(path, 'route')
    assert len(route.chain_lengths_m) >= 2
    assert sum(route.leg_lengths_m) < 15.


def test_another_chain_can_be_selected_without_connecting_the_chains(tmp_path):
    path, _ = chain_map(tmp_path, gap=True)
    first = map_profile.route_profile(path=path)
    second = map_profile.route_profile(path=path, chain_index=1)
    assert first.chain_lengths_m == second.chain_lengths_m
    assert first.route_xy != second.route_xy and second.chain_index == 1
    with pytest.raises(map_profile.MapProfileError, match='route_chain_index'):
        map_profile.route_profile(path=path, chain_index=99)


def test_single_wall_has_valid_nonzero_controller_bounds(tmp_path):
    from f1_pool_cleaning.synthetic_map import grid, write_map
    from f1_pool_cleaning.config import Config
    free, origin = grid(lambda x, y: (x >= 0.) & (x <= 4.) & (y >= 0.) & (y <= 2.),
                        (-.5, -.5, 4.5, 2.5))
    occupied, _ = grid(lambda x, y: (x >= 0.) & (x <= 4.) & (y >= -.1) & (y < 0.),
                        (-.5, -.5, 4.5, 2.5))
    path = write_map(tmp_path, 'single', free, occupied, origin)
    route = map_profile.require(path, 'route')
    assert len(route.leg_lengths_m) == 1 and not route.turns_rad
    values = route.overrides()
    values.pop('route_xy')
    Config(**values).validate()


def test_deployed_map_extends_large_l_with_three_short_walls_when_available():
    from pathlib import Path
    path = Path(__file__).resolve().parents[3] / 'maps/lidar_20261026_170703.yaml'
    if not path.is_file():
        pytest.skip('deployed map is intentionally not included in the source repository')
    route = map_profile.route_profile(path=path)
    assert len(route.leg_lengths_m) == 5
    assert route.leg_lengths_m == pytest.approx([1.0, 2.07, 1.26, 8.91, 11.84], abs=.20)
    assert [v > 0 for v in route.turns_rad] == [False, False, True, False]


@pytest.mark.parametrize('degrees', [20., 30.])
def test_successive_polygon_left_turns_are_not_mistaken_for_smooth_arcs(tmp_path, degrees):
    import numpy as np
    points = [(0., 0.)]
    for angle in (0., math.radians(degrees), math.radians(2*degrees)):
        points.append((points[-1][0] + 4*math.cos(angle), points[-1][1] + 4*math.sin(angle)))
    # Extend the mapped water past the wall end, so the endpoint isn't cut
    # short merely because the fixture's unknown frontier narrows the water.
    end = points[-1]
    extension = (end[0] + 2*math.cos(math.radians(2*degrees)),
                 end[1] + 2*math.sin(math.radians(2*degrees)))
    path, expected = chain_map(tmp_path, points=points,
                               closing=[extension, (end[0]+5., -4.), (0., -4.)])
    route = map_profile.route_profile(path=path)
    actual = np.asarray(route.route_xy).reshape(-1, 2)
    assert actual.shape == expected.shape
    # Intersections at shallow corners amplify pixel fitting error.
    assert actual == pytest.approx(expected, abs=.30)
    assert all(turn > 0 for turn in route.turns_rad)
