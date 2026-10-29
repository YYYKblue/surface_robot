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
    with pytest.raises(map_profile.MapProfileError, match="not a single closed pool"):
        map_profile.require(open_l, "rectangle")


def test_route_profile_rejects_a_closed_pool_map(rectangle):
    with pytest.raises(map_profile.MapProfileError, match="no open L route|no pair"):
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
