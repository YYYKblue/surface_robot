import math

import pytest

from f1_pool_cleaning.demo import synthetic_config
from f1_l_cleaning.route import build_l_route


ROUTE = [-1.801431, 0.681948, -10.462431, -0.786552, -11.917931, 10.962448]


def test_map_route_is_open_two_edges_with_one_right_turn():
    pool = build_l_route(ROUTE, synthetic_config().validate())
    assert [edge.id for edge in pool.edges] == ["edge_0", "edge_1"]
    assert pool.edges[0].start == pytest.approx(tuple(ROUTE[:2]))
    assert pool.edges[0].end == pytest.approx(tuple(ROUTE[2:4]))
    assert pool.edges[1].end == pytest.approx(tuple(ROUTE[4:]))
    assert pool.next_edge(pool.edges[0]) is pool.edges[1]
    assert pool.next_edge(pool.edges[1]) is None
    turn = math.atan2(math.sin(pool.edges[1].yaw_rad - pool.edges[0].yaw_rad),
                      math.cos(pool.edges[1].yaw_rad - pool.edges[0].yaw_rad))
    assert math.degrees(turn) == pytest.approx(-92.56, abs=.02)


@pytest.mark.parametrize("route", [
    [0., 0., 0.],
    [0., 0., 1., 0., math.nan, 1.],
    [0., 0., 1., 0., 0., 1., 0., 0.],
    [0., 0., 1., 0., 0., 1.],  # Left turn.
    [0., 0., 1., 0., 2., 0.],  # No turn.
])
def test_invalid_l_routes_are_rejected(route):
    with pytest.raises(ValueError):
        build_l_route(route, synthetic_config().validate())


def test_shared_open_polyline_does_not_close_or_reorder():
    from f1_pool_cleaning.geometry import PoolModel

    route = PoolModel.from_polyline([(0, 0), (4, 0), (4, 5)])
    assert len(route.edges) == 2
    assert route.edges[0].start == (0., 0.)
    assert route.edges[1].end == (4., 5.)
    assert route.next_edge(route.edges[-1]) is None


@pytest.mark.parametrize("points", [
    [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (.2, 0)],
    [(0, 0), (3, 0), (0, 0)], [(0, 0), (math.inf, 0)],
    [(0, 0), (3, 3), (0, 3), (3, 0)],
])
def test_open_polyline_rejects_degenerate_and_crossing_routes(points):
    from f1_pool_cleaning.geometry import PoolModel

    with pytest.raises(ValueError):
        PoolModel.from_polyline(points)


def test_deployed_route_and_map_derived_pose_gate(route_parameters, route_config):
    from f1_pool_cleaning.localization import LocalizationConfig, LocalizationGate, PoseSample
    from f1_pool_cleaning.map_profile import bounds_of

    route = build_l_route(ROUTE, route_config)
    assert not route_parameters["output_enabled"]
    assert route_parameters["use_map_tf"] and route_parameters["require_localization_ready"]
    assert route_parameters["default_edge_id"] == "edge_0"
    # Map geometry is no longer written in l_cleaning.yaml: the launch derives
    # the route and the pose gate from map:=.  Both must stay out of the YAML.
    for stale in ("route_xy", "origin_x_m", "origin_y_m", "length_m", "width_m"):
        assert stale not in route_parameters
    assert [edge.length_m for edge in route.edges] == pytest.approx([8.785, 11.839], abs=.002)
    # A gate built from the route extent must accept the route it was derived
    # from; this is the contract MapProfile.route.overrides() feeds the launch.
    x_min, y_min, x_max, y_max = bounds_of([ROUTE[:2], ROUTE[2:4], ROUTE[4:]])
    gate = LocalizationGate(LocalizationConfig(
        origin_x_m=x_min, origin_y_m=y_min, axis_yaw_rad=0.,
        length_m=x_max - x_min, width_m=y_max - y_min))
    for edge in route.edges:
        for point in (edge.start, edge.end):
            assert gate._boundary_ok(PoseSample(1, *point, edge.yaw_rad))
    assert not gate._boundary_ok(PoseSample(1, 50., 50., 0.))

