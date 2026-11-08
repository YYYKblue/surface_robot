from dataclasses import replace

import pytest

from f1_pool_cleaning.demo import synthetic_config
from f1_pool_cleaning.expected_track import cleaning_segments
from f1_pool_cleaning.geometry import PoolModel


@pytest.mark.parametrize("stop", [.30, .40, .55])
@pytest.mark.parametrize("yaw", [0., .7])
@pytest.mark.parametrize("started", [False, True])
def test_target_endpoint_uses_boat_center_stop_not_brush_contact_end(stop, yaw, started):
    c = synthetic_config(front_stop_m=stop, axis_yaw_rad=yaw)
    pool = PoolModel.rectangle(c)
    required = {e.id: (e.clean_start_s_m,
                       min(e.clean_end_s_m, e.length_m-stop+c.brush_x_m))
                for e in pool.edges} if started else None
    for edge, segment in zip(pool.edges, cleaning_segments(pool, c, required)):
        endpoint = segment["points"][-1]
        along = sum((endpoint[i]-edge.start[i])*edge.tangent[i] for i in (0, 1))
        lateral = -sum((endpoint[i]-edge.start[i])*edge.left_normal[i] for i in (0, 1))
        assert edge.length_m-along == pytest.approx(stop)
        assert lateral == pytest.approx(.33)
        # Changing the brush model must not move a centre-distance stop point.
        changed = cleaning_segments(pool, replace(c, brush_x_m=.15), required)
        assert changed[pool.edges.index(edge)]["points"][-1] == pytest.approx(endpoint)


def test_open_suffix_keeps_edges_separate_and_uses_current_contact_start():
    c = synthetic_config()
    pool = PoolModel.from_polyline([(0., 0.), (3., 0.), (3., -2.), (5., -2.)],
                                   wall_target_m=c.wall_target_m)
    edge = pool.edges[1]
    required = {edge.id: (.9, edge.clean_end_s_m),
                pool.edges[2].id: (.3, pool.edges[2].clean_end_s_m)}
    segments = cleaning_segments(pool, c, required)
    assert [s["edge_id"] for s in segments] == [e.id for e in pool.edges[1:]]
    start = segments[0]["points"][0]
    along = sum((start[i]-edge.start[i])*edge.tangent[i] for i in (0, 1))
    assert along == pytest.approx(.9-c.brush_x_m)
    assert all(len(s["points"]) == 2 for s in segments)
