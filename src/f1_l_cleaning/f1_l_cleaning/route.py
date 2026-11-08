"""Validate a directed open wall chain for the single left brush."""
import math

from f1_pool_cleaning.geometry import PoolModel, wrap


def build_l_route(route_xy, config):
    values = list(route_xy)
    if len(values) < 4 or len(values) % 2 or not all(math.isfinite(v) for v in values):
        raise ValueError("route_xy must contain at least two finite map points: [x0, y0, x1, y1, ...]")
    route = PoolModel.from_polyline(
        list(zip(values[::2], values[1::2])), frame=config.map_frame,
        wall_target_m=config.wall_target_m, corner_exclusion_m=config.corner_exclusion_m,
        endpoint_margin_m=config.endpoint_margin_m)
    for first, second in zip(route.edges, route.edges[1:]):
        turn = wrap(second.yaw_rad - first.yaw_rad)
        if abs(abs(turn) - math.pi) < 1e-6:
            raise ValueError("open route must not reverse along the same wall")
    if any(config.front_stop_m >= e.length_m - e.entry_s_m for e in route.edges):
        raise ValueError("front_stop_m leaves no usable straight cleaning interval")
    return route
