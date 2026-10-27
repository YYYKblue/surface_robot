"""Validate a directed A -> B -> C route for the single left brush."""
import math

from f1_pool_cleaning.geometry import PoolModel, wrap


def build_l_route(route_xy, config):
    values = list(route_xy)
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("route_xy must contain exactly three finite map points: A, B, C")
    route = PoolModel.from_polyline(
        list(zip(values[::2], values[1::2])), frame=config.map_frame,
        wall_target_m=config.wall_target_m, corner_exclusion_m=config.corner_exclusion_m,
        endpoint_margin_m=config.endpoint_margin_m)
    turn = wrap(route.edges[1].yaw_rad - route.edges[0].yaw_rad)
    # The saved map is not exactly orthogonal. Reject reversed routes,
    # straight lines and hairpins while allowing the fitted ~92.6 degree L.
    if not -math.radians(135.) <= turn <= -math.radians(45.):
        raise ValueError("L route must make one right turn between 45 and 135 degrees")
    if any(config.front_stop_m >= e.length_m - e.entry_s_m for e in route.edges):
        raise ValueError("front_stop_m leaves no usable straight cleaning interval")
    return route
