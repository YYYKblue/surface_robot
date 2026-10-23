"""Synthetic pool rays for offline examples/tests; no hardware or ROS imports."""
import math

from .perception import Scan
from .perception_config import PerceptionConfig


def pool_scan(stamp_ns, wall_distance=.48, wall_angle=0., front_x=4.,
              config=PerceptionConfig(), missing_front=False, noise=0., seed=103):
    import numpy as np
    rng = np.random.default_rng(seed)
    # Wide synthetic scan; actual 280 degree coverage must be checked separately.
    angles = np.linspace(-math.pi, math.pi, 1441)
    ranges = []
    origin = np.array([config.lidar_x_m, config.lidar_y_m])
    walls = [(np.array([-math.sin(wall_angle), math.cos(wall_angle)]), wall_distance),
             (np.array([1., 0.]), front_x), (np.array([-1., 0.]), 4.),
             (np.array([0., -1.]), 3.)]
    for sensor_angle in angles:
        theta = sensor_angle + config.lidar_yaw_rad
        ray = np.array([math.cos(theta), math.sin(theta)])
        intersections = [(d - normal @ origin) / (normal @ ray)
                         for normal, d in walls if normal @ ray > 1e-9]
        distance = min(intersections) if intersections else math.inf
        if missing_front and ray[0] > math.cos(.5):
            distance = math.inf
        elif noise:
            distance += rng.normal(0, noise)
        ranges.append(float(distance))
    return Scan(stamp_ns, config.scan_frame, -math.pi, float(angles[1] - angles[0]),
                tuple(ranges), .03, 20.)
