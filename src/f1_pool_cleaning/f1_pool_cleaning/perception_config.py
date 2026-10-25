"""Wall fitting and geometric observation settings, independent of motion control."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class PerceptionConfig:
    # Known planar calibration; height/tilt are not invented here.
    scan_frame: str = "pavo2s_frame"
    base_frame: str = "base_link"
    lidar_x_m: float = -0.2143195
    lidar_y_m: float = 0.
    lidar_yaw_rad: float = math.pi
    scan_max_age_sec: float = .15
    cluster_gap_m: float = .12
    fit_threshold_m: float = .025
    min_support_m: float = .50
    min_inliers: int = 12
    min_inlier_ratio: float = .75
    max_wall_angle_rad: float = .60
    max_wall_distance_m: float = 3.
    association_distance_m: float = .15
    association_angle_rad: float = .20
    association_timeout_sec: float = 1.
    front_valid_fraction: float = .85
    front_max_gap_rad: float = .06
    geometry_confirmed: bool = False
    # Confirmed enclosing boat rectangle and its single left-side brush.
    hull_length_m: float = .50
    hull_width_m: float = .45
    brush_x_m: float = .10
    brush_tip_y_m: float = .33
    brush_compression_m: float = .01
    brush_max_compression_m: float = .015
    rigid_margin_m: float = .03
    stopping_distance_m: float = 0.
    front_slow_distance_m: float = 0.
    # Diagnostic small-angle rigid sweep; not a full transfer/90-degree planner.
    turn_sweep_rad: float = .08

    def validate(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, (float, int)) and not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f"{f.name} must be finite")
        if not self.scan_frame or not self.base_frame:
            raise ValueError("frame names must be nonempty")
        positive = ("scan_max_age_sec", "cluster_gap_m", "fit_threshold_m", "min_support_m",
                    "min_inliers", "association_distance_m", "association_angle_rad", "association_timeout_sec",
                    "front_max_gap_rad", "max_wall_distance_m", "turn_sweep_rad")
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError("positive algorithm parameters required")
        if not (0 < self.min_inlier_ratio <= 1 and 0 < self.front_valid_fraction <= 1):
            raise ValueError("invalid quality fractions")
        if not 0 < self.max_wall_angle_rad < math.pi / 2:
            raise ValueError("invalid angle limits")
        if self.scan_max_age_sec > .15:
            raise ValueError("scan age cannot exceed 150 ms")
        if self.turn_sweep_rad > .2:
            raise ValueError("diagnostic turn sweep cannot exceed 0.2 rad")
        if self.rigid_margin_m < 0 or self.brush_compression_m < 0 or self.brush_max_compression_m < 0:
            raise ValueError("clearance/compression cannot be negative")
        if self.geometry_confirmed:
            error = self.geometry_error()
            if error:
                raise ValueError(error)
        return self

    def geometry_error(self):
        if not self.geometry_confirmed:
            return "robot and brush dimensions are not measured/confirmed"
        if self.hull_length_m <= 0 or self.hull_width_m <= 0:
            return "positive enclosing rigid hull length/width required"
        if abs(self.brush_x_m) > self.hull_length_m / 2:
            return "brush x must lie within the declared enclosing hull length"
        if self.brush_compression_m > self.brush_max_compression_m:
            return "target brush compression exceeds measured maximum"
        if self.brush_tip_y_m - self.brush_compression_m <= self.hull_width_m / 2 + self.rigid_margin_m:
            return "brush contact distance does not leave rigid hull clearance"
        if not 0 < self.stopping_distance_m < self.front_slow_distance_m:
            return "measured stop and larger slowdown distances required"
        return ""

