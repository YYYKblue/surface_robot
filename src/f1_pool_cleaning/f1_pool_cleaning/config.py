"""Wall-following and stage-2 edge-approach settings."""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
    map_frame: str = "map"
    base_frame: str = "base_link"
    origin_x_m: float = -0.9802816886364385
    origin_y_m: float = -0.18424262645062695
    axis_yaw_rad: float = -0.19739548348805863
    length_m: float = 2.8829072052510294
    width_m: float = 1.5199000413306079
    # Confirmed enclosing boat rectangle: 50 cm long by 45 cm wide.
    hull_half_length_m: float = .25
    hull_half_width_m: float = .225
    # Stable left-brush contact band measured from base_link centre to wall.
    wall_target_m: float = .34
    side_on_distance_m: float = .36
    side_off_distance_m: float = .32
    front_stop_m: float = .50
    heading_deadband_rad: float = math.radians(10.)
    heading_correction_deadband_rad: float = math.radians(3.)
    pose_max_age_sec: float = .25
    brush_percent: float = 30.
    side_percent: float = 30.
    turn_speed_percent: int = 25
    wall_max_age_sec: float = .15
    # Fresh scans may temporarily fail wall fitting while the boat yaws.
    wall_recovery_sec: float = 1.5
    command_ttl_sec: float = .10
    # Zero: run until the front-wall threshold or an explicit stop.
    run_timeout_sec: float = 0.
    # User confirmed on 2026-10-23: positive ROS thrust pushes towards left wall.
    fifth_left_sign: int = 1
    # Left brush geometry.  These values are used for contact progress and
    # endpoint checks; they are not a second right-side brush model.
    brush_x_m: float = .10
    brush_tip_y_m: float = .33
    brush_compression_m: float = .01
    # Stage-2 map-edge approach limits.
    edge_select_max_distance_m: float = .75
    edge_heading_tolerance_rad: float = math.radians(35.)
    edge_approach_timeout_sec: float = 12.
    edge_progress_jump_m: float = .35
    corner_exclusion_m: float = .30
    endpoint_margin_m: float = .50

    def validate(self):
        signed = ("origin_x_m", "origin_y_m", "axis_yaw_rad", "run_timeout_sec")
        for field in fields(self):
            v = getattr(self, field.name)
            if isinstance(v, (int, float)) and not math.isfinite(v):
                raise ValueError(f"{field.name} must be finite")
            if isinstance(v, float) and field.name not in signed and v <= 0:
                raise ValueError(f"{field.name} must be positive")
        if not self.base_frame or not self.map_frame:
            raise ValueError("frames must be nonempty")
        if type(self.fifth_left_sign) is not int or self.fifth_left_sign not in (-1, 0, 1):
            raise ValueError("fifth_left_sign must be -1, 0 (unknown), or +1")
        if not (0 < self.side_off_distance_m < self.wall_target_m < self.side_on_distance_m):
            raise ValueError("wall thresholds must order lower < target < upper")
        if self.length_m <= 2 * self.hull_half_length_m:
            raise ValueError("rectangle length must exceed hull length")
        if not 0 < self.heading_correction_deadband_rad <= self.heading_deadband_rad < math.pi / 2:
            raise ValueError("invalid heading band")
        if (not 0 < self.brush_percent <= 100 or not 0 < self.side_percent <= 30
                or type(self.turn_speed_percent) is not int or self.turn_speed_percent != 25
                or not 0 < self.command_ttl_sec <= .15 or not 0 < self.wall_max_age_sec <= .15
                or self.run_timeout_sec < 0 or self.wall_recovery_sec <= 0
                or self.edge_select_max_distance_m <= 0
                or not 0 < self.edge_heading_tolerance_rad < math.pi / 2
                or self.edge_approach_timeout_sec <= 0 or self.edge_progress_jump_m <= 0
                or self.corner_exclusion_m < 0 or self.endpoint_margin_m <= 0
                or self.brush_x_m < -self.hull_half_length_m
                or abs(self.brush_tip_y_m) <= self.hull_half_width_m
                or self.brush_compression_m < 0):
            raise ValueError("invalid effort, feedback timing or optional run timeout")
        if self.brush_tip_y_m - self.brush_compression_m <= self.hull_half_width_m:
            raise ValueError("left brush contact must clear the enclosing hull")
        return self

    def pool_xy(self, x, y):
        dx, dy = x - self.origin_x_m, y - self.origin_y_m
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return c * dx + s * dy, -s * dx + c * dy

    def map_xy(self, x, y):
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return self.origin_x_m + c * x - s * y, self.origin_y_m + s * x + c * y
