"""Single-side trial settings; rectangle dimensions are map estimates."""
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
    hull_half_length_m: float = .28
    hull_half_width_m: float = .28
    wall_target_m: float = .40
    side_on_distance_m: float = .45
    side_off_distance_m: float = .38
    min_wall_distance_m: float = .36
    max_wall_distance_m: float = .65
    front_stop_m: float = .80
    heading_deadband_rad: float = math.radians(3.)
    heading_limit_rad: float = math.radians(15.)
    map_wall_tolerance_m: float = .20
    pose_max_age_sec: float = .25
    feedback_skew_sec: float = .10
    pose_variance_max: float = .01
    yaw_variance_max: float = .03
    pose_jump_m: float = .25
    yaw_jump_rad: float = .30
    brush_percent: float = 15.
    side_percent: float = 15.
    turn_speed_percent: int = 25
    turn_pulse_sec: float = .10
    neutral_dwell_sec: float = .50
    wall_max_age_sec: float = .15
    command_ttl_sec: float = .10
    run_timeout_sec: float = 120.
    # User confirmed on 2026-10-23: positive ROS thrust pushes towards left wall.
    fifth_left_sign: int = 1

    def validate(self):
        signed = ("origin_x_m", "origin_y_m", "axis_yaw_rad")
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
        if not (self.hull_half_width_m + .03 < self.min_wall_distance_m < self.side_off_distance_m
                < self.wall_target_m < self.side_on_distance_m < self.max_wall_distance_m < self.width_m / 2):
            raise ValueError("wall thresholds must order hull margin < min < off < target < on < max < half width")
        if self.length_m <= 2 * (self.front_stop_m + self.hull_half_length_m):
            raise ValueError("rectangle must leave a positive trial segment")
        if not (0 < self.heading_deadband_rad < self.heading_limit_rad <= math.radians(15)):
            raise ValueError("invalid heading limits")
        if (not 0 < self.brush_percent <= 100 or not 0 < self.side_percent <= 20
                or type(self.turn_speed_percent) is not int or self.turn_speed_percent != 25
                or not 0 < self.turn_pulse_sec <= .10 or self.neutral_dwell_sec < .50
                or not 0 < self.command_ttl_sec <= .15 or not 0 < self.wall_max_age_sec <= .15):
            raise ValueError("single-side trial effort/timing limits exceeded")
        return self

    def pool_xy(self, x, y):
        dx, dy = x - self.origin_x_m, y - self.origin_y_m
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return c * dx + s * dy, -s * dx + c * dy

    def map_xy(self, x, y):
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return self.origin_x_m + c * x - s * y, self.origin_y_m + s * x + c * y
