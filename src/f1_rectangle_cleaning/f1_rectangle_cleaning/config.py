from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
    map_frame: str = "map"
    base_frame: str = "base_link"
    odom_frame: str = "odom"
    # Wall-band estimates from the 20261021_145054 map, NOT surveyed dimensions.
    origin_x_m: float = -0.9802816886364385
    origin_y_m: float = -0.18424262645062695
    axis_yaw_rad: float = -0.19739548348805863
    length_m: float = 2.8829072052510294
    width_m: float = 1.5199000413306079
    end_inset_m: float = .8
    hull_half_length_m: float = .28
    hull_half_width_m: float = .28
    rigid_margin_m: float = .03
    brush_tip_y_m: float = .33
    wall_target_m: float = .45
    wall_tolerance_m: float = .025
    min_wall_distance_m: float = .36
    max_wall_distance_m: float = .65
    heading_limit_rad: float = .12
    map_wall_tolerance_m: float = .20
    front_stop_m: float = .50  # Bow clearance, provisional; not calibrated braking.
    wall_max_age_sec: float = .15
    pose_max_age_sec: float = .25
    motion_max_age_sec: float = .25
    feedback_skew_sec: float = .10
    pose_variance_max: float = .01
    yaw_variance_max: float = .03
    velocity_variance_max: float = .01
    yaw_rate_variance_max: float = .01
    pose_jump_m: float = .25
    yaw_jump_rad: float = .30
    stable_sec: float = .5
    stopped_sec: float = .5
    stopped_speed_mps: float = .02
    stopped_yaw_rate_rps: float = .03
    phase_timeout_sec: float = 30.
    run_timeout_sec: float = 120.
    forward_pulse_sec: float = .10
    neutral_dwell_sec: float = 1.
    command_ttl_sec: float = .10
    side_gain_percent_per_m: float = 100.
    side_max_percent: float = 15.
    side_slew_percent_per_sec: float = 20.
    # 0 means unknown. "Reverse pushes left" does not establish wire sign.
    fifth_left_sign: int = 0

    def validate(self):
        for field in fields(self):
            v = getattr(self, field.name)
            if isinstance(v, (int, float)) and not math.isfinite(v):
                raise ValueError(f"{field.name} must be finite")
        if not self.map_frame or not self.base_frame or not self.odom_frame:
            raise ValueError("nonempty frames required")
        if self.fifth_left_sign not in (-1, 0, 1):
            raise ValueError("fifth_left_sign must be -1, 0 (unknown), or +1")
        positive = [f.name for f in fields(self) if isinstance(getattr(self, f.name), float)
                    and f.name not in ("origin_x_m", "origin_y_m", "axis_yaw_rad")]
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError("positive dimensions, thresholds and durations required")
        if not (self.hull_half_width_m + self.rigid_margin_m < self.min_wall_distance_m
                and self.brush_tip_y_m < self.min_wall_distance_m
                < self.wall_target_m - self.wall_tolerance_m
                < self.wall_target_m + self.wall_tolerance_m < self.max_wall_distance_m
                < self.width_m / 2):
            raise ValueError("v0.1 requires non-contact, unambiguous long-wall lanes")
        if not (self.hull_half_length_m + self.front_stop_m <= self.end_inset_m < self.length_m / 2):
            raise ValueError("end inset must reserve bow clearance and leave a positive segment")
        if not (self.command_ttl_sec <= .15 and self.wall_max_age_sec <= .15
                and self.forward_pulse_sec <= .10 and self.neutral_dwell_sec >= .5
                and self.side_max_percent <= 20 and self.heading_limit_rad <= .2):
            raise ValueError("preview pulse/feedback/side-thrust limits exceeded")
        if self.phase_timeout_sec > self.run_timeout_sec:
            raise ValueError("phase timeout exceeds run timeout")
        return self

    def pool_xy(self, x, y):
        dx, dy = x - self.origin_x_m, y - self.origin_y_m
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return c * dx + s * dy, -s * dx + c * dy

    def map_xy(self, x, y):
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return self.origin_x_m + c * x - s * y, self.origin_y_m + s * x + c * y
