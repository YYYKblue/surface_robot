from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class Config:
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
    front_valid_fraction: float = .85
    front_max_gap_rad: float = .06
    geometry_confirmed: bool = False
    hull_length_m: float = 0.
    hull_width_m: float = 0.
    brush_x_m: float = 0.
    brush_tip_y_m: float = 0.
    brush_compression_m: float = 0.
    brush_max_compression_m: float = 0.
    rigid_margin_m: float = .03
    stopping_distance_m: float = 0.
    front_slow_distance_m: float = 0.
    pulse_response_calibrated: bool = False
    brush_allowed: bool = True
    # Explicit operator-supervised experiment; never labels response as calibrated.
    supervised_trial: bool = False
    trial_wall_distance_m: float = .45
    # Wire v2 minimum is 25% of 500 us, not 25% measured thrust.
    speed_percent: int = 25
    forward_pulse_sec: float = .20
    turn_pulse_sec: float = .10
    neutral_dwell_sec: float = .30
    # Measured upper bound including command expiry, ramp, inertia and drift.
    turn_sweep_rad: float = .08
    distance_gain: float = 1.5
    heading_deadband_rad: float = .03
    max_approach_angle_rad: float = .15
    max_start_angle_rad: float = .40
    clean_distance_tolerance_m: float = .025
    clean_angle_tolerance_rad: float = .06
    stable_duration_sec: float = .50
    brush_percent: float = 20.
    command_ttl_sec: float = .10
    max_approach_sec: float = 20.
    max_run_sec: float = 60.

    def validate(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, (float, int)) and not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f"{f.name} must be finite")
        if not self.scan_frame or not self.base_frame:
            raise ValueError("frame names must be nonempty")
        positive = ("scan_max_age_sec", "cluster_gap_m", "fit_threshold_m", "min_support_m",
                    "min_inliers", "association_distance_m", "association_angle_rad",
                    "front_max_gap_rad", "max_wall_distance_m", "forward_pulse_sec", "turn_pulse_sec",
                    "neutral_dwell_sec", "turn_sweep_rad", "distance_gain", "heading_deadband_rad",
                    "clean_distance_tolerance_m", "clean_angle_tolerance_rad", "stable_duration_sec",
                    "command_ttl_sec", "max_approach_sec", "max_run_sec")
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError("positive algorithm parameters required")
        if not (0 < self.min_inlier_ratio <= 1 and 0 < self.front_valid_fraction <= 1):
            raise ValueError("invalid quality fractions")
        if not (0 < self.max_approach_angle_rad <= self.max_start_angle_rad <= self.max_wall_angle_rad < .8):
            raise ValueError("invalid angle limits")
        if not (self.command_ttl_sec <= .15 and self.scan_max_age_sec <= .15):
            raise ValueError("command/scan age cannot exceed bridge safety budget of 150 ms")
        if type(self.speed_percent) is not int or self.speed_percent != 25:
            raise ValueError("single-wall control requires the lowest v2 gear (25)")
        if not (-100 <= self.brush_percent <= 100 and .05 <= self.turn_pulse_sec <= .25
                and .05 <= self.forward_pulse_sec <= .5 and self.neutral_dwell_sec >= .3
                and 0 < self.turn_sweep_rad <= .2
                and 0 < self.heading_deadband_rad < self.clean_angle_tolerance_rad):
            raise ValueError("invalid pulse, dwell or angular limits")
        if self.rigid_margin_m < 0 or self.brush_compression_m < 0 or self.brush_max_compression_m < 0:
            raise ValueError("clearance/compression cannot be negative")
        if self.max_approach_sec > self.max_run_sec:
            raise ValueError("approach timeout cannot exceed run timeout")
        if self.supervised_trial:
            if (self.brush_allowed or self.max_run_sec > 10 or self.forward_pulse_sec > .10
                    or self.turn_pulse_sec > .05 or self.neutral_dwell_sec < .5
                    or self.stopping_distance_m < .5 or self.front_slow_distance_m < .8
                    or self.turn_sweep_rad < .2
                    or self.trial_wall_distance_m < self.brush_tip_y_m + .10):
                raise ValueError("supervised trial requires brush off, <=10 s run, short pulses, "
                                 "neutral dwell and non-contact wall/front/sweep margins")
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

    def control_error(self, live=False):
        return self.geometry_error() or ("v2 25% pulse, swept angle and stopping response are not calibrated"
                                        if live and not self.pulse_response_calibrated
                                        and not self.supervised_trial else "")
