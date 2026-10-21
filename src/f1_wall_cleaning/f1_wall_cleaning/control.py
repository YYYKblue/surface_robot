"""Pure single-wall state machine. No serial, ROS, force or speed estimation."""
from dataclasses import dataclass
import math

from .config import Config
from .perception import fresh

MOVING = {"ALIGN_WALL", "APPROACH_WALL", "CLEAN_WALL"}


def clip(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Effort:
    computed_ns: int
    observation_ns: int
    deadline_ns: int
    direction: int = 0
    speed_percent: int = 25
    brush_enabled: bool = False
    brush_percent: float = 0.
    direction_until_ns: int = 0


class Follower:
    def __init__(self, config=Config()):
        self.c = config.validate()
        self.phase = "IDLE"
        self.reason = "waiting for explicit start"
        self.started_ns = 0
        self.approach_since = 0
        self.stable_since = None
        self.last_observation = 0
        self.last_effort = None
        self.brush_active_sec = 0.
        self.last_tick_ns = 0
        self.pulse_direction = 0
        self.pulse_until = 0
        self.dwell_until = 0

    @property
    def active(self):
        return self.phase in MOVING

    def observation_error(self, o, now_ns):
        c = self.c
        if o is None or not fresh(o.stamp_ns, now_ns, c.scan_max_age_sec):
            return "observation missing, stale or future-dated"
        if not o.valid:
            return o.reason or "invalid wall"
        numbers = (o.distance, o.angle, o.residual, o.support, o.ratio,
                   *o.start, *o.end, o.front_clearance, o.front_fraction)
        if not all(math.isfinite(x) for x in numbers):
            return "non-finite observation"
        if (not 0 < o.distance <= c.max_wall_distance_m or abs(o.angle) > c.max_start_angle_rad
                or o.inliers < c.min_inliers or o.ratio < c.min_inlier_ratio
                or o.support < c.min_support_m or o.residual > c.fit_threshold_m):
            return "wall quality/angle outside single-wall operating envelope"
        if not o.front_observed or o.front_fraction < c.front_valid_fraction:
            return "front space unknown"
        if o.rigid_collision:
            return "return inside rigid hull clearance envelope"
        if o.front_clearance <= c.stopping_distance_m:
            return "front obstacle or corner within stopping distance"
        if max(o.start[0], o.end[0]) < c.hull_length_m / 2 + c.stopping_distance_m:
            return "wall end/corner ahead; automatic cornering is not enabled"
        body_support = abs(math.sin(o.angle)) * c.hull_length_m / 2 + abs(math.cos(o.angle)) * c.hull_width_m / 2
        if o.distance <= body_support + c.rigid_margin_m:
            return "left wall inside rigid hull clearance"
        tip = -math.sin(o.angle) * c.brush_x_m + math.cos(o.angle) * c.brush_tip_y_m
        if tip - o.distance > c.brush_max_compression_m:
            return "brush compression exceeds measured limit"
        return ""

    def start(self, observation, now_ns, live=False):
        error = self.c.control_error(live) or self.observation_error(observation, now_ns)
        if error:
            self.reason = error
            return False
        self.phase = "ALIGN_WALL"
        self.reason = "single-wall run started"
        self.started_ns = self.approach_since = now_ns
        self.stable_since = None
        self.last_observation = 0
        self.last_effort = None
        self.last_tick_ns = now_ns
        self.pulse_direction = 0
        self.pulse_until = self.dwell_until = 0
        return True

    def halt(self, phase, reason):
        self.phase, self.reason = phase, reason
        self.last_effort = None
        self.stable_since = None

    def update(self, o, now_ns):
        if not self.active:
            return None
        error = self.observation_error(o, now_ns)
        if self.last_effort and now_ns >= self.last_effort.deadline_ns:
            error = "previous command expired; explicit resume required"
        if self.last_tick_ns and now_ns < self.last_tick_ns:
            error = "ROS clock moved backward"
        if now_ns - self.started_ns >= self.c.max_run_sec * 1e9:
            error = "bounded single-wall run time reached; no pool completion claimed"
        if not error and self.phase != "CLEAN_WALL" and now_ns - self.approach_since >= self.c.max_approach_sec * 1e9:
            error = "alignment/approach timed out"
        if error:
            self.halt("PAUSED_SAFETY", error)
            return None
        if o.stamp_ns <= self.last_observation:
            # No new scan => no new computation time, command sequence or lease renewal.
            return None
        if self.last_effort and self.last_effort.brush_enabled:
            self.brush_active_sec += max(0., (min(now_ns, self.last_effort.direction_until_ns,
                                                   self.last_effort.deadline_ns) - self.last_tick_ns) / 1e9)
        self.last_tick_ns = now_ns
        self.last_observation = o.stamp_ns
        c = self.c
        normal_x, normal_y = -math.sin(o.angle), math.cos(o.angle)
        tip = normal_x * c.brush_x_m + normal_y * c.brush_tip_y_m
        target = tip - c.brush_compression_m
        error_d = o.distance - target
        contact = (0 <= tip - o.distance <= c.brush_max_compression_m
                   and abs(error_d) <= c.clean_distance_tolerance_m
                   and abs(o.angle) <= c.clean_angle_tolerance_rad)
        if self.phase == "ALIGN_WALL" and abs(o.angle) <= c.clean_angle_tolerance_rad:
            self.phase = "APPROACH_WALL"
        if contact and self.phase != "ALIGN_WALL":
            if self.stable_since is None:
                self.stable_since = now_ns
            if now_ns - self.stable_since >= c.stable_duration_sec * 1e9:
                self.phase = "CLEAN_WALL"
        else:
            self.stable_since = None
            if self.phase == "CLEAN_WALL":
                self.phase = "APPROACH_WALL"
                self.approach_since = now_ns
        bias = 0. if self.phase == "ALIGN_WALL" else clip(
            math.atan(c.distance_gain * error_d), -c.max_approach_angle_rad, c.max_approach_angle_rad)
        heading_error = o.angle + bias
        desired = 3 if heading_error > c.heading_deadband_rad else 4 if heading_error < -c.heading_deadband_rad else 1
        # Direction changes and pulse boundaries always pass through STOP + dwell.
        if self.pulse_direction and (now_ns >= self.pulse_until or desired != self.pulse_direction):
            self.pulse_direction = 0
            self.dwell_until = now_ns + int(c.neutral_dwell_sec * 1e9)
        direction = 0
        if now_ns >= self.dwell_until:
            if desired in (3, 4):
                clear = o.left_turn_clear if desired == 3 else o.right_turn_clear
                # Brush tip must stay within its measured compression limit
                # throughout the bounded turn, not just at the current pose.
                sign = 1 if desired == 3 else -1
                angles = [o.angle - sign * c.turn_sweep_rad * i / 16 for i in range(17)]
                max_tip = max(-math.sin(a) * c.brush_x_m + math.cos(a) * c.brush_tip_y_m for a in angles)
                max_tip += math.hypot(c.brush_x_m, c.brush_tip_y_m) * c.turn_sweep_rad / 32
                if not clear or max_tip - o.distance > c.brush_max_compression_m:
                    self.halt("PAUSED_SAFETY", "turn sweep unknown/blocked or brush compression unsafe")
                    return None
            if not self.pulse_direction:
                duration = c.turn_pulse_sec if desired in (3, 4) else c.forward_pulse_sec
                if desired == 1:
                    # V2 cannot lower PWM continuously; shorten forward pulses.
                    slow = clip((o.front_clearance - c.stopping_distance_m)
                                / (c.front_slow_distance_m - c.stopping_distance_m), 0., 1.)
                    duration = max(.05, duration * slow)
                self.pulse_until = now_ns + int(duration * 1e9)
                self.pulse_direction = desired
            direction = self.pulse_direction
        deadline = min(now_ns + int(c.command_ttl_sec * 1e9),
                       o.stamp_ns + int(c.scan_max_age_sec * 1e9))
        brush = self.phase == "CLEAN_WALL" and contact and direction == 1
        self.last_effort = Effort(now_ns, o.stamp_ns, deadline, direction, c.speed_percent,
                                  brush, c.brush_percent if brush else 0., self.pulse_until if direction else 0)
        self.reason = "v2 exclusive direction pulses with neutral dwell; speed/force feedback unavailable"
        return self.last_effort
