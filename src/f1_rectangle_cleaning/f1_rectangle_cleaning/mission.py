"""Pure single-left-wall controller; map endpoint, no transfer or speed assumption."""
from dataclasses import dataclass
import math

from f1_wall_cleaning.perception import Observation
from .config import Config


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Feedback:
    wall: Observation
    pose: Pose


@dataclass(frozen=True)
class Effort:
    computed_ns: int
    valid_until_ns: int
    direction_until_ns: int = 0
    direction: int = 0
    side_left_percent: float = 0.
    brush_percent: float = 0.


def command_fields(effort, config):
    percent = effort.side_left_percent * config.fifth_left_sign
    return dict(direction=effort.direction, speed_percent=config.turn_speed_percent, mode=1,
                brush_enabled=bool(effort.brush_percent), brush_percent=float(effort.brush_percent),
                thrust_enabled=bool(percent), thrust_percent=float(percent))


class Mission:
    def __init__(self, config=None):
        self.c = (config or Config()).validate()
        self.phase, self.reason = "IDLE", "manually place near and parallel to left wall A"
        self.started_ns = self.last_update_ns = 0
        self.last_scan = -1
        self.pulse_until = self.next_pulse = 0
        self.pulse_direction = 0
        self.side_output = 0.
        self.last_effort = None

    @property
    def active(self):
        return self.phase in ("FOLLOW", "CORRECT_HEADING", "NEUTRAL_DWELL")

    def halt(self, reason, phase="PAUSED"):
        self.phase, self.reason = phase, reason
        self.side_output = 0.
        self.pulse_direction = self.pulse_until = self.next_pulse = 0
        self.last_effort = None

    def front_remaining(self, pose):
        x, _ = self.c.pool_xy(pose.x, pose.y)
        yaw = wrap(pose.yaw - self.c.axis_yaw_rad)
        # Distance from the foremost corner of the enclosing hull to the fixed
        # short wall. At zero yaw this is centre distance minus hull half length.
        extent = self.c.hull_half_length_m * abs(math.cos(yaw)) + self.c.hull_half_width_m * abs(math.sin(yaw))
        return self.c.length_m - x - extent

    def error(self, f, now):
        c = self.c
        if f is None:
            return "fresh left-wall and fixed-map base_link pose required"
        w, p = f.wall, f.pose
        for stamp, age, name in ((w.stamp_ns, c.wall_max_age_sec, "wall"),
                                  (p.stamp_ns, c.pose_max_age_sec, "pose")):
            if stamp <= 0 or not 0 <= now - stamp < age * 1e9:
                return name + " stale or future stamped"
        if abs(w.stamp_ns - p.stamp_ns) > c.feedback_skew_sec * 1e9:
            return "feedback timestamp skew"
        if not all(math.isfinite(v) for v in (w.distance, w.angle, w.front_clearance,
                w.residual, w.support, w.ratio, w.front_fraction, p.x, p.y, p.yaw)):
            return "nonfinite feedback"
        if not w.valid or not 0 <= w.residual <= .025 or w.support < .5 or w.inliers < 12 or not .75 <= w.ratio <= 1:
            return "left wall invalid or insufficient support"
        if w.rigid_collision or w.distance <= c.min_wall_distance_m:
            return "rigid hull clearance violated"
        if w.distance > c.max_wall_distance_m:
            return "outside manually placed wall approach range"
        x, y = c.pool_xy(p.x, p.y)
        yaw = wrap(p.yaw - c.axis_yaw_rad)
        if abs(w.angle) > c.heading_limit_rad or abs(yaw) > c.heading_limit_rad:
            return "heading exceeds first-trial limit; manual alignment required"
        if not c.hull_half_length_m < x < c.length_m - c.hull_half_length_m:
            return "pose outside long-wall A working corridor"
        if abs(c.width_m - y - w.distance) > c.map_wall_tolerance_m:
            return "map and observed left wall disagree; check A identity"
        if abs(wrap(yaw + w.angle)) > c.heading_limit_rad:
            return "map and observed wall heading disagree"
        # The user selected fixed-map endpoint stopping because the bow is blind.
        # Do not fabricate front_observed or interpret unknown clearance as zero.
        return ""

    def start(self, f, now):
        if self.phase not in ("IDLE", "CANCELLED", "STOPPED"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self.error(f, now)
        if error:
            return False, error
        if self.front_remaining(f.pose) <= self.c.front_stop_m or self._visible_front_stop(f.wall):
            return False, "already inside front-wall stopping zone"
        self.halt("single left side started", "FOLLOW")
        self.started_ns = self.last_update_ns = now
        self.last_scan = -1
        return True, "single-side task started"

    def _visible_front_stop(self, w):
        # Available positive evidence remains useful even in map endpoint mode.
        return w.front_observed and w.front_fraction >= .85 and w.front_clearance <= self.c.front_stop_m

    def update(self, f, now):
        if not self.active:
            return None
        error = self.error(f, now)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or now - self.started_ns >= self.c.run_timeout_sec * 1e9:
            self.halt("clock reversed or run timeout")
            return None
        self.last_update_ns = now
        # A new map pose reaching the endpoint stops even without a new scan.
        if self.front_remaining(f.pose) <= self.c.front_stop_m or self._visible_front_stop(f.wall):
            self.halt("front stop threshold reached; outputs off, drift unmeasured", "STOPPED")
            self.last_effort = Effort(now, now + round(self.c.command_ttl_sec * 1e9))
            return self.last_effort
        if f.wall.stamp_ns <= self.last_scan:
            if self.last_effort and now >= self.last_effort.valid_until_ns:
                self.halt("command deadline expired without a fresh scan")
            return None
        self.last_scan = f.wall.stamp_ns
        w, c = f.wall, self.c
        deadline = min(now + round(c.command_ttl_sec * 1e9),
                       w.stamp_ns + round(c.wall_max_age_sec * 1e9),
                       f.pose.stamp_ns + round(c.pose_max_age_sec * 1e9))
        # Strict hysteresis: equality and the band preserve the previous state.
        if w.distance > c.side_on_distance_m:
            self.side_output = c.side_percent
        elif w.distance < c.side_off_distance_m:
            self.side_output = 0.
        aligned = abs(w.angle) <= c.heading_deadband_rad
        desired = 0 if aligned else (3 if w.angle > 0 else 4)
        direction = direction_until = 0
        if self.pulse_direction:
            if now >= self.pulse_until or desired != self.pulse_direction:
                ended = min(now, self.pulse_until)
                self.next_pulse = ended + round(c.neutral_dwell_sec * 1e9)
                self.pulse_direction = 0
        if desired and not self.pulse_direction and now >= self.next_pulse:
            self.pulse_direction = desired
            self.pulse_until = now + round(c.turn_pulse_sec * 1e9)
        if self.pulse_direction:
            clear = w.left_turn_clear if self.pulse_direction == 3 else w.right_turn_clear
            if not clear:
                self.halt("required heading-correction sweep is unknown or blocked")
                return None
            direction, direction_until = self.pulse_direction, self.pulse_until
            self.phase = "CORRECT_HEADING"
        else:
            self.phase = "NEUTRAL_DWELL" if now < self.next_pulse else "FOLLOW"
        self.reason = ""
        effort = Effort(now, deadline, direction_until, direction, self.side_output,
                        c.brush_percent if aligned else 0.)
        self.last_effort = effort
        return effort
