"""Direct left-wall control: angle gate, continuous yaw correction and side distance."""
from dataclasses import dataclass
import math

from f1_pool_cleaning.perception import Observation
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
        self.phase, self.reason = "IDLE", "waiting for start"
        self.started_ns = self.last_update_ns = 0
        self.last_scan = -1
        self.last_effort = None
        self.wall_lost_since_ns = None

    @property
    def active(self):
        return self.phase in ("FOLLOW", "CORRECT_HEADING", "WAIT_WALL")

    def halt(self, reason, phase="PAUSED"):
        self.phase, self.reason = phase, reason
        self.last_effort = None
        self.wall_lost_since_ns = None

    def front_remaining(self, pose):
        x, _ = self.c.pool_xy(pose.x, pose.y)
        yaw = wrap(pose.yaw - self.c.axis_yaw_rad)
        # Use the short wall ahead of the boat, for either long-wall direction.
        # A consistent 180-degree map relocalization therefore gives the same
        # clearance without a manually rotated "current side" configuration.
        extent = self.c.hull_half_length_m * abs(math.cos(yaw)) + self.c.hull_half_width_m * abs(math.sin(yaw))
        centre_distance = self.c.length_m - x if math.cos(yaw) >= 0 else x
        return centre_distance - extent

    def error(self, f, now, allow_invalid_wall=False):
        c = self.c
        if f is None:
            return "fresh left-wall and fixed-map base_link pose required"
        w, p = f.wall, f.pose
        for stamp, age, name in ((w.stamp_ns, c.wall_max_age_sec, "wall"),
                                  (p.stamp_ns, c.pose_max_age_sec, "pose")):
            if stamp <= 0 or not 0 <= now - stamp < age * 1e9:
                return name + " stale or future stamped"
        if not all(math.isfinite(v) for v in (w.distance, w.angle, p.x, p.y, p.yaw)):
            return "nonfinite feedback"
        if not w.valid:
            return "" if allow_invalid_wall else "left wall invalid: " + (w.reason or "no wall fit")
        if w.distance <= 0:
            return "left wall invalid distance"
        # Perception supplies the measured wall. No second wall-quality gate,
        # hull/brush/sweep checks, or map-to-wall identity gates here.
        return ""

    def start(self, f, now, allow_wall_recovery=False):
        if self.phase not in ("IDLE", "CANCELLED", "STOPPED"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self.error(f, now, allow_invalid_wall=allow_wall_recovery)
        if error:
            return False, error
        if self.front_remaining(f.pose) <= self.c.front_stop_m or self._visible_front_stop(f.wall):
            return False, "already inside front-wall stopping zone"
        self.halt("single left side started", "FOLLOW" if f.wall.valid else "WAIT_WALL")
        if not f.wall.valid:
            self.wall_lost_since_ns = now
            self.reason = "lease acquired; waiting for left wall: " + (f.wall.reason or "no wall fit")
        self.started_ns = self.last_update_ns = now
        self.last_scan = -1
        return True, "single-side task started"

    def _visible_front_stop(self, w):
        # Available positive evidence remains useful even in map endpoint mode.
        return (w.front_observed and math.isfinite(w.front_fraction) and w.front_fraction >= .85
                and math.isfinite(w.front_clearance) and w.front_clearance <= self.c.front_stop_m)

    def update(self, f, now):
        if not self.active:
            return None
        error = self.error(f, now, allow_invalid_wall=True)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or (self.c.run_timeout_sec > 0
                and now - self.started_ns >= self.c.run_timeout_sec * 1e9):
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
        if not w.valid:
            if self.wall_lost_since_ns is None:
                self.wall_lost_since_ns = now
            elapsed = (now - self.wall_lost_since_ns) / 1e9
            detail = w.reason or "no wall fit"
            if elapsed >= c.wall_recovery_sec:
                self.halt(f"left wall lost for {elapsed:.2f}s: {detail}")
                return None
            self.phase = "WAIT_WALL"
            self.reason = f"waiting for left wall ({elapsed:.2f}s): {detail}"
            # Keep the active task/lease with fresh neutral commands. Never
            # steer from a remembered angle or turn on the brush without a fit.
            self.last_effort = Effort(now, deadline)
            return self.last_effort
        self.wall_lost_since_ns = None
        # Signed fifth-thruster correction about 40 cm, with a 38--42 cm band.
        if w.distance > c.side_on_distance_m:
            side_output = c.side_percent
        elif w.distance < c.side_off_distance_m:
            side_output = -c.side_percent
        else:
            side_output = 0.
        brush_aligned = abs(w.angle) <= c.heading_deadband_rad
        direction = 0 if abs(w.angle) <= c.heading_correction_deadband_rad else (3 if w.angle > 0 else 4)
        self.phase = "CORRECT_HEADING" if direction else "FOLLOW"
        self.reason = ("correcting heading; brush propulsion" if direction and brush_aligned else
                       "correcting heading into brush band" if direction else "within heading band; brush propulsion")
        # No pulse/dwell state: each fresh scan directly renews yaw correction.
        # Zero direction_until means the normal command deadline applies.
        effort = Effort(now, deadline, 0, direction, side_output,
                        c.brush_percent if brush_aligned else 0.)
        self.last_effort = effort
        return effort
