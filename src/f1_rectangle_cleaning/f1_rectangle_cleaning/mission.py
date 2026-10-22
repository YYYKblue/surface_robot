"""Pure mission logic. No ROS, serial access, lease acquisition or actuation.

Transfer is deliberately a manual checkpoint: small-angle wall perception does
not provide the full swept-volume evidence needed for an autonomous 180° turn.
"""
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
class Motion:
    stamp_ns: int
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class Feedback:
    wall: Observation
    pose: Pose
    motion: Motion


@dataclass(frozen=True)
class Effort:
    computed_ns: int
    valid_until_ns: int
    direction_until_ns: int = 0
    direction: int = 0
    side_left_percent: float = 0.


def command_fields(effort, config):
    """One combined bridge-shaped preview, never a second auxiliary publisher.

    Unknown polarity inhibits fifth-channel encoding; the logical requested
    leftward thrust remains visible in mission status. Brush is always off.
    """
    percent = effort.side_left_percent * config.fifth_left_sign
    return dict(direction=effort.direction, speed_percent=25, mode=1,
                brush_enabled=False, brush_percent=0.,
                thrust_enabled=bool(percent), thrust_percent=float(percent))


class Mission:
    def __init__(self, config=None):
        self.c = (config or Config()).validate()
        self.phase, self.reason = "IDLE", "manual initial placement on long wall A required"
        self.side = "A"
        self.completed_segments = []
        self.last_scan = -1
        self.started_ns = self.phase_ns = self.last_update_ns = 0
        self.stable_since = self.stopped_since = None
        self.last_motion_stamp = -1
        self.stopped_samples = 0
        self.pulse_until = self.next_pulse = 0
        self.side_output = 0.
        self.last_effort = None

    @property
    def active(self):
        return self.phase.startswith(("APPROACH_", "FOLLOW_", "STOP_"))

    def halt(self, reason, phase="PAUSED"):
        self.phase, self.reason = phase, reason
        self.side_output = 0.
        self.last_effort = None

    def _enter(self, phase, now):
        self.phase, self.phase_ns, self.reason = phase, now, ""
        self.stable_since = self.stopped_since = None
        self.stopped_samples = 0
        self.last_motion_stamp = -1
        self.pulse_until = self.next_pulse = 0
        self.side_output = 0.

    def error(self, f, now, side=None):
        if f is None:
            return "wall, fixed-map pose and local motion feedback required"
        c, w, p, m = self.c, f.wall, f.pose, f.motion
        for stamp, max_age, name in ((w.stamp_ns, c.wall_max_age_sec, "wall"),
                                     (p.stamp_ns, c.pose_max_age_sec, "pose"),
                                     (m.stamp_ns, c.motion_max_age_sec, "motion")):
            if stamp <= 0 or not 0 <= now - stamp < max_age * 1e9:
                return f"{name} stale or future stamped"
        if max(w.stamp_ns, p.stamp_ns, m.stamp_ns) - min(w.stamp_ns, p.stamp_ns, m.stamp_ns) > c.feedback_skew_sec * 1e9:
            return "feedback timestamp skew"
        if not all(math.isfinite(v) for v in (w.distance, w.angle, w.front_clearance,
                w.residual, w.support, w.ratio, w.front_fraction, p.x, p.y, p.yaw, m.vx, m.vy, m.wz)):
            return "nonfinite feedback"
        if (not w.valid or w.residual > .025 or w.support < .5 or w.inliers < 12 or w.ratio < .75):
            return "left wall invalid or insufficient support"
        if w.rigid_collision or w.distance <= c.min_wall_distance_m:
            return "rigid or non-contact clearance violated"
        if w.distance > c.max_wall_distance_m:
            return "outside manually placed wall approach range"
        if not w.front_observed or w.front_fraction < .85 or w.front_clearance <= 0:
            return "front corridor unknown or blocked"
        x, y = c.pool_xy(p.x, p.y)
        side = side or self.side
        expected_wall = c.width_m - y if side == "A" else y
        expected_yaw = c.axis_yaw_rad + (0 if side == "A" else math.pi)
        if abs(w.angle) > c.heading_limit_rad or abs(wrap(p.yaw - expected_yaw)) > c.heading_limit_rad:
            return "heading error; manual alignment required"
        if not c.hull_half_length_m < x < c.length_m - c.hull_half_length_m:
            return "pose outside long-wall working corridor"
        if abs(expected_wall - w.distance) > c.map_wall_tolerance_m:
            return "map and observed wall disagree; check initial wall identity"
        return ""

    def start(self, f, now):
        if self.phase not in ("IDLE", "CANCELLED", "FINISHED_SEGMENTS"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self.error(f, now, "A")
        if error:
            return False, error
        if not self._stationary(f.motion):
            return False, "manually placed boat must be stationary before start"
        if self._at_end(f, "A"):
            return False, "A start already inside short-wall stopping zone"
        self.side, self.completed_segments = "A", []
        self.started_ns = now
        self.last_scan = -1
        self.last_update_ns = now
        self._enter("APPROACH_A", now)
        return True, "preview started; no hardware output"

    def confirm_second_wall(self, f, now):
        if self.phase != "WAIT_MANUAL_TRANSFER":
            return False, "only allowed after A has stopped"
        error = self.error(f, now, "B")
        if error:
            return False, error
        if self._at_end(f, "B") or not self._stationary(f.motion):
            return False, "place stationary boat at B start, facing opposite to A"
        self.side, self.started_ns = "B", now
        self.last_scan = -1
        self.last_update_ns = now
        self._enter("APPROACH_B", now)
        return True, "B placement confirmed; preview only"

    def _at_end(self, f, side=None):
        x, _ = self.c.pool_xy(f.pose.x, f.pose.y)
        remaining = self.c.length_m - x if (side or self.side) == "A" else x
        return remaining <= self.c.end_inset_m or f.wall.front_clearance <= self.c.front_stop_m

    def _stationary(self, motion):
        return (math.hypot(motion.vx, motion.vy) <= self.c.stopped_speed_mps
                and abs(motion.wz) <= self.c.stopped_yaw_rate_rps)

    def update(self, f, now):
        if not self.active:
            return None
        error = self.error(f, now)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or now - self.started_ns > self.c.run_timeout_sec * 1e9:
            self.halt("clock reversed or segment run timeout")
            return None
        if now - self.phase_ns > self.c.phase_timeout_sec * 1e9:
            self.halt("phase timeout")
            return None
        if f.wall.stamp_ns <= self.last_scan:
            if self.last_effort and now >= self.last_effort.valid_until_ns:
                self.halt("command deadline expired without a fresh scan")
            return None
        self.last_scan = f.wall.stamp_ns
        dt = min((now - self.last_update_ns) / 1e9, self.c.wall_max_age_sec)
        self.last_update_ns = now
        deadline = min(now + int(self.c.command_ttl_sec * 1e9),
                       f.wall.stamp_ns + int(self.c.wall_max_age_sec * 1e9),
                       f.pose.stamp_ns + int(self.c.pose_max_age_sec * 1e9),
                       f.motion.stamp_ns + int(self.c.motion_max_age_sec * 1e9))
        if self._at_end(f) and not self.phase.startswith("STOP_"):
            x, _ = self.c.pool_xy(f.pose.x, f.pose.y)
            remaining = self.c.length_m - x if self.side == "A" else x
            if (f.wall.front_clearance <= self.c.front_stop_m
                    and remaining > self.c.end_inset_m + self.c.map_wall_tolerance_m):
                self.halt("unexpected front obstacle or map/short-wall disagreement")
                return None
            if f.wall.front_clearance > remaining - self.c.hull_half_length_m + self.c.map_wall_tolerance_m:
                self.halt("map endpoint and front wall disagree")
                return None
            self._enter("STOP_" + self.side, now)
        direction, direction_until = 0, 0
        if self.phase.startswith("STOP_"):
            # STOP command is not evidence of a stationary hull. Require distinct
            # local-motion samples over a continuous dwell before segment completion.
            if f.motion.stamp_ns > self.last_motion_stamp:
                self.last_motion_stamp = f.motion.stamp_ns
                if self._stationary(f.motion):
                    if self.stopped_since is None:
                        self.stopped_since = f.motion.stamp_ns
                    self.stopped_samples += 1
                else:
                    self.stopped_since, self.stopped_samples = None, 0
            if (self.stopped_since is not None and self.stopped_samples >= 3
                    and f.motion.stamp_ns - self.stopped_since >= self.c.stopped_sec * 1e9):
                self.completed_segments.append(self.side)
                self._enter("WAIT_MANUAL_TRANSFER" if self.side == "A" else "FINISHED_SEGMENTS", now)
                self.reason = "all outputs off; automatic transfer/180-degree turn not implemented"
        else:
            e = f.wall.distance - self.c.wall_target_m
            # Dry preview: approach left only. No presumed outward thrust and no
            # continuous contact preload; too-close feedback removes thrust now.
            target = min(self.c.side_max_percent, max(0., self.c.side_gain_percent_per_m * e))
            self.side_output = (0. if e <= 0 else min(target, self.side_output + self.c.side_slew_percent_per_sec * dt))
            if abs(e) <= self.c.wall_tolerance_m:
                if self.stable_since is None:
                    self.stable_since = now
                if self.phase.startswith("APPROACH_") and now - self.stable_since >= self.c.stable_sec * 1e9:
                    self._enter("FOLLOW_" + self.side, now)
            else:
                self.stable_since = None
                if self.phase.startswith("FOLLOW_"):
                    self._enter("APPROACH_" + self.side, now)
            if self.phase.startswith("FOLLOW_"):
                if now >= self.pulse_until and now >= self.next_pulse:
                    self.pulse_until = now + int(self.c.forward_pulse_sec * 1e9)
                    self.next_pulse = self.pulse_until + int(self.c.neutral_dwell_sec * 1e9)
                if now < self.pulse_until:
                    direction, direction_until = 1, self.pulse_until
        if direction_until:
            deadline = min(deadline, direction_until)
        effort = Effort(now, deadline, direction_until, direction, self.side_output)
        self.last_effort = effort
        return effort
