"""Left-wall control and the rectangle edge-transfer state machine."""
from dataclasses import dataclass
import math

from f1_pool_cleaning.perception import Observation
from .config import Config
from .geometry import PoolModel


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
    """One left-brush task, optionally traversing every configured rectangle edge."""

    WALL_PHASES = ("APPROACH_WALL", "APPROACH_EDGE", "FOLLOW",
                   "CORRECT_HEADING", "WAIT_WALL")
    TRANSFER_PHASES = ("EDGE_STOP", "MOVE_INWARD", "TURN_TO_NEXT_EDGE",
                       "APPROACH_NEXT_EDGE")

    def __init__(self, config=None):
        self.c = (config or Config()).validate()
        self.phase, self.reason = "IDLE", "waiting for start"
        self.started_ns = self.last_update_ns = 0
        self.last_scan = -1
        self.last_effort = None
        self.wall_lost_since_ns = None

        self.pool = None
        self.edge = None
        self.edge_task_id = ""
        self.edge_projection = None
        self.last_edge_s = None
        self.last_contact_s = None
        self.last_coverage_ns = None
        self.cleaned_intervals = []
        self.cleaned_by_edge = {}
        self.completed_edges = []
        self.edge_index = None

        # Reverse and right-side thrust are a short simultaneous command; the
        # 90-degree turn remains pose closed-loop.
        self.transfer_next_edge = None
        self.transfer_started_ns = None
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = None
        self.transfer_turn_started_ns = None

    @property
    def active(self):
        return self.phase in self.WALL_PHASES + self.TRANSFER_PHASES

    @property
    def edge_active(self):
        return self.edge is not None and self.pool is not None

    @property
    def wall_required(self):
        return self.phase in self.WALL_PHASES or self.phase == "APPROACH_NEXT_EDGE"

    def _reset_edge_progress(self):
        self.edge_projection = None
        self.last_edge_s = self.last_contact_s = None
        self.last_coverage_ns = None
        self.cleaned_intervals = []

    def _reset_route(self):
        self.cleaned_by_edge = {}
        self.completed_edges = []
        self.edge_index = self.pool.edges.index(self.edge)
        self.transfer_next_edge = None
        self.transfer_started_ns = None
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = None
        self.transfer_turn_started_ns = None
        self._reset_edge_progress()

    def configure_edge(self, pool, edge, task_id=""):
        if self.active or self.phase == "ACQUIRING":
            return False, "cancel before changing the current edge"
        if (not isinstance(pool, PoolModel) or not hasattr(edge, "id")
                or pool.by_id.get(edge.id) is not edge):
            return False, "unknown edge model"
        self.pool, self.edge, self.edge_task_id = pool, edge, task_id
        self._reset_route()
        self.reason = f"edge {edge.id} selected; waiting for start"
        return True, self.reason

    def halt(self, reason, phase="PAUSED"):
        self.phase, self.reason = phase, reason
        self.last_effort = None
        self.wall_lost_since_ns = None
        self.last_coverage_ns = None

    def front_remaining(self, pose):
        x, _ = self.c.pool_xy(pose.x, pose.y)
        yaw = wrap(pose.yaw - self.c.axis_yaw_rad)
        # Measure the stop distance from the boat centre to the wall.
        return self.c.length_m - x if math.cos(yaw) >= 0 else x

    def _pose_error(self, f, now):
        if f is None or f.pose is None:
            return "fresh fixed-map base_link pose required"
        p = f.pose
        if p.stamp_ns <= 0 or not 0 <= now - p.stamp_ns < self.c.pose_max_age_sec * 1e9:
            return "pose stale or future stamped"
        if not all(math.isfinite(v) for v in (p.x, p.y, p.yaw)):
            return "nonfinite pose feedback"
        return ""

    def error(self, f, now, allow_invalid_wall=False):
        c = self.c
        if f is None or f.pose is None:
            return "fresh left-wall and fixed-map base_link pose required"
        pose_error = self._pose_error(f, now)
        if pose_error:
            return pose_error
        # During a transfer, the old left-wall fit is intentionally invalidated.
        if self.phase in self.TRANSFER_PHASES and self.phase != "APPROACH_NEXT_EDGE":
            return ""
        if f.wall is None:
            return "fresh left-wall observation required"
        w, p = f.wall, f.pose
        if w.stamp_ns <= 0 or not 0 <= now - w.stamp_ns < c.wall_max_age_sec * 1e9:
            return "wall stale or future stamped"
        if not all(math.isfinite(v) for v in (w.distance, w.angle, p.x, p.y, p.yaw)):
            return "nonfinite feedback"
        if not w.valid:
            return "" if allow_invalid_wall else "left wall invalid: " + (w.reason or "no wall fit")
        if w.distance <= 0:
            return "left wall invalid distance"
        return ""

    def start(self, f, now, allow_wall_recovery=False):
        if self.edge_active:
            return self._start_edge(f, now, allow_wall_recovery)
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

    def _start_edge(self, f, now, allow_wall_recovery=False):
        if self.phase not in ("IDLE", "CANCELLED", "STOPPED"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self.error(f, now, allow_invalid_wall=allow_wall_recovery)
        if error:
            return False, error
        projection = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        if projection.distance_to_segment_m > self.c.edge_select_max_distance_m:
            return False, "localized pose is too far from the selected edge"
        if projection.left_offset_m <= 0.:
            return False, "selected edge is not on the robot's left side"
        if projection.lateral_distance_m <= 0.:
            return False, "localized pose is outside the selected edge; wall would be on the right"
        if self.edge.length_m - projection.s_m < self.c.front_stop_m:
            return False, "localized pose is already inside the front-wall stopping zone"
        self._reset_route()
        self.halt("edge selected; adjusting wall distance", "APPROACH_WALL")
        self.started_ns = self.last_update_ns = now
        self.last_scan = -1
        self.edge_projection = projection
        self.last_edge_s = projection.s_m
        self.last_contact_s = projection.contact_s_m
        self.last_coverage_ns = None
        self.reason = f"approaching {self.edge.id}; target wall band 0.32--0.36 m"
        return True, f"edge {self.edge.id} started"

    def _visible_front_stop(self, w):
        return (w.front_observed and math.isfinite(w.front_fraction) and w.front_fraction >= .85
                and math.isfinite(w.front_clearance)
                and w.front_clearance + self.c.hull_half_length_m <= self.c.front_stop_m)

    def _deadline(self, now, pose=None, wall=None):
        deadline = now + round(self.c.command_ttl_sec * 1e9)
        if pose is not None:
            deadline = min(deadline, pose.stamp_ns + round(self.c.pose_max_age_sec * 1e9))
        if wall is not None:
            deadline = min(deadline, wall.stamp_ns + round(self.c.wall_max_age_sec * 1e9))
        return deadline

    def _stop_effort(self, now, pose=None, wall=None):
        return Effort(now, self._deadline(now, pose, wall))

    def _mark_edge_completed(self):
        if self.edge is None:
            return
        self.cleaned_by_edge[self.edge.id] = list(self.cleaned_intervals)
        if self.edge.id not in self.completed_edges:
            self.completed_edges.append(self.edge.id)

    def _begin_transfer(self, f, now):
        next_edge = self.pool.next_edge(self.edge)
        self._mark_edge_completed()
        if next_edge is None:
            self.phase = "COMPLETE"
            self.reason = "all rectangle edges completed"
            self.last_effort = self._stop_effort(now, f.pose)
            return
        self.transfer_next_edge = next_edge
        self.transfer_started_ns = now
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = next_edge.yaw_rad
        self.transfer_turn_started_ns = None
        self.phase = "EDGE_STOP"
        self.reason = (f"{self.edge.id} stopped at {self.c.front_stop_m:.2f} m; "
                       f"preparing transfer to {next_edge.id}")
        self.last_effort = self._stop_effort(now, f.pose)

    def _finish_transfer_to_next_edge(self, pose, now):
        self.edge = self.transfer_next_edge
        self.edge_index = self.pool.edges.index(self.edge)
        self.transfer_next_edge = None
        self.transfer_started_ns = None
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = None
        self.transfer_turn_started_ns = None
        self._reset_edge_progress()
        self.last_scan = -1
        self.wall_lost_since_ns = None
        self.phase = "APPROACH_NEXT_EDGE"
        self.reason = f"turned to {self.edge.id}; re-acquiring target wall"
        self.edge_projection = self.edge.project(pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        self.last_edge_s = self.edge_projection.s_m
        self.last_contact_s = self.edge_projection.contact_s_m
        self.last_effort = self._stop_effort(now, pose)

    def update(self, f, now):
        if not self.active:
            return None
        if self.edge_active:
            return self._update_edge(f, now)
        error = self.error(f, now, allow_invalid_wall=True)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or (self.c.run_timeout_sec > 0
                and now - self.started_ns >= self.c.run_timeout_sec * 1e9):
            self.halt("clock reversed or run timeout")
            return None
        self.last_update_ns = now
        if self.front_remaining(f.pose) <= self.c.front_stop_m or self._visible_front_stop(f.wall):
            self.halt("front stop threshold reached; outputs off, drift unmeasured", "STOPPED")
            self.last_effort = self._stop_effort(now, f.pose, f.wall)
            return self.last_effort
        if f.wall.stamp_ns <= self.last_scan:
            if self.last_effort and now >= self.last_effort.valid_until_ns:
                self.halt("command deadline expired without a fresh scan")
            return None
        self.last_scan = f.wall.stamp_ns
        w, c = f.wall, self.c
        deadline = self._deadline(now, f.pose, w)
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
            self.last_effort = Effort(now, deadline)
            return self.last_effort
        self.wall_lost_since_ns = None
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
        effort = Effort(now, deadline, 0, direction, side_output,
                        c.brush_percent if brush_aligned else 0.)
        self.last_effort = effort
        return effort

    def _record_coverage(self, projection, now):
        """Record only continuous left-brush contact-center progress."""
        if self.edge is None or self.last_contact_s is None:
            self.last_contact_s = projection.contact_s_m
            self.last_coverage_ns = now
            return
        if (self.last_coverage_ns is None
                or now - self.last_coverage_ns > 250_000_000
                or projection.contact_s_m < self.last_contact_s - self.c.edge_progress_jump_m):
            self.last_contact_s = projection.contact_s_m
            self.last_coverage_ns = now
            return
        low = max(self.edge.clean_start_s_m, min(self.last_contact_s, projection.contact_s_m))
        high = min(self.edge.clean_end_s_m, max(self.last_contact_s, projection.contact_s_m))
        if high > low:
            if self.cleaned_intervals and low <= self.cleaned_intervals[-1][1] + .01:
                self.cleaned_intervals[-1] = (self.cleaned_intervals[-1][0],
                                              max(self.cleaned_intervals[-1][1], high))
            else:
                self.cleaned_intervals.append((low, high))
        self.last_contact_s = projection.contact_s_m
        self.last_coverage_ns = now

    def _update_transfer(self, f, now):
        error = self._pose_error(f, now)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns:
            self.halt("clock reversed during edge transfer")
            return None
        if (self.c.transfer_timeout_sec > 0
                and self.transfer_started_ns is not None
                and now - self.transfer_started_ns >= round(self.c.transfer_timeout_sec * 1e9)):
            self.halt("edge transfer timeout")
            return None
        pose = f.pose
        if self.phase == "EDGE_STOP":
            self.phase = "MOVE_INWARD"
            self.transfer_action_started_ns = now
            self.reason = ("moving inward: reverse and right push "
                           f"for {self.c.transfer_move_duration_sec:.2f}s")
            self.last_update_ns = now
            self.last_effort = self._stop_effort(now, pose)
            return self.last_effort
        if self.phase == "MOVE_INWARD":
            self.last_update_ns = now
            elapsed = now - self.transfer_action_started_ns
            if elapsed >= round(self.c.transfer_move_duration_sec * 1e9):
                self.phase = "TURN_TO_NEXT_EDGE"
                self.transfer_turn_started_ns = now
                self.reason = f"turning right toward {self.transfer_next_edge.id}"
                self.last_effort = self._stop_effort(now, pose)
                return self.last_effort
            self.last_effort = Effort(
                now, self._deadline(now, pose), direction=2,
                side_left_percent=-self.c.side_percent, brush_percent=0.)
            return self.last_effort
        if self.phase == "TURN_TO_NEXT_EDGE":
            self.last_update_ns = now
            target = self.transfer_turn_target_yaw
            error = wrap(target - pose.yaw)
            if abs(error) <= self.c.transfer_turn_tolerance_rad:
                self._finish_transfer_to_next_edge(pose, now)
                return self.last_effort
            # The turn-only watchdog is optional.  A zero value deliberately
            # leaves completion to heading feedback, allowing
            # APPROACH_NEXT_EDGE to recover a rough turn.
            if (self.c.transfer_turn_timeout_sec > 0
                    and now - self.transfer_turn_started_ns >= round(
                        self.c.transfer_turn_timeout_sec * 1e9)):
                self.halt("right turn timeout")
                return None
            # Canonical pool edges are clockwise; a positive error means the
            # feedback or configured route is inconsistent with RIGHT.
            if error > 0:
                self.halt("next edge does not require the configured right turn")
                return None
            self.reason = f"turning right; heading error {math.degrees(error):.1f} deg"
            self.last_effort = Effort(now, self._deadline(now, pose), direction=4)
            return self.last_effort
        # APPROACH_NEXT_EDGE uses the ordinary wall controller on the new edge.
        return self._update_edge(f, now)

    def _update_edge(self, f, now):
        if self.phase in self.TRANSFER_PHASES and self.phase != "APPROACH_NEXT_EDGE":
            return self._update_transfer(f, now)
        error = self.error(f, now, allow_invalid_wall=True)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or (self.c.run_timeout_sec > 0
                and now - self.started_ns >= self.c.run_timeout_sec * 1e9):
            self.halt("clock reversed or run timeout")
            return None
        projection = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        if (self.last_edge_s is not None
                and projection.s_m < self.last_edge_s - self.c.edge_progress_jump_m):
            self.halt("edge progress moved backwards beyond the configured jump")
            return None
        if projection.distance_to_segment_m > self.c.edge_select_max_distance_m:
            self.halt("localized pose left the selected edge corridor")
            return None
        if projection.lateral_distance_m <= 0.:
            self.halt("localized pose is outside the selected edge; wall would be on the right")
            return None
        self.edge_projection = projection
        self.last_edge_s = projection.s_m
        self.last_update_ns = now

        front_center_distance = self.edge.length_m - projection.s_m
        if front_center_distance <= self.c.front_stop_m:
            self._begin_transfer(f, now)
            return self.last_effort

        if f.wall is None or f.wall.stamp_ns <= self.last_scan:
            if self.last_effort and now >= self.last_effort.valid_until_ns:
                self.halt("command deadline expired without a fresh scan")
            return None
        self.last_scan = f.wall.stamp_ns
        w, c = f.wall, self.c
        deadline = self._deadline(now, f.pose, w)
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
            self.last_effort = Effort(now, deadline)
            return self.last_effort

        self.wall_lost_since_ns = None
        if w.distance > c.side_on_distance_m:
            side_output = c.side_percent
        elif w.distance < c.side_off_distance_m:
            side_output = -c.side_percent
        else:
            side_output = 0.
        wall_in_band = c.side_off_distance_m <= w.distance <= c.side_on_distance_m
        brush_aligned = abs(w.angle) <= c.heading_deadband_rad
        direction = 0 if abs(w.angle) <= c.heading_correction_deadband_rad else (3 if w.angle > 0 else 4)
        if not wall_in_band:
            self.phase = "APPROACH_WALL"
            self.reason = "adjusting centre-to-left-wall distance"
        elif projection.s_m < self.edge.entry_s_m:
            self.phase = "APPROACH_EDGE"
            self.reason = "wall stable; approaching selected edge entry"
        else:
            self.phase = "CORRECT_HEADING" if direction else "FOLLOW"
            self.reason = ("correcting heading; brush propulsion" if direction and brush_aligned else
                           "correcting heading into brush band" if direction else
                           "within 32--36 cm wall band; brush propulsion")
        brush = c.brush_percent if wall_in_band and brush_aligned else 0.
        effort = Effort(now, deadline, 0, direction, side_output, brush)
        self.last_effort = effort
        if brush and projection.s_m >= self.edge.clean_start_s_m:
            self._record_coverage(projection, now)
        else:
            self.last_coverage_ns = None
            self.last_contact_s = projection.contact_s_m
        return effort
