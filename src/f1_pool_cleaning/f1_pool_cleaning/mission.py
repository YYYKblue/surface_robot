"""Left-wall control and the rectangle edge-transfer state machine."""
from dataclasses import dataclass, replace
import math

from f1_pool_cleaning.perception import Observation
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
    scan: object = None  # compatibility only; direct approach does not consume raw scans


@dataclass(frozen=True)
class Effort:
    computed_ns: int
    valid_until_ns: int
    direction_until_ns: int = 0
    direction: int = 0
    side_left_percent: float = 0.
    brush_percent: float = 0.
    forward_boost: bool = False


def command_fields(effort, config):
    percent = effort.side_left_percent * config.fifth_left_sign
    return dict(direction=effort.direction, speed_percent=config.turn_speed_percent, mode=1,
                brush_enabled=bool(effort.brush_percent), brush_percent=float(effort.brush_percent),
                thrust_enabled=bool(percent), thrust_percent=float(percent))


class Mission:
    """One left-brush task, optionally traversing every configured rectangle edge."""

    BOOST_CUTOFF_M = .80
    BOOST_EDIT_PHASES = ("IDLE", "CANCELLED", "STOPPED", "COMPLETE")

    WALL_PHASES = ("APPROACH_WALL", "APPROACH_EDGE", "FOLLOW",
                   "CORRECT_HEADING", "WAIT_WALL")
    TRANSFER_PHASES = ("EDGE_STOP", "MOVE_INWARD", "TURN_TO_NEXT_EDGE",
                       "LEFT_EXIT_WALL", "LEFT_SHIFT_TO_ENTRY", "TURN_LEFT_TO_NEXT_EDGE",
                       "APPROACH_NEXT_EDGE")

    def __init__(self, config):
        if config is None:
            raise ValueError(
                "Mission needs map-derived geometry; launch with map:=<pbstream or map yaml>")
        self.c = config.validate()
        self.phase, self.reason = "IDLE", "waiting for start"
        self.started_ns = self.last_update_ns = 0
        self.last_scan = -1
        self.last_effort = None
        self.wall_lost_since_ns = None
        self.pause_context = None
        self.pause_kind = ""
        self.resume_sequence = 0
        self.forward_boost_enabled = False
        self.forward_boost_cutoff_reached = False

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
        self.transfer_turn_rad = 0.
        self.left_exit_s_m = None

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
        self.forward_boost_cutoff_reached = False
        self.edge_projection = None
        self.last_edge_s = self.last_contact_s = None
        self.last_coverage_ns = None
        self.cleaned_intervals = []

    def set_forward_boost(self, enabled):
        if type(enabled) is not bool:
            return False, "forward boost requires a boolean"
        if self.phase not in self.BOOST_EDIT_PHASES:
            return False, "主电机提速只能在任务开始前设置；暂停后请先取消任务"
        self.forward_boost_enabled = enabled
        return True, "主电机提速已开启" if enabled else "主电机提速已关闭"

    def boost_remaining(self, f):
        if self.edge_active:
            return self.edge.length_m - self.edge.project(f.pose).s_m
        return self.front_remaining(f.pose)

    def _check_boost_cutoff(self, f):
        remaining = self.boost_remaining(f)
        # Tolerate only floating-point roundoff at the inclusive 80 cm boundary.
        if math.isfinite(remaining) and remaining <= self.BOOST_CUTOFF_M + 1e-9:
            self.forward_boost_cutoff_reached = True

    def _without_new_scan(self, now):
        previous = self.last_effort
        if previous and now >= previous.valid_until_ns:
            self.halt("command deadline expired without a fresh scan")
        elif previous and previous.forward_boost and self.forward_boost_cutoff_reached:
            # Revoke forward assistance on fresh pose alone. Never extend the
            # old wall command's computation time or validity deadline.
            self.last_effort = replace(previous, direction=0, forward_boost=False)
            self.reason = "near endpoint; brush propulsion"
            return self.last_effort
        return None

    def _with_forward_boost(self, effort, f):
        remaining = self.boost_remaining(f)
        if (self.forward_boost_enabled and not self.forward_boost_cutoff_reached
                and self.phase == "FOLLOW" and effort.direction == 0
                and effort.brush_percent > 0 and f.wall.valid
                and self.c.side_off_distance_m <= f.wall.distance <= self.c.side_on_distance_m
                and math.isfinite(remaining) and remaining > self.BOOST_CUTOFF_M + 1e-9):
            self.reason = "within wall and heading bands; brush with main-motor forward boost"
            return replace(effort, direction=1, forward_boost=True)
        return effort

    def forward_boost_status(self, now):
        effort = self.last_effort
        active = bool(self.active and effort and effort.forward_boost
                      and now < effort.valid_until_ns)
        reason = ("disabled" if not self.forward_boost_enabled else
                  "active" if active else
                  "inactive_task" if not self.active else
                  "heading_correction" if self.phase == "CORRECT_HEADING" else
                  "endpoint_cutoff" if self.forward_boost_cutoff_reached else
                  "waiting_follow")
        return dict(forward_boost_enabled=self.forward_boost_enabled,
                    forward_boost_active=active, forward_boost_reason=reason,
                    forward_boost_editable=self.phase in self.BOOST_EDIT_PHASES,
                    forward_boost_cutoff_m=self.BOOST_CUTOFF_M,
                    forward_boost_cutoff_reached=self.forward_boost_cutoff_reached)

    def _reset_route(self):
        self.cleaned_by_edge = {}
        self.completed_edges = []
        self.edge_index = self.pool.edges.index(self.edge)
        self.transfer_next_edge = None
        self.transfer_started_ns = None
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = None
        self.transfer_turn_started_ns = None
        self.transfer_turn_rad = 0.
        self.left_exit_s_m = None
        self._reset_edge_progress()

    def configure_edge(self, pool, edge, task_id=""):
        if self.active or self.phase in ("ACQUIRING", "PAUSED"):
            return False, "cancel before changing the current edge"
        if (not isinstance(pool, PoolModel) or not hasattr(edge, "id")
                or pool.by_id.get(edge.id) is not edge):
            return False, "unknown edge model"
        self.pool, self.edge, self.edge_task_id = pool, edge, task_id
        self._reset_route()
        self.reason = f"edge {edge.id} selected; waiting for start"
        return True, self.reason

    def halt(self, reason, phase="PAUSED"):
        self.pause_context = None
        self.pause_kind = "fault" if phase == "PAUSED" else ""
        self.phase, self.reason = phase, reason
        self.last_effort = None
        self.wall_lost_since_ns = None
        self.last_coverage_ns = None

    def pause(self, f, now):
        if self.phase == "PAUSED":
            return self.pause_kind == "user", self.reason
        if not self.active:
            return False, "task is not executing"
        error = self._pose_error(f, now)
        if error:
            self.halt(error)
            return False, error
        context = dict(phase=self.phase, pose=f.pose, since=now)
        self.halt("user paused; resume or cancel")
        self.pause_context, self.pause_kind = context, "user"
        return True, self.reason

    def resume_error(self, f, now):
        context = self.pause_context
        if not context or self.pause_kind != "user" or self.phase not in ("PAUSED", "ACQUIRING"):
            return "no resumable user pause; cancel and start a new task"
        error = self._pose_error(f, now)
        if error:
            return error
        if now < context["since"]:
            return "clock reversed during pause"
        old = context["pose"]
        if (math.hypot(f.pose.x-old.x, f.pose.y-old.y) > self.c.edge_progress_jump_m
                or abs(wrap(f.pose.yaw-old.yaw)) > math.radians(20)):
            return "pose drift exceeds resume limits; cancel and start a new task"
        wall_required = context["phase"] in self.WALL_PHASES + ("APPROACH_NEXT_EDGE",)
        if wall_required:
            w = f.wall
            if (w is None or not w.valid or w.stamp_ns <= context["since"]
                    or not 0 <= now-w.stamp_ns < self.c.wall_max_age_sec*1e9
                    or not math.isfinite(w.distance) or w.distance <= 0 or not math.isfinite(w.angle)):
                return "fresh post-pause target wall required"
        return ""

    def resume(self, f, now):
        error = self.resume_error(f, now)
        if error:
            return False, error
        context = self.pause_context
        duration = now-context["since"]
        # Shift execution timers, never sensor timestamps or command deadlines.
        for key in ("started_ns", "transfer_started_ns", "transfer_action_started_ns",
                    "transfer_turn_started_ns", "stage_since"):
            value = getattr(self, key, None)
            if value is not None:
                setattr(self, key, value+duration)
        self.phase, self.reason = context["phase"], "user resumed"
        self.pause_context, self.pause_kind = None, ""
        self.resume_sequence += 1
        self.last_update_ns, self.last_scan = now, -1
        self.last_effort = None
        self.last_contact_s = self.last_coverage_ns = self.wall_lost_since_ns = None
        if hasattr(self, "coverage_anchor"):
            self.coverage_anchor = self.arrived_since = None
        if self.edge_active:
            self.last_edge_s = self.edge.project(f.pose).s_m
        if hasattr(self, "wall_after_ns"):
            self.wall_after_ns = now
        return True, self.reason

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
        self.forward_boost_cutoff_reached = False
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
        self.reason = (f"approaching {self.edge.id}; target wall band "
                       f"{self.c.side_off_distance_m:.2f}--{self.c.side_on_distance_m:.2f} m")
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

    def _next_task_edge(self):
        return self.pool.next_edge(self.edge)

    def _begin_transfer(self, f, now):
        next_edge = self._next_task_edge()
        self._mark_edge_completed()
        if next_edge is None:
            self.phase = "COMPLETE"
            self.reason = "all configured edges completed"
            self.last_effort = self._stop_effort(now, f.pose)
            return
        self.transfer_next_edge = next_edge
        self.transfer_started_ns = now
        self.transfer_action_started_ns = None
        self.transfer_turn_target_yaw = next_edge.yaw_rad
        self.transfer_turn_started_ns = None
        self.transfer_turn_rad = wrap(next_edge.yaw_rad - self.edge.yaw_rad)
        if self.transfer_turn_rad > 0:
            # Project a nominal next-entry centre onto the current heading.
            # Keep a minimum forward exit; the following left shift is ended
            # by next-edge progress, not by distance to the next wall.
            entry = tuple(next_edge.start[i] + next_edge.tangent[i] * next_edge.entry_s_m
                          - next_edge.left_normal[i] * self.c.wall_target_m for i in (0, 1))
            along = sum((entry[i] - self.edge.end[i]) * self.edge.tangent[i] for i in (0, 1))
            self.left_exit_s_m = self.edge.length_m + max(self.c.left_exit_forward_m, along)
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
        self.transfer_turn_rad = 0.
        self.left_exit_s_m = None
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
        front_center_distance = self.front_remaining(f.pose)
        self._check_boost_cutoff(f)
        if front_center_distance <= self.c.front_stop_m or self._visible_front_stop(f.wall):
            self.halt("front stop threshold reached; outputs off, drift unmeasured", "STOPPED")
            self.last_effort = self._stop_effort(now, f.pose, f.wall)
            return self.last_effort
        if f.wall.stamp_ns <= self.last_scan:
            return self._without_new_scan(now)
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
        turn_direction = 0 if abs(w.angle) <= c.heading_correction_deadband_rad else (3 if w.angle > 0 else 4)
        self.phase = "CORRECT_HEADING" if turn_direction else "FOLLOW"
        self.reason = ("correcting heading; brush propulsion" if turn_direction and brush_aligned else
                       "correcting heading into brush band" if turn_direction else
                       "within heading band; brush propulsion")
        brush = c.brush_percent if brush_aligned else 0.
        effort = Effort(now, deadline, 0, turn_direction, side_output, brush)
        effort = self._with_forward_boost(effort, f)
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
        if self.phase in ("LEFT_EXIT_WALL", "LEFT_SHIFT_TO_ENTRY", "TURN_LEFT_TO_NEXT_EDGE"):
            return self._update_left_transfer(pose, now)
        if self.phase == "EDGE_STOP":
            if self.transfer_turn_rad > 1e-6:
                self.phase = "LEFT_EXIT_WALL"
                self.reason = "left transfer: moving forward beyond current wall endpoint"
                self.last_update_ns = now
                self.last_effort = self._stop_effort(now, pose)
                return self.last_effort
            if abs(self.transfer_turn_rad) <= 1e-6:
                self._finish_transfer_to_next_edge(pose, now)
                return self.last_effort
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
                side_left_percent=-self.c.transfer_side_percent, brush_percent=0.)
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

    def _update_left_transfer(self, pose, now):
        """Pose-only forward/left/turn sequence, without spatial clearance gates.

        The original right-transfer implementation above is unchanged. The
        old wall remains selected but disabled until the left turn finishes.
        """
        self.last_update_ns = now
        direction, side = 0, 0.
        if self.phase == "TURN_LEFT_TO_NEXT_EDGE":
            error = wrap(self.transfer_turn_target_yaw - pose.yaw)
            if abs(error) <= self.c.transfer_turn_tolerance_rad:
                self._finish_transfer_to_next_edge(pose, now)
                return self.last_effort
            if (self.c.transfer_turn_timeout_sec > 0 and
                    now - self.transfer_turn_started_ns >= self.c.transfer_turn_timeout_sec * 1e9):
                self.halt("left turn timeout")
                return None
            if error < 0:
                self.halt("next edge does not require the configured left turn")
                return None
            direction = 3
            self.reason = f"turning left; heading error {math.degrees(error):.1f} deg"
        else:
            # Keep the incoming heading during both translations. A heading
            # correction interrupts translation and inserts a stop cycle.
            heading = wrap(self.edge.yaw_rad - pose.yaw)
            if abs(heading) > self.c.heading_correction_deadband_rad:
                direction = 3 if heading > 0 else 4
                self.reason = "left transfer: holding incoming wall heading"
            elif self.phase == "LEFT_EXIT_WALL":
                p = self.edge.project(pose)
                if p.s_m >= self.left_exit_s_m - self.c.left_transfer_position_tolerance_m:
                    self.phase = "LEFT_SHIFT_TO_ENTRY"
                    self.reason = "left transfer: shifting left into next wall entry region"
                    self.last_effort = self._stop_effort(now, pose)
                    return self.last_effort
                direction = 1
                self.reason = "left transfer: moving forward beyond current wall endpoint"
            else:
                p = self.transfer_next_edge.project(pose)
                if p.s_m >= self.transfer_next_edge.entry_s_m - self.c.left_transfer_position_tolerance_m:
                    self.phase = "TURN_LEFT_TO_NEXT_EDGE"
                    self.transfer_turn_started_ns = now
                    self.reason = f"turning left toward {self.transfer_next_edge.id}"
                    self.last_effort = self._stop_effort(now, pose)
                    return self.last_effort
                side = self.c.side_percent
                self.reason = "left transfer: shifting left into next wall entry region"
        previous = self.last_effort
        if (previous and (previous.direction or previous.side_left_percent)
                and (previous.direction, previous.side_left_percent) != (direction, side)):
            self.last_effort = self._stop_effort(now, pose)
        else:
            self.last_effort = Effort(now, self._deadline(now, pose), direction=direction,
                                      side_left_percent=side)
        return self.last_effort

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
        self._check_boost_cutoff(f)
        if front_center_distance <= self.c.front_stop_m:
            self._begin_transfer(f, now)
            return self.last_effort

        if f.wall is None or f.wall.stamp_ns <= self.last_scan:
            return self._without_new_scan(now)
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
        turn_direction = 0 if abs(w.angle) <= c.heading_correction_deadband_rad else (3 if w.angle > 0 else 4)
        if not wall_in_band:
            self.phase = "APPROACH_WALL"
            self.reason = "adjusting centre-to-left-wall distance"
        elif projection.s_m < self.edge.entry_s_m:
            self.phase = "APPROACH_EDGE"
            self.reason = "wall stable; approaching selected edge entry"
        else:
            self.phase = "CORRECT_HEADING" if turn_direction else "FOLLOW"
            self.reason = ("correcting heading; brush propulsion" if turn_direction and brush_aligned else
                           "correcting heading into brush band" if turn_direction else
                           "within configured wall band; brush propulsion")
        brush = c.brush_percent if wall_in_band and brush_aligned else 0.
        effort = Effort(now, deadline, 0, turn_direction, side_output, brush)
        effort = self._with_forward_boost(effort, f)
        self.last_effort = effort
        if brush and projection.s_m >= self.edge.clean_start_s_m:
            self._record_coverage(projection, now)
        else:
            self.last_coverage_ns = None
            self.last_contact_s = projection.contact_s_m
        return effort
