"""Left-preferred wall acquisition and complete interval coverage of a task route.

Planning runs off the control thread. While waiting, the controller keeps
publishing STOP with the ordinary lease and deadlines. Transit never brushes.
"""
from concurrent.futures import ThreadPoolExecutor
import math

from .coverage import CleaningLeg, initial_legs, merge_intervals, missing_intervals
from .geometry import wrap
from .mission import Mission, Effort
from .navigation import PlanningError, Waypoint


class AutonomousMission(Mission):
    NAV_PHASES = ("PLANNING", "NAVIGATING", "SETTLING", "ACQUIRE_WALL", "DOCK_WALL")

    def __init__(self, config, pool, navigator):
        super().__init__(config)
        self.pool, self.navigator, self.n = pool, navigator, navigator.n
        self.navigator.candidate_edges = pool.edges
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="wall-navigation")
        self.future = None
        self.requested_edge_id = None
        self.target_generation = 0
        self.required = {}
        self.legs = []
        self.leg = None
        self.path = []
        self.path_index = 0
        self.repair_pass = 0
        self.stage_since = 0
        self.previous_pose = None
        self.coverage_anchor = None
        self.first_contact = None
        self.wall_after_ns = 0
        self.settle_since = None
        self.best_progress = math.inf
        self.progress_since = 0
        self.replan_goal = None
        self.replans = 0

    @property
    def active(self):
        return self.phase in self.NAV_PHASES or super().active

    @property
    def wall_required(self):
        if self.phase in self.TRANSFER_PHASES:
            return super().wall_required
        return self.phase not in ("PLANNING", "NAVIGATING", "SETTLING", "ACQUIRE_WALL", "ACQUIRING", "IDLE")

    @property
    def target_wall_enabled(self):
        return self.phase not in ("PLANNING", "NAVIGATING", "SETTLING", "COMPLETE", "CANCELLED")

    def close(self):
        if self.future is not None:
            self.future.cancel()
        self.executor.shutdown(wait=False, cancel_futures=True)

    def halt(self, reason, phase="PAUSED"):
        super().halt(reason, phase)
        self.coverage_anchor = None
        if self.future is not None:
            self.future.cancel()
            self.future = None

    def configure_edge(self, pool, edge, task_id=""):
        if self.active or self.phase == "ACQUIRING":
            return False, "cancel before selecting a new start edge"
        if pool is not self.pool or pool.by_id.get(edge.id) is not edge:
            return False, "unknown task edge"
        self.requested_edge_id = edge.id
        return True, f"next task starts on {edge.id}"

    def scan_error(self, f, now):
        scan = getattr(f, "scan", None)
        if scan is None or scan.stamp_ns <= 0 or not 0 <= now-scan.stamp_ns < self.n.scan_max_age_sec*1e9:
            return "fresh navigation laser scan required"
        return ""

    def error(self, f, now, allow_invalid_wall=False):
        error = self._pose_error(f, now) or self.scan_error(f, now)
        if error or not self.wall_required:
            return error
        return super().error(f, now, allow_invalid_wall)

    def start(self, f, now, allow_wall_recovery=False):
        if self.phase not in ("IDLE", "CANCELLED", "STOPPED", "COMPLETE"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self._pose_error(f, now) or self.scan_error(f, now)
        if error:
            return False, error
        self.halt("selecting a reachable start edge", "PLANNING")
        self.started_ns = self.last_update_ns = self.stage_since = now
        self.previous_pose = f.pose
        self.required, self.cleaned_by_edge, self.completed_edges = {}, {}, []
        self.cleaned_intervals, self.legs, self.path = [], [], []
        self.edge = self.leg = self.first_contact = None
        self.repair_pass = 0
        self.replans = 0
        self.replan_goal = None
        self.plan_start = f.pose
        self.future = None  # submitted only after a hardware lease is granted
        self.last_effort = self._stop_effort(now, f.pose)
        return True, "automatic task accepted; inspect status for planning and wall acquisition"

    def _initial_plan(self, pose, requested):
        nav = self.navigator
        required = {edge.id: nav.usable_interval(edge) for edge in self.pool.edges}
        candidates = []
        for edge in self.pool.edges:
            if requested and edge.id != requested:
                continue
            low, high = required[edge.id]
            projection = edge.project(pose, self.c.brush_x_m, self.c.brush_tip_y_m)
            contact = min(high-self.n.coverage_overlap_m, max(low, projection.contact_s_m))
            goal = nav.edge_pose(edge, contact, self.n.approach_distance_m)
            # Prefer walls on the current brush side (base_link +y), then
            # distance and heading. Ignore numerical noise at the fore/aft
            # axis; non-left walls remain candidates if left ones cannot be reached.
            non_left = projection.left_offset_m <= 1e-6
            candidates.append((non_left, projection.distance_to_segment_m,
                               abs(wrap(edge.yaw_rad-pose.yaw)), edge.id, edge, contact, goal))
        failures = []
        for _, _, _, _, edge, contact, goal in sorted(candidates, key=lambda v: v[:4]):
            try:
                path = nav.plan(pose, goal)
            except PlanningError as exc:
                failures.append(f"{edge.id}: {exc}")
                continue
            legs = initial_legs(self.pool.edges, required, edge.id, contact, self.n.coverage_overlap_m)
            return required, legs, path
        raise PlanningError("no reachable task wall: "+"; ".join(failures))

    def _set_leg(self, leg):
        self.leg = leg
        self.edge = self.pool.by_id[leg.edge_id]
        self.edge_index = self.pool.edges.index(self.edge)
        self.cleaned_intervals = list(self.cleaned_by_edge.get(leg.edge_id, []))
        self._reset_edge_progress()
        self.cleaned_intervals = list(self.cleaned_by_edge.get(leg.edge_id, []))
        self.coverage_anchor = None
        self.target_generation += 1

    def _planning(self, f, now):
        if self.future is None:
            self.plan_start = f.pose
            if not self.required:
                self.future = self.executor.submit(self._initial_plan, f.pose, self.requested_edge_id)
            elif self.replan_goal is not None:
                self.future = self.executor.submit(self.navigator.plan, f.pose, self.replan_goal,
                                                   self.departure_edge)
            else:
                goal = self.navigator.edge_pose(self.edge, self.leg.start, self.n.approach_distance_m)
                self.future = self.executor.submit(self.navigator.plan, f.pose, goal, self.departure_edge)
        if not self.future.done():
            return self._stop_effort(now, f.pose)
        try:
            result = self.future.result()
        except Exception as exc:
            self.halt(f"route planning failed: {exc}")
            return None
        self.future = None
        if (math.hypot(f.pose.x-self.plan_start.x, f.pose.y-self.plan_start.y) > self.n.position_tolerance_m
                or abs(wrap(f.pose.yaw-self.plan_start.yaw)) > self.n.heading_tolerance_rad):
            self.reason = "boat moved during planning; replan from current pose"
            return self._stop_effort(now, f.pose)
        if not self.required:
            self.required, self.legs, self.path = result
            self._set_leg(self.legs.pop(0))
        else:
            self.path = result
            self.replan_goal = None
        self.path_index = 0
        self.phase, self.reason = "NAVIGATING", f"brush off; approaching {self.edge.id}"
        self.stage_since = self.progress_since = now
        self.best_progress = math.inf
        return self._stop_effort(now, f.pose)

    def _guarded(self, f, now, direction=0, side=0., contact=False, travel=None, edge=None):
        p = f.pose
        yaw = p.yaw
        distance = self.n.lookahead_m if travel is None else min(self.n.lookahead_m, max(.01, travel))
        dx = dy = 0.
        if direction in (1,2):
            sign = 1 if direction == 1 else -1
            dx, dy = sign*distance*math.cos(yaw), sign*distance*math.sin(yaw)
        if side:
            sign = 1 if side > 0 else -1
            dx += -sign*distance*math.sin(yaw)
            dy += sign*distance*math.cos(yaw)
        if direction in (3,4):
            yaw += math.radians(8.)*(1 if direction == 3 else -1)
        target = Waypoint(p.x+dx,p.y+dy,yaw)
        turning = direction in (3, 4)
        if (not turning
                and not self.navigator.motion_clear(p, target, f.scan, contact, edge or self.edge)):
            return None
        return Effort(now, min(self._deadline(now,p), f.scan.stamp_ns+round(self.n.scan_max_age_sec*1e9)),
                      direction=direction, side_left_percent=side)

    def _navigate(self, f, now):
        if self.path_index >= len(self.path):
            self.phase, self.reason = "SETTLING", "at approach pose; waiting for pose to settle"
            self.settle_since = None
            return self._stop_effort(now, f.pose)
        target, pose = self.path[self.path_index], f.pose
        contact_edge = self.pool.by_id.get(target.contact_edge_id)
        dx, dy = target.x-pose.x, target.y-pose.y
        distance, angle = math.hypot(dx,dy), wrap(target.yaw-pose.yaw)
        # Do not discard a short undocking/sideways step just because it is
        # inside the general arrival tolerance: that step may clear the wall
        # before a long translation parallel to it.
        tolerance = min(self.n.position_tolerance_m, .01)
        if distance <= tolerance and abs(angle) <= self.n.heading_tolerance_rad:
            self.path_index += 1
            self.best_progress = math.inf
            self.progress_since = now
            return self._stop_effort(now, pose)
        metric = distance+.3*abs(angle)
        if metric < self.best_progress-.01:
            self.best_progress, self.progress_since = metric, now
        elif now-self.progress_since > self.n.no_progress_sec*1e9:
            self.halt("navigation made no progress")
            return None
        # Keep heading feedback aligned before translating near a wall.
        if abs(angle) > self.n.heading_tolerance_rad:
            return self._guarded(f, now, direction=3 if angle > 0 else 4,
                                 contact=target.contact, edge=contact_edge)
        along = dx*math.cos(pose.yaw)+dy*math.sin(pose.yaw)
        across = -dx*math.sin(pose.yaw)+dy*math.cos(pose.yaw)
        command = None
        if distance > tolerance:
            if abs(along) >= abs(across):
                command = dict(direction=1 if along > 0 else 2, travel=abs(along))
            else:
                command = dict(side=self.c.side_percent*(1 if across > 0 else -1),
                               travel=abs(across))
        if command is not None:
            effort = self._guarded(f,now,contact=target.contact,edge=contact_edge,**command)
            if effort is None:
                self.replans += 1
                if self.replans > 3:
                    self.halt("translation blocked after bounded replanning attempts")
                    return None
                self.replan_goal, self.departure_edge = self.path[-1], contact_edge or self.edge
                self.phase, self.future = "PLANNING", None
                self.reason = "live pose differs from path; replanning to next safe waypoint"
                self.stage_since = now
                return self._stop_effort(now,pose)
            return effort
        self.halt("waypoint position error cannot be reduced with available motions")
        return None

    def _settle(self, f, now, previous):
        goal = self.navigator.edge_pose(self.edge, self.leg.start, self.n.approach_distance_m)
        if (math.hypot(f.pose.x-goal.x,f.pose.y-goal.y) > 2*self.n.position_tolerance_m
                or abs(wrap(f.pose.yaw-goal.yaw)) > 2*self.n.heading_tolerance_rad):
            self.departure_edge = self.edge
            self.phase, self.future = "PLANNING", None
            return self._stop_effort(now,f.pose)
        if previous and f.pose.stamp_ns > previous.stamp_ns:
            dt = (f.pose.stamp_ns-previous.stamp_ns)/1e9
            stable = (math.hypot(f.pose.x-previous.x,f.pose.y-previous.y)/dt <= self.n.settled_speed_mps
                      and abs(wrap(f.pose.yaw-previous.yaw))/dt <= self.n.settled_yaw_rate_radps)
            if not stable:
                self.settle_since = None
            elif self.settle_since is None:
                self.settle_since = now
            elif now-self.settle_since >= self.n.settle_sec*1e9:
                self.phase, self.reason = "ACQUIRE_WALL", "waiting for a fresh target-wall observation"
                self.wall_after_ns = self.stage_since = now
                self.target_generation += 1
        return self._stop_effort(now,f.pose)

    def _dock(self, f, now):
        w = f.wall
        if (w is None or not w.valid or w.stamp_ns <= self.wall_after_ns
                or not 0 <= now-w.stamp_ns < self.c.wall_max_age_sec*1e9):
            if now-self.stage_since > self.n.wall_acquire_timeout_sec*1e9:
                self.halt("target wall acquisition timed out")
                return None
            return self._stop_effort(now,f.pose)
        if not all(math.isfinite(v) for v in (w.angle,w.distance)) or w.distance <= 0:
            self.halt("invalid target wall measurement")
            return None
        # Tight alignment precedes side motion. The map and observed wall must
        # agree; the perception target filter already enforces that identity.
        # Once acquired, use the deployed wall controller's distance/heading
        # commands. Raster wall thickness must not prevent physical contact.
        deadline = min(self._deadline(now, f.pose, w),
                       f.scan.stamp_ns + round(self.n.scan_max_age_sec*1e9))
        if abs(w.angle) > self.c.heading_correction_deadband_rad:
            self.phase = "DOCK_WALL"
            return Effort(now, deadline, direction=3 if w.angle > 0 else 4)
        if not self.c.side_off_distance_m <= w.distance <= self.c.side_on_distance_m:
            self.phase = "DOCK_WALL"
            side = self.c.side_percent*(1 if w.distance > self.c.side_on_distance_m else -1)
            return Effort(now, deadline, side_left_percent=side)
        self.phase, self.reason = "FOLLOW", f"cleaning {self.edge.id} interval"
        self.last_edge_s = None
        self.last_scan = -1
        self.wall_lost_since_ns = None
        self.coverage_anchor = None
        return self._stop_effort(now,f.pose)

    def _record_coverage(self, projection, now):
        # Coverage is integrated from the *previous* still-valid brush command,
        # never from a new command applied retroactively to the last interval.
        pass

    def _coverage(self, f, now):
        projection = self.edge.project(f.pose,self.c.brush_x_m,self.c.brush_tip_y_m)
        w, effort = f.wall, self.last_effort
        good = (w is not None and w.valid and effort is not None and effort.brush_percent > 0
                and now < effort.valid_until_ns
                and self.c.side_off_distance_m <= w.distance <= self.c.side_on_distance_m
                and abs(w.angle) <= self.c.heading_deadband_rad)
        if good:
            contact = projection.contact_s_m
            if self.first_contact is None:
                self.first_contact = (self.edge.id, contact)
            if self.coverage_anchor is not None:
                last, stamp = self.coverage_anchor
                if 0 < f.pose.stamp_ns-stamp <= 250_000_000 and 0 <= contact-last <= self.c.edge_progress_jump_m:
                    lo, hi = self.required[self.edge.id]
                    intervals = self.cleaned_by_edge.get(self.edge.id, [])+[(max(lo,last),min(hi,contact))]
                    self.cleaned_by_edge[self.edge.id] = merge_intervals(intervals)
            self.coverage_anchor = (contact, f.pose.stamp_ns)
        else:
            self.coverage_anchor = None
        self.cleaned_intervals = list(self.cleaned_by_edge.get(self.edge.id, []))
        self.completed_edges = [edge.id for edge in self.pool.edges if not missing_intervals(
            self.required[edge.id],self.cleaned_by_edge.get(edge.id,[]),self.n.coverage_tolerance_m)]
        return projection

    def _begin_transfer(self, f, now):
        return self._finish_leg(f,now)

    def _next_task_edge(self):
        if not self.legs:
            return None
        candidate = self.pool.by_id[self.legs[0].edge_id]
        # Includes the closing corner of a rectangle started on any edge.
        # The endpoints of an open L never meet here.
        if math.dist(self.edge.end, candidate.start) <= 1e-6:
            return candidate
        return None

    def _mark_edge_completed(self):
        # Finishing a leg is not evidence that its earlier prefix was cleaned.
        self.completed_edges = [edge.id for edge in self.pool.edges if not missing_intervals(
            self.required[edge.id], self.cleaned_by_edge.get(edge.id, []), self.n.coverage_tolerance_m)]

    def _finish_leg(self, f, now):
        self.coverage_anchor = None
        # Keep the established reverse/right-push and closed-loop right turn
        # for adjacent route edges. Navigation is reserved for acquisition,
        # open-route returns, and repair legs that cannot use that transition.
        next_leg = self.legs[0] if self.legs else None
        next_edge = self._next_task_edge()
        at_edge_end = self.leg.end >= self.required[self.edge.id][1] - self.n.coverage_tolerance_m
        # Retain the original base_link-centre stopping distance on full legs.
        # Contact coverage ends earlier because the brush is ahead of centre.
        if next_leg and next_edge and next_leg.edge_id == next_edge.id and at_edge_end:
            projection = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
            if self.edge.length_m-projection.s_m > self.c.front_stop_m:
                return super()._update_edge(f, now)
            super()._begin_transfer(f, now)
            return self.last_effort
        if not self.legs:
            missing = [CleaningLeg(edge.id, max(self.required[edge.id][0],a-self.n.coverage_overlap_m),
                                   min(self.required[edge.id][1],b+self.n.coverage_overlap_m))
                       for edge in self.pool.edges
                       for a,b in missing_intervals(self.required[edge.id],self.cleaned_by_edge.get(edge.id,[]),
                                                    self.n.coverage_tolerance_m)]
            if not missing:
                self.halt("all reachable cleaning intervals covered", "COMPLETE")
                self.last_effort = self._stop_effort(now,f.pose)
                return self.last_effort
            if self.repair_pass >= self.n.repair_passes:
                self.halt("coverage gaps remain after bounded repair passes")
                return None
            self.repair_pass += 1
            self.legs = missing
        self.departure_edge = self.edge
        self.replans = 0
        self._set_leg(self.legs.pop(0))
        self.phase, self.reason = "PLANNING", f"brush off; planning transit to {self.edge.id}"
        self.future = None
        self.stage_since = now
        self.last_effort = self._stop_effort(now,f.pose)
        return self.last_effort

    def update(self, f, now):
        if not self.active:
            return None
        error = self.error(f,now,allow_invalid_wall=True)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or (self.c.run_timeout_sec > 0 and now-self.started_ns >= self.c.run_timeout_sec*1e9):
            self.halt("clock reversed or task timeout")
            return None
        previous = self.previous_pose
        if previous and (math.hypot(f.pose.x-previous.x,f.pose.y-previous.y) > self.c.edge_progress_jump_m
                         or abs(wrap(f.pose.yaw-previous.yaw)) > math.radians(25.)):
            self.halt("pose jumped during automatic task")
            return None
        self.previous_pose, self.last_update_ns = f.pose, now
        if self.phase in self.NAV_PHASES and now-self.stage_since > self.n.timeout_sec*1e9:
            self.halt("navigation/acquisition timeout")
            return None
        if self.phase in self.TRANSFER_PHASES:
            old_edge = self.edge
            effort = super()._update_edge(f, now)
            if self.edge is not old_edge:
                if not self.legs or self.legs[0].edge_id != self.edge.id:
                    self.halt("legacy edge transfer did not match the planned cleaning route")
                    return None
                self._set_leg(self.legs.pop(0))
            if effort is not None:
                self.last_effort = effort
            return effort
        if self.phase == "PLANNING":
            effort = self._planning(f,now)
        elif self.phase == "NAVIGATING":
            effort = self._navigate(f,now)
        elif self.phase == "SETTLING":
            effort = self._settle(f,now,previous)
        elif self.phase in ("ACQUIRE_WALL", "DOCK_WALL"):
            effort = self._dock(f,now)
        else:
            projection = self._coverage(f,now)
            self.edge_projection = projection
            if projection.contact_s_m >= self.leg.end-self.n.coverage_tolerance_m/2:
                effort = self._finish_leg(f,now)
            else:
                effort = super()._update_edge(f,now)
        if effort is not None:
            self.last_effort = effort
        return effort

    def status(self):
        return dict(automatic_start=True, requested_start_edge=self.requested_edge_id,
                    first_brush_contact=self.first_contact, required_contact_intervals=self.required,
                    remaining_contact_intervals={edge: missing_intervals(bounds,self.cleaned_by_edge.get(edge,[]),
                                                                         self.n.coverage_tolerance_m)
                                                 for edge,bounds in self.required.items()},
                    active_cleaning_interval=None if self.leg is None else vars(self.leg),
                    remaining_legs=[vars(leg) for leg in self.legs], repair_pass=self.repair_pass,
                    navigation_waypoint=self.path_index, navigation_waypoint_count=len(self.path))
