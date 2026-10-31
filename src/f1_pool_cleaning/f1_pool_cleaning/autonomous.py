"""Left-ray acquisition with pose feedback, followed by the legacy wall control.

Acquisition uses turn/forward/reverse/side commands only. It does not read an
occupancy grid, search a path or gate motion on hull/brush/laser collisions.
Rectangle routes close once to clean the starting prefix; open routes never
return. Lease, pose freshness and wall feedback remain owned by the adapters
and the original Mission state machine.
"""
from dataclasses import dataclass
import math

from .coverage import merge_intervals, missing_intervals
from .geometry import wrap
from .mission import Mission, Effort


@dataclass(frozen=True)
class ApproachConfig:
    along_tolerance_m: float = .10
    heading_tolerance_rad: float = math.radians(20.)
    arrival_hold_sec: float = .20
    corner_tolerance_m: float = .01
    coverage_tolerance_m: float = .05
    coverage_overlap_m: float = .10

    def validate(self, control):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError("approach parameters must be positive and finite")
        if (self.heading_tolerance_rad >= control.edge_heading_tolerance_rad
                or self.along_tolerance_m >= control.edge_select_max_distance_m/2
                or self.coverage_overlap_m < self.coverage_tolerance_m):
            raise ValueError("invalid approach heading, position or coverage tolerances")
        return self


class AutonomousMission(Mission):
    APPROACH_PHASES = ("APPROACH_START", "ALIGN_START", "MOVE_START_ALONG", "MOVE_START_SIDE")

    def __init__(self, config, pool, approach=ApproachConfig()):
        super().__init__(config)
        self.pool, self.a = pool, approach.validate(config)
        self.closed = math.dist(pool.edges[-1].end, pool.edges[0].start) <= 1e-8
        self.requested_edge_id = None
        self.target_generation = 0
        self.required = {}
        self.start_edge_id = None
        self.start_s = None
        self.nearest_wall_point = None
        self.remaining_edges = []
        self.finished_edges = []
        self.first_contact = None
        self.prefix_end = None
        self.returning_to_start = False
        self.coverage_anchor = None
        self.arrived_since = None
        self.wall_after_ns = 0
        self.stage_since = 0

    @property
    def active(self):
        return self.phase in self.APPROACH_PHASES + ("ACQUIRE_WALL",) or super().active

    @property
    def wall_required(self):
        return super().wall_required

    @property
    def target_wall_enabled(self):
        return (self.edge is not None and self.phase not in self.APPROACH_PHASES
                + ("ACQUIRING", "IDLE", "COMPLETE", "CANCELLED", "STOPPED"))

    def close(self):
        pass  # no background planning worker

    def halt(self, reason, phase="PAUSED"):
        super().halt(reason, phase)
        self.coverage_anchor = None
        self.arrived_since = None

    def configure_edge(self, pool, edge, task_id=""):
        if self.active or self.phase == "ACQUIRING":
            return False, "cancel before selecting a new start edge"
        if pool is not self.pool or pool.by_id.get(edge.id) is not edge:
            return False, "unknown task edge"
        self.requested_edge_id = edge.id
        return True, f"next task starts on {edge.id}"

    def error(self, f, now, allow_invalid_wall=False):
        error = self._pose_error(f, now)
        if error or not self.wall_required:
            return error
        return super().error(f, now, allow_invalid_wall)

    def start(self, f, now, allow_wall_recovery=False):
        if self.phase not in ("IDLE", "CANCELLED", "STOPPED", "COMPLETE"):
            return False, "cancel before restarting; paused tasks never resume automatically"
        error = self._pose_error(f, now)
        if error:
            return False, error
        try:
            edge = (self.pool.by_id[self.requested_edge_id] if self.requested_edge_id else
                    self.pool.select_left_ray(f.pose, self.a.corner_tolerance_m))
        except (KeyError, ValueError) as exc:
            return False, str(exc)
        projection = edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        nearest_s = min(edge.length_m, max(0., projection.s_m))
        low = edge.entry_s_m
        high = edge.length_m-self.c.front_stop_m-self.a.along_tolerance_m-.01
        if high < low:
            return False, f"{edge.id} has no starting interval within the endpoint margins"
        self.edge, self.start_edge_id = edge, edge.id
        self.start_s = min(high, max(low, nearest_s))
        self.nearest_wall_point = [edge.start[i]+edge.tangent[i]*nearest_s for i in (0, 1)]
        self._reset_route()
        index = self.pool.edges.index(edge)
        ordered = list(self.pool.edges[index:])
        if self.closed:
            ordered += list(self.pool.edges[:index])
        self.remaining_edges = [e.id for e in ordered[1:]]
        self.required = {e.id: (e.clean_start_s_m,
                               min(e.clean_end_s_m, e.length_m-self.c.front_stop_m+self.c.brush_x_m))
                         for e in ordered}
        self.finished_edges = []
        self.first_contact = self.prefix_end = None
        self.returning_to_start = False
        self.started_ns = self.last_update_ns = now
        self.target_generation += 1
        return self.resume_after_acquire(f, now)

    def resume_after_acquire(self, f, now):
        """Refresh feedback after the lease without changing the locked target."""
        error = self._pose_error(f, now)
        if error:
            return False, error
        self.halt(f"start wall {self.start_edge_id} locked; approaching starting range", "APPROACH_START")
        self.stage_since = self.last_update_ns = now
        self.last_effort = self._stop_effort(now, f.pose)
        return True, "task accepted; approaching the selected wall's starting range"

    def _inside_start(self, projection):
        return (abs(projection.heading_error_rad) <= self.a.heading_tolerance_rad
                and abs(projection.s_m-self.start_s) <= self.a.along_tolerance_m
                and 0 < projection.lateral_distance_m
                and projection.left_offset_m > 0
                and projection.distance_to_segment_m <= self.c.edge_select_max_distance_m
                and self.edge.length_m-projection.s_m > self.c.front_stop_m)

    def _approach(self, f, now):
        p = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        self.edge_projection = p
        if self._inside_start(p):
            if self.arrived_since is None:
                self.arrived_since = now
            elif now-self.arrived_since >= self.a.arrival_hold_sec*1e9:
                self.phase, self.reason = "ACQUIRE_WALL", "in starting range; waiting for a fresh target wall"
                self.wall_after_ns = self.stage_since = now
                self.target_generation += 1
            return self._stop_effort(now, f.pose)
        self.arrived_since = None
        along = self.start_s-p.s_m
        direction, side = 0, 0.
        # A small exit band prevents action chatter. Exact entry bounds still
        # gate handoff; the controller never requires an exact target point.
        angle_limit = self.a.heading_tolerance_rad
        if self.phase in ("MOVE_START_ALONG", "MOVE_START_SIDE"):
            angle_limit += math.radians(5.)
        if abs(p.heading_error_rad) > angle_limit:
            phase, reason = "ALIGN_START", "aligning with the selected wall"
            direction = 3 if p.heading_error_rad > 0 else 4
        elif abs(along) > self.a.along_tolerance_m:
            phase, reason = "MOVE_START_ALONG", "adjusting along-wall starting position"
            direction = 1 if along > 0 else 2
        elif p.lateral_distance_m <= 0 or p.distance_to_segment_m > self.c.edge_select_max_distance_m:
            phase, reason = "MOVE_START_SIDE", "moving sideways into the starting range"
            side = self.c.side_percent*(1 if p.lateral_distance_m > 0 else -1)
        else:
            phase, reason = "ALIGN_START", "aligning for wall-control handoff"
            direction = 3 if p.heading_error_rad > 0 else 4
        previous = self.last_effort
        changing_motion = (previous is not None and (previous.direction or previous.side_left_percent)
                           and (previous.direction, previous.side_left_percent) != (direction, side))
        if phase != self.phase or changing_motion:
            self.phase, self.reason = phase, reason
            return self._stop_effort(now, f.pose)
        return Effort(now, self._deadline(now, f.pose), direction=direction, side_left_percent=side)

    def _acquire_wall(self, f, now):
        p = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        if not self._inside_start(p):
            self.phase, self.reason = "APPROACH_START", "drifted outside starting range; correcting pose"
            self.arrived_since = None
            self.target_generation += 1
            return self._stop_effort(now, f.pose)
        wall = f.wall
        fresh = (wall is not None and wall.valid and wall.stamp_ns > self.wall_after_ns
                 and 0 <= now-wall.stamp_ns < self.c.wall_max_age_sec*1e9
                 and math.isfinite(wall.distance) and wall.distance > 0 and math.isfinite(wall.angle))
        if not fresh:
            if now-self.stage_since >= self.c.wall_recovery_sec*1e9:
                self.halt("target wall acquisition timed out")
                return None
            return self._stop_effort(now, f.pose)
        # The old _start_edge geometry gates were satisfied by _inside_start.
        # Keep the run's route/coverage state and hand over to _update_edge.
        self._reset_edge_progress()
        self.phase, self.reason = "APPROACH_WALL", "target wall confirmed; legacy wall control"
        self.last_scan = -1
        self.wall_lost_since_ns = None
        return self._stop_effort(now, f.pose, wall)

    def _record_coverage(self, projection, now):
        pass  # record the previous effective brush command in _coverage instead

    def _first_brush(self, f, effort):
        if self.first_contact is not None or effort is None or effort.brush_percent <= 0:
            return
        contact = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m).contact_s_m
        self.first_contact = (self.edge.id, contact)
        low, high = self.required[self.edge.id]
        if self.closed and contact > low+self.a.coverage_tolerance_m:
            self.prefix_end = min(high, contact+self.a.coverage_overlap_m)
            self.remaining_edges.append(self.start_edge_id)
        elif not self.closed:
            self.required[self.edge.id] = (min(high, max(low, contact)), high)
        self.coverage_anchor = (contact, f.pose.stamp_ns)

    def _coverage(self, f, now):
        p = self.edge.project(f.pose, self.c.brush_x_m, self.c.brush_tip_y_m)
        wall, previous = f.wall, self.last_effort
        good = (wall is not None and wall.valid and previous is not None
                and previous.brush_percent > 0 and now < previous.valid_until_ns
                and self.c.side_off_distance_m <= wall.distance <= self.c.side_on_distance_m
                and abs(wall.angle) <= self.c.heading_deadband_rad)
        if good:
            if self.coverage_anchor is not None:
                contact, stamp = self.coverage_anchor
                if (0 < f.pose.stamp_ns-stamp <= 250_000_000
                        and 0 <= p.contact_s_m-contact <= self.c.edge_progress_jump_m):
                    low, high = self.required[self.edge.id]
                    intervals = self.cleaned_by_edge.get(self.edge.id, [])
                    self.cleaned_by_edge[self.edge.id] = merge_intervals(
                        intervals+[(max(low, contact), min(high, p.contact_s_m))])
            self.coverage_anchor = (p.contact_s_m, f.pose.stamp_ns)
        else:
            self.coverage_anchor = None
        self.cleaned_intervals = list(self.cleaned_by_edge.get(self.edge.id, []))
        self._mark_edge_completed()
        return p

    def _mark_edge_completed(self):
        self.completed_edges = [edge_id for edge_id, bounds in self.required.items()
                                if not missing_intervals(bounds, self.cleaned_by_edge.get(edge_id, []),
                                                         self.a.coverage_tolerance_m)]

    def _next_task_edge(self):
        return self.pool.by_id[self.remaining_edges[0]] if self.remaining_edges else None

    def _complete(self, f, now):
        self.halt("rectangle route and starting prefix finished; inspect coverage gaps" if self.closed else
                  "remaining L route finished; no return or repair", "COMPLETE")
        self.last_effort = self._stop_effort(now, f.pose)

    def _begin_transfer(self, f, now):
        self.finished_edges.append(self.edge.id)
        self.coverage_anchor = None
        if not self.remaining_edges:
            self._complete(f, now)
        else:
            super()._begin_transfer(f, now)

    def _finish_transfer_to_next_edge(self, pose, now):
        next_id = self.remaining_edges.pop(0)
        super()._finish_transfer_to_next_edge(pose, now)
        assert self.edge.id == next_id
        self.returning_to_start = self.closed and self.edge.id == self.start_edge_id
        self.cleaned_intervals = list(self.cleaned_by_edge.get(self.edge.id, []))
        self.coverage_anchor = None
        self.target_generation += 1

    def update(self, f, now):
        if not self.active:
            return None
        error = self.error(f, now, allow_invalid_wall=True)
        if error:
            self.halt(error)
            return None
        if now < self.last_update_ns or (self.c.run_timeout_sec > 0
                                        and now-self.started_ns >= self.c.run_timeout_sec*1e9):
            self.halt("clock reversed or run timeout")
            return None
        self.last_update_ns = now
        if self.phase in self.APPROACH_PHASES:
            effort = self._approach(f, now)
        elif self.phase == "ACQUIRE_WALL":
            effort = self._acquire_wall(f, now)
        elif self.phase in self.TRANSFER_PHASES and self.phase != "APPROACH_NEXT_EDGE":
            effort = super()._update_edge(f, now)
        else:
            p = self._coverage(f, now)
            if (self.returning_to_start and self.prefix_end is not None
                    and p.contact_s_m >= self.prefix_end-self.a.coverage_tolerance_m/2):
                self.finished_edges.append(self.edge.id)
                self._complete(f, now)
                effort = self.last_effort
            else:
                effort = super()._update_edge(f, now)
                self._first_brush(f, effort)
        if effort is not None:
            self.last_effort = effort
        return effort

    def start_region(self):
        if self.start_edge_id is None:
            return None
        edge = self.pool.by_id[self.start_edge_id]
        def point(along, distance):
            return [edge.start[i]+edge.tangent[i]*along-edge.left_normal[i]*distance for i in (0, 1)]
        low, high = self.start_s-self.a.along_tolerance_m, self.start_s+self.a.along_tolerance_m
        maximum = self.c.edge_select_max_distance_m
        return dict(edge_id=edge.id, nearest_wall_point=self.nearest_wall_point,
                    reference_wall_point=point(self.start_s, 0.), heading_rad=edge.yaw_rad,
                    along_tolerance_m=self.a.along_tolerance_m, maximum_wall_distance_m=maximum,
                    polygon=[point(low, 0.), point(high, 0.), point(high, maximum), point(low, maximum)])

    def status(self):
        gaps = {edge_id: missing_intervals(bounds, self.cleaned_by_edge.get(edge_id, []),
                                           self.a.coverage_tolerance_m)
                for edge_id, bounds in self.required.items()}
        return dict(automatic_start=True,
                    acquisition_mode="specified_direct" if self.requested_edge_id else "left_ray_direct",
                    requested_start_edge=self.requested_edge_id, start_edge_id=self.start_edge_id,
                    start_region=self.start_region(), first_brush_contact=self.first_contact,
                    required_contact_intervals=self.required, remaining_contact_intervals=gaps,
                    coverage_complete=bool(self.required) and not any(gaps.values()),
                    returning_to_start=self.returning_to_start, remaining_edges=list(self.remaining_edges),
                    executed_edge_ids=list(dict.fromkeys(self.finished_edges)),
                    completion_scope="rectangle_route" if self.closed else "remaining_l_route")
