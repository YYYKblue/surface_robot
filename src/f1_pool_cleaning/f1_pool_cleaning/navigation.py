"""Map-space transit for a boat with discrete forward/turn/side commands.

Unknown cells are blocked. Translation uses swept hull and brush checks.
In-place turns retain the deployed heading control without collision gates.
"""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

from .geometry import wrap


@dataclass(frozen=True)
class NavigationConfig:
    approach_distance_m: float = .60
    clearance_m: float = .01
    step_m: float = .15
    position_tolerance_m: float = .04
    heading_tolerance_rad: float = math.radians(1.)
    scan_max_age_sec: float = .25
    lookahead_m: float = .15
    settle_sec: float = .30
    settled_speed_mps: float = .04
    settled_yaw_rate_radps: float = .15
    timeout_sec: float = 180.
    no_progress_sec: float = 8.
    wall_acquire_timeout_sec: float = 5.
    plan_timeout_sec: float = 8.
    max_expansions: int = 60000
    coverage_tolerance_m: float = .05
    coverage_overlap_m: float = .10
    repair_passes: int = 2

    def validate(self, control):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"navigation_{name} must be positive and finite")
        if (self.approach_distance_m <= control.side_on_distance_m
                or self.approach_distance_m >= control.edge_select_max_distance_m
                or self.position_tolerance_m >= self.step_m / 2
                or self.heading_tolerance_rad >= control.heading_deadband_rad
                or self.coverage_overlap_m < self.coverage_tolerance_m
                or type(self.max_expansions) is not int or type(self.repair_passes) is not int):
            raise ValueError("invalid navigation approach, tolerances or search limits")
        return self


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float
    # Only undocking permits the brush to retain contact with the known wall.
    contact: bool = False
    contact_edge_id: str = ""


@dataclass(frozen=True)
class NavigationScan:
    stamp_ns: int
    points: tuple  # observed endpoints in base_link, never map coordinates


class PlanningError(ValueError):
    pass


class Navigator:
    def __init__(self, occupancy, control, config=NavigationConfig()):
        self.map, self.c, self.n = occupancy, control, config.validate(control)
        self._candidate_edges = ()
        # Include cell extent, not just cell centres, in the footprint test.
        self.padding = config.clearance_m + occupancy.resolution_m / 2
        spacing = occupancy.resolution_m / 2
        def rectangle(x0, x1, y0, y1):
            x, y = np.meshgrid(np.linspace(x0, x1, math.ceil((x1-x0)/spacing)+1),
                               np.linspace(y0, y1, math.ceil((y1-y0)/spacing)+1))
            return np.column_stack((x.ravel(), y.ravel()))
        p = self.padding
        self.hull = rectangle(-control.hull_half_length_m-p, control.hull_half_length_m+p,
                              -control.hull_half_width_m-p, control.hull_half_width_m+p)
        brush = rectangle(control.brush_x_m-.025-p, control.brush_x_m+.025+p,
                          control.hull_half_width_m, control.brush_tip_y_m+p)
        self.footprint = np.concatenate((self.hull, brush))

    def free(self, pose, contact=False, edge=None):
        points = self.footprint
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        xs = pose.x + c*points[:, 0] - s*points[:, 1]
        ys = pose.y + s*points[:, 0] + c*points[:, 1]
        m = self.map
        cols = np.floor((xs-m.origin_x_m)/m.resolution_m).astype(int)
        rows = m.height-1-np.floor((ys-m.origin_y_m)/m.resolution_m).astype(int)
        if np.any(cols < 0) or np.any(cols >= m.width) or np.any(rows < 0) or np.any(rows >= m.height):
            return False
        free = m.region[rows, cols].astype(bool)
        if contact and edge is not None:
            # Only the brush may occupy the fitted target wall's narrow band.
            # Its raster surface includes unknown transition cells as well as
            # occupied cells. Every hull conflict remains blocked.
            hull_count = len(self.hull)
            brush_mask = np.zeros(len(points), dtype=bool)
            brush_mask[hull_count:] = True
            boundary = (np.abs((xs-edge.start[0])*edge.left_normal[0]
                               +(ys-edge.start[1])*edge.left_normal[1]) <= .08)
            along = ((xs-edge.start[0])*edge.tangent[0]
                     +(ys-edge.start[1])*edge.tangent[1])
            target_wall = boundary & (along >= -.05) & (along <= edge.length_m+.05)
            allowed = brush_mask & target_wall
            free |= allowed
        return bool(np.all(free))

    def sweep(self, a, b, contact=False, edge=None):
        distance = math.hypot(b.x-a.x, b.y-a.y)
        if distance <= 1e-8:
            # Keep the deployed turn behaviour: turns are closed-loop heading
            # commands and are not gated by a new swept-footprint model.
            return True
        count = max(1, math.ceil(distance/(self.map.resolution_m/2)))
        return all(self.free(Waypoint(a.x+(b.x-a.x)*i/count,
                                     a.y+(b.y-a.y)*i/count, a.yaw), contact, edge)
                   for i in range(count+1))

    def _sweep_edge(self, a, b, edge):
        distance = math.hypot(b.x-a.x,b.y-a.y)
        count=max(1,math.ceil(distance/(self.map.resolution_m/2)))
        return all(self.free(Waypoint(a.x+(b.x-a.x)*i/count,
                                     a.y+(b.y-a.y)*i/count,a.yaw),
                             contact=True,edge=edge) for i in range(count+1))

    def connect(self, start, goal):
        """Connect with body-axis translations and in-place turns, if feasible."""
        for turn_first in (False, True):
            yaw = goal.yaw if turn_first else start.yaw
            first = Waypoint(start.x, start.y, yaw)
            dx, dy = goal.x-first.x, goal.y-first.y
            along = dx*math.cos(yaw)+dy*math.sin(yaw)
            across = -dx*math.sin(yaw)+dy*math.cos(yaw)
            for side_first in (False, True):
                mid = (Waypoint(first.x-math.sin(yaw)*across, first.y+math.cos(yaw)*across, yaw)
                       if side_first else
                       Waypoint(first.x+math.cos(yaw)*along, first.y+math.sin(yaw)*along, yaw))
                path = [first, mid, Waypoint(goal.x, goal.y, yaw), goal]
                previous = start
                if all(self.sweep(a, b) for a, b in zip([previous]+path, path)):
                    return [p for a, p in zip([start]+path, path)
                            if math.hypot(p.x-a.x, p.y-a.y) > 1e-8 or abs(wrap(p.yaw-a.yaw)) > 1e-8]
        return None

    def plan(self, start, goal, departure_edge=None):
        deadline = time.monotonic()+self.n.plan_timeout_sec
        prefix = []
        if not self.free(start):
            # A boat may start with its brush against a known wall. Escape along
            # that wall's inward normal while preserving the initial heading;
            # never rotate a brush through an unknown/occupied cell.
            departures = [departure_edge] if departure_edge is not None else list(self.candidate_edges)
            escaped = None
            for edge in departures:
                if edge is None:
                    continue
                projection = edge.project(start, self.c.brush_x_m, self.c.brush_tip_y_m)
                if (not .02 <= projection.lateral_distance_m <= self.n.approach_distance_m
                        or projection.distance_to_segment_m > self.n.approach_distance_m
                        or not -.05 <= projection.s_m <= edge.length_m+.05):
                    continue
                for distance in np.arange(.05, self.n.approach_distance_m-projection.lateral_distance_m+.051, .05):
                    candidate = Waypoint(start.x-edge.left_normal[0]*distance,
                                         start.y-edge.left_normal[1]*distance,start.yaw,True,edge.id)
                    dx,dy=candidate.x-start.x,candidate.y-start.y
                    along=dx*math.cos(start.yaw)+dy*math.sin(start.yaw)
                    across=-dx*math.sin(start.yaw)+dy*math.cos(start.yaw)
                    for side_first in (True,False):
                        mid=(Waypoint(start.x-math.sin(start.yaw)*across,
                                      start.y+math.cos(start.yaw)*across,start.yaw,True,edge.id)
                             if side_first else
                             Waypoint(start.x+math.cos(start.yaw)*along,
                                      start.y+math.sin(start.yaw)*along,start.yaw,True,edge.id))
                        if self._sweep_edge(start,mid,edge) and self._sweep_edge(mid,candidate,edge) and self.free(candidate):
                            escaped=[mid,candidate]
                            break
                    if escaped:
                        break
                if escaped:
                    break
            if escaped is None:
                raise PlanningError("initial hull/brush footprint cannot safely leave its mapped wall")
            prefix.extend(escaped)
            start = escaped[-1]
        if not self.free(goal):
            raise PlanningError("approach pose is not reachable with this footprint")
        direct = self.connect(start, goal)
        if direct is not None:
            return prefix+direct
        # Lattice axes are anchored to the initial heading, so every primitive
        # is an executable forward/reverse/side translation or 45-degree turn.
        step, yaw0 = self.n.step_m, start.yaw
        c, s = math.cos(yaw0), math.sin(yaw0)
        directions = [(1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1)]
        def pose(key):
            x, y, heading = key
            return Waypoint(start.x+step*(c*x-s*y), start.y+step*(s*x+c*y), yaw0+heading*math.pi/4)
        def heuristic(p):
            return math.hypot(p.x-goal.x, p.y-goal.y)
        root = (0, 0, 0)
        cost, parent, poses = {root: 0.}, {}, {root: pose(root)}
        heap = [(heuristic(start), 0., root)]
        expanded = 0
        while heap and expanded < self.n.max_expansions:
            if time.monotonic() >= deadline:
                raise PlanningError("navigation search timed out")
            _, distance, key = heapq.heappop(heap)
            if distance != cost[key]:
                continue
            expanded += 1
            current = poses[key]
            if heuristic(current) <= 2*step:
                tail = self.connect(current, goal)
                if tail is not None:
                    path = []
                    while key != root:
                        path.append(poses[key])
                        key = parent[key]
                    return prefix+list(reversed(path))+tail
            x, y, h = key
            neighbours = [((x,y,(h+dh)%8), .15) for dh in (-1,1)]
            for offset in (0, 4, 2, 6):
                dx, dy = directions[(h+offset)%8]
                neighbours.append(((x+dx,y+dy,h), step*math.hypot(dx,dy)))
            for nxt, increment in neighbours:
                candidate_cost = distance+increment
                if candidate_cost >= cost.get(nxt, math.inf):
                    continue
                target = poses.setdefault(nxt, pose(nxt))
                if not self.sweep(current, target):
                    continue
                cost[nxt], parent[nxt] = candidate_cost, key
                heapq.heappush(heap, (candidate_cost+heuristic(target), candidate_cost, nxt))
        raise PlanningError("no reachable approach path within navigation search limits")

    def edge_pose(self, edge, contact_s, distance):
        along = contact_s-self.c.brush_x_m
        return Waypoint(edge.start[0]+edge.tangent[0]*along-edge.left_normal[0]*distance,
                        edge.start[1]+edge.tangent[1]*along-edge.left_normal[1]*distance,
                        edge.yaw_rad)

    def usable_interval(self, edge):
        """Contact interval with reachable brush-off acquisition endpoints.

        Keep existing wall-following geometry; do not reject a whole cleaning
        edge because raster wall thickness differs from its fitted centreline.
        """
        lo = edge.clean_start_s_m
        hi = min(edge.clean_end_s_m, edge.length_m-self.c.front_stop_m+self.c.brush_x_m)
        samples = np.linspace(lo, hi, max(2, math.ceil((hi-lo)/self.map.resolution_m)+1))
        valid = [float(v) for v in samples
                 if self.free(self.edge_pose(edge, v, self.n.approach_distance_m))]
        if len(valid) < 2 or valid[-1]-valid[0] < .2:
            raise PlanningError(f"{edge.id} has no usable cleaning interval for this boat")
        return valid[0], valid[-1]

    @property
    def candidate_edges(self):
        return self._candidate_edges

    @candidate_edges.setter
    def candidate_edges(self, edges):
        self._candidate_edges=tuple(edges)

    def motion_clear(self, pose, target, scan, contact=False, edge=None):
        """Check the next swept motion against map and observed laser endpoints."""
        if math.hypot(target.x-pose.x,target.y-pose.y) <= 1e-8:
            return True
        if not self.sweep(pose, target, contact, edge):
            return False
        points = np.asarray(scan.points, dtype=float).reshape(-1, 2)
        if not len(points):
            return True
        cp, sp = math.cos(pose.yaw), math.sin(pose.yaw)
        wx = pose.x+cp*points[:,0]-sp*points[:,1]
        wy = pose.y+sp*points[:,0]+cp*points[:,1]
        angle = wrap(target.yaw-pose.yaw)
        count = max(2, math.ceil(math.hypot(target.x-pose.x,target.y-pose.y)/.025),
                    math.ceil(abs(angle)/math.radians(3.)))
        for i in range(count+1):
            x = pose.x+(target.x-pose.x)*i/count
            y = pose.y+(target.y-pose.y)*i/count
            yaw = pose.yaw+angle*i/count
            lx = math.cos(yaw)*(wx-x)+math.sin(yaw)*(wy-y)
            ly = -math.sin(yaw)*(wx-x)+math.cos(yaw)*(wy-y)
            p = self.n.clearance_m
            hull_collision = ((np.abs(lx) <= self.c.hull_half_length_m+p)
                              & (np.abs(ly) <= self.c.hull_half_width_m+p))
            if np.any(hull_collision):
                return False
            if not contact:
                brush_collision = ((np.abs(lx-self.c.brush_x_m) <= .025+p)
                                   & (ly >= self.c.hull_half_width_m)
                                   & (ly <= self.c.brush_tip_y_m+p))
                if np.any(brush_collision):
                    return False
            elif edge is not None:
                world_x = x+math.cos(yaw)*lx-math.sin(yaw)*ly
                world_y = y+math.sin(yaw)*lx+math.cos(yaw)*ly
                normal = ((world_x-edge.start[0])*edge.left_normal[0]
                          +(world_y-edge.start[1])*edge.left_normal[1])
                along = ((world_x-edge.start[0])*edge.tangent[0]
                         +(world_y-edge.start[1])*edge.tangent[1])
                allowed = ((np.abs(lx-self.c.brush_x_m) <= .08)
                           & (ly >= self.c.hull_half_width_m)
                           & (normal >= -.04) & (normal <= .18)
                           & (along >= -.05) & (along <= edge.length_m+.05))
                brush_collision = ((np.abs(lx-self.c.brush_x_m) <= .025+p)
                                   & (ly >= self.c.hull_half_width_m)
                                   & (ly <= self.c.brush_tip_y_m+p))
                if np.any(brush_collision & ~allowed):
                    return False
        return True
