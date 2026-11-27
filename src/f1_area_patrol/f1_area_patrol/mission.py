"""Pose-driven execution and measured coverage. The caller owns time and output."""
from dataclasses import dataclass, fields
import math

import numpy as np

from .planner import Leg, mark_band


@dataclass(frozen=True)
class Settings:
    lane_spacing_m: float = .40
    arrival_m: float = .18
    lookahead_m: float = .45
    pose_jump_m: float = .35
    slowdown_m: float = .25
    turn_enter_deg: float = 20.
    turn_exit_deg: float = 10.
    drive_percent: int = 22
    slow_percent: int = 14
    turn_percent: int = 20
    side_percent: int = 25
    stall_sec: float = 20.

    def validate(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0 for f in fields(self)):
            raise ValueError("巡游参数必须是有限正数")
        if not (.10 <= self.lane_spacing_m <= 10.
                and self.arrival_m < min(self.lookahead_m, self.slowdown_m)
                and self.turn_exit_deg < self.turn_enter_deg < 90
                and self.slow_percent <= self.drive_percent
                and max(self.drive_percent, self.side_percent, self.turn_percent) <= 100):
            raise ValueError("巡游间距、控制阈值或推力百分比无效")
        return self


@dataclass(frozen=True)
class Output:
    direction: str = "STOP"
    percent: int = 0
    side: int = 0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Mission:
    def __init__(self, grid, route, settings):
        self.grid, self.route, self.settings = grid, route, settings
        self.legs = list(route.legs)
        self.index = self.finished_lanes = 0
        self.total_lanes = sum(l.kind == "lane" for l in self.legs)
        self.region_cells = int(route.region.sum())
        self.observed = np.zeros_like(route.region)
        self.observed_cells = 0
        self.previous = None
        self.turning = False
        self.reset_watchdog()

    def reset_watchdog(self):
        self.best, self.last_improvement, self.watch_mode = math.inf, None, ""

    def watch(self, error, mode, now):
        if mode != self.watch_mode or self.last_improvement is None or error < self.best-.012:
            self.best, self.last_improvement, self.watch_mode = error, now, mode
        elif now-self.last_improvement > self.settings.stall_sec:
            raise ValueError("推进或转向持续无进展")

    def _target(self, position):
        leg = self.legs[self.index]
        dx, dy = leg.end[0] - leg.start[0], leg.end[1] - leg.start[1]
        length = math.hypot(dx, dy)
        if not length:
            return leg.end
        along = ((position[0]-leg.start[0])*dx + (position[1]-leg.start[1])*dy) / length
        scale = min(length, max(0., along) + self.settings.lookahead_m) / length
        return leg.start[0] + scale*dx, leg.start[1] + scale*dy

    def rejoin(self, position):
        # Aim along the unfinished leg from the latest position. If the direct
        # approach crosses an island, join the same target over safe connectors.
        target = self._target(position)
        if not self.grid.clear(position, target):
            points = self.grid.connect(position, target)
            self.legs[self.index:self.index] = [Leg(a, b, "entry") for a, b in zip(points, points[1:]) if a != b]
        self.previous = None
        self.turning = False
        self.reset_watchdog()

    def update(self, pose, now):
        position, yaw = pose[:2], pose[2]
        leg = self.legs[self.index]
        c = self.settings
        if not self.grid.label(position):
            raise ValueError("船位离开可航区域")
        if self.previous is not None:
            if math.dist(position, self.previous) > c.pose_jump_m or not self.grid.clear(self.previous, position):
                raise ValueError("相邻位姿跳变或跨越边界")
        if leg.kind == "lane":
            self.observed_cells += mark_band(self.observed, self.grid, self.previous or position,
                                             position, self.route.spacing, self.route.region)
        self.previous = position
        remaining = math.dist(position, leg.end)
        # Keep short transfers meaningful when a fine sweep spacing is selected.
        tolerance = min(c.arrival_m, self.route.spacing * .45)
        dx, dy = leg.end[0] - leg.start[0], leg.end[1] - leg.start[1]
        length = math.hypot(dx, dy)
        ex, ey = position[0] - leg.end[0], position[1] - leg.end[1]
        # Accept crossing a lane's end within its half-width. A transfer or
        # obstacle detour still has to reach its waypoint before changing leg.
        passed = (leg.kind == "lane" and length > 0 and ex*dx + ey*dy >= 0
                  and abs(ex*dy - ey*dx) <= tolerance*length)
        if remaining <= tolerance or passed:
            self.finished_lanes += leg.kind == "lane"
            self.index += 1
            self.turning = False
            self.reset_watchdog()
            return Output()
        aim = self._target(position)
        if not self.grid.clear(position, aim):
            raise ValueError("前视点连接越过可航边界")
        bearing = wrap(math.atan2(aim[1]-position[1], aim[0]-position[0])-yaw)
        # Use a lateral transfer when the hull is already broadside to a straight connector.
        if leg.kind == "transfer" and abs(abs(bearing)-math.pi/2) <= math.radians(c.turn_enter_deg):
            self.watch(remaining, "side", now)
            return Output(side=c.side_percent if bearing>0 else -c.side_percent)
        limit = c.turn_exit_deg if self.turning else c.turn_enter_deg
        # At a map edge a small heading error can point outside the known
        # cells even when the segment to the target stays entirely inside.
        distance = math.dist(position, aim)
        forward = (position[0] + distance*math.cos(yaw), position[1] + distance*math.sin(yaw))
        self.turning = abs(bearing) > math.radians(limit) or not self.grid.clear(position, forward)
        if self.turning:
            self.watch(abs(bearing), "turn", now)
            return Output("LEFT" if bearing>0 else "RIGHT", c.turn_percent)
        self.watch(remaining, "forward", now)
        return Output("FORWARD", c.slow_percent if remaining < c.slowdown_m else c.drive_percent)
