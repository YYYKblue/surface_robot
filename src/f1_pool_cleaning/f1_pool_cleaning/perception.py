"""Bounded planar left-wall extraction using contiguous clusters and robust TLS."""
from dataclasses import dataclass
import math

import numpy as np

from .perception_config import PerceptionConfig


@dataclass(frozen=True)
class Scan:
    stamp_ns: int
    frame: str
    angle_min: float
    angle_increment: float
    ranges: tuple
    range_min: float
    range_max: float
    time_increment: float = 0.


@dataclass(frozen=True)
class Observation:
    stamp_ns: int
    valid: bool = False
    reason: str = ""
    distance: float = 0.
    angle: float = 0.
    residual: float = 0.
    support: float = 0.
    inliers: int = 0
    ratio: float = 0.
    start: tuple = (0., 0.)
    end: tuple = (0., 0.)
    front_observed: bool = False
    front_clearance: float = 0.
    front_fraction: float = 0.
    rigid_collision: bool = False
    left_turn_clear: bool = False
    right_turn_clear: bool = False


@dataclass(frozen=True)
class EdgeTargetContext:
    """Map-derived target used to keep a left-wall identity across loss."""
    stamp_ns: int
    generation: int
    edge_id: str
    valid: bool
    reason: str = ""
    expected_distance: float = 0.
    expected_angle: float = 0.
    distance_tolerance: float = .15
    angle_tolerance: float = .35


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def fresh(stamp_ns, now_ns, max_age):
    return stamp_ns > 0 and -5_000_000 <= now_ns - stamp_ns < int(max_age * 1e9)


class Perception:
    def __init__(self, config=PerceptionConfig()):
        self.c = config.validate()
        self.previous = None
        self.last_stamp = 0
        self.target = None
        # Config is frozen and ROS declares its parameters read-only. Only the
        # hull sweep is invariant; visibility and collisions use every new scan.
        self._turn_sweep = self._turn_geometry()

    def reset(self):
        self.previous = None
        self.last_stamp = 0

    def set_target(self, target):
        """Replace target context and invalidate an old wall association."""
        old = getattr(self, "target", None)
        self.target = target
        if (old is None or target is None
                or (old.generation, old.edge_id, old.valid)
                != (target.generation, target.edge_id, target.valid)):
            self.previous = None

    def _split_corners(self, cluster, depth=0):
        """Split connected wall corners before line fitting, preserving quality gates.

        Adjacent perpendicular walls have no range gap at their shared corner.
        Fitting both as one line can reject a well-supported left wall near the
        end of a pool. Bound recursion and retain the corner in both children.
        """
        if depth >= 5 or len(cluster) < 2 * self.c.min_inliers:
            return [cluster]
        chord = cluster[-1] - cluster[0]
        length = np.linalg.norm(chord)
        if length < self.c.min_support_m:
            return [cluster]
        normal = np.array([-chord[1], chord[0]]) / length
        errors = np.abs((cluster - cluster[0]) @ normal)
        index = int(np.argmax(errors))
        if (errors[index] <= 3 * self.c.fit_threshold_m
                or index + 1 < self.c.min_inliers
                or len(cluster) - index < self.c.min_inliers):
            return [cluster]
        return (self._split_corners(cluster[:index + 1], depth + 1)
                + self._split_corners(cluster[index:], depth + 1))

    def _fit(self, cluster):
        c = self.c
        # At most 240 points enter RANSAC; front safety still uses every ray.
        points = cluster[np.linspace(0, len(cluster) - 1, min(len(cluster), 240), dtype=int)]
        rng = np.random.default_rng(103)
        best = None
        best_count = 0
        for _ in range(64):
            a, b = points[rng.choice(len(points), 2, replace=False)]
            delta = b - a
            length = np.linalg.norm(delta)
            if length < c.min_support_m / 3:
                continue
            normal = np.array([-delta[1], delta[0]]) / length
            mask = np.abs((points - a) @ normal) <= c.fit_threshold_m
            count = int(mask.sum())
            if count > best_count:
                best, best_count = mask, count
        if best is None or best_count < c.min_inliers:
            return None
        selected = points[best]
        centre = selected.mean(axis=0)
        _, _, vt = np.linalg.svd(selected - centre, full_matrices=False)
        tangent = vt[0]
        if tangent[0] < 0:
            tangent = -tangent
        normal = np.array([-tangent[1], tangent[0]])
        distance = float(centre @ normal)
        angle = math.atan2(tangent[1], tangent[0])
        errors = np.abs(points @ normal - distance)
        mask = errors <= c.fit_threshold_m
        selected = points[mask]
        if len(selected) < c.min_inliers:
            return None
        projection = selected @ tangent
        support = float(np.ptp(projection))
        residual = float(np.sqrt(np.mean(errors[mask] ** 2)))
        ratio = float(mask.mean())
        if not (0 < distance <= c.max_wall_distance_m and abs(angle) <= c.max_wall_angle_rad
                and support >= c.min_support_m and ratio >= c.min_inlier_ratio):
            return None
        start = normal * distance + tangent * projection.min()
        end = normal * distance + tangent * projection.max()
        return distance, angle, residual, support, len(selected), ratio, tuple(start), tuple(end)

    def _front(self, angles, valid, points, range_max):
        c = self.c
        if c.geometry_error():
            return False, 0., 0., False
        half_x, half_y = c.hull_length_m / 2, c.hull_width_m / 2 + c.rigid_margin_m
        # A blind bow does not remove evidence of obstacles in visible side or
        # stern sectors. Compute this before the front-coverage early return.
        # False only means no observed point in the envelope, not that unseen
        # parts of the envelope have been checked.
        finite_points = points[valid]
        rigid = bool(np.any((np.abs(finite_points[:, 0]) <= half_x + c.rigid_margin_m)
                            & (np.abs(finite_points[:, 1]) <= half_y)))
        horizon = half_x + c.front_slow_distance_m
        low = math.atan2(-half_y - c.lidar_y_m, horizon - c.lidar_x_m)
        high = math.atan2(half_y - c.lidar_y_m, horizon - c.lidar_x_m)
        sector = (angles >= low) & (angles <= high)
        indices = np.flatnonzero(sector)
        if len(indices) < 3:
            return False, 0., 0., rigid
        coverage = float(valid[indices].mean())
        visible_angles = np.sort(angles[indices][valid[indices]])
        gaps = np.diff(np.r_[low, visible_angles, high])
        observed = coverage >= c.front_valid_fraction and max(gaps) <= c.front_max_gap_rad
        # NaN/inf are never interpreted as free space.
        corridor = finite_points[(finite_points[:, 0] > 0) & (np.abs(finite_points[:, 1]) <= half_y)]
        clearance = min(c.front_slow_distance_m, range_max - half_x)
        if len(corridor):
            clearance = min(clearance, float(corridor[:, 0].min()) - half_x)
        return bool(observed), max(0., clearance), coverage, rigid

    def process(self, scan, now_ns, target=None):
        c = self.c
        if target is not None:
            self.set_target(target)
        else:
            target = getattr(self, "target", None)
        invalid = lambda reason: Observation(scan.stamp_ns, reason=reason)
        if not fresh(scan.stamp_ns, now_ns, c.scan_max_age_sec):
            return invalid("scan stale or future-dated")
        if scan.stamp_ns <= self.last_stamp:
            return invalid("duplicate/backward scan stamp")
        self.last_stamp = scan.stamp_ns
        n = len(scan.ranges)
        if scan.frame != c.scan_frame:
            return invalid("unexpected scan frame")
        if (not 12 <= n <= 20000 or not all(math.isfinite(v) for v in
                (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, scan.time_increment))
                or scan.angle_increment == 0 or scan.range_min < 0 or scan.range_max <= scan.range_min
                or scan.time_increment < 0 or scan.time_increment * (n - 1) > .1):
            return invalid("invalid scan metadata or scan duration")
        if (abs(scan.angle_increment) * (n - 1) > 2 * math.pi + .01
                or scan.stamp_ns + round(scan.time_increment * (n - 1) * 1e9) > now_ns + 5_000_000):
            return invalid("invalid angular coverage or future scan end time")
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment + c.lidar_yaw_rad
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        safe_ranges = np.where(valid, ranges, 0.)
        points = np.column_stack((safe_ranges * np.cos(angles) + c.lidar_x_m,
                                  safe_ranges * np.sin(angles) + c.lidar_y_m))
        front = self._front(angles, valid, points, scan.range_max)
        selected = valid & (points[:, 1] > .02) & (points[:, 0] >= -1.5) & (points[:, 0] <= 3.)
        indices = np.flatnonzero(selected)
        gaps = ((np.diff(indices) > 2)
                | (np.linalg.norm(np.diff(points[indices], axis=0), axis=1) > c.cluster_gap_m))
        clusters = [points[group] for group in np.split(indices, np.flatnonzero(gaps) + 1)
                    if len(group) >= c.min_inliers]
        clusters = [part for cluster in clusters for part in self._split_corners(cluster)]
        # Limit work even on highly fragmented scans; longest clusters first.
        candidates = [f for cluster in sorted(clusters, key=len, reverse=True)[:12]
                      if (f := self._fit(cluster)) is not None]
        fitted = bool(candidates)
        if target is not None:
            if not target.valid:
                return Observation(scan.stamp_ns, reason=target.reason or "edge target invalid",
                                   front_observed=front[0], front_clearance=front[1],
                                   front_fraction=front[2], rigid_collision=front[3])
            if not fresh(target.stamp_ns, now_ns, c.scan_max_age_sec):
                return Observation(scan.stamp_ns, reason="edge target stale",
                                   front_observed=front[0], front_clearance=front[1],
                                   front_fraction=front[2], rigid_collision=front[3])

            def line_delta(a, b):
                # A fitted line has no arrow.  Compare its orientation modulo
                # pi while retaining the positive-left normal from _fit.
                delta = wrap(a - b)
                if delta > math.pi / 2:
                    delta -= math.pi
                elif delta < -math.pi / 2:
                    delta += math.pi
                return delta

            candidates = [f for f in candidates
                          if abs(f[0] - target.expected_distance) <= target.distance_tolerance
                          and abs(line_delta(f[1], target.expected_angle)) <= target.angle_tolerance]
            if not candidates:
                return Observation(scan.stamp_ns,
                                   reason=f"edge {target.edge_id} wall target mismatch",
                                   front_observed=front[0], front_clearance=front[1],
                                   front_fraction=front[2], rigid_collision=front[3])
        previous = self.previous
        if previous and scan.stamp_ns - previous.stamp_ns <= c.association_timeout_sec * 1e9:
            candidates = [f for f in candidates if abs(f[0] - previous.distance) <= c.association_distance_m
                          and abs(wrap(f[1] - previous.angle)) <= c.association_angle_rad]
        if not candidates:
            reason = ("left wall association lost: distance/angle change exceeds tracking limits" if fitted
                      else f"left wall absent or fit rejected (angle limit {math.degrees(c.max_wall_angle_rad):.1f} deg)")
            return Observation(scan.stamp_ns, reason=reason,
                               front_observed=front[0], front_clearance=front[1],
                               front_fraction=front[2], rigid_collision=front[3])
        candidates.sort(key=lambda f: abs(f[0] - previous.distance) if previous else f[0])
        best = candidates[0]
        turns = self._turn_clearance(angles, valid, ranges)
        result = Observation(scan.stamp_ns, True, "left wall tracked", *best, *front, *turns)
        self.previous = result
        return result

    def _turn_geometry(self):
        """Precompute the conservative bounded rigid-hull sweep once.

        Ray/box intersections use the actual lidar offset and a margin-expanded
        hull. Analytic rotation extrema cover continuous rotation between poses;
        missing rays in newly swept sectors veto the corresponding turn.
        These are diagnostic flags for the configured small-angle envelope;
        the current single-edge controller does not use them to gate output.
        """
        c = self.c
        if c.geometry_error():
            return None
        hx, hy = c.hull_length_m / 2 + c.rigid_margin_m, c.hull_width_m / 2 + c.rigid_margin_m
        origin = np.array([c.lidar_x_m, c.lidar_y_m])
        if np.linalg.norm(origin) >= min(hx, hy):
            return None
        # Dense target angles also check gaps and missing sectors in the real scan.
        phi = np.linspace(-math.pi, math.pi, 1440, endpoint=False)
        rays = np.column_stack((np.cos(phi), np.sin(phi)))

        def exit_ranges(theta):
            # theta can be a scalar or one candidate rotation for each ray.
            cs, sn = np.cos(theta), np.sin(theta)
            local_origin = np.column_stack((origin[0] * cs + origin[1] * sn + np.zeros(len(phi)),
                                            -origin[0] * sn + origin[1] * cs + np.zeros(len(phi))))
            local_rays = np.column_stack((rays[:, 0] * cs + rays[:, 1] * sn,
                                          -rays[:, 0] * sn + rays[:, 1] * cs))
            bounds = np.array([hx, hy])
            with np.errstate(divide="ignore", invalid="ignore"):
                crossing = (np.where(local_rays >= 0, bounds, -bounds) - local_origin) / local_rays
            return np.min(crossing, axis=1)

        def envelope(end):
            # Exact candidates for maxima of ray/rotating-box intersections:
            # interval endpoints, stationary face intersections and corner crossings.
            low, high = min(0., end), max(0., end)
            candidates = [exit_ranges(0.), exit_ranges(end)]
            cross = origin[0] * rays[:, 1] - origin[1] * rays[:, 0]

            def add(theta, valid):
                theta = (theta + math.pi) % (2 * math.pi) - math.pi
                valid = valid & (theta >= low) & (theta <= high)
                candidates.append(exit_ranges(np.where(valid, theta, 0.)))

            for axis, bound in ((0., hx), (0., -hx), (math.pi / 2, hy), (math.pi / 2, -hy)):
                ratio = cross / bound
                arc = np.arcsin(np.clip(ratio, -1., 1.))
                add(phi - axis - arc, np.abs(ratio) <= 1.)
                add(phi - axis - math.pi + arc, np.abs(ratio) <= 1.)
            radius = math.hypot(hx, hy)
            arc = np.arcsin(np.clip(-cross / radius, -1., 1.))
            for x, y in ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)):
                alpha = math.atan2(y, x)
                add(phi - alpha + arc, np.abs(cross) <= radius)
                add(phi - alpha + math.pi - arc, np.abs(cross) <= radius)
            return np.max(candidates, axis=0)

        baseline = exit_ranges(0.)
        sweeps = []
        for sign in (1, -1):
            extent = envelope(sign * c.turn_sweep_rad)
            required = extent > baseline + 1e-6
            # Include angular boundary neighbours instead of assuming a point ray
            # observes an entire sweep sector. Gaps still veto the affected turn.
            required = np.logical_or.reduce([np.roll(required, i) for i in range(-3, 4)])
            sweeps.append((extent, required))
        return phi, tuple(sweeps)

    def _turn_clearance(self, angles, valid, ranges):
        """Check current returns and blind sectors against the cached hull sweep."""
        if self._turn_sweep is None:
            return False, False
        c = self.c
        phi, sweeps = self._turn_sweep
        order = np.argsort(angles)
        ordered = angles[order]
        positions = np.searchsorted(ordered, phi)
        a, b = order[(positions - 1) % len(order)], order[positions % len(order)]
        da = np.abs((angles[a] - phi + math.pi) % (2 * math.pi) - math.pi)
        db = np.abs((angles[b] - phi + math.pi) % (2 * math.pi) - math.pi)
        nearest = np.where(da < db, a, b)
        answers = []
        for extent, required in sweeps:
            known = valid[nearest] & (np.minimum(da, db) <= min(c.front_max_gap_rad / 2, .01))
            free = ranges[nearest] > extent + c.fit_threshold_m
            # All finite returns must also lie outside the swept box union.
            # Reuse angular envelope with conservative radial padding.
            measured_extent = np.interp(angles, phi, extent, period=2 * math.pi)
            collision = np.any(valid & (ranges <= measured_extent + c.fit_threshold_m))
            answers.append(bool(np.any(required) and np.all((known & free)[required]) and not collision))
        return tuple(answers)
