"""Derive task geometry from a saved map instead of hand-copied constants.

The launch files take one ``map`` argument.  This module turns that path into
the numbers the controller and the localization gate need, so changing maps
means changing the map path:

* closed pool task -- ``origin_*``/``axis_yaw_rad``/``length_m``/``width_m``
  and the ``boundary_xy`` route;
* open L task -- the ``route_xy`` polyline;
* localization -- the known-free extents used as a coarse pose gate.

The map is the single source of truth.  Derivation never falls back to a
built-in rectangle: a map that does not yield a usable enclosure or route
raises, so a wrong or ambiguous map fails at launch instead of running on
stale geometry.

Geometry conventions match :mod:`f1_pool_cleaning.config`: ``map_xy(0, 0)``
is ``origin``, +x is ``axis_yaw_rad`` and +y is its left normal, so the
``boundary_xy`` corners are ``map_xy(0, W)``, ``map_xy(L, W)``,
``map_xy(L, 0)``, ``map_xy(0, 0)`` -- the canonical clockwise route with the
pool interior on the right and the wall on the boat's left.
"""

from dataclasses import dataclass, field
import math
from pathlib import Path

# Re-export the old map helpers for existing callers; implementation lives in
# the localization package so it never depends on cleaning task geometry.
from f1_localization.map_io import (
    MapProfileError, Occupancy, bounds_of, load_occupancy, resolve_map_files,
    resolve_pbstream, free_bounds as _free_bounds, occupancy_points as _points,
)
from f1_localization.localization import bounds_overrides

# Wall bands are sampled this far in from the free-region extremes, so corner
# returns and stray scans never share a median with the wall they belong to.
BAND_INSET_M = .10
# Minimum occupied cells before a side counts as measured.
MIN_BAND_CELLS = 8
# A closed-pool map must be mostly free water inside its wall rectangle; an
# L-shaped or obstacle-filled map is not a rectangle task.
MIN_POOL_FILL = .90
# Shortest wall segment considered part of an open route.
MIN_ROUTE_SEGMENT_M = 2.0
# A route corner must be this close to both wall segments to count as a join.
CORNER_JOIN_TOLERANCE_M = 1.0
# Route quality: fraction of samples with wall alongside and open water on the
# right (the single left-brush convention).
ROUTE_WATER_OFFSETS_M = (.35, .55, .75, .95)
ROUTE_WALL_OFFSETS_M = (.05, .10, .15, .20)
MIN_ROUTE_QUALITY = .75
MIN_ROUTE_MARGIN = .10


@dataclass(frozen=True)
class RectangleProfile:
    """Closed pool enclosure, expressed in the controller's pool frame."""

    origin_x_m: float
    origin_y_m: float
    axis_yaw_rad: float
    length_m: float
    width_m: float
    wall_thickness_m: float
    fill_fraction: float
    boundary_xy: tuple = ()

    def map_xy(self, x, y):
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return (self.origin_x_m + c * x - s * y,
                self.origin_y_m + s * x + c * y)

    def overrides(self):
        """Controller/localization parameter overrides for this map."""
        return dict(origin_x_m=self.origin_x_m, origin_y_m=self.origin_y_m,
                    axis_yaw_rad=self.axis_yaw_rad,
                    length_m=self.length_m, width_m=self.width_m,
                    boundary_xy=list(self.boundary_xy))


@dataclass(frozen=True)
class RouteProfile:
    """Open directed route with the wall on the boat's left."""

    route_xy: tuple
    leg_lengths_m: tuple
    turn_rad: float
    quality: float
    bounds: tuple  # (x_min, y_min, x_max, y_max) of the route vertices

    def overrides(self):
        """Controller overrides: the route plus the box its pose gate covers.

        The open route has no pool frame of its own, so ``axis_yaw_rad`` is
        zero and the box is axis-aligned in ``map``.  It is only a containment
        bound; edge progress always comes from ``route_xy``.
        """
        x_min, y_min, x_max, y_max = self.bounds
        return dict(route_xy=list(self.route_xy), axis_yaw_rad=0.,
                    origin_x_m=x_min, origin_y_m=y_min,
                    length_m=x_max - x_min, width_m=y_max - y_min)


@dataclass(frozen=True)
class MapProfile:
    """Everything derived from one map, with the failures kept for reporting."""

    map_yaml: Path
    resolution_m: float
    free_bounds: tuple  # (x_min, y_min, x_max, y_max) of the largest free region
    free_area_m2: float
    rectangle: RectangleProfile | None = None
    route: RouteProfile | None = None
    errors: dict = field(default_factory=dict)


def _axis(occupancy, free_points):
    """Longest minimum-area-rectangle edge of the free region, as unit +x."""
    import cv2
    import numpy as np

    box = cv2.boxPoints(cv2.minAreaRect(free_points.astype(np.float32)))
    edges = np.roll(box, -1, axis=0) - box
    axis = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))].astype(np.float64)
    if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-6:
        raise MapProfileError("free region has no usable principal axis")
    axis /= np.linalg.norm(axis)
    return -axis if axis[0] < 0 else axis


def _median_band(values, mask, what):
    import numpy as np

    if int(np.count_nonzero(mask)) < MIN_BAND_CELLS:
        raise MapProfileError(
            f"fewer than {MIN_BAND_CELLS} occupied cells for the {what} wall band; "
            "the map may be incomplete or the thresholds wrong")
    return float(np.median(values[mask]))


def rectangle_profile(occupancy=None, path=None):
    """Enclosure of the largest free region, measured on the wall cells.

    The free region gives the orientation; the *occupied* bands give the wall
    centre lines, so the result matches the wall-centre distances the cleaning
    controller was tuned for rather than the inset free-cell rectangle.
    """
    import numpy as np

    occupancy = occupancy or load_occupancy(path)
    free_points = _points(occupancy, occupancy.region)
    occupied_points = _points(occupancy, occupancy.occupied)
    if len(occupied_points) < 4 * MIN_BAND_CELLS:
        raise MapProfileError("map has too few occupied cells to locate the walls")
    if len(free_points) < 3:
        raise MapProfileError("map free region is too small to fit a pool")
    axis = _axis(occupancy, free_points)
    normal = np.array([-axis[1], axis[0]])
    fu, fv = free_points @ axis, free_points @ normal
    ou, ov = occupied_points @ axis, occupied_points @ normal
    middle_v = (ov > fv.min() + BAND_INSET_M) & (ov < fv.max() - BAND_INSET_M)
    middle_u = (ou > fu.min() + BAND_INSET_M) & (ou < fu.max() - BAND_INSET_M)
    u0 = _median_band(ou, middle_v & (ou < fu.min() + BAND_INSET_M), "lower-x")
    u1 = _median_band(ou, middle_v & (ou > fu.max() - BAND_INSET_M), "upper-x")
    v0 = _median_band(ov, middle_u & (ov < fv.min() + BAND_INSET_M), "lower-y")
    v1 = _median_band(ov, middle_u & (ov > fv.max() - BAND_INSET_M), "upper-y")
    length, width = u1 - u0, v1 - v0
    if not (math.isfinite(length) and math.isfinite(width) and length > 0 and width > 0):
        raise MapProfileError("map wall bands are not finite or not ordered")
    # A rectangle task needs one enclosing pool, not merely some free water.
    # Fill is measured against the free region's own extent: a rectangular
    # basin fills it almost completely, while an L-shaped or obstacle-strewn
    # map leaves large gaps around the water.
    free_span = (float(fu.max() - fu.min())) * (float(fv.max() - fv.min()))
    area = len(free_points) * occupancy.resolution_m ** 2
    fill = area / free_span if free_span > 0 else 0.
    if fill < MIN_POOL_FILL:
        raise MapProfileError(
            f"free water fills only {fill:.0%} of its {fu.max() - fu.min():.2f} x "
            f"{fv.max() - fv.min():.2f} m bounding rectangle; this map is not a "
            "single closed pool")
    origin = axis * u0 + normal * v0
    thickness = ((abs(fu.min() - u0) + abs(u1 - fu.max()))
                 + (abs(fv.min() - v0) + abs(v1 - fv.max()))) / 4.
    profile = RectangleProfile(
        origin_x_m=float(origin[0]), origin_y_m=float(origin[1]),
        axis_yaw_rad=math.atan2(float(axis[1]), float(axis[0])),
        length_m=float(length), width_m=float(width),
        wall_thickness_m=float(thickness), fill_fraction=float(fill))
    corners = (profile.map_xy(0., width), profile.map_xy(length, width),
               profile.map_xy(length, 0.), profile.map_xy(0., 0.))
    return RectangleProfile(**{**profile.__dict__,
                               "boundary_xy": tuple(v for point in corners for v in point)})


def _clean_segments(occupancy, min_length):
    """Straight water/wall boundary segments in map metres.

    Detection runs on the free-region edge rather than on occupied cells: the
    water boundary is one continuous line, while occupied returns can be
    sparse or doubled by the far wall across a walkway.
    """
    import cv2
    import numpy as np

    edge = cv2.morphologyEx(occupancy.region, cv2.MORPH_GRADIENT,
                            np.ones((3, 3), np.uint8))
    lines = cv2.HoughLinesP(edge * 255, 1, math.pi / 720, threshold=60,
                            minLineLength=int(min_length / occupancy.resolution_m),
                            maxLineGap=25)
    if lines is None:
        return []
    height = occupancy.height
    segments = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        start = occupancy.xy(x1, y1)
        end = occupancy.xy(x2, y2)
        if math.dist(start, end) >= min_length:
            segments.append((start, end))
    return segments


def _line_intersection(a, b):
    (x1, y1), (x2, y2) = a
    (x3, y3), (x4, y4) = b
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-9:
        return None
    return (((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator,
            ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator)


def _leg_quality(occupancy, start, end):
    """Fraction of samples with a wall on the left and open water on the right.

    This is exactly the geometry the single left brush needs, and it is what
    separates a real cleaning leg from a boundary of the mapped area.
    """
    import numpy as np

    length = math.dist(start, end)
    if length <= 0:
        return 0.
    ux, uy = (end[0] - start[0]) / length, (end[1] - start[1]) / length
    nx, ny = -uy, ux  # left normal
    samples = np.linspace(.05, .95, 13)
    good = 0
    for fraction in samples:
        bx, by = start[0] + ux * length * fraction, start[1] + uy * length * fraction
        wall = [occupancy.cell(bx + nx * o, by + ny * o) for o in ROUTE_WALL_OFFSETS_M]
        water = [occupancy.cell(bx - nx * o, by - ny * o) for o in ROUTE_WATER_OFFSETS_M]
        if "occupied" in wall and water.count("free") >= 3 and "occupied" not in water:
            good += 1
    return good / len(samples)


def _route_candidates(occupancy):
    """All clockwise-corner wall pairs that could be an open cleaning route."""
    segments = _clean_segments(occupancy, MIN_ROUTE_SEGMENT_M)
    candidates = []
    for index, first in enumerate(segments):
        for second in segments[index + 1:]:
            corner = _line_intersection(first, second)
            if corner is None:
                continue
            if min(min(math.dist(corner, point) for point in first),
                   min(math.dist(corner, point) for point in second)) > CORNER_JOIN_TOLERANCE_M:
                continue
            for a, b in ((first, second), (second, first)):
                start = max(a, key=lambda point: math.dist(point, corner))
                finish = max(b, key=lambda point: math.dist(point, corner))
                first_leg = math.dist(start, corner)
                second_leg = math.dist(corner, finish)
                if min(first_leg, second_leg) < MIN_ROUTE_SEGMENT_M:
                    continue
                # The L route has exactly one right-angled clockwise corner.
                inbound = math.atan2(corner[1] - start[1], corner[0] - start[0])
                outbound = math.atan2(finish[1] - corner[1], finish[0] - corner[0])
                turn = math.atan2(math.sin(outbound - inbound), math.cos(outbound - inbound))
                if not -math.radians(135.) <= turn <= -math.radians(45.):
                    continue
                quality = min(_leg_quality(occupancy, start, corner),
                              _leg_quality(occupancy, corner, finish))
                candidates.append(dict(start=start, corner=corner, finish=finish,
                                       lengths=(first_leg, second_leg),
                                       turn=turn, quality=quality))
    return candidates


def route_profile(occupancy=None, path=None):
    """Best two-leg clockwise route, or a :class:`MapProfileError`.

    Candidate routes are ranked by how well each leg matches the left-brush
    convention.  A map with several comparable candidates is reported as
    ambiguous rather than resolved by guesswork.
    """
    occupancy = occupancy or load_occupancy(path)
    candidates = _route_candidates(occupancy)
    if not candidates:
        raise MapProfileError(
            "no pair of water boundaries forms a single clockwise 45-135 degree "
            "corner with two usable legs; this map has no open L route")
    candidates.sort(key=lambda item: (item["quality"], sum(item["lengths"])),
                    reverse=True)
    best = candidates[0]
    if best["quality"] < MIN_ROUTE_QUALITY:
        raise MapProfileError(
            f"best L candidate only scores {best['quality']:.2f} (< {MIN_ROUTE_QUALITY}); "
            "the walls do not keep water on the right along both legs")
    runners = [item for item in candidates[1:]
               if math.dist(item["corner"], best["corner"]) > 1.5]
    if runners and best["quality"] - runners[0]["quality"] < MIN_ROUTE_MARGIN:
        raise MapProfileError(
            "map has two comparable L candidates "
            f"(quality {best['quality']:.2f} vs {runners[0]['quality']:.2f}); "
            "the route is ambiguous")
    points = (best["start"], best["corner"], best["finish"])
    return RouteProfile(
        route_xy=tuple(v for point in points for v in point),
        leg_lengths_m=tuple(best["lengths"]), turn_rad=best["turn"],
        quality=best["quality"], bounds=bounds_of(points))


def profile(path):
    """Load one map once and derive every geometry that it supports.

    Faults are collected in :attr:`MapProfile.errors` so a launch can report
    what the selected map offers and what it is missing, instead of failing on
    the first check.
    """
    occupancy = load_occupancy(path)
    errors = {}
    rectangle = route = None
    try:
        rectangle = rectangle_profile(occupancy)
    except MapProfileError as exc:
        errors["rectangle"] = str(exc)
    try:
        route = route_profile(occupancy)
    except MapProfileError as exc:
        errors["route"] = str(exc)
    return MapProfile(map_yaml=resolve_map_files(path)[0],
                      resolution_m=occupancy.resolution_m,
                      free_bounds=_free_bounds(occupancy),
                      free_area_m2=float(occupancy.region.sum()) * occupancy.resolution_m ** 2,
                      rectangle=rectangle, route=route, errors=errors)


def require(path, kind):
    """Derived overrides for ``kind`` in ``{'rectangle', 'route'}``.

    Only the requested geometry is computed: the route search is quadratic in
    the number of wall lines, so a closed-pool launch must not pay for it.
    """
    if kind not in ("rectangle", "route"):
        raise ValueError("kind must be 'rectangle' or 'route'")
    return {"rectangle": rectangle_profile, "route": route_profile}[kind](
        load_occupancy(path))


def describe(path):
    """Operator-facing summary used by the CLI and by launch logging."""
    derived = profile(path)
    summary = dict(map=str(derived.map_yaml),
                   resolution_m=derived.resolution_m,
                   free_area_m2=round(derived.free_area_m2, 2),
                   free_bounds=[round(v, 3) for v in derived.free_bounds])
    if derived.rectangle:
        summary["rectangle"] = dict(
            origin_x_m=round(derived.rectangle.origin_x_m, 6),
            origin_y_m=round(derived.rectangle.origin_y_m, 6),
            axis_yaw_rad=round(derived.rectangle.axis_yaw_rad, 6),
            length_m=round(derived.rectangle.length_m, 4),
            width_m=round(derived.rectangle.width_m, 4),
            fill_fraction=round(derived.rectangle.fill_fraction, 3),
            boundary_xy=[round(v, 4) for v in derived.rectangle.boundary_xy])
    if derived.route:
        summary["route"] = dict(
            route_xy=[round(v, 4) for v in derived.route.route_xy],
            leg_lengths_m=[round(v, 3) for v in derived.route.leg_lengths_m],
            turn_deg=round(math.degrees(derived.route.turn_rad), 2),
            quality=round(derived.route.quality, 3))
    summary.update(errors=derived.errors)
    return summary


def main(args=None):
    """``ros2 run f1_pool_cleaning map_profile <map>`` diagnostic entry point."""
    import argparse
    import json

    parser = argparse.ArgumentParser(
        description="Derive cleaning geometry from a saved map and report it.")
    parser.add_argument("map", help=".pbstream or map .yaml")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    options = parser.parse_args(args)
    try:
        summary = describe(options.map)
    except MapProfileError as exc:
        parser.error(str(exc))
    print(json.dumps(summary, indent=2, ensure_ascii=False) if options.json
          else "\n".join(f"{key}: {value}" for key, value in summary.items()))
    # A map that drives neither task is a configuration error, not a warning.
    if summary["errors"].get("rectangle") and summary["errors"].get("route"):
        raise SystemExit(1)
    return 0
