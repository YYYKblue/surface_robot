"""Extract directed open straight-wall chains from the water boundary.

Contour order supplies topology; line intersections only refine nearby joins.
Unknown-space frontiers, unsupported gaps and smooth arcs break a chain. This
module does not test vessel footprints or spacing between neighbouring turns.
"""
from dataclasses import dataclass
import math

from f1_localization.map_io import MapProfileError


MIN_SEGMENT_M = .65
JOIN_GAP_M = .50
LINE_ERROR_M = .10


@dataclass
class Piece:
    points: object
    curved: bool = False
    gap: bool = False

    @property
    def start(self):
        return tuple(self.points[0])

    @property
    def end(self):
        return tuple(self.points[-1])

    @property
    def length(self):
        return math.dist(self.start, self.end)

    @property
    def yaw(self):
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])


def turn(first, second):
    angle = second.yaw - first.yaw
    return math.atan2(math.sin(angle), math.cos(angle))


def _smooth_arc(pieces, resolution):
    """Require smooth curvature evidence, not merely several same-sign turns.

    Real polygon corners can also turn repeatedly in one direction. Their
    source samples must not be discarded solely because of the turn angles.
    """
    import numpy as np

    points = np.vstack([p.points for p in pieces])
    axis = points[-1] - points[0]
    length = float(np.linalg.norm(axis))
    if length < 1e-6:
        return False
    axis /= length
    xy = (points - points[0]) @ np.column_stack((axis, [-axis[1], axis[0]]))
    if np.any(np.diff(xy[:, 0]) < -resolution):
        return False
    fit = np.polynomial.polynomial.polyfit(xy[:, 0], xy[:, 1], 2)
    residual = np.abs(np.polynomial.polynomial.polyval(xy[:, 0], fit) - xy[:, 1])
    return float(np.quantile(residual, .95)) <= resolution


def _pieces(occupancy):
    import cv2
    import numpy as np

    contours, _ = cv2.findContours(occupancy.region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return []
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
    # Clockwise in map coordinates: the water is on the right of travel.
    if cv2.contourArea(contour, oriented=True) < 0:
        contour = contour[::-1].copy()
    vertices = cv2.approxPolyDP(contour, max(1., .075 / occupancy.resolution_m), True).reshape(-1, 2)
    offset = int(np.flatnonzero(np.all(contour == vertices[0], axis=1))[0])
    contour = np.roll(contour, -offset, axis=0)
    indices, cursor = [], 0
    for vertex in vertices:
        matches = np.flatnonzero(np.all(contour[cursor:] == vertex, axis=1))
        if not len(matches):
            raise MapProfileError("water contour cannot be ordered into wall segments")
        index = cursor + int(matches[0])
        indices.append(index)
        cursor = index + 1
    pixels = np.vstack((contour, contour[:1]))
    pieces = [Piece(np.column_stack(occupancy.xy(pixels[a:b + 1, 0], pixels[a:b + 1, 1])))
              for a, b in zip(indices, indices[1:] + [len(contour)])]

    # A series of small same-sign bends is a smooth arc, not several new
    # cleaning walls. Keep those breaks even when adjacent chords fit well.
    for i in range(len(pieces)):
        triple = [pieces[(i + j) % len(pieces)] for j in range(3)]
        a, b = turn(triple[0], triple[1]), turn(triple[1], triple[2])
        if (all(p.length >= MIN_SEGMENT_M for p in triple) and a * b > 0
                and all(math.radians(2) <= abs(v) <= math.radians(40) for v in (a, b))
                and abs(a + b) > math.radians(10)
                and _smooth_arc(triple, occupancy.resolution_m)):
            for piece in triple:
                piece.curved = True

    # Merge raster stair-steps and tiny corner blemishes, checking all source
    # contour samples rather than extending Hough lines through unknown gaps.
    changed = True
    while changed and len(pieces) > 2:
        changed = False
        for i in range(len(pieces)):
            j = (i + 1) % len(pieces)
            first, second = pieces[i], pieces[j]
            if first.curved or second.curved or abs(turn(first, second)) >= math.pi / 2:
                continue
            if abs(turn(first, second)) > math.radians(15) and min(first.length, second.length) > .45:
                continue
            points = np.vstack((first.points, second.points[1:]))
            delta = points[-1] - points[0]
            length = float(np.linalg.norm(delta))
            if length <= 1e-8:
                continue
            normal = np.array([-delta[1], delta[0]]) / length
            if float(np.max(np.abs((points - points[0]) @ normal))) > max(LINE_ERROR_M, occupancy.resolution_m * 2):
                continue
            merged = Piece(points)
            if j == 0:
                pieces = [merged] + pieces[1:i]
            else:
                pieces[i:j + 1] = [merged]
            changed = True
            break
    return pieces


def _join(first, second):
    """Refine a real neighbouring corner; never join infinite extensions."""
    import numpy as np

    angle = abs(turn(first, second))
    if not 1e-3 <= angle < math.pi - 1e-3:
        return None
    a, b, c, d = (np.asarray(p) for p in (first.start, first.end, second.start, second.end))
    u, v = b - a, d - c
    matrix = np.column_stack((u, -v))
    if abs(float(np.linalg.det(matrix))) < 1e-9:
        return None
    intersection = a + float(np.linalg.solve(matrix, c - a)[0]) * u
    if max(math.dist(intersection, b), math.dist(intersection, c)) > JOIN_GAP_M:
        return None
    return tuple(float(x) for x in intersection)


def _split_support(pieces, occupancy, supported):
    """Split long fitted lines at missing wall evidence, not just mean score."""
    import numpy as np

    result = []
    for piece in pieces:
        if piece.curved or piece.length < MIN_SEGMENT_M:
            result.append(piece)
            continue
        count = max(2, math.ceil(piece.length / max(.05, occupancy.resolution_m)) + 1)
        samples = np.linspace(piece.start, piece.end, count)
        ux, uy = math.cos(piece.yaw), math.sin(piece.yaw)
        good = np.array([supported(occupancy, *p, ux, uy) for p in samples])
        spacing = piece.length / (count - 1)
        # Tolerate small raster holes, but preserve sustained unknown spans.
        i = 0
        while i < count:
            j = i + 1
            while j < count and good[j] == good[i]:
                j += 1
            if not good[i] and (j - i) * spacing <= .20 + 1e-9 and good.any():
                good[i:j] = True
            i = j
        i = 0
        while i < count:
            j = i + 1
            while j < count and good[j] == good[i]:
                j += 1
            a = samples[0] if i == 0 else (samples[i - 1] + samples[i]) / 2
            b = samples[-1] if j == count else (samples[j - 1] + samples[j]) / 2
            result.append(Piece(np.array([a, b]), gap=not bool(good[i])))
            i = j
    return result


def wall_chains(occupancy, quality, supported, minimum_quality=.75):
    """Return all supported open chains, longest first, with quality scores.

    Separate chains are reported independently: no cleaning edge or transit
    is invented between them. Fully closed contours are not open routes.
    """
    pieces = _split_support(_pieces(occupancy), occupancy, supported)
    accepted = [i for i, p in enumerate(pieces)
                if not p.curved and not p.gap and p.length >= MIN_SEGMENT_M
                and quality(occupancy, p.start, p.end) >= minimum_quality]
    if not accepted:
        raise MapProfileError("no supported open straight-wall chain found")
    joins = [None] * len(accepted)
    for k, i in enumerate(accepted):
        j = accepted[(k + 1) % len(accepted)]
        skipped = []
        cursor = (i + 1) % len(pieces)
        while cursor != j:
            skipped.append(pieces[cursor])
            cursor = (cursor + 1) % len(pieces)
        if any(p.curved or p.gap for p in skipped) or sum(p.length for p in skipped) > JOIN_GAP_M:
            continue
        if len(accepted) > 1:
            joins[k] = _join(pieces[i], pieces[j])
    starts = [k for k in range(len(accepted)) if joins[(k - 1) % len(accepted)] is None]
    chains = []
    for start in starts:
        first = pieces[accepted[start]]
        points = [first.start]
        cursor = start
        while joins[cursor] is not None:
            points.append(joins[cursor])
            cursor = (cursor + 1) % len(accepted)
        points.append(pieces[accepted[cursor]].end)
        lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
        if min(lengths) < MIN_SEGMENT_M:
            continue
        scores = [quality(occupancy, a, b) for a, b in zip(points, points[1:])]
        if min(scores) < minimum_quality:
            continue
        # Geometry validation only. Do not introduce footprint or turn-space
        # checks here; execution retains the configured per-edge margins.
        from .geometry import PoolModel
        try:
            PoolModel.from_polyline(points)
        except ValueError:
            continue
        chains.append((tuple(points), min(scores)))
    if not chains:
        raise MapProfileError("no open wall route; supported boundaries are closed or degenerate")
    chains.sort(key=lambda item: (-sum(math.dist(a, b) for a, b in zip(item[0], item[0][1:])),
                                  -item[1], item[0]))
    return chains
