"""Coverage planning on the selected boat-centre component of a frozen map."""
from copy import copy
from dataclasses import dataclass
import heapq
import math

import cv2
import numpy as np

from f1_localization.map_io import MapProfileError


@dataclass(frozen=True)
class Leg:
    start: tuple
    end: tuple
    kind: str  # lane, transfer, entry


class WaterGrid:
    def __init__(self, occupancy, polygon=()):
        self.map = occupancy
        self.res = occupancy.resolution_m
        free = occupancy.free.copy()
        if polygon:
            if len(polygon) < 6 or len(polygon) % 2 or not np.isfinite(polygon).all():
                raise ValueError("work_polygon_xy requires at least three finite map-frame vertices")
            vertices = [self.cell(x, y)[::-1] for x, y in zip(polygon[::2], polygon[1::2])]
            roi = np.zeros_like(free)
            cv2.fillPoly(roi, [np.int32(vertices)], 1)
            free &= roi
        self.free = free
        _, self.labels = cv2.connectedComponents(free, connectivity=4)

    def cell(self, x, y):
        col = math.floor((x - self.map.origin_x_m) / self.res)
        row = self.map.height - 1 - math.floor((y - self.map.origin_y_m) / self.res)
        return row, col

    def xy(self, cell):
        return tuple(float(v) for v in self.map.xy(cell[1], cell[0]))

    def label(self, xy):
        r, c = self.cell(*xy)
        if 0 <= r < self.map.height and 0 <= c < self.map.width:
            return int(self.labels[r, c])
        return 0

    def clear(self, start, end):
        """Supercover traversal, including both neighbours at a grid corner."""
        r, c = self.cell(*start)
        er, ec = self.cell(*end)
        if not self.label(start) or not self.label(end):
            return False
        dx, dy = (end[0]-start[0])/self.res, (end[1]-start[1])/self.res
        sx, sy = int(np.sign(dx)), int(np.sign(dy))
        x = (start[0]-self.map.origin_x_m)/self.res
        y = (start[1]-self.map.origin_y_m)/self.res
        tx = (c + (sx > 0) - x)/dx if sx else math.inf
        ty = (self.map.height-1-r + (sy > 0) - y)/dy if sy else math.inf
        while (r, c) != (er, ec):
            ax = tx if c != ec else math.inf
            ay = ty if r != er else math.inf
            if abs(ax-ay) < 1e-10:
                if not self.free[r, c+sx] or not self.free[r-sy, c]:
                    return False
            if ax <= ay + 1e-10:
                c += sx
                tx += 1/abs(dx)
            if ay <= ax + 1e-10:
                r -= sy
                ty += 1/abs(dy)
            if not self.free[r, c]:
                return False
        return True

    def connect(self, start, end):
        if self.clear(start, end):
            return [start, end]
        source, target = self.cell(*start), self.cell(*end)
        if not self.label(start) or self.label(start) != self.label(end):
            raise MapProfileError("连接路径不在同一片可航水域")
        queue = [(0, 0, source)]
        cost, parent = {source: 0}, {}
        while queue:
            _, g, here = heapq.heappop(queue)
            if here == target:
                break
            if g != cost[here]:
                continue
            r, c = here
            for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = r+dr, c+dc
                if not (0 <= nxt[0] < self.map.height and 0 <= nxt[1] < self.map.width):
                    continue
                if not self.free[nxt] or (dr and dc and not (self.free[r+dr, c] and self.free[r, c+dc])):
                    continue
                distance = g + (math.sqrt(2) if dr and dc else 1.)
                if distance < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = distance, here
                    heapq.heappush(queue, (distance+math.dist(nxt, target), distance, nxt))
        path = [target]
        while path[-1] != source:
            path.append(parent[path[-1]])
        points = [start, *(self.xy(p) for p in reversed(path)), end]
        result = [start]
        for i in range(1, len(points)-1):
            if not self.clear(result[-1], points[i+1]):
                result.append(points[i])
        return result + [end]


def runs(line):
    changes = np.flatnonzero(np.diff(np.pad(line.astype(np.int8), 1)))
    return list(zip(changes[::2].tolist(), (changes[1::2]-1).tolist()))


def sweep_coordinates(component, start_cell):
    """Digital parallel lines whose normal coordinate is zero at the boat."""
    rows, cols = np.nonzero(component)
    points = np.column_stack((cols, rows)).astype(np.float64)
    # A square or a single cell has no preferred long axis. Use map x on ties.
    axis = np.array([1., 0.])
    if len(points) >= 2:
        _, vectors, values = cv2.PCACompute2(points, mean=None)
        if values[0, 0] > values[1, 0] * 1.001:
            axis = vectors[0]
    transpose = abs(axis[1]) > abs(axis[0])
    major, minor = (axis[1], axis[0]) if transpose else axis
    slope = minor / major
    row, col = start_cell[::-1] if transpose else start_cell
    width = component.shape[0 if transpose else 1]
    baseline = row + np.rint((np.arange(width) - col) * slope).astype(int)
    return transpose, baseline, math.hypot(1., slope)


def larger_side(grid, component, start_cell, transpose, baseline):
    """Count only cells reachable from the divider without crossing to the other side."""
    oriented = component.T if transpose else component
    offset = np.arange(oriented.shape[0])[:, None] - baseline
    row, col = start_cell[::-1] if transpose else start_cell
    choices = []
    for sign in (1, -1):
        candidate = (oriented & (sign * offset >= 0)).astype(np.uint8)
        count, _, _, _ = cv2.floodFill(candidate, None, (col, row), 2, flags=4)
        choices.append((count, sign, candidate == 2))
    _, sign, selected = max(choices, key=lambda item: item[0])
    selected = selected.T if transpose else selected
    # Restrict every planned connector and shortcut to the chosen half.
    local = copy(grid)
    local.free = np.ascontiguousarray(selected, dtype=np.uint8)
    local.labels = local.free
    return local, selected, sign


@dataclass
class Plan:
    legs: list
    region: np.ndarray
    band: np.ndarray
    disconnected_m2: float
    skipped_m2: float
    spacing: float


def mark_band(mask, grid, start, end, width, region=None):
    a, b = grid.cell(*start), grid.cell(*end)
    # Disk in metres around the observed segment, sampled at cell centres.
    radius = width/(2*grid.res)
    # Clip against the bounding box before computing segment distances.
    lo = np.maximum(np.minimum(a, b)-math.ceil(radius), 0)
    hi = np.minimum(np.maximum(a, b)+math.ceil(radius)+1, mask.shape)
    rr, cc = np.ogrid[lo[0]:hi[0], lo[1]:hi[1]]
    x, y = grid.map.xy(cc, rr)
    dx, dy = end[0]-start[0], end[1]-start[1]
    t = np.clip(((x-start[0])*dx+(y-start[1])*dy)/(dx*dx+dy*dy), 0, 1) if dx or dy else 0
    view = np.s_[lo[0]:hi[0], lo[1]:hi[1]]
    before = np.count_nonzero(mask[view])
    covered = (x-start[0]-t*dx)**2+(y-start[1]-t*dy)**2 <= (width/2)**2+1e-12
    mask[view] |= covered if region is None else covered & region[view]
    return int(np.count_nonzero(mask[view])-before)


def plan(grid, start, spacing):
    if not math.isfinite(spacing) or spacing < grid.res:
        raise MapProfileError("航线间距必须至少为一个地图栅格")
    label = grid.label(start)
    if not label:
        raise MapProfileError("船位在已知可航水域外")
    component = grid.labels == label
    start_cell = grid.cell(*start)
    transpose, baseline, scale = sweep_coordinates(component, start_cell)
    local, region, sign = larger_side(grid, component, start_cell, transpose, baseline)
    oriented = region.T if transpose else region
    rows, cols = np.nonzero(oriented)
    depth = int(np.max(sign * (rows - baseline[cols])))
    step = max(1, round(spacing * scale / grid.res))
    legs = []
    band = np.zeros_like(region)
    cursor = start
    forward = True
    offsets = list(range(0, depth + 1, step))
    # Even a narrow pool can support a return pass. Place it inside the
    # remaining width rather than on the last, often single-cell, tip.
    if len(offsets) == 1 and depth >= 2:
        offsets.append(max(1, round(.5 * depth)))
    for offset in offsets:
        indices = baseline + sign * offset
        columns = np.arange(len(indices))
        valid = (indices >= 0) & (indices < oriented.shape[0])
        valid[valid] &= oriented[indices[valid], columns[valid]]
        intervals = runs(valid)
        if not intervals:
            continue

        def xy(column):
            cell = (int(indices[column]), column)
            return grid.xy(cell[::-1] if transpose else cell)

        # On the divider, go straight from the boat down the longer available
        # stretch. Following rows alternate from the end just reached.
        if offset == 0:
            column = start_cell[0 if transpose else 1]
            left, right = next(pair for pair in intervals if pair[0] <= column <= pair[1])
            forward = right - column >= column - left
            intervals = [(column, right) if forward else (left, column)]
        for left, right in (intervals if forward else reversed(intervals)):
            # A jagged bank can intersect a straight chord between digital-line
            # endpoints. Split until every emitted sweep segment is certified.
            pending = [(left, right)]
            while pending:
                lo, hi = pending.pop()
                a, b = (xy(lo), xy(hi)) if forward else (xy(hi), xy(lo))
                if offset == 0 and (lo if forward else hi) == column:
                    a = start
                # The first chord lies on the divider. Its grid supercover
                # can touch either side by one cell; check the original free-space mask.
                check_grid = grid if offset == 0 else local
                if hi > lo and not check_grid.clear(a, b):
                    mid = (lo + hi) // 2
                    parts = [(lo, mid), (mid+1, hi)]
                    pending.extend(reversed(parts) if forward else parts)
                    continue
                if a == b:
                    continue
                path = local.connect(cursor, a)
                kind = "transfer" if legs else "entry"
                legs.extend(Leg(p, q, kind) for p, q in zip(path, path[1:]) if p != q)
                legs.append(Leg(a, b, "lane"))
                mark_band(band, grid, a, b, spacing)
                cursor = b
        if legs:
            forward = not forward

    if not legs:
        raise MapProfileError("所选水域没有可执行的巡游航段")
    area = grid.res**2
    return Plan(legs, region, band, float(grid.free.sum()-component.sum())*area,
                float(component.sum()-region.sum())*area, spacing)
