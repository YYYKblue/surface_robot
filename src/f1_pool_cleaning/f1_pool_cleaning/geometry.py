"""Pool boundary and straight-edge geometry used by the stage-2 task.

The map is the global source of truth for edge identity and along-edge
progress.  The local wall observation is still used for the actual wall
following controller.  Edges are oriented so that the wall is on the boat's
left while travelling along the edge; this is the only orientation supported
by the single left-side brush.
"""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_point(point):
    return (len(point) == 2 and all(math.isfinite(float(v)) for v in point))


@dataclass(frozen=True)
class PoseProjection:
    s_m: float
    lateral_distance_m: float
    heading_error_rad: float
    distance_to_segment_m: float
    contact_s_m: float
    # Offset of the nearest point on the map edge in the robot frame.
    # Positive left_offset_m means the edge is on base_link +y.
    left_offset_m: float = 0.
    forward_offset_m: float = 0.


@dataclass(frozen=True)
class EdgeSegment:
    id: str
    start: tuple
    end: tuple
    length_m: float
    tangent: tuple
    left_normal: tuple
    yaw_rad: float
    entry_s_m: float
    clean_start_s_m: float
    clean_end_s_m: float
    stop_s_m: float
    wall_target_m: float
    corner_exclusion_m: float
    endpoint_margin_m: float

    def project(self, pose, brush_x_m=.10, brush_tip_y_m=.33):
        """Project a base pose and its left brush contact onto this edge."""
        px, py = pose.x, pose.y
        dx, dy = px - self.start[0], py - self.start[1]
        tx, ty = self.tangent
        nx, ny = self.left_normal
        s = dx * tx + dy * ty
        # The interior is to the right of the clockwise route.  A positive
        # value therefore means the boat is inside the pool relative to wall.
        lateral = -(dx * nx + dy * ny)
        clamped = min(self.length_m, max(0., s))
        cx = self.start[0] + clamped * tx
        cy = self.start[1] + clamped * ty
        distance = math.hypot(px - cx, py - cy)
        yaw = getattr(pose, "yaw", self.yaw_rad)
        c, sn = math.cos(yaw), math.sin(yaw)
        edge_dx, edge_dy = cx - px, cy - py
        left_offset = -sn * edge_dx + c * edge_dy
        forward_offset = c * edge_dx + sn * edge_dy
        brush_x = px + c * brush_x_m - sn * brush_tip_y_m
        brush_y = py + sn * brush_x_m + c * brush_tip_y_m
        contact_s = (brush_x - self.start[0]) * tx + (brush_y - self.start[1]) * ty
        return PoseProjection(s, lateral, wrap(self.yaw_rad - yaw), distance, contact_s,
                              left_offset, forward_offset)


class PoolModel:
    """Validated ordered boundary and its left-wall edge segments."""

    def __init__(self, edges, frame="map"):
        self.edges = tuple(edges)
        self.frame = frame
        self.by_id = {edge.id: edge for edge in self.edges}

    @staticmethod
    def _signed_area(points):
        return .5 * sum(a[0] * b[1] - b[0] * a[1]
                        for a, b in zip(points, points[1:] + points[:1]))

    @staticmethod
    def _segments_cross(a, b, c, d):
        def orient(p, q, r):
            return ((q[0] - p[0]) * (r[1] - p[1])
                    - (q[1] - p[1]) * (r[0] - p[0]))

        def between(p, q, r):
            return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
                    and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)

        ab_c, ab_d = orient(a, b, c), orient(a, b, d)
        cd_a, cd_b = orient(c, d, a), orient(c, d, b)
        if ab_c * ab_d < -1e-12 and cd_a * cd_b < -1e-12:
            return True
        return ((abs(ab_c) <= 1e-12 and between(a, c, b))
                or (abs(ab_d) <= 1e-12 and between(a, d, b))
                or (abs(cd_a) <= 1e-12 and between(c, a, d))
                or (abs(cd_b) <= 1e-12 and between(c, b, d)))

    @classmethod
    def from_boundary(cls, points, *, boundary_order="clockwise", wall_target_m=.34,
                      corner_exclusion_m=.30, endpoint_margin_m=.50, frame="map"):
        points = [tuple(float(v) for v in p) for p in points]
        if len(points) < 3 or any(not _finite_point(p) for p in points):
            raise ValueError("boundary needs at least three finite [x, y] points")
        if points[0] == points[-1]:
            points.pop()
        if len(points) < 3:
            raise ValueError("boundary has too few distinct points")
        for a, b in zip(points, points[1:] + points[:1]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) <= 1e-6:
                raise ValueError("boundary contains a zero-length edge")
        segments = list(zip(points, points[1:] + points[:1]))
        for i, (a, b) in enumerate(segments):
            for j in range(i + 1, len(segments)):
                # Adjacent segments meet at their shared vertex by design.
                if j in (i - 1, i + 1) or (i == 0 and j == len(segments) - 1):
                    continue
                if cls._segments_cross(a, b, *segments[j]):
                    raise ValueError("boundary self-intersects")
        order = str(boundary_order).lower()
        if order not in ("clockwise", "counterclockwise"):
            raise ValueError("boundary_order must be clockwise or counterclockwise")
        # A clockwise route has the pool interior to the right and the wall to
        # the left.  The declared order documents the input; the generated
        # route is always canonical clockwise for the single left brush.
        area = cls._signed_area(points)
        is_clockwise = area < 0.
        if not is_clockwise:
            points = [points[0]] + list(reversed(points[1:]))
        if corner_exclusion_m < 0 or endpoint_margin_m <= 0:
            raise ValueError("edge margins must be nonnegative and endpoint margin positive")
        edges = []
        for index, (start, end) in enumerate(zip(points, points[1:] + points[:1])):
            vx, vy = end[0] - start[0], end[1] - start[1]
            length = math.hypot(vx, vy)
            tx, ty = vx / length, vy / length
            nx, ny = -ty, tx
            margin = max(corner_exclusion_m, endpoint_margin_m)
            if length <= 2 * margin:
                raise ValueError(f"edge_{index} is too short for configured endpoint margins")
            edges.append(EdgeSegment(
                id=f"edge_{index}", start=start, end=end, length_m=length,
                tangent=(tx, ty), left_normal=(nx, ny), yaw_rad=math.atan2(ty, tx),
                entry_s_m=margin, clean_start_s_m=corner_exclusion_m,
                clean_end_s_m=length - corner_exclusion_m,
                stop_s_m=length - margin, wall_target_m=wall_target_m,
                corner_exclusion_m=corner_exclusion_m,
                endpoint_margin_m=endpoint_margin_m))
        return cls(edges, frame=frame)

    @classmethod
    def rectangle(cls, config, **kwargs):
        """Build the legacy map rectangle in the canonical clockwise route.

        The first edge is the historical top/left-wall edge used by the
        single-side controller, so existing synthetic feedback remains useful.
        """
        points = [
            config.map_xy(0., config.width_m),
            config.map_xy(config.length_m, config.width_m),
            config.map_xy(config.length_m, 0.),
            config.map_xy(0., 0.),
        ]
        return cls.from_boundary(points, boundary_order="clockwise",
                                 wall_target_m=config.wall_target_m, **kwargs)

    def select(self, pose, *, max_distance_m=.75, heading_tolerance_rad=math.radians(35.),
               edge_id=None, brush_x_m=.10, brush_tip_y_m=.33):
        """Select the nearest feasible edge, or a requested edge.

        A requested edge is still checked for distance and left-side location.
        Heading is corrected after the left edge is selected; it is not a
        prerequisite for finding the nearest wall on base_link +y.
        """
        candidates = []
        diagnostics = []
        for edge in self.edges:
            if edge_id and edge.id != edge_id:
                continue
            projection = edge.project(pose, brush_x_m, brush_tip_y_m)
            diagnostics.append(
                f"{edge.id}: distance={projection.distance_to_segment_m:.2f}m, "
                f"left={projection.left_offset_m:.2f}m, "
                f"lateral={projection.lateral_distance_m:.2f}m, "
                f"heading={math.degrees(projection.heading_error_rad):.1f}deg")
            if projection.distance_to_segment_m > max_distance_m:
                continue
            if projection.left_offset_m <= 0.:
                continue
            # Canonical clockwise edges have the pool interior on the right.
            # A pose outside the boundary would put this wall on the boat's
            # right, which is unsupported by the single left-side brush.
            if projection.lateral_distance_m <= 0.:
                continue
            candidates.append((projection.distance_to_segment_m, edge, projection))
        if not candidates:
            return None, None, ("no edge matches the robot's left side; "
                                + "; ".join(diagnostics))
        candidates.sort(key=lambda item: item[0])
        if not edge_id and len(candidates) > 1:
            if abs(candidates[1][0] - candidates[0][0]) < .08:
                return None, None, "edge selection ambiguous near a corner"
        _, edge, projection = candidates[0]
        if projection.s_m > edge.stop_s_m:
            return None, None, "localized pose is past the safe start/stop window"
        return edge, projection, "edge selected"
