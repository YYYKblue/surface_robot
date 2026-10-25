"""Fixed-map localization lifecycle and conservative readiness checks.

The Cartographer node owns scan matching and the ``map -> base_link`` TF.  This
module owns the part around it: accepting an operator supplied initial pose,
starting one pure-localization trajectory, and deciding when that pose has
been fresh and stable long enough for a cleaning task to use it.

The small :class:`LocalizationGate` class deliberately has no ROS dependency;
it is also used by tests and keeps the safety policy separate from service and
TF plumbing in :mod:`localization_node`.
"""

from dataclasses import dataclass
import json
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    norm = sum(value * value for value in values)
    if not all(math.isfinite(value) for value in values) or not math.isfinite(norm):
        raise ValueError("nonfinite quaternion")
    if norm < 1e-9 or abs(norm - 1.0) > .02:
        raise ValueError("quaternion is not normalized")
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def occupied_map_overlap(points, resolution, width, height, data,
                         origin_x, origin_y, origin_yaw,
                         radius_m=.15, occupied_threshold=50):
    """Return the fraction of scan endpoints near occupied map cells.

    ``points`` are already expressed in the occupancy grid's frame.  The
    radius accounts for map rasterization and lidar/map calibration error; it
    does not turn the rectangle boundary into a map-match signal.
    """
    if (not points or not math.isfinite(resolution) or resolution <= 0
            or width <= 0 or height <= 0 or len(data) < width * height
            or not math.isfinite(radius_m) or radius_m < 0):
        return None
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    radius_cells = int(math.ceil(radius_m / resolution))
    matches = 0
    for x, y in points:
        dx, dy = x - origin_x, y - origin_y
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        column = math.floor(local_x / resolution)
        row = math.floor(local_y / resolution)
        found = False
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(-radius_cells, radius_cells + 1):
                test_row = row + row_offset
                test_column = column + column_offset
                if not (0 <= test_row < height and 0 <= test_column < width):
                    continue
                value = data[test_row * width + test_column]
                if value >= occupied_threshold:
                    found = True
                    break
            if found:
                break
        matches += int(found)
    return matches / len(points)


@dataclass(frozen=True)
class LocalizationConfig:
    map_frame: str = "map"
    base_frame: str = "base_link"
    pose_max_age_sec: float = .50
    scan_max_age_sec: float = .50
    stable_sec: float = 1.00
    jump_distance_m: float = .35
    jump_angle_rad: float = math.radians(25.)
    boundary_margin_m: float = .25
    scan_overlap_min: float = .10
    scan_valid_fraction_min: float = .05
    scan_map_match_radius_m: float = .15
    initial_pose_timeout_sec: float = 5.0
    status_max_age_sec: float = .60
    origin_x_m: float = -0.9802816886364385
    origin_y_m: float = -0.18424262645062695
    axis_yaw_rad: float = -0.19739548348805863
    length_m: float = 2.8829072052510294
    width_m: float = 1.5199000413306079

    def validate(self):
        if not self.map_frame or not self.base_frame:
            raise ValueError("localization frames must be nonempty")
        for name in ("pose_max_age_sec", "scan_max_age_sec", "stable_sec",
                     "jump_distance_m", "jump_angle_rad", "boundary_margin_m",
                     "scan_overlap_min", "scan_valid_fraction_min",
                     "scan_map_match_radius_m",
                     "initial_pose_timeout_sec", "status_max_age_sec",
                     "length_m", "width_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not 0 <= self.scan_overlap_min <= 1 or not 0 <= self.scan_valid_fraction_min <= 1:
            raise ValueError("scan overlap thresholds must be between zero and one")
        if self.length_m <= 0 or self.width_m <= 0:
            raise ValueError("map dimensions must be positive")
        return self

    def pool_xy(self, x, y):
        dx, dy = x - self.origin_x_m, y - self.origin_y_m
        c, s = math.cos(self.axis_yaw_rad), math.sin(self.axis_yaw_rad)
        return c * dx + s * dy, -s * dx + c * dy


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ScanQuality:
    stamp_ns: int
    valid_fraction: float
    overlap_fraction: float | None
    frame_id: str = ""


class LocalizationGate:
    """Stateful readiness policy independent of ROS messages.

    ``update`` expects source timestamps in the same clock domain as ``now``.
    A source can be unavailable while the gate is checking, but readiness is
    only granted after trajectory, pose, scan and map-boundary checks have all
    been continuously valid for ``stable_sec``.
    """

    def __init__(self, config=None):
        self.config = (config or LocalizationConfig()).validate()
        self.state = "LOCALIZATION_CHECK"
        self.ready = False
        self.reason = "waiting for RViz 2D Pose Estimate"
        self.initial_pose_received = False
        self.initial_pose_source = ""
        self.initial_pose_stamp_ns = 0
        self.stable_since_ns = None
        self.last_pose = None
        self.last_pose_jump = None
        self.pose = None
        self.scan = None
        self.trajectory_active = False
        self.scan_overlap = None

    def reset(self, reason="waiting for RViz 2D Pose Estimate"):
        self.state = "LOCALIZATION_CHECK"
        self.ready = False
        self.reason = reason
        self.initial_pose_received = False
        self.initial_pose_source = ""
        self.initial_pose_stamp_ns = 0
        self.stable_since_ns = None
        self.last_pose = None
        self.last_pose_jump = None
        self.pose = None
        self.scan = None
        self.trajectory_active = False
        self.scan_overlap = None

    def accept_initial_pose(self, stamp_ns, source="rviz"):
        if stamp_ns <= 0:
            raise ValueError("initial pose stamp must be positive")
        self.state = "LOCALIZATION_CHECK"
        self.ready = False
        self.reason = "initial pose accepted; waiting for localization trajectory"
        self.initial_pose_received = True
        self.initial_pose_source = source
        self.initial_pose_stamp_ns = stamp_ns
        self.stable_since_ns = None
        self.last_pose = None
        self.last_pose_jump = None

    def mark_paused(self, reason):
        self.ready = False
        self.state = "PAUSED"
        self.reason = reason
        self.stable_since_ns = None

    def _boundary_ok(self, pose):
        x, y = self.config.pool_xy(pose.x, pose.y)
        margin = self.config.boundary_margin_m
        return (-margin <= x <= self.config.length_m + margin
                and -margin <= y <= self.config.width_m + margin)

    def update(self, now_ns, trajectory_active, pose, scan):
        self.trajectory_active = bool(trajectory_active)
        self.pose = pose
        self.scan = scan
        self.scan_overlap = None if scan is None else scan.overlap_fraction

        # A jump or an operator/trajectory failure is latched.  Fresh data is
        # evidence for a future relocalization, never permission to resume the
        # old task automatically.
        if self.state == "PAUSED":
            self.ready = False
            return self.ready

        if not self.initial_pose_received:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "waiting for RViz 2D Pose Estimate"
            self.stable_since_ns = None
            return self.ready
        if not self.trajectory_active:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "localization trajectory is not active"
            self.stable_since_ns = None
            return self.ready
        if pose is None or pose.stamp_ns <= 0:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "map to base_link pose unavailable"
            self.stable_since_ns = None
            return self.ready
        pose_age = (now_ns - pose.stamp_ns) / 1e9
        if pose_age < 0 or pose_age >= self.config.pose_max_age_sec:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "map to base_link pose is stale"
            self.stable_since_ns = None
            return self.ready
        if not self._boundary_ok(pose):
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "localized pose is outside the configured map boundary"
            self.stable_since_ns = None
            return self.ready

        if self.last_pose is not None:
            distance = math.hypot(pose.x - self.last_pose.x, pose.y - self.last_pose.y)
            angle = abs(wrap(pose.yaw - self.last_pose.yaw))
            if distance > self.config.jump_distance_m or angle > self.config.jump_angle_rad:
                detail = f"pose jump: {distance:.2f} m, {math.degrees(angle):.1f} deg"
                self.last_pose_jump = detail
                self.stable_since_ns = None
                if self.ready:
                    self.mark_paused(detail)
                else:
                    self.state, self.ready = "LOCALIZATION_CHECK", False
                    self.reason = detail
                self.last_pose = pose
                return self.ready
        self.last_pose = pose

        if scan is None or scan.stamp_ns <= 0:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "laser scan unavailable"
            self.stable_since_ns = None
            return self.ready
        scan_age = (now_ns - scan.stamp_ns) / 1e9
        if scan_age < 0 or scan_age >= self.config.scan_max_age_sec:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "laser scan is stale"
            self.stable_since_ns = None
            return self.ready
        if scan.valid_fraction < self.config.scan_valid_fraction_min:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "laser scan has too few valid returns"
            self.stable_since_ns = None
            return self.ready
        if scan.overlap_fraction is None:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = "scan to map overlap is unknown"
            self.stable_since_ns = None
            return self.ready
        if scan.overlap_fraction < self.config.scan_overlap_min:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = (f"scan/map overlap too low: {scan.overlap_fraction:.2f} "
                           f"< {self.config.scan_overlap_min:.2f}")
            self.stable_since_ns = None
            return self.ready

        if self.stable_since_ns is None:
            self.stable_since_ns = now_ns
        elapsed = (now_ns - self.stable_since_ns) / 1e9
        if elapsed < self.config.stable_sec:
            self.state, self.ready = "LOCALIZATION_CHECK", False
            self.reason = f"localization stabilizing ({elapsed:.2f}/{self.config.stable_sec:.2f}s)"
            return self.ready
        self.state, self.ready = "LOCALIZATION_READY", True
        self.reason = "localization stable; map to base_link is ready"
        return self.ready

    def as_dict(self, now_ns):
        pose_age = None if self.pose is None else (now_ns - self.pose.stamp_ns) / 1e9
        scan_age = None if self.scan is None else (now_ns - self.scan.stamp_ns) / 1e9
        return dict(state=self.state, ready=self.ready, reason=self.reason,
                    initial_pose_received=self.initial_pose_received,
                    initial_pose_source=self.initial_pose_source,
                    initial_pose_stamp_ns=self.initial_pose_stamp_ns,
                    trajectory_active=self.trajectory_active,
                    pose_fresh=pose_age is not None and 0 <= pose_age < self.config.pose_max_age_sec,
                    pose_age_sec=pose_age,
                    scan_fresh=scan_age is not None and 0 <= scan_age < self.config.scan_max_age_sec,
                    scan_age_sec=scan_age,
                    scan_valid_fraction=None if self.scan is None else self.scan.valid_fraction,
                    scan_map_overlap=self.scan_overlap,
                    pose_jump=self.last_pose_jump,
                    stable_sec=self.config.stable_sec,
                    stable_elapsed_sec=None if self.stable_since_ns is None else
                    max(0., (now_ns - self.stable_since_ns) / 1e9))


def validate_initial_pose(message, map_frame):
    """Validate an RViz pose message and return ``(ok, reason, yaw)``."""
    if message.header.frame_id != map_frame:
        return False, f"initial pose frame must be {map_frame!r}", None
    pose = message.pose.pose
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w)
    if not all(math.isfinite(value) for value in values):
        return False, "initial pose contains a nonfinite value", None
    try:
        yaw = yaw_from_quaternion(pose.orientation)
    except ValueError as exc:
        return False, str(exc), None
    if abs(pose.position.z) > .25:
        return False, "initial pose is not planar", None
    return True, "", yaw


def json_status(status):
    """Serialize status with strict JSON so diagnostics never contain NaN."""
    return json.dumps(status, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
