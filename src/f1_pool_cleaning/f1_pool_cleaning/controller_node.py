"""Rectangle edge-task ROS adapter with isolated preview and optional lease."""
from dataclasses import fields, MISSING, replace
import json
import math
import time
import uuid
from pathlib import Path

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_interfaces.msg import ActuatorCommand, EdgeTarget, WallObservation
from f1_interfaces.srv import SetEdge
from f1_pool_cleaning.ros_support import stamp, stamp_ns, from_observation_message

from .config import Config
from .geometry import PoolModel
from .mission import Mission, Feedback, Pose, Effort, command_fields
from .lease import Lease
from .autonomous import AutonomousMission
from .navigation import Navigator, NavigationConfig, NavigationScan


class PoolController(Node):
    NODE_NAME = "pool_controller"
    EDGE_TASK_DEFAULT = False
    # Which map-derived geometry this controller consumes: a closed rectangle
    # boundary or an open L route.  The launch passes the map through unchanged.
    MAP_PROFILE_KIND = "rectangle"

    def __init__(self, **kwargs):
        super().__init__(self.NODE_NAME, **kwargs)
        for name, default in (("output_enabled", False), ("use_map_tf", False),
                              ("require_localization_ready", False),
                              ("localization_status_max_age_sec", .60)):
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.live = self.get_parameter("output_enabled").value
        self.use_map_tf = self.get_parameter("use_map_tf").value
        self.edge_task_enabled = self._declare_readonly("edge_task_enabled", self.EDGE_TASK_DEFAULT)
        self.require_localization_ready = (self.use_map_tf and
                                            self.get_parameter("require_localization_ready").value)
        self.localization_status_max_age_sec = self.get_parameter("localization_status_max_age_sec").value
        # ``map`` is the only map input.  When it is set, the geometry below is
        # derived from that file; the launch passes the same path, so swapping
        # maps never means re-deriving numbers by hand.
        self.map_path = self._declare_readonly("map", "")
        self.map_id = str(Path(self.map_path).expanduser().resolve().with_suffix("")) if self.map_path else ""
        self.run_id = ""
        self.completed_at_ns = None
        # Kept so subclasses can consume non-Config entries (the L route).
        self.derived_geometry = self._map_geometry()
        values = {}
        for field in fields(Config):
            # Map geometry has no default: it comes from map:= (or an explicit
            # override).  NaN keeps the parameter typed while validation
            # rejects a launch that supplied neither.
            default = self.derived_geometry.get(field.name, math.nan if field.default is MISSING
                                                else field.default)
            self.declare_parameter(field.name, default, ParameterDescriptor(read_only=True))
            values[field.name] = self.get_parameter(field.name).value
        missing = [name for name, value in values.items()
                   if isinstance(value, float) and math.isnan(value)]
        if missing:
            self.destroy_node()
            raise ValueError(
                f"map geometry {missing} was not supplied; launch with map:=<pbstream or map yaml>")
        self.config = Config(**values).validate()
        self.edge_distance_tolerance_m = self._declare_readonly(
            "edge_target_distance_tolerance_m", .20)
        self.edge_angle_tolerance_rad = self._declare_readonly(
            "edge_target_angle_tolerance_rad", math.radians(70.))
        self.boundary_order = self._declare_readonly("boundary_order", "clockwise")
        # ROS 2 cannot infer the type of an empty array parameter.  A single
        # zero is the typed "unset" sentinel; the map supplies the real corners.
        self.boundary_xy = self._declare_readonly(
            "boundary_xy", self.derived_geometry.get("boundary_xy") or [0.0])
        self.default_edge_id = self._declare_readonly("default_edge_id", "edge_0")
        self.start_policy = self._declare_readonly("start_policy", "specified")
        if self.start_policy not in ("specified", "nearest_reachable"):
            raise ValueError("start_policy must be specified or nearest_reachable")
        self.pool = self._build_pool()
        if self.live and (self.get_parameter("use_sim_time").value or not self.config.fifth_left_sign):
            self.destroy_node()
            raise ValueError("hardware output requires system time and known fifth_left_sign")
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.automatic = bool(self.edge_task_enabled and self.map_path)
        if self.start_policy == "nearest_reachable" and not self.automatic:
            raise ValueError("nearest_reachable requires map:= and edge_task_enabled=true")
        if self.automatic:
            from .map_profile import load_occupancy
            navigation = NavigationConfig(**{
                field.name: self._declare_readonly("navigation_"+field.name, field.default)
                for field in fields(NavigationConfig)})
            self.mission = AutonomousMission(
                self.config, self.pool, Navigator(load_occupancy(self.map_path), self.config, navigation))
        else:
            # Explicit, map-free fixtures and standalone legacy wall following.
            self.mission = Mission(self.config)
        self.requested_start_edge = None
        self.wall_after_ns = 0
        self.synced_target_generation = -1
        self.navigation_scan = None
        self.selected_edge = None
        self.edge_generation = 0
        self.edge_task_id = f"edge-task-{int(time.time() * 1000)}"
        self.wall = self.pose = None
        self.session, self.sequence = "", -1
        self.received = {}
        self.clock_pair = None
        self.clock_fault = False
        self.pose_covariance_available = False
        self.localization_ready = not self.require_localization_ready
        self.localization_state = "BYPASSED" if not self.require_localization_ready else "LOCALIZATION_CHECK"
        self.localization_reason = ("localization gate disabled" if not self.require_localization_ready
                                    else "waiting for localization status")
        self.localization_status_received = 0.
        self.localization_snapshot = {}
        self.preview_sequence = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wall_sub = self.create_subscription(WallObservation, "/pool_cleaning/wall_observation", self._wall, qos)
        self.edge_target_pub = self.create_publisher(EdgeTarget, "/pool_cleaning/edge_target", 2)
        if self.require_localization_ready:
            localization_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.localization_sub = self.create_subscription(
                String, self._declare_readonly("localization_status_topic", "/pool_cleaning/localization_status"), self._localization_status,
                localization_qos)
        if self.use_map_tf or self.automatic:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        if not self.use_map_tf:
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/pool_cleaning/map_pose", self._pose, qos)
        if self.automatic:
            self.scan_sub = self.create_subscription(
                LaserScan, self._declare_readonly("scan_topic", "/pavo2_scan"), self._navigation_scan, qos)
        self.preview_pub = self.create_publisher(ActuatorCommand, "/pool_cleaning/command_preview", 2)
        self.status_pub = self.create_publisher(String, "/pool_cleaning/status", 5)
        self.lease = Lease(self) if self.live else None
        self.task_services = [
            self.create_service(Trigger, "/pool_cleaning/start", self._start),
            self.create_service(Trigger, "/pool_cleaning/pause", self._pause),
            self.create_service(Trigger, "/pool_cleaning/stop", self._stop_task),
            self.create_service(Trigger, "/pool_cleaning/cancel", self._cancel),
            self.create_service(SetEdge, "/pool_cleaning/set_edge", self._set_edge),
        ]
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02, self._tick, clock=steady)
        self.status_timer = self.create_timer(.1, self._status, clock=steady)

    def _declare_readonly(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        return self.get_parameter(name).value

    def _map_geometry(self):
        """Map-derived parameter defaults, or ``{}`` when no map was given.

        The derivation lives in :mod:`f1_pool_cleaning.map_profile`; this keeps
        a controller run without ``map:=`` explicit about where its numbers
        came from instead of quietly applying another map's geometry.
        """
        if not self.map_path:
            return {}
        from .map_profile import MapProfileError, require
        try:
            return require(self.map_path, self.MAP_PROFILE_KIND).overrides()
        except MapProfileError as exc:
            self.destroy_node()
            raise ValueError(
                f"map {self.map_path} cannot drive the {self.MAP_PROFILE_KIND} task: {exc}") from exc

    def _build_pool(self):
        values = list(self.boundary_xy or [])
        if len(values) > 1:
            if len(values) % 2 or len(values) < 6:
                raise ValueError("boundary_xy must contain paired map x/y values")
            points = list(zip(values[::2], values[1::2]))
            return PoolModel.from_boundary(
                points, boundary_order=self.boundary_order,
                wall_target_m=self.config.wall_target_m,
                corner_exclusion_m=self.config.corner_exclusion_m,
                endpoint_margin_m=self.config.endpoint_margin_m,
                frame=self.config.map_frame)
        # No explicit boundary: fall back to the rectangle implied by the map
        # geometry parameters.  Those parameters are map-derived, so this stays
        # consistent with the selected map rather than a built-in shape.
        return PoolModel.rectangle(
            self.config, corner_exclusion_m=self.config.corner_exclusion_m,
            endpoint_margin_m=self.config.endpoint_margin_m)

    def _validate_parameters(self, parameters):
        if self.live and any(p.name == "use_sim_time" and p.value for p in parameters):
            return SetParametersResult(successful=False, reason="hardware output requires system time")
        return SetParametersResult(successful=True)

    def _localization_status(self, message):
        try:
            status = json.loads(message.data)
            ready = bool(status.get("ready", False)) and status.get("state") == "LOCALIZATION_READY"
            self.localization_snapshot = status
            self.localization_ready = ready
            self.localization_state = str(status.get("state", "UNKNOWN"))
            self.localization_reason = str(status.get("reason", "localization not ready"))
            self.localization_status_received = time.monotonic()
        except (ValueError, TypeError, AttributeError) as exc:
            self.localization_ready = False
            self.localization_state = "LOCALIZATION_CHECK"
            self.localization_reason = f"invalid localization status: {exc}"
            self.localization_status_received = time.monotonic()
        if self.require_localization_ready and self.mission.active and not self.localization_ready:
            self._halt("localization not ready: " + self.localization_reason)

    def _localization_error(self):
        if not self.require_localization_ready:
            return ""
        if not self.localization_status_received:
            return "localization status missing"
        if time.monotonic() - self.localization_status_received >= self.localization_status_max_age_sec:
            return "localization status stream stopped"
        source_stamp = self.localization_snapshot.get("stamp_ns")
        if source_stamp is not None and (type(source_stamp) is not int or not
                0 <= self.get_clock().now().nanoseconds - source_stamp < self.localization_status_max_age_sec * 1e9):
            return "localization status source timestamp stale or invalid"
        if not self.localization_ready:
            return "localization not ready: " + self.localization_reason
        if self.localization_snapshot.get("map_id") and self.map_id != self.localization_snapshot["map_id"]:
            return "localization and cleaning maps do not match"
        return ""

    def _publish_edge_target(self, valid=None, reason=""):
        """Publish the map-derived left-wall context consumed by perception."""
        if not self.edge_task_enabled:
            return
        msg = EdgeTarget()
        msg.header.frame_id = self.config.base_frame
        msg.task_id = self.edge_task_id
        msg.generation = self.edge_generation
        msg.edge_id = self.selected_edge.id if self.selected_edge else ""
        transfer_without_wall = self.mission.phase in (
            "EDGE_STOP", "MOVE_INWARD", "TURN_TO_NEXT_EDGE", "COMPLETE")
        transfer_without_wall |= not getattr(self.mission, "target_wall_enabled", True)
        msg.valid = (bool(self.selected_edge and self.pose and not transfer_without_wall)
                     if valid is None else bool(valid))
        msg.reason = reason or ("waiting for map pose" if not self.pose else "")
        msg.distance_tolerance_m = self.edge_distance_tolerance_m
        msg.tangent_tolerance_rad = self.edge_angle_tolerance_rad
        if self.selected_edge and self.pose:
            projection = self.selected_edge.project(
                self.pose, self.config.brush_x_m, self.config.brush_tip_y_m)
            msg.header.stamp = stamp(self.pose.stamp_ns)
            msg.expected_distance_m = projection.lateral_distance_m
            msg.expected_tangent_rad = projection.heading_error_rad
        else:
            msg.header.stamp = stamp(self.get_clock().now().nanoseconds)
        self.edge_target_pub.publish(msg)

    def _set_selected_edge(self, edge, reason="edge selected"):
        self.selected_edge = edge
        self.edge_generation += 1
        if self.edge_task_enabled:
            self.mission.configure_edge(self.pool, edge, self.edge_task_id)
        self._publish_edge_target(reason=reason)

    def _set_edge(self, request, response):
        if self.mission.active or self.mission.phase == "ACQUIRING":
            response.success = False
            response.message = "cancel before changing the current edge"
            return response
        edge = self.pool.by_id.get(request.edge_id)
        if edge is None:
            response.success = False
            response.message = f"unknown edge id: {request.edge_id}"
            return response
        if self.automatic:
            self.requested_start_edge = edge.id
        self._set_selected_edge(edge)
        response.success, response.message = True, f"selected {edge.id}"
        return response

    def _ensure_edge(self):
        if self.selected_edge is not None:
            if self.edge_task_enabled:
                self.mission.configure_edge(self.pool, self.selected_edge, self.edge_task_id)
            self._publish_edge_target()
            return True, "selected edge ready"
        edge_id = self.default_edge_id or None
        edge, projection, reason = self.pool.select(
            self.pose, edge_id=edge_id,
            max_distance_m=self.config.edge_select_max_distance_m,
            heading_tolerance_rad=self.config.edge_heading_tolerance_rad,
            brush_x_m=self.config.brush_x_m, brush_tip_y_m=self.config.brush_tip_y_m)
        if edge is None:
            return False, reason
        self._set_selected_edge(edge, reason)
        return True, reason

    def _sync_mission_edge(self):
        """Publish a new wall target after the mission finishes a right turn."""
        if not self.edge_task_enabled or self.mission.edge is None:
            return
        generation = getattr(self.mission, "target_generation", 0)
        if self.selected_edge is self.mission.edge and (not self.automatic or generation == self.synced_target_generation):
            return
        self.selected_edge = self.mission.edge
        self.synced_target_generation = generation
        self.edge_generation += 1
        if self.automatic:
            self.wall_after_ns = self.get_clock().now().nanoseconds
            if self.wall is not None:
                self.wall = replace(self.wall, valid=False, reason="waiting for new target-wall evidence")
        self._publish_edge_target(valid=False, reason="edge transfer completed; resetting wall association")

    def _halt(self, reason, phase="PAUSED"):
        self.mission.halt(reason, phase)
        if self.edge_task_enabled and hasattr(self, "edge_target_pub") and self.selected_edge is not None:
            self.edge_generation += 1
            self._publish_edge_target(valid=False, reason=reason)
        self._stop_outputs()

    def _invalidate(self, name, reason):
        setattr(self, name, None)
        if name == "wall" and not self.mission.wall_required:
            return
        if self.mission.active or self.mission.phase == "ACQUIRING":
            self._halt(reason)

    def _wall(self, msg):
        if self.automatic and stamp_ns(msg.header.stamp) <= self.wall_after_ns:
            return
        if msg.header.frame_id != self.config.base_frame or not msg.session:
            self._invalidate("wall", "invalid wall frame/session")
            return
        if self.session and msg.session != self.session:
            self._invalidate("wall", "perception restarted; cancel/restart required")
            self.sequence = -1
        if msg.sequence <= self.sequence:
            return
        observation = from_observation_message(msg)
        if self.wall and observation.stamp_ns <= self.wall.stamp_ns:
            return
        self.session, self.sequence = msg.session, msg.sequence
        self.wall = observation
        self.received["wall"] = time.monotonic()
        if self.mission.active:
            self._tick()

    def _pose(self, msg):
        c, p, cov = self.config, msg.pose.pose, msg.pose.covariance
        q = p.orientation
        values = (p.position.x, p.position.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (msg.header.frame_id != c.map_frame or not all(math.isfinite(v) for v in values)
                or abs(norm - 1.) > .01):
            self._invalidate("pose", "invalid map pose/frame/quaternion")
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        pose = Pose(stamp_ns(msg.header.stamp), p.position.x, p.position.y, yaw)
        covariance_available = all(math.isfinite(v) for v in cov) and all(cov[i] > 0 for i in (0, 7, 35))
        self._accept_pose(pose, covariance_available=covariance_available)

    def _accept_pose(self, pose, covariance_available=False):
        if self.pose and pose.stamp_ns <= self.pose.stamp_ns:
            return
        self.pose = pose
        self.pose_covariance_available = covariance_available
        self.received["pose"] = time.monotonic()
        if self.edge_task_enabled and hasattr(self, "edge_target_pub") and self.selected_edge is not None:
            self._publish_edge_target()

    def _read_tf_pose(self):
        from tf2_ros import TransformException
        try:
            tf = self.tf_buffer.lookup_transform(self.config.map_frame, self.config.base_frame, Time())
        except TransformException:
            return  # Original stamp and receive watchdog expire; never restamp old TF.
        t, q = tf.transform.translation, tf.transform.rotation
        if (not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01):
            self._invalidate("pose", "invalid map TF pose")
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self._accept_pose(Pose(stamp_ns(tf.header.stamp), t.x, t.y, yaw))

    def _navigation_scan(self, msg):
        """Raw scan freshness/obstacles are independent of successful wall fitting."""
        from tf2_ros import TransformException
        source_stamp = stamp_ns(msg.header.stamp)
        if self.navigation_scan and source_stamp <= self.navigation_scan.stamp_ns:
            return
        if (source_stamp <= 0 or not msg.header.frame_id or len(msg.ranges) < 12
                or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
                or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max):
            self.navigation_scan = None
            return
        x = y = yaw = 0.
        if msg.header.frame_id != self.config.base_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.config.base_frame, msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp))
            except TransformException:
                self.navigation_scan = None
                return
            t, q = tf.transform.translation, tf.transform.rotation
            if (not all(math.isfinite(v) for v in (t.x,t.y,q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                self.navigation_scan = None
                return
            x,y = t.x,t.y
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        points = tuple((x+r*math.cos(msg.angle_min+i*msg.angle_increment+yaw),
                        y+r*math.sin(msg.angle_min+i*msg.angle_increment+yaw))
                       for i,r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min <= r <= msg.range_max)
        if len(points) < max(3, .05*len(msg.ranges)):
            self.navigation_scan = None
            return
        self.navigation_scan = NavigationScan(source_stamp, points)
        self.received["scan"] = time.monotonic()

    def _feedback(self):
        return Feedback(self.wall, self.pose, self.navigation_scan) if self.pose is not None else None

    def _stream_error(self, require_wall=None):
        if require_wall is None:
            require_wall = self.mission.wall_required
        now = time.monotonic()
        streams = [("pose", self.config.pose_max_age_sec)]
        if self.automatic:
            streams.append(("scan", self.mission.n.scan_max_age_sec))
        if require_wall:
            streams.insert(0, ("wall", self.config.wall_max_age_sec))
        for name, max_age in streams:
            if name not in self.received or now - self.received[name] >= max_age:
                return name + " receive stream stopped"
        return ""

    def _start_blockers(self):
        """Side-effect-free preflight, shared by status and start (no planning)."""
        blockers = []
        def add(code, message):
            if message:
                blockers.append(dict(code=code, message=message))
        allowed = ("IDLE", "CANCELLED", "STOPPED", "COMPLETE") if self.automatic else ("IDLE", "CANCELLED", "STOPPED")
        if self.mission.phase not in allowed:
            add("phase", "cancel before restarting the task")
        add("clock", "clock fault latched; restart controller" if self.clock_fault else "")
        add("localization", self._localization_error())
        add("feedback", self._stream_error(require_wall=False if self.automatic else None))
        if self.automatic:
            now = self.get_clock().now().nanoseconds
            feedback = self._feedback()
            add("pose", self.mission._pose_error(feedback, now))
            add("scan", self.mission.scan_error(feedback, now))
        if self.lease:
            add("bridge", self.lease.error(acquiring=True))
            if not self.lease.acquire.service_is_ready():
                add("bridge_service", "bridge acquire service unavailable")
        return blockers

    def _start(self, request, response):
        if self.use_map_tf:
            self._read_tf_pose()
        if self.automatic:
            blockers = self._start_blockers()
            if blockers:
                response.success, response.message = False, blockers[0]["message"]
                return response
            self.mission.requested_edge_id = (self.requested_start_edge or
                (self.default_edge_id if self.start_policy == "specified" else None))
            if self.mission.requested_edge_id and self.mission.requested_edge_id not in self.pool.by_id:
                response.success, response.message = False, "unknown configured start edge"
                return response
            response.success, response.message = self.mission.start(self._feedback(),self.get_clock().now().nanoseconds)
            if response.success:
                self.run_id = uuid.uuid4().hex
                self.completed_at_ns = None
                self.requested_start_edge = None
                self.selected_edge = None
                self.edge_generation += 1
                self._publish_edge_target(valid=False,reason="automatic start: selecting a task wall")
                if self.lease:
                    response.success, response.message = self.lease.begin()
                    if not response.success:
                        self._halt(response.message)
                else:
                    response.message += "; preview only"
                    self._tick()
            return response
        if self.mission.phase in ("CANCELLED", "COMPLETE"):
            self.selected_edge = None
        error = ("clock fault latched; restart controller" if self.clock_fault else
                 self._localization_error() or self._stream_error())
        if error:
            response.success, response.message = False, error
            return response
        if self.edge_task_enabled and self.selected_edge is None:
            ok, reason = self._ensure_edge()
            if not ok:
                response.success, response.message = False, reason
                return response
        elif self.edge_task_enabled:
            self.mission.configure_edge(self.pool, self.selected_edge, self.edge_task_id)
            self._publish_edge_target()
        response.success, response.message = self.mission.start(self._feedback(), self.get_clock().now().nanoseconds)
        if response.success:
            if self.lease:
                response.success, response.message = self.lease.begin()
                if not response.success:
                    self._halt(response.message)
            else:
                response.message += "; preview only"
                self._tick()
        return response

    def _pause(self, request, response):
        self._halt("user pause; cancel before restarting")
        response.success, response.message = True, self.mission.reason
        return response

    def _stop_task(self, request, response):
        self._halt("user stopped task; start creates a new run", "STOPPED")
        response.success, response.message = True, self.mission.reason
        return response

    def _cancel(self, request, response):
        self._halt("user cancelled", "CANCELLED")
        if self.edge_task_enabled:
            self.edge_generation += 1
            self._publish_edge_target(valid=False, reason="task cancelled; edge association reset")
            self.selected_edge = None
            self.requested_start_edge = None
            if not self.automatic:
                self.mission.pool = None
            self.mission.edge = None
            self.mission.edge_projection = None
        response.success, response.message = True, "cancelled"
        return response

    def _tick(self):
        now, mono = self.get_clock().now().nanoseconds, time.monotonic()
        if self.clock_pair:
            old_ros, old_mono = self.clock_pair
            if now < old_ros or (self.live and abs((now - old_ros) / 1e9 - (mono - old_mono)) > .05):
                self.wall = self.pose = None
                self.received.clear()
                self.sequence, self.session = -1, ""
                self.clock_fault = self.live
                self._halt("clock discontinuity; reset task")
        self.clock_pair = now, mono
        if self.use_map_tf:
            self._read_tf_pose()
        if self.automatic and self.selected_edge is not None:
            if self.mission.target_wall_enabled:
                self._publish_edge_target()
        if (self.edge_task_enabled and self.selected_edge is not None
                and self.mission.phase in ("EDGE_STOP", "MOVE_INWARD", "TURN_TO_NEXT_EDGE")):
            # Break the old wall association as soon as a transfer begins;
            # transfer motion is driven by map pose, not a remembered wall fit.
            self._publish_edge_target(valid=False, reason=self.mission.reason)
        if self.edge_task_enabled and self.selected_edge is not None and not self.mission.active:
            # Keep the selected map edge available while idle/paused so the
            # perception node can refresh a valid wall association before a
            # start retry.  _halt() already publishes one invalid generation
            # to break the old association; repeatedly publishing invalid
            # targets here makes a later start impossible after a lease error.
            self._publish_edge_target(reason=self.mission.reason)
        if self.lease:
            error = self.lease.tick_error()
            if error:
                self._halt(error)
                return
        if not self.mission.active and self.mission.phase != "ACQUIRING":
            return
        error = self._stream_error(require_wall=self.mission.wall_required)
        if error:
            self._halt(error)
            return
        error = self._localization_error()
        if error:
            self._halt(error)
            return
        if self.mission.phase == "ACQUIRING":
            return
        effort = self.mission.update(self._feedback(), now)
        self._sync_mission_edge()
        if effort:
            self._publish(effort)
        if not self.mission.active:
            self._stop_outputs()

    def _message(self, effort):
        msg = ActuatorCommand()
        msg.computed_at, msg.valid_until = stamp(effort.computed_ns), stamp(effort.valid_until_ns)
        msg.direction_until = stamp(effort.direction_until_ns)
        for key, value in command_fields(effort, self.config).items():
            setattr(msg, key, value)
        return msg

    def _publish(self, effort):
        msg = self._message(effort)
        msg.token, msg.sequence = "", self.preview_sequence
        self.preview_sequence += 1
        self.preview_pub.publish(msg)
        if self.lease:
            self.lease.publish(self._message(effort))

    def _stop_outputs(self):
        now = self.get_clock().now().nanoseconds
        self._publish(Effort(now, now + round(self.config.command_ttl_sec * 1e9)))
        if self.lease:
            self.lease.stop()

    def _expected_track(self):
        """Independent boat-centre segments; never invent corner/transit joins."""
        segments = []
        if self.edge_task_enabled:
            required = self.mission.required if self.automatic else {}
            for edge in self.pool.edges:
                low, high = required.get(edge.id, (edge.clean_start_s_m, edge.clean_end_s_m))
                points = [[edge.start[i] + edge.tangent[i]*(contact-self.config.brush_x_m)
                           - edge.left_normal[i]*self.config.wall_target_m for i in (0, 1)]
                          for contact in (low, high)]
                segments.append(dict(edge_id=edge.id, points=points))
        navigation = []
        if self.automatic and self.mission.phase in ("NAVIGATING", "SETTLING"):
            navigation = [[point.x, point.y] for point in self.mission.path]
        return dict(frame_id=self.config.map_frame, map_id=self.map_id,
                    cleaning_segments=segments, navigation_path=navigation)

    def _status(self):
        f = self._feedback()
        now = self.get_clock().now().nanoseconds
        effort = self.mission.last_effort
        valid = effort is not None and now < effort.valid_until_ns and self.mission.active
        edge = ((self.mission.edge if self.mission.edge_active else self.selected_edge)
                if self.edge_task_enabled else None)
        projection = self.mission.edge_projection
        cleaned = ([list(interval) for interval in self.mission.cleaned_intervals]
                   if self.mission.edge_active else [])
        cleaned_by_edge = {
            edge_id: [list(interval) for interval in intervals]
            for edge_id, intervals in self.mission.cleaned_by_edge.items()
        }
        uncovered = []
        if edge and cleaned:
            cursor = edge.clean_start_s_m
            for low, high in self.mission.cleaned_intervals:
                if low > cursor:
                    uncovered.append([cursor, low])
                cursor = max(cursor, high)
            if cursor < edge.clean_end_s_m:
                uncovered.append([cursor, edge.clean_end_s_m])
        elif edge:
            uncovered = [[edge.clean_start_s_m, edge.clean_end_s_m]]
        wall_age = None if self.wall is None else (now-self.wall.stamp_ns)/1e9
        wall_fresh = (wall_age is not None and 0 <= wall_age < self.config.wall_max_age_sec
                      and time.monotonic()-self.received.get("wall", 0) < self.config.wall_max_age_sec)
        left_distance = (self.wall.distance if wall_fresh and self.wall.valid
                         and math.isfinite(self.wall.distance) and self.wall.distance > 0 else None)
        if self.mission.phase == "COMPLETE" and self.completed_at_ns is None:
            self.completed_at_ns = now
        blockers = self._start_blockers()
        status = dict(phase=self.mission.phase,
                      task_type="l" if self.MAP_PROFILE_KIND == "route" else "rectangle",
                      run_id=self.run_id, completed_at_ns=self.completed_at_ns,
                      map_id=self.map_id, stamp_ns=now,
                      can_start=not blockers, start_blockers=blockers,
                      expected_track=self._expected_track(),
                      wall_fresh=wall_fresh, wall_age_sec=wall_age,
                      center_wall_distance_m=left_distance,
                      edge_task_enabled=self.edge_task_enabled,
                      left_wall_distance_cm=round(left_distance * 100, 1) if left_distance is not None else None,
                      wall_angle_deg=round(math.degrees(self.wall.angle), 2) if self.wall and self.wall.valid
                          and math.isfinite(self.wall.angle) else None,
                      direction=effort.direction if valid and (not effort.direction_until_ns or now < effort.direction_until_ns) else 0,
                      reason=self.mission.reason,
                      feedback_error=self._stream_error() or self.mission.error(f, now),
                      output_enabled=self.live, lease_held=bool(self.lease and self.lease.token),
                      brush_enabled=bool(valid and effort.brush_percent),
                      logical_side_left_percent=effort.side_left_percent if valid else 0.,
                      wall_distance_m=left_distance,
                      wall_valid=bool(wall_fresh and self.wall.valid),
                      wall_reason=self.wall.reason if self.wall else "wall observation missing",
                      wall_recovery_sec=self.config.wall_recovery_sec,
                      heading_band_deg=math.degrees(self.config.heading_deadband_rad),
                      heading_correction_deadband_deg=math.degrees(self.config.heading_correction_deadband_rad),
                      wall_target_m=self.config.wall_target_m,
                      wall_band_min_m=self.config.side_off_distance_m,
                      wall_band_max_m=self.config.side_on_distance_m,
                      hull_length_m=2 * self.config.hull_half_length_m,
                      hull_width_m=2 * self.config.hull_half_width_m,
                      front_stop_m=self.config.front_stop_m,
                      front_distance_source=("edge_endpoint_center_distance" if edge else
                                             "map_short_wall_center_distance"),
                      map_front_clearance_m=(
                          None if not f else
                          (edge.length_m - projection.s_m if edge and projection else
                           self.mission.front_remaining(f.pose))),
                      current_edge_id=edge.id if edge else "",
                      edge_count=len(self.pool.edges),
                      edge_progress_m=None if projection is None else projection.s_m,
                      edge_contact_progress_m=None if projection is None else projection.contact_s_m,
                      edge_entry_m=None if edge is None else edge.entry_s_m,
                      edge_stop_m=None if edge is None else edge.stop_s_m,
                      edge_clean_start_m=None if edge is None else edge.clean_start_s_m,
                      edge_clean_end_m=None if edge is None else edge.clean_end_s_m,
                      cleaned_intervals=cleaned,
                      uncovered_intervals=uncovered,
                      coverage_basis=("left_brush_contact_centerline" if edge else None),
                      front_observed=bool(self.wall and self.wall.front_observed),
                      blind_front_obstacle_detection_available=False,
                      pose_source="tf" if self.use_map_tf else "map_pose",
                      pose_covariance_available=self.pose_covariance_available,
                      localization_ready=self.localization_ready,
                      localization_state=self.localization_state,
                      localization_reason=self.localization_reason,
                      localization_status_age_sec=(None if not self.localization_status_received else
                                                    time.monotonic() - self.localization_status_received),
                      localization_snapshot=self.localization_snapshot,
                      speed_feedback_available=False,
                      # Kept for status-schema compatibility; simplified
                      # transfer no longer performs physical stop verification.
                      stopped_verified=False,
                      stop_confirmation_available=False,
                      automatic_transfer_available=True,
                      transfer_next_edge=(None if self.mission.transfer_next_edge is None
                                          else self.mission.transfer_next_edge.id),
                      completed_edges=list(self.mission.completed_edges),
                      cleaned_intervals_by_edge=cleaned_by_edge,
                      coverage_available=bool(edge))
        status["start_policy"] = self.start_policy
        if self.automatic:
            status.update(self.mission.status())
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))

    def destroy_node(self):
        if hasattr(self, "lease") and self.context.ok():
            self._halt("controller shutdown", "CANCELLED")
        if hasattr(self, "mission") and isinstance(self.mission, AutonomousMission):
            self.mission.close()
        return super().destroy_node()


def main(args=None):
    from f1_pool_cleaning.runner import run
    run(PoolController, args)
