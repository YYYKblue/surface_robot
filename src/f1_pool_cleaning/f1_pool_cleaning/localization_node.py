"""ROS adapter for operator-initialized Cartographer localization."""

import math
import time

from cartographer_ros_msgs.msg import StatusCode
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from f1_interfaces.srv import SetInitialPose

from .localization import (LocalizationConfig, LocalizationGate, PoseSample,
                           ScanQuality, json_status, validate_initial_pose,
                           yaw_from_quaternion)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def planar_yaw(q):
    return yaw_from_quaternion(q)


class LocalizationManager(Node):
    """Manage exactly one pure-localization trajectory.

    The manager never publishes an actuator command.  Its readiness topic is
    transient-local so a controller started after localization still receives
    the current gate result.
    """

    def __init__(self, **kwargs):
        super().__init__("pool_localization_manager", **kwargs)
        self._declare_parameters()
        self.config = self._read_config()
        self.configuration_directory = self.get_parameter("configuration_directory").value
        self.configuration_basename = self.get_parameter("configuration_basename").value
        self.reference_trajectory_id = self.get_parameter("reference_trajectory_id").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.trajectory_query_period_sec = self.get_parameter("trajectory_query_period_sec").value
        self.gate = LocalizationGate(self.config)
        self.trajectory_id = None
        self._known_active_ids = set()
        self._pending_pose = None
        self._pending_source = ""
        self._pending_since = None
        self._finish_future = None
        self._start_future = None
        self._states_future = None
        self._states_checked = False
        self._last_state_query = 0.
        self._last_trajectory_error = ""
        self._last_scan = None
        self._last_scan_received = 0.
        self._pose = None
        self._pose_received = 0.
        self._scan_map_overlap = None
        self._scan_overlap_reason = "waiting for scan/map transform"
        self._last_status = {}

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status_pub = self.create_publisher(String, "/pool_cleaning/localization_status", status_qos)
        self.ready_pub = self.create_publisher(Bool, "/pool_cleaning/localization_ready", status_qos)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initial_pose, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan, sensor_qos)
        self.set_pose_srv = self.create_service(SetInitialPose,
                                                "/pool_cleaning/set_initial_pose",
                                                self._set_initial_pose)
        self.relocalize_srv = self.create_service(Trigger,
                                                  "/pool_cleaning/relocalize",
                                                  self._relocalize)
        self.start_client = self.create_client(StartTrajectory, "/start_trajectory")
        self.finish_client = self.create_client(FinishTrajectory, "/finish_trajectory")
        self.states_client = self.create_client(GetTrajectoryStates, "/get_trajectory_states")
        from tf2_ros import Buffer, TransformException, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._transform_exception = TransformException
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self._tick, clock=steady)

    def _declare_parameters(self):
        defaults = LocalizationConfig()
        for name in ("map_frame", "base_frame", "pose_max_age_sec", "scan_max_age_sec",
                     "stable_sec", "jump_distance_m", "jump_angle_rad", "boundary_margin_m",
                     "scan_overlap_min", "scan_valid_fraction_min", "initial_pose_timeout_sec",
                     "status_max_age_sec", "origin_x_m", "origin_y_m", "axis_yaw_rad",
                     "length_m", "width_m"):
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("configuration_directory", "")
        self.declare_parameter("configuration_basename", "pool_localization.lua")
        self.declare_parameter("reference_trajectory_id", 0)
        self.declare_parameter("scan_topic", "/pavo2_scan")
        self.declare_parameter("trajectory_query_period_sec", .50)

    def _read_config(self):
        values = {name: self.get_parameter(name).value for name in (
            "map_frame", "base_frame", "pose_max_age_sec", "scan_max_age_sec",
            "stable_sec", "jump_distance_m", "jump_angle_rad", "boundary_margin_m",
            "scan_overlap_min", "scan_valid_fraction_min", "initial_pose_timeout_sec",
            "status_max_age_sec", "origin_x_m", "origin_y_m", "axis_yaw_rad",
            "length_m", "width_m")}
        return LocalizationConfig(**values).validate()

    @staticmethod
    def _service_ok(response):
        status = getattr(response, "status", None)
        return status is not None and status.code == StatusCode.OK

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _set_gate_reason(self, reason):
        self.gate.ready = False
        self.gate.state = "LOCALIZATION_CHECK"
        self.gate.reason = reason
        self.gate.stable_since_ns = None

    def _reject_initial_pose(self, reason):
        # Once a valid pose has been accepted, a malformed replacement is a
        # safety event.  Keep it latched until the operator explicitly
        # relocalizes instead of silently returning to the old ready pose.
        if self.gate.initial_pose_received:
            self.gate.mark_paused(reason)
        else:
            self.gate.reset(reason)
        self._publish_status(self._now_ns())

    def _initial_pose(self, message):
        self._accept_initial_pose(message, "rviz")

    def _set_initial_pose(self, request, response):
        ok, reason, _ = validate_initial_pose(request.pose, self.config.map_frame)
        if not ok:
            response.success, response.message = False, reason
            self._reject_initial_pose(reason)
            return response
        accepted = self._accept_initial_pose(request.pose, "service")
        response.success = accepted
        response.message = "initial pose accepted; starting localization trajectory" if accepted else self.gate.reason
        return response

    def _accept_initial_pose(self, message, source):
        ok, reason, _ = validate_initial_pose(message, self.config.map_frame)
        if not ok:
            self._reject_initial_pose(reason)
            self.get_logger().warning(reason)
            return False
        now = self._now_ns()
        source_stamp = stamp_ns(message.header.stamp) or max(now, 1)
        try:
            self.gate.accept_initial_pose(source_stamp, source)
        except ValueError as exc:
            self._set_gate_reason(str(exc))
            return False
        # Close the controller gate before any asynchronous finish/start call.
        self._publish_status(now)
        # A new initial pose invalidates the previous trajectory and all of its
        # stability evidence.  The task controller sees not-ready immediately.
        self._pending_pose = message.pose.pose
        self._pending_source = source
        self._pending_since = time.monotonic()
        self._last_trajectory_error = ""
        self._start_future = None
        if self.trajectory_id is None:
            candidates = sorted(self._known_active_ids - {int(self.reference_trajectory_id)})
            if candidates:
                self.trajectory_id = candidates[-1]
        if self._finish_future is None and self.trajectory_id is None:
            self._request_start()
        elif self._finish_future is None:
            self._request_finish_current()
        return True

    def _relocalize(self, request, response):
        self._pending_pose = None
        self._pending_source = ""
        self._pending_since = None
        self._start_future = None
        self.gate.reset("relocalization requested; waiting for a new RViz 2D Pose Estimate")
        self._publish_status(self._now_ns())
        if self.trajectory_id is None:
            candidates = sorted(self._known_active_ids - {int(self.reference_trajectory_id)})
            if candidates:
                self.trajectory_id = candidates[-1]
        if self.trajectory_id is not None and self._finish_future is None:
            self._request_finish_current()
        response.success, response.message = True, "relocalization requested; submit a new initial pose"
        return response

    def _request_finish_current(self):
        if self.trajectory_id is None:
            return False
        if not self.finish_client.service_is_ready():
            self._last_trajectory_error = "Cartographer finish_trajectory service unavailable"
            return False
        request = FinishTrajectory.Request(trajectory_id=int(self.trajectory_id))
        self._finish_future = self.finish_client.call_async(request)
        self._finish_future.add_done_callback(self._finish_done)
        self._set_gate_reason("finishing previous localization trajectory")
        return True

    def _finish_done(self, future):
        self._finish_future = None
        try:
            response = future.result()
            if not self._service_ok(response):
                self._last_trajectory_error = getattr(response.status, "message", "finish_trajectory failed")
                self._set_gate_reason(self._last_trajectory_error)
                return
        except Exception as exc:  # service futures surface DDS errors here
            self._last_trajectory_error = f"finish_trajectory failed: {exc}"
            self._set_gate_reason(self._last_trajectory_error)
            return
        self.trajectory_id = None
        if self._pending_pose is not None:
            self._request_start()

    def _request_start(self):
        if self._pending_pose is None or self._start_future is not None:
            return False
        # The first state query adopts a trajectory left by a restarted
        # manager.  Never create a second active trajectory before that check.
        if not self._states_checked:
            self._set_gate_reason("waiting for Cartographer trajectory state")
            return False
        if not self.start_client.service_is_ready():
            self._set_gate_reason("Cartographer start_trajectory service unavailable")
            return False
        request = StartTrajectory.Request(
            configuration_directory=str(self.configuration_directory),
            configuration_basename=str(self.configuration_basename),
            use_initial_pose=True,
            initial_pose=self._pending_pose,
            relative_to_trajectory_id=int(self.reference_trajectory_id))
        self._start_future = self.start_client.call_async(request)
        self._start_future.add_done_callback(self._start_done)
        self._set_gate_reason("initial pose submitted; waiting for Cartographer trajectory")
        return True

    def _start_done(self, future):
        self._start_future = None
        try:
            response = future.result()
            if not self._service_ok(response):
                self._last_trajectory_error = getattr(response.status, "message", "start_trajectory failed")
                self.trajectory_id = None
                self._set_gate_reason(self._last_trajectory_error)
                return
            self.trajectory_id = int(response.trajectory_id)
            self._pending_pose = None
            self._pending_since = None
            self.gate.last_pose = None
            self.gate.stable_since_ns = None
            self.gate.reason = "trajectory active; checking pose, scan and map overlap"
        except Exception as exc:
            self.trajectory_id = None
            self._last_trajectory_error = f"start_trajectory failed: {exc}"
            self._set_gate_reason(self._last_trajectory_error)

    def _states_done(self, future):
        self._states_future = None
        try:
            response = future.result()
            if not self._service_ok(response):
                self._last_trajectory_error = getattr(response.status, "message", "get_trajectory_states failed")
                return
            ids = response.trajectory_states.trajectory_id
            states = response.trajectory_states.trajectory_state
            active = {int(i) for i, state in zip(ids, states) if state == 0}
            self._known_active_ids = active
            self._states_checked = True
            if self.trajectory_id is None and active:
                candidates = sorted(active - {int(self.reference_trajectory_id)})
                if candidates:
                    self.trajectory_id = candidates[-1]
        except Exception as exc:
            self._last_trajectory_error = f"get_trajectory_states failed: {exc}"

    def _query_states(self):
        now = time.monotonic()
        if self._states_future is not None or now - self._last_state_query < self.trajectory_query_period_sec:
            return
        self._last_state_query = now
        if self.states_client.service_is_ready():
            self._states_future = self.states_client.call_async(GetTrajectoryStates.Request())
            self._states_future.add_done_callback(self._states_done)

    def _scan(self, message):
        stamp = stamp_ns(message.header.stamp)
        if stamp <= 0 or not message.header.frame_id:
            self._last_scan = None
            self._scan_map_overlap = None
            self._scan_overlap_reason = "scan has no timestamp or frame"
            return
        values = list(message.ranges)
        valid = [value for value in values if math.isfinite(value)
                 and message.range_min <= value <= message.range_max]
        fraction = len(valid) / len(values) if values else 0.
        overlap = self._scan_overlap(message, valid)
        self._last_scan = ScanQuality(stamp, fraction, overlap, message.header.frame_id)
        self._last_scan_received = time.monotonic()

    def _scan_overlap(self, message, valid_values):
        if not valid_values:
            self._scan_overlap_reason = "scan has no valid range returns"
            return 0.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.config.map_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp))
        except self._transform_exception:
            self._scan_overlap_reason = "scan/map transform unavailable"
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        try:
            yaw = planar_yaw(q)
        except ValueError:
            self._scan_overlap_reason = "scan/map transform has invalid quaternion"
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        # Sample a bounded number of rays.  This is a consistency check, not a
        # replacement for Cartographer's scan matcher.
        ranges = list(message.ranges)
        valid_indices = [i for i, value in enumerate(ranges)
                         if math.isfinite(value) and message.range_min <= value <= message.range_max]
        step = max(1, len(valid_indices) // 120)
        inside = 0
        margin = self.config.boundary_margin_m
        for index in valid_indices[::step]:
            distance = ranges[index]
            angle = message.angle_min + index * message.angle_increment
            sx, sy = distance * math.cos(angle), distance * math.sin(angle)
            x = t.x + c * sx - s * sy
            y = t.y + s * sx + c * sy
            px, py = self.config.pool_xy(x, y)
            if -margin <= px <= self.config.length_m + margin and -margin <= py <= self.config.width_m + margin:
                inside += 1
        fraction = inside / max(1, len(valid_indices[::step]))
        self._scan_overlap_reason = "scan endpoints overlap configured map boundary"
        return fraction

    def _read_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.config.map_frame, self.config.base_frame, Time())
        except self._transform_exception:
            self._pose = None
            self._pose_received = 0.
            return
        t, q = tf.transform.translation, tf.transform.rotation
        values = (t.x, t.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            self._pose = None
            self._pose_received = 0.
            return
        try:
            yaw = planar_yaw(q)
        except ValueError:
            self._pose = None
            self._pose_received = 0.
            return
        stamp = stamp_ns(tf.header.stamp)
        self._pose = PoseSample(stamp, t.x, t.y, yaw)
        self._pose_received = time.monotonic()

    def _trajectory_active(self):
        return self.trajectory_id is not None and int(self.trajectory_id) in self._known_active_ids

    def _tick(self):
        now_ns = self._now_ns()
        self._query_states()
        self._read_pose()
        if self._pending_pose is not None:
            if self._pending_since is not None and time.monotonic() - self._pending_since > self.config.initial_pose_timeout_sec:
                self._set_gate_reason("initial pose submission timed out")
            elif self._finish_future is None and self._start_future is None:
                if self.trajectory_id is not None:
                    self._request_finish_current()
                else:
                    self._request_start()
        active = self._trajectory_active()
        if self.trajectory_id is not None and self._known_active_ids and not active:
            # Cartographer finished or deleted the current trajectory.  Keep
            # the task gate closed until the operator explicitly relocalizes.
            self.gate.mark_paused("localization trajectory is no longer active")
        self.gate.update(now_ns, active, self._pose, self._last_scan)
        self._publish_status(now_ns)

    def _publish_status(self, now_ns):
        status = self.gate.as_dict(now_ns)
        status.update(dict(trajectory_id=-1 if self.trajectory_id is None else int(self.trajectory_id),
                           reference_trajectory_id=int(self.reference_trajectory_id),
                           scan_topic=self.scan_topic,
                           scan_overlap_reason=self._scan_overlap_reason,
                           last_trajectory_error=self._last_trajectory_error,
                           stamp_ns=now_ns))
        self._last_status = status
        self.status_pub.publish(String(data=json_status(status)))
        ready = Bool(data=bool(self.gate.ready))
        self.ready_pub.publish(ready)

    def destroy_node(self):
        self.gate.mark_paused("localization manager shutdown")
        return super().destroy_node()


def main(args=None):
    from .runner import run
    run(LocalizationManager, args)
