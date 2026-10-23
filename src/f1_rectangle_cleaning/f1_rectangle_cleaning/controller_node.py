"""Single-side ROS adapter with isolated preview and optional bridge lease."""
from dataclasses import fields
import json
import math
import time

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_interfaces.msg import ActuatorCommand, WallObservation
from f1_wall_cleaning.ros_support import stamp, stamp_ns, from_observation_message

from .config import Config
from .mission import Mission, Feedback, Pose, Effort, command_fields
from .lease import Lease


class RectangleController(Node):
    def __init__(self, **kwargs):
        super().__init__("rectangle_controller", **kwargs)
        for name, default in (("output_enabled", False), ("use_map_tf", False)):
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.live = self.get_parameter("output_enabled").value
        self.use_map_tf = self.get_parameter("use_map_tf").value
        values = {}
        for field in fields(Config()):
            self.declare_parameter(field.name, getattr(Config(), field.name), ParameterDescriptor(read_only=True))
            values[field.name] = self.get_parameter(field.name).value
        self.config = Config(**values).validate()
        if self.live and (self.get_parameter("use_sim_time").value or not self.config.fifth_left_sign):
            self.destroy_node()
            raise ValueError("hardware output requires system time and known fifth_left_sign")
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.mission = Mission(self.config)
        self.wall = self.pose = None
        self.session, self.sequence = "", -1
        self.received = {}
        self.clock_pair = None
        self.clock_fault = False
        self.pose_covariance_available = False
        self.preview_sequence = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wall_sub = self.create_subscription(WallObservation, "/rectangle_cleaning/wall_observation", self._wall, qos)
        if self.use_map_tf:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/rectangle_cleaning/map_pose", self._pose, qos)
        self.preview_pub = self.create_publisher(ActuatorCommand, "/rectangle_cleaning/command_preview", 2)
        self.status_pub = self.create_publisher(String, "/rectangle_cleaning/status", 5)
        self.lease = Lease(self) if self.live else None
        self.task_services = [
            self.create_service(Trigger, "/rectangle_cleaning/start", self._start),
            self.create_service(Trigger, "/rectangle_cleaning/pause", self._pause),
            self.create_service(Trigger, "/rectangle_cleaning/cancel", self._cancel),
        ]
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02, self._tick, clock=steady)
        self.status_timer = self.create_timer(.1, self._status, clock=steady)

    def _validate_parameters(self, parameters):
        if self.live and any(p.name == "use_sim_time" and p.value for p in parameters):
            return SetParametersResult(successful=False, reason="hardware output requires system time")
        return SetParametersResult(successful=True)

    def _halt(self, reason, phase="PAUSED"):
        self.mission.halt(reason, phase)
        self._stop_outputs()

    def _invalidate(self, name, reason):
        setattr(self, name, None)
        if self.mission.active or self.mission.phase == "ACQUIRING":
            self._halt(reason)

    def _wall(self, msg):
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

    def _feedback(self):
        return Feedback(self.wall, self.pose) if self.wall is not None and self.pose is not None else None

    def _stream_error(self):
        now = time.monotonic()
        for name, max_age in (("wall", self.config.wall_max_age_sec), ("pose", self.config.pose_max_age_sec)):
            if name not in self.received or now - self.received[name] >= max_age:
                return name + " receive stream stopped"
        return ""

    def _start(self, request, response):
        if self.use_map_tf:
            self._read_tf_pose()
        error = "clock fault latched; restart controller" if self.clock_fault else self._stream_error()
        if error:
            response.success, response.message = False, error
            return response
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

    def _cancel(self, request, response):
        self._halt("user cancelled", "CANCELLED")
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
        if self.lease:
            error = self.lease.tick_error()
            if error:
                self._halt(error)
                return
        if not self.mission.active and self.mission.phase != "ACQUIRING":
            return
        error = self._stream_error()
        if error:
            self._halt(error)
            return
        if self.mission.phase == "ACQUIRING":
            return
        effort = self.mission.update(self._feedback(), now)
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

    def _status(self):
        f = self._feedback()
        now = self.get_clock().now().nanoseconds
        effort = self.mission.last_effort
        valid = effort is not None and now < effort.valid_until_ns and self.mission.active
        status = dict(phase=self.mission.phase, reason=self.mission.reason,
                      feedback_error=self._stream_error() or self.mission.error(f, now),
                      output_enabled=self.live, lease_held=bool(self.lease and self.lease.token),
                      brush_enabled=bool(valid and effort.brush_percent),
                      logical_side_left_percent=effort.side_left_percent if valid else 0.,
                      direction=effort.direction if valid and (not effort.direction_until_ns or now < effort.direction_until_ns) else 0,
                      wall_distance_m=self.wall.distance if self.wall and math.isfinite(self.wall.distance) else None,
                      wall_angle_deg=math.degrees(self.wall.angle) if self.wall and math.isfinite(self.wall.angle) else None,
                      heading_band_deg=math.degrees(self.config.heading_deadband_rad),
                      wall_target_m=self.config.wall_target_m,
                      front_stop_m=self.config.front_stop_m,
                      front_distance_source="map_short_wall_in_heading_direction",
                      map_front_clearance_m=self.mission.front_remaining(f.pose) if f else None,
                      front_observed=bool(self.wall and self.wall.front_observed),
                      blind_front_obstacle_detection_available=False,
                      pose_source="tf" if self.use_map_tf else "map_pose",
                      pose_covariance_available=self.pose_covariance_available,
                      speed_feedback_available=False, stopped_verified=False,
                      automatic_transfer_available=False, coverage_available=False)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))

    def destroy_node(self):
        if hasattr(self, "lease") and self.context.ok():
            self._halt("controller shutdown", "CANCELLED")
        return super().destroy_node()


def main(args=None):
    from f1_wall_cleaning.runner import run
    run(RectangleController, args)
