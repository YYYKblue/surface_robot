"""Preview adapter. Intentionally has no hardware publisher or lease clients."""
from dataclasses import fields
import json
import math
import time

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_interfaces.msg import ActuatorCommand, WallObservation
from f1_wall_cleaning.ros_support import stamp, stamp_ns, from_observation_message

from .config import Config
from .mission import Mission, Feedback, Pose, Motion, Effort, command_fields, wrap


class RectangleController(Node):
    def __init__(self, **kwargs):
        super().__init__("rectangle_controller", **kwargs)
        self.declare_parameter("output_enabled", False, ParameterDescriptor(read_only=True))
        if self.get_parameter("output_enabled").value:
            self.destroy_node()
            raise ValueError("v0.1 is preview-only: hardware lease/output adapter is not implemented")
        values = {}
        for field in fields(Config()):
            self.declare_parameter(field.name, getattr(Config(), field.name), ParameterDescriptor(read_only=True))
            values[field.name] = self.get_parameter(field.name).value
        self.config = Config(**values).validate()
        self.mission = Mission(self.config)
        self.wall = self.pose = self.motion = None
        self.session, self.sequence = "", -1
        self.received = {}
        self.clock_pair = None
        self.preview_sequence = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wall_sub = self.create_subscription(WallObservation, "/rectangle_cleaning/wall_observation", self._wall, qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/rectangle_cleaning/map_pose", self._pose, qos)
        self.motion_sub = self.create_subscription(Odometry, "/rectangle_cleaning/local_motion", self._motion, qos)
        self.preview_pub = self.create_publisher(ActuatorCommand, "/rectangle_cleaning/command_preview", 2)
        self.status_pub = self.create_publisher(String, "/rectangle_cleaning/status", 5)
        self.task_services = [
            self.create_service(Trigger, "/rectangle_cleaning/start", self._start),
            self.create_service(Trigger, "/rectangle_cleaning/pause", self._pause),
            self.create_service(Trigger, "/rectangle_cleaning/cancel", self._cancel),
            self.create_service(Trigger, "/rectangle_cleaning/confirm_second_wall", self._second),
        ]
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self._tick, clock=steady)
        self.status_timer = self.create_timer(.1, self._status, clock=steady)

    def _invalidate(self, name, reason):
        setattr(self, name, None)
        if self.mission.active:
            self.mission.halt(reason)
            self._stop_preview()

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

    @staticmethod
    def _covariance_ok(cov, indices, limits):
        return all(math.isfinite(v) for v in cov) and all(
            0 < cov[i] <= limit for i, limit in zip(indices, limits))

    def _pose(self, msg):
        c, p, cov = self.config, msg.pose.pose, msg.pose.covariance
        q = p.orientation
        values = (p.position.x, p.position.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (msg.header.frame_id != c.map_frame or not all(math.isfinite(v) for v in values)
                or abs(norm - 1.) > .01
                or not self._covariance_ok(cov, (0, 7, 35), (c.pose_variance_max, c.pose_variance_max, c.yaw_variance_max))):
            self._invalidate("pose", "invalid map pose/frame/quaternion/covariance")
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        pose = Pose(stamp_ns(msg.header.stamp), p.position.x, p.position.y, yaw)
        if self.pose and pose.stamp_ns <= self.pose.stamp_ns:
            return
        if self.pose and self.mission.active and (
                math.hypot(pose.x - self.pose.x, pose.y - self.pose.y) > c.pose_jump_m
                or abs(wrap(pose.yaw - self.pose.yaw)) > c.yaw_jump_rad):
            self._invalidate("pose", "map localization jump; manual reassociation required")
            return
        self.pose = pose
        self.received["pose"] = time.monotonic()

    def _motion(self, msg):
        c, t = self.config, msg.twist.twist
        if (msg.header.frame_id != c.odom_frame or msg.child_frame_id != c.base_frame
                or not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z))
                or not self._covariance_ok(msg.twist.covariance, (0, 7, 35),
                                          (c.velocity_variance_max, c.velocity_variance_max, c.yaw_rate_variance_max))):
            self._invalidate("motion", "invalid local motion/frame/covariance; zero is not unknown-speed evidence")
            return
        motion = Motion(stamp_ns(msg.header.stamp), t.linear.x, t.linear.y, t.angular.z)
        if self.motion and motion.stamp_ns <= self.motion.stamp_ns:
            return
        self.motion = motion
        self.received["motion"] = time.monotonic()

    def _feedback(self):
        if self.wall is None or self.pose is None or self.motion is None:
            return None
        return Feedback(self.wall, self.pose, self.motion)

    def _stream_error(self):
        now = time.monotonic()
        for name, max_age in (("wall", self.config.wall_max_age_sec),
                              ("pose", self.config.pose_max_age_sec),
                              ("motion", self.config.motion_max_age_sec)):
            if name not in self.received or now - self.received[name] >= max_age:
                return name + " receive stream stopped"
        return ""

    def _begin(self, response, second=False):
        error = self._stream_error()
        if error:
            response.success, response.message = False, error
            return response
        method = self.mission.confirm_second_wall if second else self.mission.start
        response.success, response.message = method(self._feedback(), self.get_clock().now().nanoseconds)
        if response.success:
            self._tick()
        return response

    def _start(self, request, response):
        return self._begin(response)

    def _second(self, request, response):
        return self._begin(response, second=True)

    def _pause(self, request, response):
        self.mission.halt("user pause; cancel and manually replace at A to restart")
        self._stop_preview()
        response.success, response.message = True, self.mission.reason
        return response

    def _cancel(self, request, response):
        self.mission.halt("user cancelled", "CANCELLED")
        self._stop_preview()
        response.success, response.message = True, "cancelled"
        return response

    def _tick(self):
        now, mono = self.get_clock().now().nanoseconds, time.monotonic()
        if self.clock_pair and now < self.clock_pair[0]:
            self.wall = self.pose = self.motion = None
            self.received.clear()
            self.sequence, self.session = -1, ""
            self.mission.halt("ROS clock reversed; cancel/restart required")
            self._stop_preview()
        self.clock_pair = now, mono
        if not self.mission.active:
            return
        error = self._stream_error()
        if error:
            self.mission.halt(error)
            self._stop_preview()
            return
        effort = self.mission.update(self._feedback(), now)
        if effort:
            self._publish(effort)
        elif not self.mission.active:
            self._stop_preview()

    def _publish(self, effort):
        msg = ActuatorCommand()
        # Empty token cannot authorize actuation. Do NOT remap this topic to /f1.
        msg.token, msg.sequence = "", self.preview_sequence
        self.preview_sequence += 1
        msg.computed_at, msg.valid_until = stamp(effort.computed_ns), stamp(effort.valid_until_ns)
        msg.direction_until = stamp(effort.direction_until_ns)
        for key, value in command_fields(effort, self.config).items():
            setattr(msg, key, value)
        self.preview_pub.publish(msg)

    def _stop_preview(self):
        now = self.get_clock().now().nanoseconds
        self._publish(Effort(now, now))

    def _status(self):
        f = self._feedback()
        now = self.get_clock().now().nanoseconds
        effort = self.mission.last_effort
        status = dict(phase=self.mission.phase, reason=self.mission.reason,
                      feedback_error=self._stream_error() or self.mission.error(f, now),
                      output_enabled=False, lease_held=False, brush_enabled=False,
                      fifth_polarity_known=self.config.fifth_left_sign != 0,
                      logical_side_left_percent=effort.side_left_percent if effort and now < effort.valid_until_ns else 0.,
                      completed_segments=self.mission.completed_segments,
                      automatic_transfer_available=False, coverage_available=False)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    from f1_wall_cleaning.runner import run
    run(RectangleController, args)
