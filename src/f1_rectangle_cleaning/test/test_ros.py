"""Real DDS/service preview tests on localhost; no bridge or serial devices."""
import json
import math
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_wall_interfaces.msg import WallObservation
from f1_bridge_interfaces.msg import ActuatorCommand
from f1_wall_cleaning.ros_support import stamp, to_observation_message
from f1_rectangle_cleaning.controller_node import RectangleController
from f1_rectangle_cleaning.demo import sample


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "178")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class Harness:
    def __init__(self):
        self.controller = RectangleController()
        self.peer = Node("rectangle_test_feedback")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.controller)
        self.executor.add_node(self.peer)
        self.wall_pub = self.peer.create_publisher(WallObservation, "/rectangle_cleaning/wall_observation", qos_profile_sensor_data)
        self.pose_pub = self.peer.create_publisher(PoseWithCovarianceStamped, "/rectangle_cleaning/map_pose", qos_profile_sensor_data)
        self.motion_pub = self.peer.create_publisher(Odometry, "/rectangle_cleaning/local_motion", qos_profile_sensor_data)
        self.commands, self.statuses = [], []
        self.command_sub = self.peer.create_subscription(ActuatorCommand, "/rectangle_cleaning/command_preview", self.commands.append, 10)
        self.status_sub = self.peer.create_subscription(String, "/rectangle_cleaning/status", lambda m: self.statuses.append(json.loads(m.data)), 10)
        self.start_client = self.peer.create_client(Trigger, "/rectangle_cleaning/start")
        self.sequence, self.session, self.enabled = 0, "test-perception", True
        self.pose_offset = 0.
        self.timer = self.peer.create_timer(.02, self.publish)

    def messages(self):
        c = self.controller.config
        now = self.peer.get_clock().now().nanoseconds
        f = sample(c, now)
        p = PoseWithCovarianceStamped()
        p.header.frame_id, p.header.stamp = c.map_frame, stamp(now)
        p.pose.pose.position.x, p.pose.pose.position.y = f.pose.x + self.pose_offset, f.pose.y
        p.pose.pose.orientation.z = math.sin(f.pose.yaw / 2)
        p.pose.pose.orientation.w = math.cos(f.pose.yaw / 2)
        for i in (0, 7, 35):
            p.pose.covariance[i] = .001
        m = Odometry()
        m.header.frame_id, m.header.stamp, m.child_frame_id = c.odom_frame, stamp(now), c.base_frame
        for i in (0, 7, 35):
            m.twist.covariance[i] = .001
        self.sequence += 1
        w = to_observation_message(f.wall, c.base_frame, self.session, self.sequence)
        return p, m, w

    def publish(self):
        if self.enabled:
            p, m, w = self.messages()
            self.pose_pub.publish(p)
            self.motion_pub.publish(m)
            self.wall_pub.publish(w)

    def wait(self, condition, timeout=3.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=.01)
            if condition():
                return
        raise AssertionError(f"timeout: {self.controller.mission.phase}: {self.controller.mission.reason}")

    def start(self):
        self.wait(lambda: self.controller._feedback() is not None and self.start_client.service_is_ready())
        future = self.start_client.call_async(Trigger.Request())
        self.wait(future.done)
        assert future.result().success, future.result().message

    def close(self):
        self.executor.shutdown()
        self.peer.destroy_node()
        self.controller.destroy_node()


@pytest.fixture
def harness():
    h = Harness()
    yield h
    h.close()


def test_real_ros_start_preview_forward_and_stream_watchdog(harness):
    h = harness
    h.start()
    h.wait(lambda: any(m.direction == 1 for m in h.commands))
    assert all(not m.token and not m.brush_enabled and not m.thrust_enabled for m in h.commands)
    h.wait(lambda: bool(h.statuses))
    assert all(not s["output_enabled"] and not s["lease_held"] for s in h.statuses)
    publishers = h.peer.get_publisher_names_and_types_by_node("rectangle_controller", "/")
    assert not any(name == "/f1/actuator_command" for name, _ in publishers)
    assert list(h.controller.clients) == []
    h.enabled = False
    h.wait(lambda: h.controller.mission.phase == "PAUSED")
    h.wait(lambda: h.commands[-1].direction == 0)
    h.enabled = True
    h.wait(lambda: h.controller.wall is not None and h.controller._stream_error() == "")
    assert h.controller.mission.phase == "PAUSED"


def test_perception_restart_latches_pause(harness):
    h = harness
    h.start()
    h.session = "restarted-perception"
    h.wait(lambda: h.controller.mission.phase == "PAUSED")
    assert "perception restarted" in h.controller.mission.reason


def test_map_jump_latches_pause(harness):
    h = harness
    h.start()
    h.pose_offset = .4
    h.wait(lambda: h.controller.mission.phase == "PAUSED")
    assert "localization jump" in h.controller.mission.reason


def test_invalid_covariance_clears_previous_feedback(harness):
    h = harness
    h.start()
    h.enabled = False
    p, m, _ = h.messages()
    p.pose.covariance[0] = 0.
    h.controller._pose(p)
    assert h.controller.pose is None and h.controller.mission.phase == "PAUSED"
    m.twist.covariance[35] = float("nan")
    h.controller._motion(m)
    assert h.controller.motion is None


def test_hardware_output_flag_is_rejected():
    with pytest.raises(ValueError, match="preview-only"):
        RectangleController(parameter_overrides=[Parameter("output_enabled", value=True)])
