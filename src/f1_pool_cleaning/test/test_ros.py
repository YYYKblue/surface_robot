"""Real DDS/service preview tests on localhost; no bridge or serial devices."""
import json
import math
import time
from dataclasses import replace

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_interfaces.msg import ActuatorCommand, WallObservation
from f1_pool_cleaning.ros_support import stamp, to_observation_message
from f1_pool_cleaning.controller_node import PoolController
from f1_pool_cleaning.demo import sample


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
    def __init__(self, parameters=None):
        self.controller = PoolController(parameter_overrides=parameters or [])
        self.peer = Node("pool_test_feedback")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.controller)
        self.executor.add_node(self.peer)
        self.wall_pub = self.peer.create_publisher(WallObservation, "/pool_cleaning/wall_observation", qos_profile_sensor_data)
        self.pose_pub = self.peer.create_publisher(PoseWithCovarianceStamped, "/pool_cleaning/map_pose", qos_profile_sensor_data)
        self.commands, self.statuses = [], []
        self.command_sub = self.peer.create_subscription(ActuatorCommand, "/pool_cleaning/command_preview", self.commands.append, 10)
        self.status_sub = self.peer.create_subscription(String, "/pool_cleaning/status", lambda m: self.statuses.append(json.loads(m.data)), 10)
        self.start_client = self.peer.create_client(Trigger, "/pool_cleaning/start")
        self.sequence, self.session, self.enabled = 0, "test-perception", True
        self.pose_offset = 0.
        self.wall_valid = True
        self.along, self.distance, self.angle = .8, .4, 0.
        self.timer = self.peer.create_timer(.02, self.publish)

    def messages(self):
        c = self.controller.config
        now = self.peer.get_clock().now().nanoseconds
        f = sample(c, now, along=self.along, distance=self.distance, angle=self.angle)
        p = PoseWithCovarianceStamped()
        p.header.frame_id, p.header.stamp = c.map_frame, stamp(now)
        p.pose.pose.position.x, p.pose.pose.position.y = f.pose.x + self.pose_offset, f.pose.y
        p.pose.pose.orientation.z = math.sin(f.pose.yaw / 2)
        p.pose.pose.orientation.w = math.cos(f.pose.yaw / 2)
        for i in (0, 7, 35):
            p.pose.covariance[i] = .001
        self.sequence += 1
        wall = replace(f.wall, left_turn_clear=False, right_turn_clear=False,
                       valid=self.wall_valid,
                       reason=f.wall.reason if self.wall_valid else "left wall association lost")
        w = to_observation_message(wall, c.base_frame, self.session, self.sequence)
        return p, w

    def publish(self):
        if self.enabled:
            p, w = self.messages()
            self.pose_pub.publish(p)
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


def test_real_ros_brush_preview_and_stream_watchdog(harness):
    h = harness
    h.start()
    h.wait(lambda: any(m.brush_enabled for m in h.commands))
    assert all(not m.token and m.direction == 0 and not m.thrust_enabled for m in h.commands)
    h.wait(lambda: bool(h.statuses))
    h.wait(lambda: any(s["left_wall_distance_cm"] == 40. for s in h.statuses))
    distance_status = next(s for s in h.statuses if s["left_wall_distance_cm"] == 40.)
    assert distance_status["wall_distance_m"] == pytest.approx(.4)
    assert all(not s["output_enabled"] and not s["lease_held"] for s in h.statuses)
    publishers = h.peer.get_publisher_names_and_types_by_node("pool_controller", "/")
    assert not any(name == "/f1/actuator_command" for name, _ in publishers)
    assert list(h.controller.clients) == []
    h.enabled = False
    h.wait(lambda: h.controller.mission.phase == "PAUSED")
    h.wait(lambda: h.commands[-1].direction == 0 and not h.commands[-1].brush_enabled)
    h.enabled = True
    h.wait(lambda: h.controller.wall is not None and h.controller._stream_error() == "")
    assert h.controller.mission.phase == "PAUSED"


def test_perception_restart_latches_pause(harness):
    h = harness
    h.start()
    h.session = "restarted-perception"
    h.wait(lambda: h.controller.mission.phase == "PAUSED")
    assert "perception restarted" in h.controller.mission.reason


def test_map_jump_is_accepted_without_extra_startup_or_identity_gates(harness):
    h = harness
    h.start()
    old_x = h.controller.pose.x
    h.pose_offset = .4
    h.wait(lambda: h.controller.pose and h.controller.pose.x > old_x + .3)
    assert h.controller.mission.active
    assert h.controller.mission.error(h.controller._feedback(), h.peer.get_clock().now().nanoseconds) == ""


def test_missing_covariance_is_diagnostic_and_does_not_block_control(harness):
    h = harness
    h.start()
    h.enabled = False
    p, _ = h.messages()
    p.pose.covariance[0] = 0.
    h.controller._pose(p)
    assert h.controller.pose is not None and h.controller.mission.active
    assert not h.controller.pose_covariance_available


@pytest.mark.parametrize("angle,direction", [(10., 3), (-10., 4), (12., 3), (-12., 4)])
def test_ros_ten_degree_gate_and_continuous_correction_without_turn_clearance(harness, angle, direction):
    h = harness
    h.angle = math.radians(angle)
    h.distance = .5
    h.start()
    h.wait(lambda: len(h.commands) >= 10)
    commands = h.commands[-8:]
    assert all(m.direction == direction and m.mode == 1 for m in commands)
    assert all(m.brush_enabled == (abs(angle) <= 10) and m.thrust_percent == 30. for m in commands)
    assert all(m.direction_until.sec == 0 and m.direction_until.nanosec == 0 for m in commands)
    assert h.controller.mission.active
    # The next scan restores brush propulsion and reverses side correction.
    h.angle, h.distance = 0., .37
    h.wait(lambda: h.commands[-1].direction == 0 and h.commands[-1].brush_enabled
           and h.commands[-1].thrust_percent == -30.)


@pytest.mark.parametrize("extra", [Parameter("use_sim_time", value=True), Parameter("fifth_left_sign", value=0)])
def test_hardware_output_requires_clock_and_polarity(extra):
    with pytest.raises(ValueError, match="hardware output requires"):
        PoolController(parameter_overrides=[Parameter("output_enabled", value=True), extra])


def test_ros_map_endpoint_closes_all_outputs(harness):
    h = harness
    h.distance = .46
    h.start()
    h.wait(lambda: any(m.thrust_enabled and m.brush_enabled for m in h.commands))
    c = h.controller.config
    endpoint = c.length_m - c.hull_half_length_m - c.front_stop_m
    # Feed a sequence of updated positions until the front threshold is reached.
    while h.along < endpoint:
        h.along = min(endpoint + .001, h.along + .15)
        target = h.along
        h.wait(lambda: h.controller.pose and c.pool_xy(h.controller.pose.x, h.controller.pose.y)[0] >= target - 1e-8)
    h.wait(lambda: h.controller.mission.phase == "STOPPED")
    h.wait(lambda: not h.commands[-1].brush_enabled and not h.commands[-1].thrust_enabled)
    assert h.commands[-1].direction == 0


def test_tf_pose_preserves_source_stamp_and_accepts_updated_pose():
    from geometry_msgs.msg import TransformStamped
    h = Harness([Parameter("use_map_tf", value=True)])
    try:
        h.enabled = False
        p, w = h.messages()
        tf = TransformStamped()
        tf.header, tf.child_frame_id = p.header, "base_link"
        tf.transform.translation.x = p.pose.pose.position.x
        tf.transform.translation.y = p.pose.pose.position.y
        tf.transform.rotation = p.pose.pose.orientation
        h.controller.tf_buffer.set_transform(tf, "test")
        h.controller._read_tf_pose()
        assert h.controller.pose.stamp_ns == w.header.stamp.sec * 10**9 + w.header.stamp.nanosec
        assert not h.controller.pose_covariance_available
        received = h.controller.received["pose"]
        h.controller._read_tf_pose()
        assert h.controller.received["pose"] == received
        h.controller._wall(w)
        assert h.controller._start(None, Trigger.Response()).success
        tf.header.stamp = stamp(h.peer.get_clock().now().nanoseconds)
        tf.transform.translation.x += .4
        h.controller.tf_buffer.set_transform(tf, "test")
        h.controller._read_tf_pose()
        assert h.controller.mission.active
        assert h.controller.pose.x == tf.transform.translation.x
    finally:
        h.close()


def test_map_tf_start_requires_localization_ready_gate():
    from geometry_msgs.msg import TransformStamped
    h = Harness([Parameter("use_map_tf", value=True),
                 Parameter("require_localization_ready", value=True)])
    try:
        h.enabled = False
        p, w = h.messages()
        tf = TransformStamped()
        tf.header, tf.child_frame_id = p.header, "base_link"
        tf.transform.translation.x = p.pose.pose.position.x
        tf.transform.translation.y = p.pose.pose.position.y
        tf.transform.rotation = p.pose.pose.orientation
        h.controller.tf_buffer.set_transform(tf, "test")
        h.controller._read_tf_pose()
        h.controller._wall(w)
        blocked = h.controller._start(None, Trigger.Response())
        assert not blocked.success and "localization" in blocked.message
        h.controller._localization_status(String(data=json.dumps({
            "state": "LOCALIZATION_READY", "ready": True, "reason": "stable"})))
        allowed = h.controller._start(None, Trigger.Response())
        assert allowed.success
    finally:
        h.close()


class FakeBridge:
    """DDS lease peer only; never opens a serial port."""
    def __init__(self, h):
        from f1_interfaces.msg import ControlState
        from f1_interfaces.srv import AcquireControl, ReleaseControl
        self.h = h
        self.state = ControlState(connected=True, connection_id="fake-connection", epoch=7,
                                  state="READY", source="NANO", armed=True, result="OK",
                                  allow_fifth_thruster=True, ctrl_age_sec=0., ack_age_sec=0.)
        self.token = ""
        self.on_acquire = None
        self.commands, self.releases = [], []
        self.pub = h.peer.create_publisher(ControlState, "/f1/control_state", 5)
        self.sub = h.peer.create_subscription(ActuatorCommand, "/f1/actuator_command", self.commands.append, qos_profile_sensor_data)
        self.acquire = h.peer.create_service(AcquireControl, "/f1/acquire_control", self._acquire)
        self.release = h.peer.create_service(ReleaseControl, "/f1/release_control", self._release)
        self.timer = h.peer.create_timer(.02, self.publish)

    def publish(self):
        self.state.stamp = stamp(self.h.peer.get_clock().now().nanoseconds)
        self.pub.publish(self.state)

    def _acquire(self, req, res):
        self.token = "fake-lease"
        self.state.state, self.state.owner, self.state.motion_authorized = "ACTIVE", req.owner, True
        res.success, res.token, res.epoch = True, self.token, self.state.epoch
        if self.on_acquire:
            self.on_acquire()
        return res

    def _release(self, req, res):
        self.releases.append(req.token)
        res.success = req.token == self.token
        self.state.state, self.state.owner, self.state.motion_authorized = "READY", "", False
        self.token = ""
        return res


def test_live_lease_continuous_yaw_and_signed_fifth_output():
    h = Harness([Parameter("output_enabled", value=True)])
    b = FakeBridge(h)
    h.angle, h.distance = math.radians(12), .37
    try:
        h.wait(lambda: h.controller.lease.bridge is not None and h.controller.lease.acquire.service_is_ready())
        h.start()
        h.wait(lambda: len(b.commands) >= 12)
        assert all(m.token == "fake-lease" and m.direction == 3 and m.mode == 1
                   and not m.brush_enabled and m.thrust_percent == -30.
                   and m.direction_until.sec == 0 and m.direction_until.nanosec == 0
                   for m in b.commands[-10:])
        h.angle, h.distance = math.radians(-12), .5
        h.wait(lambda: b.commands[-1].direction == 4 and b.commands[-1].thrust_percent == 30.)
        h.angle, h.distance = math.radians(10), .4
        h.wait(lambda: b.commands[-1].brush_enabled and b.commands[-1].direction == 3
               and not b.commands[-1].thrust_enabled)
        h.controller._cancel(None, Trigger.Response())
        h.wait(lambda: bool(b.releases))
    finally:
        h.close()


def test_live_lease_survives_brief_wall_fit_loss_and_resumes_correction():
    h = Harness([Parameter("output_enabled", value=True)])
    b = FakeBridge(h)
    h.distance = .5
    try:
        h.wait(lambda: h.controller.lease.bridge is not None and h.controller.lease.acquire.service_is_ready())
        h.start()
        h.wait(lambda: any(m.brush_enabled and m.thrust_enabled for m in b.commands))
        h.wall_valid = False
        h.wait(lambda: h.controller.mission.phase == "WAIT_WALL")
        h.wait(lambda: b.commands and not b.commands[-1].brush_enabled and not b.commands[-1].thrust_enabled)
        begin = len(b.commands)
        h.wait(lambda: len(b.commands) >= begin + 8)
        assert all(m.direction == 0 and not m.brush_enabled and not m.thrust_enabled
                   and m.token == "fake-lease" for m in b.commands[begin:])
        assert h.controller.lease.token and not b.releases
        h.wait(lambda: any(s["phase"] == "WAIT_WALL" and not s["wall_valid"]
                          and s["left_wall_distance_cm"] is None and s["wall_distance_m"] is None
                          and "association lost" in s["wall_reason"] for s in h.statuses))
        h.angle, h.wall_valid = math.radians(40), True
        h.wait(lambda: h.controller.mission.phase == "CORRECT_HEADING" and b.commands[-1].direction == 3)
        assert not b.commands[-1].brush_enabled
        h.angle = math.radians(8)
        h.wait(lambda: b.commands[-1].brush_enabled and b.commands[-1].direction == 3)
        assert h.controller.lease.token and not b.releases
        h.controller._cancel(None, Trigger.Response())
        h.wait(lambda: bool(b.releases))
    finally:
        h.close()


def test_wall_fit_lost_during_acquisition_starts_neutral_recovery():
    h = Harness([Parameter("output_enabled", value=True)])
    b = FakeBridge(h)
    def lose_wall_before_reply():
        h.wall_valid = False
        p, w = h.messages()
        h.controller._pose(p)
        h.controller._wall(w)
    b.on_acquire = lose_wall_before_reply
    try:
        h.wait(lambda: h.controller.lease.bridge is not None and h.controller.lease.acquire.service_is_ready())
        h.start()
        h.wait(lambda: h.controller.mission.phase == "WAIT_WALL" and bool(b.commands))
        assert h.controller.lease.token and not b.releases
        assert all(m.direction == 0 and not m.brush_enabled and not m.thrust_enabled for m in b.commands)
        h.angle, h.wall_valid = math.radians(40), True
        h.wait(lambda: b.commands[-1].direction == 3 and h.controller.mission.phase == "CORRECT_HEADING")
        h.controller._cancel(None, Trigger.Response())
        h.wait(lambda: bool(b.releases))
    finally:
        h.close()


def test_live_persistent_wall_fit_loss_pauses_and_releases_lease():
    h = Harness([Parameter("output_enabled", value=True), Parameter("wall_recovery_sec", value=.15)])
    b = FakeBridge(h)
    try:
        h.wait(lambda: h.controller.lease.bridge is not None and h.controller.lease.acquire.service_is_ready())
        h.start()
        h.wait(lambda: any(m.brush_enabled for m in b.commands))
        h.wall_valid = False
        h.wait(lambda: h.controller.mission.phase == "PAUSED")
        assert "left wall lost for" in h.controller.mission.reason
        assert "association lost" in h.controller.mission.reason
        h.wait(lambda: bool(b.releases))
        assert not h.controller.lease.token
        h.wall_valid = True
        h.wait(lambda: h.controller.wall and h.controller.wall.valid)
        assert h.controller.mission.phase == "PAUSED"
    finally:
        h.close()


@pytest.mark.parametrize("fault", ["RC", "epoch", "stream", "cancel", "bridge_stream"])
def test_live_leased_output_and_fault_release(fault):
    h = Harness([Parameter("output_enabled", value=True)])
    b = FakeBridge(h)
    h.distance = .46
    try:
        h.wait(lambda: h.controller.lease.bridge is not None and h.controller.lease.acquire.service_is_ready())
        h.start()
        h.wait(lambda: any(m.brush_enabled and m.thrust_enabled for m in b.commands))
        moving = [m for m in b.commands if m.brush_enabled]
        assert all(m.token == "fake-lease" and m.direction == 0 and m.brush_percent == 30.
                   and m.thrust_percent == 30. for m in moving)
        assert all(not m.token for m in h.commands)
        if fault == "RC":
            b.state.source = "RC"
        elif fault == "epoch":
            b.state.epoch += 1
        elif fault == "stream":
            h.enabled = False
        elif fault == "bridge_stream":
            b.timer.cancel()
        else:
            h.controller._cancel(None, Trigger.Response())
        h.wait(lambda: h.controller.mission.phase in ("PAUSED", "CANCELLED"))
        h.wait(lambda: bool(b.releases))
        h.wait(lambda: b.commands and not b.commands[-1].brush_enabled and not b.commands[-1].thrust_enabled)
        assert not h.controller.lease.token
        h.enabled = True
        b.state.source = "NANO"
        h.wait(lambda: h.controller._stream_error() == "")
        assert not h.controller.mission.active
    finally:
        h.close()


def test_cancel_during_acquire_releases_late_token():
    from concurrent.futures import Future
    from f1_interfaces.srv import AcquireControl
    h = Harness([Parameter("output_enabled", value=True)])
    b = FakeBridge(h)
    try:
        lease = h.controller.lease
        h.wait(lambda: lease.bridge is not None)
        generation = lease.generation
        future = Future()
        lease.pending = future
        h.controller.mission.halt("waiting", "ACQUIRING")
        h.controller._cancel(None, Trigger.Response())
        b.token = "late-token"
        future.set_result(AcquireControl.Response(success=True, token="late-token", epoch=7))
        lease._acquired(future, generation, "fake-connection", 7)
        h.wait(lambda: "late-token" in b.releases)
        assert not lease.token and h.controller.mission.phase == "CANCELLED"
        assert not b.commands
    finally:
        h.close()
