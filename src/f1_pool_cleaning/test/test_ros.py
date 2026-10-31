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
from f1_interfaces.srv import SetEdge
from f1_pool_cleaning.ros_support import stamp, to_observation_message
from f1_pool_cleaning.controller_node import PoolController
from f1_pool_cleaning.demo import SYNTHETIC_GEOMETRY, sample


def synthetic_geometry(extra=None):
    """Map-derived geometry overrides for a controller started without map:=."""
    return ([Parameter(name, value=value) for name, value in SYNTHETIC_GEOMETRY.items()]
            + list(extra or []))



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
        self.controller = PoolController(
            parameter_overrides=synthetic_geometry() + list(parameters or []))
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
        self.along, self.distance, self.angle = .8, .34, 0.
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
    h.wait(lambda: any(s["left_wall_distance_cm"] == 34. for s in h.statuses))
    distance_status = next(s for s in h.statuses if s["left_wall_distance_cm"] == 34.)
    assert distance_status["wall_distance_m"] == pytest.approx(.34)
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


def test_stage2_uses_map_pose_to_select_edge_and_enter_wall_band():
    h = Harness([Parameter("edge_task_enabled", value=True)])
    try:
        h.distance = .34
        h.start()
        h.wait(lambda: any(m.brush_enabled for m in h.commands))
        assert h.controller.selected_edge.id == "edge_0"
        assert h.controller.mission.phase == "FOLLOW"
        h.wait(lambda: any(s.get("current_edge_id") == "edge_0" and
                           s.get("edge_progress_m") is not None and
                           s.get("coverage_basis") == "left_brush_contact_centerline"
                           for s in h.statuses))
        h.distance = .31
        h.wait(lambda: h.controller.mission.phase == "APPROACH_WALL"
               and h.commands[-1].thrust_percent == -30.)
    finally:
        h.close()


def test_stage2_set_edge_is_explicit_and_wrong_pose_cannot_start():
    h = Harness([Parameter("edge_task_enabled", value=True)])
    try:
        h.wait(lambda: h.controller._feedback() is not None)
        response = h.controller._set_edge(SetEdge.Request(edge_id="edge_1"), SetEdge.Response())
        assert response.success and h.controller.selected_edge.id == "edge_1"
        start = h.controller._start(None, Trigger.Response())
        assert not start.success
        assert "selected edge" in start.message or "heading" in start.message
        assert not h.controller.mission.active
    finally:
        h.close()


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
    h.angle, h.distance = 0., .31
    h.wait(lambda: h.commands[-1].direction == 0 and h.commands[-1].brush_enabled
           and h.commands[-1].thrust_percent == -30.)


@pytest.mark.parametrize("extra", [Parameter("use_sim_time", value=True), Parameter("fifth_left_sign", value=0)])
def test_hardware_output_requires_clock_and_polarity(extra):
    with pytest.raises(ValueError, match="hardware output requires"):
        PoolController(parameter_overrides=synthetic_geometry()
                       + [Parameter("output_enabled", value=True), extra])


def test_ros_map_endpoint_closes_all_outputs(harness):
    h = harness
    h.distance = .46
    h.start()
    h.wait(lambda: any(m.thrust_enabled and m.brush_enabled for m in h.commands))
    c = h.controller.config
    endpoint = c.length_m - c.front_stop_m
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
    h.angle, h.distance = math.radians(12), .31
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
        h.angle, h.distance = math.radians(10), .34
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


def automatic_controller(tmp_path, **overrides):
    from f1_pool_cleaning.synthetic_map import rectangle_map
    values = dict(map=str(rectangle_map(tmp_path)), edge_task_enabled=True,
                  start_policy="nearest_reachable", use_map_tf=True,
                  require_localization_ready=True, scan_topic="/test_navigation_scan")
    values.update(overrides)
    return PoolController(parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])


def automatic_feedback(node, *, ready=True):
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    now = node.get_clock().now().nanoseconds
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "map", "base_link"
    tf.header.stamp = stamp(now)
    tf.transform.translation.x, tf.transform.translation.y = 2.8, 1.45
    tf.transform.rotation.w = 1.
    node.tf_buffer.set_transform(tf, "test")
    node._read_tf_pose()
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = "base_link", stamp(now)
    scan.angle_min, scan.angle_increment = -math.pi, math.pi/12
    scan.range_min, scan.range_max = .05, 20.
    scan.ranges = [10.]*24
    node._navigation_scan(scan)
    node._localization_status(String(data=json.dumps(dict(
        state="LOCALIZATION_READY" if ready else "LOCALIZATION_CHECK", ready=ready))))
    return scan


def test_automatic_start_tf_scan_gate_and_explicit_selection(tmp_path):
    node = automatic_controller(tmp_path)
    try:
        scan = automatic_feedback(node, ready=False)
        assert not node._start(None, Trigger.Response()).success
        automatic_feedback(node)
        assert node.scan_sub.topic_name == "/test_navigation_scan"
        assert node._set_edge(SetEdge.Request(edge_id="edge_2"), SetEdge.Response()).success
        assert node._start(None, Trigger.Response()).success
        assert node.mission.requested_edge_id == "edge_2"
        assert node.requested_start_edge is None  # consumed for this task only
        node.mission.future.result(timeout=5)
        automatic_feedback(node)
        node._tick()
        assert node.selected_edge.id == "edge_2"
        assert node.mission.phase == "NAVIGATING"
        assert not node.mission.target_wall_enabled
        assert not node.mission.last_effort.brush_percent
        # A newly invalid scan cannot be hidden behind the receive watchdog.
        scan.header.stamp = stamp(node.get_clock().now().nanoseconds)
        scan.ranges = [float("nan")]*24
        node._navigation_scan(scan)
        node._tick()
        assert node.mission.phase == "PAUSED"
        node._cancel(None, Trigger.Response())
        automatic_feedback(node)
        assert node._start(None, Trigger.Response()).success
        node.mission.future.result(timeout=5)
        automatic_feedback(node)
        node._tick()
        assert node.selected_edge.id == "edge_0"
    finally:
        node.destroy_node()


@pytest.mark.parametrize("lose_localization", [False, True])
def test_automatic_planning_waits_for_lease_and_rechecks_localization(tmp_path, lose_localization):
    from concurrent.futures import Future
    from types import SimpleNamespace
    from f1_interfaces.msg import ControlState
    from f1_interfaces.srv import AcquireControl
    node = automatic_controller(tmp_path, output_enabled=True)
    try:
        automatic_feedback(node)
        lease = node.lease
        pending = Future()
        lease.acquire = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: pending)
        bridge = ControlState(connected=True, connection_id="automatic-test", epoch=7,
                              state="READY", source="NANO", armed=True, result="OK",
                              allow_fifth_thruster=True, ctrl_age_sec=0., ack_age_sec=0.)
        bridge.stamp = stamp(node.get_clock().now().nanoseconds)
        lease._bridge(bridge)
        assert node._start(None, Trigger.Response()).success
        assert node.mission.phase == "ACQUIRING" and node.mission.future is None
        automatic_feedback(node, ready=not lose_localization)
        bridge.state, bridge.owner, bridge.motion_authorized = "ACTIVE", lease.owner, True
        bridge.stamp = stamp(node.get_clock().now().nanoseconds)
        lease._bridge(bridge)
        pending.set_result(AcquireControl.Response(success=True, token="test-token", epoch=7))
        if lose_localization:
            assert not lease.token and node.mission.phase == "PAUSED"
            assert node.mission.future is None
        else:
            assert lease.token == "test-token"
            assert node.mission.phase in ("PLANNING", "NAVIGATING")
            assert not node.mission.last_effort.brush_percent
    finally:
        node.destroy_node()


def test_automatic_preflight_matches_start_and_run_identity_changes(tmp_path):
    node = automatic_controller(tmp_path)
    try:
        assert not node._start(None, Trigger.Response()).success
        assert node._start_blockers()
        automatic_feedback(node)
        assert not node._start_blockers()
        assert node._start(None, Trigger.Response()).success
        first_run = node.run_id
        assert first_run
        node._cancel(None, Trigger.Response())
        automatic_feedback(node)
        assert node._start(None, Trigger.Response()).success
        assert node.run_id and node.run_id != first_run
        node._halt("test complete", "COMPLETE")
        node._status()
        completed_at = node.completed_at_ns
        node._status()
        assert completed_at is not None and node.completed_at_ns == completed_at
    finally:
        node.destroy_node()


def test_automatic_start_rejects_recently_received_but_old_localization(tmp_path):
    node = automatic_controller(tmp_path)
    try:
        automatic_feedback(node)
        assert not node._start_blockers()
        node._localization_status(String(data=json.dumps(dict(ready=True, state="LOCALIZATION_READY",
            stamp_ns=node.get_clock().now().nanoseconds-2_000_000_000))))
        response = node._start(None, Trigger.Response())
        assert not response.success and "source timestamp" in response.message
    finally:
        node.destroy_node()


def test_stop_cuts_outputs_retains_coverage_and_restart_creates_new_run(tmp_path):
    node = automatic_controller(tmp_path)
    try:
        automatic_feedback(node)
        assert node._start(None, Trigger.Response()).success
        run = node.run_id
        node.mission.cleaned_by_edge = {"edge_0": [(1., 2.)]}
        node._set_selected_edge(node.pool.edges[0])
        stopped = []
        node._stop_outputs = lambda: stopped.append(True)
        assert node._stop_task(None, Trigger.Response()).success
        assert stopped and node.mission.phase == "STOPPED"
        assert node.mission.last_effort is None and not node.mission.active
        assert node.mission.cleaned_by_edge == {"edge_0": [(1., 2.)]}
        assert node.selected_edge is not None
        automatic_feedback(node)
        assert node._start(None, Trigger.Response()).success
        assert node.run_id != run and not node.mission.cleaned_by_edge
        node._set_selected_edge(node.pool.edges[0])
        assert node._cancel(None, Trigger.Response()).success
        assert node.selected_edge is None and node.mission.edge is None
        automatic_feedback(node)
        assert node._start(None, Trigger.Response()).success
        assert node.run_id != run and not node.mission.cleaned_by_edge
    finally:
        node.destroy_node()


def test_expected_tracks_are_boat_center_segments_and_only_active_navigation(tmp_path):
    from f1_pool_cleaning.navigation import Waypoint
    node = automatic_controller(tmp_path)
    try:
        track = node._expected_track()
        assert track["map_id"] == node.map_id and track["frame_id"] == "map"
        assert len(track["cleaning_segments"]) == 4
        for edge, segment in zip(node.pool.edges, track["cleaning_segments"]):
            point = node.mission.navigator.edge_pose(edge, edge.clean_start_s_m, node.config.wall_target_m)
            assert segment["points"][0] == pytest.approx([point.x, point.y])
        node.mission.path = [Waypoint(1., 2., 0.), Waypoint(3., 4., 0.)]
        node.mission.phase = "NAVIGATING"
        assert node._expected_track()["navigation_path"] == [[1., 2.], [3., 4.]]
        node._stop_task(None, Trigger.Response())
        assert not node._expected_track()["navigation_path"]
    finally:
        node.destroy_node()


@pytest.mark.parametrize("failure", ["source_age", "receive_age", "invalid", "nan"])
def test_center_wall_distance_never_reuses_invalid_or_stale_observation(tmp_path, failure):
    from types import SimpleNamespace
    from f1_pool_cleaning.perception import Observation
    node = automatic_controller(tmp_path)
    try:
        snapshots = []
        node.status_pub = SimpleNamespace(publish=lambda msg: snapshots.append(json.loads(msg.data)))
        now = node.get_clock().now().nanoseconds
        node.wall = Observation(now, valid=True, distance=.34)
        node.received["wall"] = time.monotonic()
        node._status()
        assert snapshots[-1]["center_wall_distance_m"] == .34
        if failure == "source_age":
            node.wall = replace(node.wall, stamp_ns=now-2_000_000_000)
        elif failure == "receive_age":
            node.received["wall"] -= 2
        elif failure == "invalid":
            node.wall = replace(node.wall, valid=False)
        else:
            node.wall = replace(node.wall, distance=float("nan"))
        node._status()
        assert snapshots[-1]["center_wall_distance_m"] is None
    finally:
        node.destroy_node()
