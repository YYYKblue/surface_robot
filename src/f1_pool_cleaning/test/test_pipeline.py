"""Synthetic scan -> real ROS nodes -> actual bridge Session; never opens serial."""
from dataclasses import fields, replace
from pathlib import Path
import math
import yaml
from collections import deque
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl
from nano_ros_bridge.session import Session, Command
from nano_ros_bridge.protocol import Control
from f1_pool_cleaning.perception_config import PerceptionConfig
from f1_pool_cleaning.controller_node import PoolController
from f1_pool_cleaning.demo import SYNTHETIC_GEOMETRY, sample
from f1_pool_cleaning.perception_node import WallPerception
from f1_pool_cleaning.ros_support import stamp, stamp_ns
from f1_pool_cleaning.synthetic import pool_scan
from f1_pool_cleaning.synthetic_map import rectangle_map


def synthetic_geometry(extra=None):
    """Map-derived geometry overrides for a controller started without map:=."""
    return ([Parameter(name, value=value) for name, value in SYNTHETIC_GEOMETRY.items()]
            + list(extra or []))


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class RecordingSession(Session):
    def __init__(self):
        super().__init__(motion_enabled=True, allow_fifth=True)
        self.revocations = []
        self.revocation_details = []

    def stop(self, reason):
        if self.token:
            self.revocations.append(reason)
            now = time.monotonic()
            self.revocation_details.append(dict(
                reason=reason, expired_by_ms=round((now - self.expires) * 1000, 2),
                command_sequence=self.command_seq))
        super().stop(reason)


class SimBridge(Node):
    def __init__(self):
        super().__init__("wall_test_bridge")
        self.core = RecordingSession()
        self.core.opened()
        self.source, self.epoch = "NANO", 1
        self.commands, self.releases = [], []
        self.command_timings = deque(maxlen=12)
        self.last_command_at = None
        self.acquires = 0
        self.pub = self.create_publisher(ControlState, "/f1/control_state", 5)
        self.sub = self.create_subscription(ActuatorCommand, "/f1/actuator_command", self.command, qos_profile_sensor_data)
        self.acquire_srv = self.create_service(AcquireControl, "/f1/acquire_control", self.acquire)
        self.release_srv = self.create_service(ReleaseControl, "/f1/release_control", self.release)
        self.timer = self.create_timer(.02, self.heartbeat)

    def command(self, m):
        cmd = Command(m.token, m.sequence, stamp_ns(m.computed_at), stamp_ns(m.valid_until),
                      m.direction, m.speed_percent, m.mode,
                      m.brush_enabled, m.brush_percent, m.thrust_enabled, m.thrust_percent, stamp_ns(m.direction_until))
        now, unix_ns = time.monotonic(), self.get_clock().now().nanoseconds
        accepted = self.core.accept(cmd, now, unix_ns)
        self.command_timings.append(dict(
            sequence=m.sequence, accepted=accepted,
            receive_gap_ms=None if self.last_command_at is None else round((now - self.last_command_at) * 1000, 2),
            computed_age_ms=round((unix_ns - cmd.computed_ns) / 1e6, 2),
            remaining_ms=round((cmd.deadline_ns - unix_ns) / 1e6, 2)))
        self.last_command_at = now
        self.commands.append((m, accepted))

    def acquire(self, request, response):
        self.acquires += 1
        response.token, response.message = self.core.acquire(request.owner, time.monotonic())
        response.success = bool(response.token)
        response.epoch = self.epoch
        response.first_command_timeout_ms = response.max_command_ttl_ms = 150
        return response

    def release(self, request, response):
        self.releases.append(request.token)
        response.success = self.core.release(request.token)
        return response

    def heartbeat(self):
        now = time.monotonic()
        packet = self.core.frame(now)
        if packet:
            self.core.sent(now)
        ctrl = Control(int(now * 1000) & 0xffffffff, self.source, self.epoch,
                       self.core.last_sent is not None, self.core.last_sent or 0,
                       "OK", "BOTH", True, False, 0, False, 0, (1500,) * 4)
        self.core.receive(ctrl, now)
        snap = self.core.snapshot(now)
        m = ControlState()
        m.stamp = self.get_clock().now().to_msg()
        for key, value in snap.items():
            if key != "ctrl":
                setattr(m, key, value)
        m.source, m.epoch = self.source, self.epoch
        m.armed, m.have_sequence, m.result, m.mode = True, True, "OK", "BOTH"
        self.pub.publish(m)


class Harness:
    def __init__(self, config, live, perception_delay=0.):
        parameters = [Parameter(f.name, value=getattr(config, f.name)) for f in fields(config)]
        self.perception = WallPerception(parameter_overrides=parameters)
        self.perception_delay = perception_delay
        self.perception_timings = deque(maxlen=12)
        process = self.perception.estimator.process

        def timed_process(scan, now_ns):
            begin = time.monotonic()
            observation = process(scan, now_ns)
            if self.perception_delay:
                time.sleep(self.perception_delay)
            elapsed_ms = (time.monotonic() - begin) * 1000
            queue_age_ms = (now_ns - scan.stamp_ns) / 1e6
            self.perception_timings.append(dict(
                queue_age_ms=round(queue_age_ms, 2), process_ms=round(elapsed_ms, 2),
                result_age_ms=round(queue_age_ms + elapsed_ms, 2)))
            return observation

        self.perception.estimator.process = timed_process
        self.controller = PoolController(parameter_overrides=synthetic_geometry(
            [Parameter("output_enabled", value=live)]))
        self.bridge = SimBridge()
        self.driver = Node("wall_test_scan_driver")
        self.pub = self.driver.create_publisher(LaserScan, "/pavo2_scan", qos_profile_sensor_data)
        self.scanning = True
        self.pose_pub = self.driver.create_publisher(PoseWithCovarianceStamped, "/pool_cleaning/map_pose", qos_profile_sensor_data)
        self.scan_template = pool_scan(1_000_000_000, wall_distance=.34, config=config)
        # 20 Hz leaves scan processing time within the 100/150 ms deadlines;
        # a 50 Hz synthetic source can build an artificial backlog on the Nano.
        self.timer = self.driver.create_timer(.05, self.scan)
        self.executor = SingleThreadedExecutor()
        self.nodes = [self.driver, self.perception, self.controller, self.bridge]
        for node in self.nodes:
            if node is not self.perception:
                self.executor.add_node(node)
        # Production launches perception separately and the bridge has its own
        # serial worker. A slow scan must not block this test's control/watchdog
        # callbacks merely because the harness puts everything in one process.
        self.perception_executor = SingleThreadedExecutor()
        self.perception_executor.add_node(self.perception)
        self.perception_thread = threading.Thread(target=self.perception_executor.spin)
        self.perception_thread.start()

    def scan(self):
        if not self.scanning:
            return
        # Match the installed rear-facing 280-degree lidar. A full synthetic
        # ring would also see the tilted left wall in the bow corridor and
        # correctly trigger the front stop in the signed-heading test.
        s = replace(self.scan_template, angle_min=math.radians(-140),
                    ranges=self.scan_template.ranges[160:-160])
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.driver.get_clock().now().to_msg(), s.frame
        m.angle_min, m.angle_increment = s.angle_min, s.angle_increment
        m.angle_max = s.angle_min + (len(s.ranges) - 1) * s.angle_increment
        m.range_min, m.range_max, m.ranges = s.range_min, s.range_max, list(s.ranges)
        self.pub.publish(m)
        f = sample(self.controller.config, self.driver.get_clock().now().nanoseconds)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = "map", stamp(f.pose.stamp_ns)
        pose.pose.pose.position.x, pose.pose.pose.position.y = f.pose.x, f.pose.y
        pose.pose.pose.orientation.z = math.sin(f.pose.yaw / 2)
        pose.pose.pose.orientation.w = math.cos(f.pose.yaw / 2)
        self.pose_pub.publish(pose)

    def until(self, predicate, timeout=4.):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)
        assert predicate(), dict(
            phase=self.controller.mission.phase, controller_reason=self.controller.mission.reason,
            bridge_reason=self.bridge.core.reason, rejected=self.bridge.core.rejected,
            revocations=self.bridge.core.revocation_details,
            max_tx_gap_ms=round(self.bridge.core.max_tx_gap * 1000, 2),
            commands=list(self.bridge.command_timings), perception=list(self.perception_timings))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)

    def ready(self):
        self.until(lambda: self.controller.wall is not None and self.controller.wall.valid
                   and self.controller.pose is not None
                   and (not self.controller.live or self.controller.lease.bridge is not None
                        and self.controller.lease.bridge.state == "READY"))

    def call(self, operation):
        client = self.driver.create_client(Trigger, "/pool_cleaning/" + operation)
        self.until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        self.until(future.done)
        result = future.result()
        self.driver.destroy_client(client)
        return result

    def close(self):
        self.controller.destroy_node()
        self.executor.remove_node(self.controller)
        self.spin_for(.03)
        self.perception_executor.shutdown()
        self.perception_thread.join()
        self.executor.shutdown()
        for node in self.nodes:
            if node is not self.controller:
                node.destroy_node()


@pytest.fixture
def profile():
    parameters = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/perception.yaml").read_text())
    return PerceptionConfig(**parameters["pool_perception"]["ros__parameters"]).validate()


def test_preview_scan_pipeline_has_no_actuator_output(profile):
    h = Harness(profile, live=False)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: h.controller.mission.last_effort is not None)
        assert h.controller.mission.last_effort.brush_percent == 30.
        assert h.controller.lease is None
        assert not h.bridge.commands and h.bridge.acquires == 0
        assert h.call("pause").success
        assert h.call("cancel").success
    finally:
        h.close()


@pytest.mark.parametrize("perception_delay", [0., .03])
def test_live_scan_pipeline_rc_return_requires_cancel_and_start(profile, perception_delay):
    h = Harness(profile, live=True, perception_delay=perception_delay)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: any(m.brush_enabled and accepted for m, accepted in h.bridge.commands))
        token = h.controller.lease.token
        assert token
        for m, accepted in h.bridge.commands:
            assert accepted
            assert 0 < stamp_ns(m.valid_until) - stamp_ns(m.computed_at) <= 100_000_000
            assert not m.thrust_enabled and m.mode == ActuatorCommand.BOTH
            assert m.direction == ActuatorCommand.STOP
            assert m.speed_percent == 25
        h.bridge.source, h.bridge.epoch = "RC", 2
        h.until(lambda: h.controller.mission.phase == "PAUSED")
        assert not h.controller.lease.token
        h.bridge.source, h.bridge.epoch = "NANO", 3
        h.until(lambda: h.controller.lease.bridge.state == "READY")
        count = len(h.bridge.commands)
        h.spin_for(.15)
        assert h.controller.mission.phase == "PAUSED" and len(h.bridge.commands) == count
        assert not h.call("start").success
        assert h.call("cancel").success
        assert h.call("start").success
        h.until(lambda: h.controller.lease.token and len(h.bridge.commands) > count)
        assert h.controller.lease.token != token
        h.scanning = False
        h.until(lambda: h.controller.mission.phase == "PAUSED")
        h.until(lambda: not h.bridge.core.token)
        assert h.bridge.core.command is None
    finally:
        h.close()


def test_live_scan_pipeline_signed_side_and_heading_commands(profile):
    h = Harness(profile, live=True)
    h.scan_template = pool_scan(10**9, wall_distance=.44, wall_angle=math.radians(12), config=profile)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: any(accepted and m.direction == 3 and m.thrust_percent == 30.
                            and not m.brush_enabled for m, accepted in h.bridge.commands))
        h.scan_template = pool_scan(10**9, wall_distance=.31, wall_angle=math.radians(-12), config=profile)
        h.until(lambda: any(accepted and m.direction == 4 and m.thrust_percent == -30.
                            and not m.brush_enabled for m, accepted in h.bridge.commands))
        h.scan_template = pool_scan(10**9, wall_distance=.34, config=profile)
        h.until(lambda: any(accepted and m.direction == 0 and m.brush_enabled
                            and not m.thrust_enabled for m, accepted in h.bridge.commands))
        assert all(accepted for _, accepted in h.bridge.commands)
        assert h.call("cancel").success
        h.until(lambda: not h.bridge.core.token)
    finally:
        h.close()


def test_perception_over_budget_stops_without_automatic_restart(profile):
    h = Harness(profile, live=True)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: any(accepted for _, accepted in h.bridge.commands))
        h.perception_delay = .18
        h.until(lambda: h.controller.mission.phase == "PAUSED")
        h.until(lambda: not h.bridge.core.token)
        assert not h.controller.lease.token and h.bridge.core.command is None
        h.perception_delay = 0.
        h.until(lambda: h.controller.wall is not None
                and h.controller.get_clock().now().nanoseconds - h.controller.wall.stamp_ns < 100_000_000)
        count = len(h.bridge.commands)
        h.spin_for(.2)
        assert h.controller.mission.phase == "PAUSED"
        assert len(h.bridge.commands) == count and h.bridge.acquires == 1
    finally:
        h.close()


def test_perception_reset_pauses_active_task_and_releases_lease(profile):
    h = Harness(profile, live=True)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: bool(h.bridge.commands))
        assert h.call("reset_observation").success
        h.until(lambda: h.controller.mission.phase == "PAUSED")
        assert "perception restarted" in h.controller.mission.reason
        h.until(lambda: not h.bridge.core.token)
        h.spin_for(.15)
        assert h.controller.mission.phase == "PAUSED"
    finally:
        h.close()


def test_live_clock_fault_latches_and_sim_time_cannot_be_enabled(profile):
    h = Harness(profile, live=True)
    try:
        h.ready()
        assert not h.controller.set_parameters([Parameter("use_sim_time", value=True)])[0].successful
        assert h.call("start").success
        h.until(lambda: bool(h.bridge.commands))
        h.controller.clock_pair = (h.controller.get_clock().now().nanoseconds - 1_000_000_000, time.monotonic())
        h.controller._tick()
        assert h.controller.clock_fault and h.controller.mission.phase == "PAUSED"
        assert h.call("cancel").success
        assert not h.call("start").success
        h.until(lambda: not h.bridge.core.token)
    finally:
        h.close()


def test_default_launch_exits_cleanly_without_bridge(tmp_path):
    reference_map = rectangle_map(tmp_path)
    process = subprocess.Popen(["ros2", "launch", "f1_pool_cleaning", "pool_cleaning.launch.py",
                                f"map:={reference_map}"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, start_new_session=True)
    try:
        time.sleep(1.5)
        assert process.poll() is None, process.communicate()[0]
        os.killpg(process.pid, signal.SIGINT)
        output = process.communicate(timeout=5)[0]
        assert output.count("process has finished cleanly") == 2, output
        assert process.returncode == 0, output
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
