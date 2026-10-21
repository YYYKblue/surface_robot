"""Synthetic scan -> real ROS nodes -> actual bridge Session; never opens serial."""
from dataclasses import fields
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from f1_bridge_interfaces.msg import ActuatorCommand, ControlState
from f1_bridge_interfaces.srv import AcquireControl, ReleaseControl
from nano_ros_bridge.session import Session, Command
from nano_ros_bridge.protocol import Control
from f1_wall_cleaning.config import Config
from f1_wall_cleaning.controller_node import WallController
from f1_wall_cleaning.perception_node import WallPerception
from f1_wall_cleaning.ros_support import stamp, stamp_ns
from f1_wall_cleaning.synthetic import pool_scan


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class SimBridge(Node):
    def __init__(self):
        super().__init__("wall_test_bridge")
        self.core = Session(motion_enabled=True)
        self.core.opened()
        self.source, self.epoch = "NANO", 1
        self.commands, self.releases = [], []
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
        accepted = self.core.accept(cmd, time.monotonic(), self.get_clock().now().nanoseconds)
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
    def __init__(self, config, live):
        parameters = [Parameter(f.name, value=getattr(config, f.name)) for f in fields(config)]
        self.perception = WallPerception(parameter_overrides=parameters)
        self.controller = WallController(parameter_overrides=parameters + [Parameter("output_enabled", value=live)])
        self.bridge = SimBridge()
        self.driver = Node("wall_test_scan_driver")
        self.pub = self.driver.create_publisher(LaserScan, "/pavo2_scan", qos_profile_sensor_data)
        self.scanning = True
        self.front_missing = False
        self.scan_template = pool_scan(1_000_000_000, config=config)
        self.timer = self.driver.create_timer(.02, self.scan)
        self.executor = SingleThreadedExecutor()
        self.nodes = [self.driver, self.perception, self.controller, self.bridge]
        for node in self.nodes:
            self.executor.add_node(node)

    def scan(self):
        if not self.scanning:
            return
        s = self.scan_template
        if self.front_missing:
            s = pool_scan(1_000_000_000, missing_front=True)
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.driver.get_clock().now().to_msg(), s.frame
        m.angle_min, m.angle_increment = s.angle_min, s.angle_increment
        m.angle_max = s.angle_min + (len(s.ranges) - 1) * s.angle_increment
        m.range_min, m.range_max, m.ranges = s.range_min, s.range_max, list(s.ranges)
        self.pub.publish(m)

    def until(self, predicate, timeout=4.):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)
        assert predicate(), (self.controller.follower.phase, self.controller.follower.reason,
                             self.bridge.core.reason, self.bridge.core.rejected)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.005)

    def ready(self):
        self.until(lambda: self.controller.observation is not None and self.controller.observation.valid
                   and (not self.controller.live or self.controller.bridge is not None
                        and self.controller.bridge.state == "READY"))

    def call(self, operation):
        client = self.driver.create_client(Trigger, "/wall_cleaning/" + operation)
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
        self.executor.shutdown()
        for node in self.nodes:
            if node is not self.controller:
                node.destroy_node()


def test_default_unmeasured_geometry_and_preview_have_no_actuator_publisher(calibrated):
    for config, should_start in [(Config(), False), (calibrated, True)]:
        h = Harness(config, live=False)
        try:
            h.ready()
            assert h.call("start").success == should_start
            if should_start:
                h.until(lambda: h.controller.follower.phase == "CLEAN_WALL")
                assert h.call("pause").success
                assert h.call("resume").success
                assert h.call("cancel").success
            assert h.controller.command_pub is None
            assert not h.bridge.commands and h.bridge.acquires == 0
        finally:
            h.close()


def test_ros_live_pipeline_rc_return_requires_explicit_resume(calibrated):
    h = Harness(calibrated, live=True)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: any(m.brush_enabled and accepted for m, accepted in h.bridge.commands))
        token = h.controller.token
        assert token
        for m, accepted in h.bridge.commands:
            assert accepted
            assert 0 < stamp_ns(m.valid_until) - stamp_ns(m.computed_at) <= 100_000_000
            assert not m.thrust_enabled and m.mode == ActuatorCommand.BOTH
            assert m.direction in (ActuatorCommand.STOP, ActuatorCommand.FORWARD)
            assert m.speed_percent == 25
        h.bridge.source, h.bridge.epoch = "RC", 2
        h.until(lambda: h.controller.follower.phase == "PAUSED_RC")
        assert not h.controller.token
        h.bridge.source, h.bridge.epoch = "NANO", 3
        h.until(lambda: h.controller.bridge.state == "READY")
        count = len(h.bridge.commands)
        h.spin_for(.15)
        assert h.controller.follower.phase == "PAUSED_RC" and len(h.bridge.commands) == count
        assert h.call("resume").success
        h.until(lambda: h.controller.token and len(h.bridge.commands) > count)
        assert h.controller.token != token
        h.scanning = False
        h.until(lambda: h.controller.follower.phase == "PAUSED_SAFETY")
        h.until(lambda: not h.bridge.core.token)
        assert h.bridge.core.command is None
    finally:
        h.close()


def test_late_acquire_after_pause_is_released_without_motion(calibrated):
    h = Harness(calibrated, live=True)
    try:
        h.ready()
        response = h.controller._begin(Trigger.Response())
        assert response.success and h.controller.pending is not None
        h.controller._halt("PAUSED_USER", "pause before service response")
        h.until(lambda: h.bridge.acquires == 1 and bool(h.bridge.releases))
        assert not h.bridge.commands and not h.controller.token
        assert h.controller.follower.phase == "PAUSED_USER"
        h.until(lambda: not h.bridge.core.token)
    finally:
        h.close()


def test_unknown_front_revokes_running_lease(calibrated):
    h = Harness(calibrated, live=True)
    try:
        h.ready()
        assert h.call("start").success
        h.until(lambda: bool(h.bridge.commands))
        h.front_missing = True
        h.until(lambda: h.controller.follower.phase == "PAUSED_SAFETY")
        assert "front" in h.controller.follower.reason
        h.until(lambda: not h.bridge.core.token)
    finally:
        h.close()


def test_default_launch_exits_cleanly_without_bridge():
    process = subprocess.Popen(["ros2", "launch", "f1_wall_cleaning", "wall_cleaning.launch.py"],
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


@pytest.mark.parametrize("parameters,reason", [
    (["output_enabled:=true"], "not measured/confirmed"),
    (["output_enabled:=true", "use_sim_time:=true"], "rejects use_sim_time"),
])
def test_hardware_startup_gates(parameters, reason):
    command = ["ros2", "run", "f1_wall_cleaning", "wall_controller", "--ros-args"]
    for parameter in parameters:
        command += ["-p", parameter]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode != 0 and reason in result.stderr


def test_live_clock_fault_latches_and_sim_time_cannot_be_enabled(calibrated):
    h = Harness(calibrated, live=True)
    try:
        h.ready()
        assert not h.controller.set_parameters([Parameter("use_sim_time", value=True)])[0].successful
        assert h.call("start").success
        h.until(lambda: bool(h.bridge.commands))
        h.controller.clock_pair = (h.controller.get_clock().now().nanoseconds - 1_000_000_000, time.monotonic())
        h.controller._tick()
        assert h.controller.clock_fault and h.controller.follower.phase == "PAUSED_SAFETY"
        assert not h.call("resume").success
        h.until(lambda: not h.bridge.core.token)
    finally:
        h.close()
