"""ROS transport checks with serial disabled; never opens a device."""
import time
import os
import signal
import subprocess

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl
from nano_ros_bridge.node import Bridge


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "174")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))


def test_ros_state_stop_and_unauthorized_commands():
    rclpy.init()
    bridge = Bridge()
    client = Node("bridge_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(client)
    states = []
    imu_messages = []
    sub = client.create_subscription(ControlState, "/f1/control_state", states.append, 5)
    from rclpy.qos import qos_profile_sensor_data
    imu_sub = client.create_subscription(Imu, "/f1/imu/data_raw", imu_messages.append,
                                         qos_profile_sensor_data)
    pub = client.create_publisher(ActuatorCommand, "/f1/actuator_command", 1)
    acquire = client.create_client(AcquireControl, "/f1/acquire_control")
    stop = client.create_client(Trigger, "/f1/stop")

    def until(predicate):
        end = time.monotonic() + 4.
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        assert predicate()

    try:
        until(lambda: states and acquire.service_is_ready() and stop.service_is_ready())
        assert states[-1].state == "DISCONNECTED" and not states[-1].connected
        assert not bridge.set_parameters([Parameter("use_sim_time", value=True)])[0].successful
        future = acquire.call_async(AcquireControl.Request(owner="test"))
        until(future.done)
        assert not future.result().success and not future.result().token
        until(lambda: pub.get_subscription_count() == 1)
        pub.publish(ActuatorCommand(token="replayed", sequence=1, direction=ActuatorCommand.FORWARD))
        until(lambda: bridge.worker.core.rejected > 0)
        future = stop.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success
        assert bridge.worker.thread is None and not bridge.worker.core.token
        until(lambda: bridge.imu_pub.get_subscription_count() == 1)
        sample = dict(kind="IMU", sequence=0, stm32_ms=100, status=6,
                      acceleration=[0., 0., 9.8], angular_velocity=[0., 0., .2])
        bridge.worker.imu.append(("offline", sample, time.time_ns(), time.monotonic_ns()))
        until(lambda: bool(imu_messages))
        assert imu_messages[-1].header.frame_id == "imu_link"
        assert imu_messages[-1].orientation_covariance[0] == -1.
        assert imu_messages[-1].linear_acceleration.z == 9.8
        assert imu_messages[-1].angular_velocity.z == .2
        sample = dict(sample, sequence=1, stm32_ms=120, status=2)
        bridge.worker.imu.append(("offline", sample, time.time_ns(), time.monotonic_ns()))
        until(lambda: bridge.converter.counts["status_rejected"] == 1)
        assert len(imu_messages) == 1
        assert sub is not None and imu_sub is not None
    finally:
        executor.shutdown()
        client.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("launch_file,fifth", [("bridge.launch.py", "false")])
def test_default_launch_and_group_sigint(launch_file, fifth):
    process = subprocess.Popen(
        ["ros2", "launch", "f1_nano_bridge", launch_file, "allow_fifth_thruster:=" + fifth],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        time.sleep(1.5)
        assert process.poll() is None, process.communicate()[0]
        os.killpg(process.pid, signal.SIGINT)
        output = process.communicate(timeout=5)[0]
        assert "serial=False, motion=False" in output, output
        assert "fifth=" + fifth.capitalize() in output, output
        assert "process has finished cleanly" in output, output
        assert process.returncode == 0, output
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


@pytest.mark.parametrize("parameters,reason", [
    (["use_sim_time:=true"], "rejects use_sim_time"),
    (["serial_enabled:=true", "motion_enabled:=true"], "direction_confirmed"),
    (["firmware_profile:=old"], "unsupported firmware profile"),
])
def test_bad_startup_configuration_rejected_before_serial(parameters, reason):
    command = ["ros2", "run", "f1_nano_bridge", "f1_bridge", "--ros-args"]
    for parameter in parameters:
        command += ["-p", parameter]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert reason in result.stderr


@pytest.mark.parametrize("source", ["worker_forward", "worker_backward", "imu"])
def test_main_exits_for_clock_recovery_and_cleans_up(monkeypatch, source):
    """Inject clock observations, never change host time or open hardware."""
    from nano_ros_bridge import node as bridge_module
    from nano_ros_bridge.imu.conversion import ClockDiscontinuity
    observed = []
    spins = []
    real_bridge = bridge_module.Bridge
    def make_bridge():
        bridge = real_bridge()
        observed.append(bridge)
        return bridge
    def spin_once(bridge, **kwargs):
        spins.append(True)
        assert len(spins) == 1, "recovery must not depend on a ROS timer firing"
        bridge.worker.core.token = "old-lease"
        if source == "imu":
            def stepped(*args, **kwargs):
                raise ClockDiscontinuity("injected IMU clock step")
            bridge.converter.convert = stepped
            bridge.worker.imu.append(("test", {}, 0, 0))
            bridge._imu()
        else:
            jump = 100_000_000 if source == "worker_forward" else -100_000_000
            offset = bridge.worker.clock_offset + jump
            bridge.worker.wall_clock_ns = lambda: time.monotonic_ns() + offset
            bridge.worker._check_clock()
    monkeypatch.setattr(bridge_module, "Bridge", make_bridge)
    monkeypatch.setattr(bridge_module.rclpy, "spin_once", spin_once)
    # main temporarily owns signal handling; restore it for the pytest runner.
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with pytest.raises(SystemExit) as error:
            bridge_module.main(args=[])
        assert error.value.code == 75
        assert len(observed) == 1 and not rclpy.ok()
        worker = observed[0].worker
        assert worker.clock_restart.is_set() and worker.quit.is_set()
        assert not worker.core.token and worker.handle is None
        # A fresh process starts without the old lease/fault; it cannot resume motion.
        rclpy.init(args=[])
        fresh = real_bridge()
        try:
            assert not fresh.worker.core.token and not fresh.worker.core.fault
            assert not fresh.worker.clock_restart.is_set()
            assert fresh.worker.snapshot()["state"] == "DISCONNECTED"
        finally:
            fresh.destroy_node()
            rclpy.shutdown()
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
