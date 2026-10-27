"""Exercise installed launch, including both controller and perception configs."""
import json
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from f1_interfaces.msg import ControlState


@pytest.mark.parametrize("start_bridge", [False, True])
def test_preview_launch_and_reuse_existing_localization(monkeypatch, tmp_path, start_bridge):
    monkeypatch.setenv("ROS_DOMAIN_ID", "182")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    peer = Node("l_bringup_test")
    statuses, states = [], []
    peer.create_subscription(String, "/l_cleaning/status",
                             lambda m: statuses.append(json.loads(m.data)), 10)
    peer.create_subscription(ControlState, "/f1/control_state", states.append, 10)
    log_path = tmp_path / "launch.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "f1_l_cleaning", "l_bringup.launch.py",
            "start_localization:=false", f"start_bridge:={str(start_bridge).lower()}",
            "load_state_filename:=/missing/map.pbstream",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            until = time.monotonic() + 20.
            while time.monotonic() < until:
                rclpy.spin_once(peer, timeout_sec=.05)
                if statuses and (states or not start_bridge):
                    break
                assert process.poll() is None, log_path.read_text()
            assert statuses, log_path.read_text()
            status = statuses[-1]
            assert status["edge_count"] == 2 and status["edge_task_enabled"]
            assert status["phase"] == "IDLE"
            assert status["pose_source"] == "tf"
            assert not status["output_enabled"] and not status["lease_held"]
            assert not status["brush_enabled"] and not status["localization_ready"]
            assert status["localization_state"] == "LOCALIZATION_CHECK"
            names = peer.get_node_names()
            assert names.count("l_controller") == 1
            assert names.count("l_perception") == 1
            assert "pool_controller" not in names and "pool_localization" not in names
            # Verify the renamed perception node actually loaded its calibrated
            # YAML, rather than silently using the uncalibrated node defaults.
            from rcl_interfaces.srv import GetParameters
            client = peer.create_client(GetParameters, "/l_perception/get_parameters")
            assert client.wait_for_service(timeout_sec=3.)
            future = client.call_async(GetParameters.Request(names=["geometry_confirmed", "lidar_yaw_rad"]))
            rclpy.spin_until_future_complete(peer, future, timeout_sec=3.)
            assert future.done()
            assert future.result().values[0].bool_value
            assert future.result().values[1].double_value == pytest.approx(3.141592653589793)
            if start_bridge:
                assert states and not states[-1].connected
                assert not states[-1].motion_authorized and not states[-1].allow_fifth_thruster
            else:
                assert "nano_bridge" not in names and not states
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
            peer.destroy_node()
            rclpy.shutdown()
    assert "process has died" not in log_path.read_text()
