"""Exercise combined launch on localhost, with serial and motion disabled."""
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


@pytest.mark.parametrize("start_bridge", [True, False])
def test_combined_preview_and_reuse_existing_nodes(monkeypatch, tmp_path, start_bridge):
    monkeypatch.setenv("ROS_DOMAIN_ID", "179")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    # A distinct target detects launch argument leakage between child launches.
    config = tmp_path / "cleaning.yaml"
    config.write_text("pool_controller:\n  ros__parameters:\n    wall_target_m: 0.345\n    side_off_distance_m: 0.325\n    side_on_distance_m: 0.365\n")
    rclpy.init()
    peer = Node("combined_launch_test")
    statuses, states = [], []
    status_sub = peer.create_subscription(
        String, "/pool_cleaning/status", lambda m: statuses.append(json.loads(m.data)), 10)
    state_sub = peer.create_subscription(ControlState, "/f1/control_state", states.append, 10)
    log_path = tmp_path / "launch.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([
            "ros2", "launch", "f1_pool_cleaning", "pool_bringup.launch.py",
            "start_localization:=false", f"start_bridge:={str(start_bridge).lower()}",
            "load_state_filename:=/missing/map.pbstream", f"config:={config}",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            until = time.monotonic() + 15.
            while time.monotonic() < until:
                rclpy.spin_once(peer, timeout_sec=.05)
                if statuses and (states or not start_bridge):
                    break
                assert process.poll() is None, log_path.read_text()
            assert statuses, log_path.read_text()
            status = statuses[-1]
            assert status["wall_target_m"] == pytest.approx(.345)
            assert status["phase"] == "IDLE"
            assert status["pose_source"] == "tf"
            assert not status["output_enabled"] and not status["lease_held"]
            assert not status["brush_enabled"]
            names = peer.get_node_names()
            assert "pool_localization" not in names
            assert names.count("pool_controller") == 1
            if start_bridge:
                assert states, log_path.read_text()
                assert not states[-1].connected
                assert not states[-1].motion_authorized
                assert not states[-1].allow_fifth_thruster
            else:
                assert "nano_bridge" not in names
                assert not states
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
