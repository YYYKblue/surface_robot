"""Installed combined launch: one bridge and shared localization subscription."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from f1_pool_cleaning.synthetic_map import l_map, rectangle_map


@pytest.mark.parametrize("kind,make_map,expected_edges", [("rectangle", rectangle_map, 4), ("l", l_map, 2)])
def test_operator_bringup_with_shared_external_localization(tmp_path, monkeypatch, kind, make_map, expected_edges):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "176")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    map_path = make_map(tmp_path)
    rclpy.init()
    peer = Node("operator_bringup_test")
    statuses = []
    prefix = "/pool_cleaning" if kind == "rectangle" else "/l_cleaning"
    peer.create_subscription(String, prefix+"/status", lambda msg: statuses.append(json.loads(msg.data)), 5)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    loc = peer.create_publisher(String, "/f1_localization/localization_status", qos)
    log_path = tmp_path / "launch.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(["ros2", "launch", "f1_operator_gateway", "operator_bringup.launch.py",
            f"map:={map_path}", "start_localization:=false", "port:=0"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                loc.publish(String(data=json.dumps(dict(state="LOCALIZATION_READY", ready=True,
                    reason="synthetic stable", stamp_ns=peer.get_clock().now().nanoseconds,
                    map_id=str(Path(map_path).resolve().with_suffix(""))))))
                rclpy.spin_once(peer, timeout_sec=.05)
                assert process.poll() is None, log_path.read_text()
                names = peer.get_node_names()
                if statuses and statuses[-1]["localization_ready"] and "f1_operator_gateway" in names:
                    break
            assert statuses, (f"nodes={peer.get_node_names()}, publishers={peer.count_publishers(prefix + '/status')}, loc_subscribers={loc.get_subscription_count()}\n" + log_path.read_text())
            status = statuses[-1]
            assert status["localization_ready"], log_path.read_text()
            assert status["task_type"] == kind and status["edge_count"] == expected_edges
            assert status["phase"] == "IDLE" and not status["output_enabled"]
            assert not status["can_start"]  # No pose/scan feedback in this fixture.
            names = peer.get_node_names()
            assert names.count("nano_bridge") == 1
            assert names.count("f1_operator_gateway") == 1
            assert "pool_localization" not in names and "pool_localization_manager" not in names
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            peer.destroy_node()
            rclpy.shutdown()
    assert "process has died" not in log_path.read_text()
