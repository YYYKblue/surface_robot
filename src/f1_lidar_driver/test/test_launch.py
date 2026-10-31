"""Installed entry points and config overrides; no physical device is contacted."""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def environment(tmp_path):
    return dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="178",
                RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
                CYCLONEDDS_URI=(ROOT / "test/cyclonedds_localhost.xml").as_uri(),
                ROS_LOG_DIR=str(tmp_path / "ros_logs"))


def test_installed_config_and_executables_preserve_deployed_parameters(tmp_path):
    share = Path(get_package_share_directory("f1_lidar_driver"))
    values = yaml.safe_load((share / "config/pavo2.yaml").read_text())["pavo2_scan_node"]["ros__parameters"]
    # These are the deployed Nano launch's parameters, not node default values.
    assert values == dict(lidar_ip="10.10.10.121", lidar_port=2368, local_ip="10.10.10.100",
        local_port=0, conn_protocol=0, frame_id="pavo2s_frame", topic="pavo2_scan",
        range_min=.10, range_max=100., inverted=False, timeout=180)
    prefix = Path(get_package_prefix("f1_lidar_driver"))
    for executable in ("pavo2_scan_node", "pavo2_fov_marker_node"):
        assert os.access(prefix / "lib/f1_lidar_driver" / executable, os.X_OK)
    result = subprocess.run(["ros2", "launch", "f1_lidar_driver", "lidar.launch.py", "--show-args"],
                            env=environment(tmp_path), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "'config'" in result.stdout and "pavo2.yaml" in result.stdout


@pytest.mark.parametrize("service_entry", [False, True])
def test_launch_reads_custom_yaml_and_retains_initial_connection_failure_exit(tmp_path, service_entry):
    config = tmp_path / "invalid-address.yaml"
    # Invalid local address is rejected before any radar socket is opened.
    config.write_text(yaml.safe_dump({"pavo2_scan_node": {"ros__parameters": {
        "local_ip": "invalid-test-address", "lidar_ip": "127.0.0.1"}}}))
    env = environment(tmp_path)
    command = ["ros2", "launch", "f1_lidar_driver", "lidar.launch.py", f"config:={config}"]
    if service_entry:
        # Run the actual installed service wrapper, including its ROS overlay setup.
        share = Path(get_package_share_directory("f1_lidar_driver"))
        env.update(F1_LIDAR_WORKSPACE=str(ROOT.parents[1]), F1_LIDAR_CONFIG=str(config))
        command = ["bash", str(share / "deploy/start-lidar.sh")]
    result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=10)
    output = result.stdout + result.stderr
    assert "Invalid local_ip [invalid-test-address]" in output, output
    assert "exit code 1" in output, output
    assert "Connecting to Pavo2" not in output


def test_no_device_startup_fails_cleanly_using_loopback_only(tmp_path):
    import socket
    binary = Path(get_package_prefix("f1_lidar_driver")) / "lib/f1_lidar_driver/pavo2_scan_node"
    # Reserve a loopback TCP port without listening, to guarantee no device endpoint.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as closed_endpoint:
        closed_endpoint.bind(("127.0.0.1", 0))
        port = closed_endpoint.getsockname()[1]
        result = subprocess.run([str(binary), "--ros-args", "-p", "local_ip:=127.0.0.1",
            "-p", "lidar_ip:=127.0.0.1", "-p", f"lidar_port:={port}", "-p", "timeout:=50"],
            env=environment(tmp_path), capture_output=True, text=True, timeout=10)
    assert result.returncode == 1, result.stdout + result.stderr
    assert f"Could not connect to Pavo2 at 127.0.0.1:{port}" in result.stdout + result.stderr
