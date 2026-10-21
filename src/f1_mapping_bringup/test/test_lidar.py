import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from f1_mapping_bringup.scan_quality import ScanQuality


PACKAGE = Path(__file__).resolve().parents[1]
STAMP = 1_800_000_000_000_000_000


def scan(ranges=(1.0, 2.0, 3.0), stamp=STAMP, **fields):
    seconds, nanoseconds = divmod(stamp, 10**9)
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="pavo2s_frame",
                               stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)),
        ranges=list(ranges), angle_min=-math.pi / 2, angle_max=math.pi / 2,
        angle_increment=math.pi / 2, range_min=0.1, range_max=100.0,
        time_increment=0.001, scan_time=0.02,
    )
    for name, value in fields.items():
        setattr(message, name, value)
    return message


def test_valid_scan_counts_geometry_rate_and_directions():
    quality = ScanQuality()
    quality.add(scan(), 10.0, STAMP + 10_000_000)
    quality.add(scan(stamp=STAMP + 20_000_000), 10.02, STAMP + 30_000_000)
    report = quality.report()
    assert report["issues"] == []
    assert report["arrival_hz"] == pytest.approx(50)
    assert report["valid_fraction"] == 1.0
    assert report["nearest_valid_m"] == 1.0
    assert report["sector_medians_m"] == {
        "right_minus_90_deg": 1.0, "forward_0_deg": 2.0, "left_plus_90_deg": 3.0}


def test_invalid_returns_are_distinguished_without_arbitrary_validity_threshold():
    quality = ScanQuality()
    quality.add(scan([1.0, float("nan"), float("inf"), 0.0, 101.0],
                     angle_increment=math.pi / 4), 10.0, STAMP)
    report = quality.report()
    assert report["valid_fraction"] == 0.2
    assert report["nan_points"] == 1
    assert report["infinite_points"] == 1
    assert report["out_of_range_points"] == 2
    assert report["issues"] == []
    json.dumps(report, allow_nan=False)


def test_all_missing_returns_are_reported():
    quality = ScanQuality()
    quality.add(scan([float("inf")] * 3), 10.0, STAMP)
    assert "no finite in-range returns" in quality.report()["issues"]


def test_observed_pavo_partial_scan_metadata_is_consistent():
    quality = ScanQuality()
    message = scan([1.0] * 3500, angle_min=-2.4434609413146973,
                   angle_max=2.4420647621154785, angle_increment=0.0013962633674964309,
                   time_increment=4.4444441300584e-06, scan_time=0.019999999552965164)
    quality.add(message, 10.0, STAMP)
    report = quality.report()
    assert report["issues"] == []
    assert report["geometry"]["angular_span_deg"] == pytest.approx(280, abs=0.1)
    assert report["geometry"]["ray_span_sec"] < report["geometry"]["scan_time_sec"]


@pytest.mark.parametrize("fields", [
    {"angle_increment": 0.0}, {"angle_min": float("nan")},
    {"range_max": -1.0}, {"angle_max": 10.0}, {"time_increment": -0.1},
])
def test_invalid_geometry_is_reported(fields):
    quality = ScanQuality()
    quality.add(scan(**fields), 10.0, STAMP)
    assert "invalid scan geometry metadata" in quality.report()["issues"]
    json.dumps(quality.report(), allow_nan=False)


def test_repeated_backwards_stale_and_wrong_frame_are_reported():
    quality = ScanQuality()
    quality.add(scan(), 10.0, STAMP)
    quality.add(scan(), 10.02, STAMP)
    wrong = scan(stamp=STAMP - 1)
    wrong.header.frame_id = "wrong_frame"
    quality.add(wrong, 10.04, STAMP + 2_000_000_000)
    report = quality.report()
    assert report["backward_stamps"] == 1
    assert report["repeated_stamps"] == 1
    assert "unexpected sensor frame" in report["issues"]
    assert "scan timestamp age exceeds 1 second" in report["issues"]


def test_no_input_does_not_pass():
    assert "no scans received" in ScanQuality().report()["issues"]


def launch_module():
    spec = importlib.util.spec_from_file_location("lidar_mapping", PACKAGE / "launch/lidar_mapping.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("grid_name", ["cartographer_occupancy_grid_node", "occupancy_grid_node"])
def test_lidar_launch_publishes_planar_mounting_without_imu(tmp_path, monkeypatch, grid_name):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    from launch import LaunchContext

    directory = tmp_path / "lib/cartographer_ros"
    directory.mkdir(parents=True)
    for name in ("cartographer_node", grid_name):
        path = directory / name
        path.write_text("#!/bin/sh\nexit 0\n")
        path.chmod(0o755)
    module = launch_module()
    monkeypatch.setattr(module, "get_package_prefix", lambda _: str(tmp_path))
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(
        resolution="0.05", scan_topic="/pavo2_scan", configuration_directory=str(PACKAGE / "config"),
        configuration_basename="pavo2_2d.lua", use_sim_time="false", use_rviz="false",
        publish_lidar_tf="true", lidar_extrinsics_file=str(PACKAGE / "config/lidar_planar.yaml"),
    )
    nodes = module.lidar_nodes(context)
    assert len(nodes) == 3
    mounting, slam, grid = nodes
    assert mounting["executable"] == "static_transform_publisher"
    options = dict(zip(mounting["arguments"][::2], mounting["arguments"][1::2]))
    assert options["--frame-id"] == "base_link"
    assert options["--child-frame-id"] == "pavo2s_frame"
    assert float(options["--x"]) == pytest.approx(-0.2143195)
    assert float(options["--yaw"]) == pytest.approx(math.pi)
    assert slam["remappings"] == [("scan", "/pavo2_scan")]
    assert grid["executable"] == grid_name
    # Existing TF publishers / recorded bags must not get a duplicate parent edge.
    context.launch_configurations.update(publish_lidar_tf="false", lidar_extrinsics_file="/missing.yaml")
    nodes = module.lidar_nodes(context)
    assert len(nodes) == 2
    assert all(node["package"] == "cartographer_ros" for node in nodes)


def test_missing_cartographer_executable_gives_actionable_error(tmp_path):
    with pytest.raises(RuntimeError, match="ros2 pkg executables"):
        launch_module().executable(tmp_path, ("cartographer_node",))
