import importlib.util
from pathlib import Path

import pytest
import yaml

from f1_mapping_bringup.configuration import load_lidar_planar_transform, load_transforms, static_tf_arguments


PACKAGE = Path(__file__).resolve().parents[1]


def test_planar_mounting_maps_lidar_forward_to_vehicle_rear():
    import math

    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer
    from rclpy.time import Time

    values = load_lidar_planar_transform(PACKAGE / "config/lidar_planar.yaml")
    x, y, z, roll, pitch, yaw = values
    assert (z, roll, pitch) == (0.0, 0.0, 0.0)
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "pavo2s_frame"
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2)
    transform.transform.rotation.w = math.cos(yaw / 2)
    buffer = Buffer()
    buffer.set_transform_static(transform, "test")
    # Check the actual TF direction and inverse, including the 180 degree yaw.
    forward = buffer.lookup_transform("base_link", "pavo2s_frame", Time()).transform
    assert forward.translation.x == pytest.approx(-0.2143195)
    assert forward.rotation.z == pytest.approx(1.0)
    inverse = buffer.lookup_transform("pavo2s_frame", "base_link", Time()).transform
    assert inverse.translation.x == pytest.approx(-0.2143195)
    assert inverse.translation.y == pytest.approx(0.0, abs=1e-10)


@pytest.mark.parametrize("key,bad", [("x_m", None), ("y_m", True), ("yaw_rad", float("nan")),
                                     ("yaw_rad", float("inf")), ("parent_frame", "odom"),
                                     ("child_frame", "base_link")])
def test_invalid_planar_mounting_is_rejected(tmp_path, key, bad):
    document = yaml.safe_load((PACKAGE / "config/lidar_planar.yaml").read_text())
    document[key] = bad
    path = tmp_path / "planar.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_lidar_planar_transform(path)


def measured_file(tmp_path):
    document = yaml.safe_load((PACKAGE / "config/sensors.yaml").read_text())
    document["transforms"]["pavo2s_frame"]["xyz_m"][2] = 0.12
    document["transforms"]["imu_link"] = {"xyz_m": [0.02, -0.01, 0.03],
                                           "rpy_rad": [0.1, 0.2, 0.3]}
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(document))
    return path, document


def test_unknown_mounting_cannot_silently_become_identity():
    with pytest.raises(ValueError, match="fill measured"):
        load_transforms(PACKAGE / "config/sensors.yaml")


def test_measured_mounting_uses_named_euler_arguments(tmp_path):
    path, _ = measured_file(tmp_path)
    transforms = dict(load_transforms(path))
    args = static_tf_arguments("imu_link", transforms["imu_link"])
    options = dict(zip(args[::2], args[1::2]))
    assert options["--x"] == "0.02"
    assert options["--roll"] == "0.1"
    assert options["--pitch"] == "0.2"
    assert options["--yaw"] == "0.3"
    assert options["--frame-id"] == "base_link"
    assert options["--child-frame-id"] == "imu_link"


@pytest.mark.parametrize("bad", [None, True, "0.1", float("nan"), float("inf")])
def test_invalid_mounting_is_rejected(tmp_path, bad):
    path, document = measured_file(tmp_path)
    document["transforms"]["imu_link"]["rpy_rad"][0] = bad
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="finite number"):
        load_transforms(path)


def test_frame_mismatch_is_rejected(tmp_path):
    path, document = measured_file(tmp_path)
    document["transforms"]["imu"] = document["transforms"].pop("imu_link")
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="imu_link"):
        load_transforms(path)


def test_tf_launch_constructs_both_sensor_nodes(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    from launch import LaunchContext

    path, _ = measured_file(tmp_path)
    spec = importlib.util.spec_from_file_location("sensor_tf", PACKAGE / "launch/sensor_tf.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(sensors_file=str(path), use_sim_time="false")
    assert len(module.sensor_nodes(context)) == 2
