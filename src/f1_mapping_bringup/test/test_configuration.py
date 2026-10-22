from pathlib import Path

import pytest
import yaml

from f1_mapping_bringup.configuration import load_lidar_planar_transform, static_tf_arguments


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


def test_planar_mounting_uses_named_euler_arguments():
    values = load_lidar_planar_transform(PACKAGE / "config/lidar_planar.yaml")
    args = static_tf_arguments("pavo2s_frame", values)
    options = dict(zip(args[::2], args[1::2]))
    assert float(options["--x"]) == pytest.approx(-0.2143195)
    assert float(options["--y"]) == 0.0
    assert float(options["--z"]) == 0.0
    assert float(options["--roll"]) == 0.0
    assert float(options["--pitch"]) == 0.0
    assert float(options["--yaw"]) == pytest.approx(3.141592653589793)
    assert options["--frame-id"] == "base_link"
    assert options["--child-frame-id"] == "pavo2s_frame"
