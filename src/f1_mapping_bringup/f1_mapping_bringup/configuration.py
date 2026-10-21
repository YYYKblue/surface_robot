"""Validate planar models and measured 3D mounting transforms before launch."""
import math
from pathlib import Path

import yaml


def load_lidar_planar_transform(filename):
    """Return the explicit 2D mounting model, without assuming a physical height."""
    document = yaml.safe_load(Path(filename).read_text(encoding="utf-8"))
    if (not isinstance(document, dict) or document.get("parent_frame") != "base_link"
            or document.get("child_frame") != "pavo2s_frame"):
        raise ValueError("lidar_planar.yaml: expected base_link -> pavo2s_frame")
    values = []
    for key in ("x_m", "y_m", "yaw_rad"):
        value = document.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"lidar_planar.yaml: {key} must be a finite number")
        values.append(value)
    x, y, yaw = values
    # Coplanar coordinates are intentional; physical sensor height is unknown.
    return [x, y, 0.0, 0.0, 0.0, yaw]


def load_transforms(filename):
    document = yaml.safe_load(Path(filename).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("parent_frame") != "base_link":
        raise ValueError("sensors.yaml: parent_frame must be base_link")
    transforms = document.get("transforms")
    if not isinstance(transforms, dict) or set(transforms) != {"pavo2s_frame", "imu_link"}:
        raise ValueError("sensors.yaml: transforms must contain pavo2s_frame and imu_link")
    result = []
    for child, transform in transforms.items():
        if not isinstance(transform, dict):
            raise ValueError(f"sensors.yaml: {child} must contain xyz_m and rpy_rad")
        values = []
        for key in ("xyz_m", "rpy_rad"):
            vector = transform.get(key)
            if not isinstance(vector, list) or len(vector) != 3:
                raise ValueError(f"sensors.yaml: {child}.{key} must have three values")
            for index, value in enumerate(vector):
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(
                        f"sensors.yaml: fill measured {child}.{key}[{index}] "
                        "with a finite number; null means unknown"
                    )
            values.extend(vector)
        result.append((child, values))
    return result


def static_tf_arguments(child, values):
    arguments = []
    for flag, value in zip(("--x", "--y", "--z", "--roll", "--pitch", "--yaw"), values):
        arguments.extend((flag, str(value)))
    return arguments + ["--frame-id", "base_link", "--child-frame-id", child]
