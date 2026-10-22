"""Validate the lidar's planar mounting model before launch."""
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


def static_tf_arguments(child, values):
    arguments = []
    for flag, value in zip(("--x", "--y", "--z", "--roll", "--pitch", "--yaw"), values):
        arguments.extend((flag, str(value)))
    return arguments + ["--frame-id", "base_link", "--child-frame-id", child]
