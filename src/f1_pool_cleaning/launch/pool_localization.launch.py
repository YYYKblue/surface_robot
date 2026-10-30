"""Compatibility entry point; localization is owned by f1_localization.

The included launch exposes its arguments and inherits the launch context,
preserving map, cleaning_prefix and all other existing arguments.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory("f1_localization"))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / "launch/localization.launch.py"))),
    ])
