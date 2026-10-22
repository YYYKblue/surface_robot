"""Start both UI control nodes with a single fifth-motor setting."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("f1_nano_bridge"))
    flags = ("serial_enabled", "motion_enabled", "direction_confirmed", "allow_fifth_thruster")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config" / "bridge.yaml")),
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        *[DeclareLaunchArgument(flag, default_value="false") for flag in flags],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "bridge.launch.py")),
            launch_arguments={name: LaunchConfiguration(name) for name in ("config", *flags)}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "gateway.launch.py")),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ("bind_address", "allow_fifth_thruster")}.items()),
    ])
