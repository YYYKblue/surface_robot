from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default = str(Path(get_package_share_directory("f1_nano_bridge")) / "config" / "bridge.yaml")
    flags = ("serial_enabled", "motion_enabled", "direction_confirmed")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        *[DeclareLaunchArgument(flag, default_value="false") for flag in flags],
        Node(package="f1_nano_bridge", executable="f1_bridge", name="nano_bridge", output="screen",
             parameters=[LaunchConfiguration("config"),
                         {flag: ParameterValue(LaunchConfiguration(flag), value_type=bool)
                          for flag in flags}]),
    ])
