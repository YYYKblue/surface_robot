from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def bridge_node(context):
    flags = ("serial_enabled", "motion_enabled", "direction_confirmed")
    overrides = {flag: ParameterValue(LaunchConfiguration(flag), value_type=bool) for flag in flags}
    # An omitted fifth-motor argument preserves existing custom YAML settings.
    fifth = LaunchConfiguration("allow_fifth_thruster").perform(context)
    if fifth:
        overrides["allow_fifth_thruster"] = ParameterValue(
            LaunchConfiguration("allow_fifth_thruster"), value_type=bool)
    return [Node(package="f1_nano_bridge", executable="f1_bridge", name="nano_bridge", output="screen",
                 parameters=[LaunchConfiguration("config"), overrides])]


def generate_launch_description():
    default = str(Path(get_package_share_directory("f1_nano_bridge")) / "config" / "bridge.yaml")
    flags = ("serial_enabled", "motion_enabled", "direction_confirmed")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        *[DeclareLaunchArgument(flag, default_value="false") for flag in flags],
        DeclareLaunchArgument("allow_fifth_thruster", default_value="",
                              description="true/false to override YAML; empty preserves YAML"),
        OpaqueFunction(function=bridge_node),
    ])
