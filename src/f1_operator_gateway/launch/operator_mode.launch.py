"""Owned child mode: any child exit shuts down the entire mode."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if value("mode") == "mapping":
        package, filename = "f1_mapping_bringup", "lidar_mapping.launch.py"
        arguments = {key: value(key) for key in ("scan_topic", "publish_lidar_tf", "resolution")}
    else:
        package, filename = "f1_operator_gateway", "operator_bringup.launch.py"
        arguments = {key: value(key) for key in ("map", "scan_topic", "publish_lidar_tf", "output_enabled")}
        arguments.update(start_bridge="false", start_gateway="false")
        for key in ("rectangle_config", "l_config", "localization_config"):
            if value(key):
                arguments[key] = value(key)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory(package)) / "launch" / filename)), launch_arguments=arguments.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", choices=["mapping", "localization"]),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in dict(
            map="", scan_topic="/pavo2_scan", publish_lidar_tf="true", output_enabled="false",
            resolution="0.05", rectangle_config="", l_config="", localization_config="").items()],
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="owned mode process exited"))])),
        OpaqueFunction(function=nodes),
    ])
