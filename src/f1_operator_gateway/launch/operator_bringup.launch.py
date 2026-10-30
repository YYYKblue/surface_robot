"""One map, one localization owner, one bridge, and all valid task types."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    from f1_pool_cleaning.map_profile import profile
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def include(package, filename, arguments):
        path = Path(get_package_share_directory(package)) / "launch" / filename
        return GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())])
    derived = profile(value("map"))
    unavailable = {}
    result = []
    if value("start_localization") == "true":
        result.append(include("f1_localization", "localization.launch.py", {
            "map": value("map"), "cleaning_prefix": "f1_localization",
            "scan_topic": value("scan_topic"), "start_rviz": value("start_rviz"),
            "localization_config": value("localization_config"),
            "publish_lidar_tf": value("publish_lidar_tf"),
        }))
    if value("start_bridge") == "true":
        result.append(include("f1_nano_bridge", "bridge.launch.py", {
            "config": value("bridge_config"),
            **{key: value("output_enabled") for key in
               ("serial_enabled", "motion_enabled", "direction_confirmed", "allow_fifth_thruster")},
        }))
    for kind, geometry, package, launch, config in (
        ("rectangle", derived.rectangle, "f1_pool_cleaning", "pool_cleaning.launch.py", "rectangle_config"),
        ("l", derived.route, "f1_l_cleaning", "l_cleaning.launch.py", "l_config"),
    ):
        if geometry is None:
            unavailable[kind] = derived.errors.get("route" if kind == "l" else "rectangle", "地图不支持该任务")
            continue
        result.append(include(package, launch, {
            "map": value("map"), "config": value(config),
            "scan_topic": value("scan_topic"), "output_enabled": value("output_enabled"),
            "localization_status_topic": "/f1_localization/localization_status",
        }))
    result.append(Node(package="f1_operator_gateway", executable="f1_operator_gateway", output="screen",
        parameters=[dict(bind_address=value("bind_address"), port=int(value("port")),
                         localization_prefix="/f1_localization",
                         task_unavailable_reasons=ParameterValue(json.dumps(unavailable), value_type=str))]))
    return result


def generate_launch_description():
    def config(package, name):
        return str(Path(get_package_share_directory(package)) / "config" / name)
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Existing .pbstream or matching map YAML"),
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_bridge", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_localization", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("localization_config", default_value=""),
        DeclareLaunchArgument("rectangle_config", default_value=config("f1_pool_cleaning", "pool_cleaning.yaml")),
        DeclareLaunchArgument("l_config", default_value=config("f1_l_cleaning", "l_cleaning.yaml")),
        DeclareLaunchArgument("bridge_config", default_value=config("f1_nano_bridge", "bridge.yaml")),
        OpaqueFunction(function=nodes),
    ])
