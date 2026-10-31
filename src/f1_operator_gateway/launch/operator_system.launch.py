"""Map-free app entry point. Keeps bridge/gateway alive across mode changes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge = Path(get_package_share_directory("f1_nano_bridge"))
    parameters = {name: LaunchConfiguration(name) for name in (
        "workspace_root", "bind_address", "scan_topic", "map_timezone",
        "rectangle_config", "l_config", "localization_config")}
    parameters.update(managed_runtime=True, localization_prefix="/f1_localization")
    for name, kind in (("port", int), ("output_enabled", bool), ("publish_lidar_tf", bool), ("mapping_resolution", float)):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=kind)
    gateway = Node(package="f1_operator_gateway", executable="f1_operator_gateway", output="screen", parameters=[parameters])
    return LaunchDescription([
        DeclareLaunchArgument("workspace_root", description="Absolute workspace root; maps are stored under maps/"),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in dict(
            bind_address="127.0.0.1", port="8765", scan_topic="/pavo2_scan", map_timezone="Asia/Shanghai",
            start_bridge="true", output_enabled="false", publish_lidar_tf="true", mapping_resolution="0.05",
            bridge_config=str(bridge / "config/bridge.yaml"), rectangle_config="", l_config="", localization_config="").items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge / "launch/bridge.launch.py")),
            condition=IfCondition(LaunchConfiguration("start_bridge")), launch_arguments={
                "config": LaunchConfiguration("bridge_config"),
                **{key: LaunchConfiguration("output_enabled") for key in
                   ("serial_enabled", "motion_enabled", "direction_confirmed", "allow_fifth_thruster")},
            }.items()),
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="operator system process exited"))])),
        gateway,
    ])
