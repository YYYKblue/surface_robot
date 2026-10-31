"""Combined localization, bridge and cleaning startup; task start stays manual."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("f1_pool_cleaning"))
    localization = Path(get_package_share_directory("f1_localization"))
    bridge = Path(get_package_share_directory("f1_nano_bridge"))
    mapping = Path(get_package_share_directory("f1_mapping_bringup"))

    def include(package_share, filename, arguments, condition=None):
        launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch" / filename)),
            launch_arguments=arguments.items())
        # Each child declares "config"; keep its launch arguments local.
        return GroupAction(actions=[launch], condition=condition)

    def config(name):
        return LaunchConfiguration(name)

    return LaunchDescription([
        # One map input drives everything: the frozen localization state, the
        # closed-pool rectangle, the boundary route and the localization pose
        # gate.  Point this at the new map and the numbers follow.
        DeclareLaunchArgument("map", default_value="",
                              description="Frozen map: .pbstream or the map .yaml beside it"),
        DeclareLaunchArgument("start_localization", default_value="true", choices=["true", "false"],
                              description="Set false when localization is already running"),
        DeclareLaunchArgument("start_bridge", default_value="true", choices=["true", "false"],
                              description="Set false when the bridge is already running"),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"],
                              description="Enable bridge hardware permissions and cleaning output"),
        DeclareLaunchArgument("edge_task_enabled", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_policy", default_value="left_ray", choices=["left_ray", "specified", "nearest_reachable"]),
        DeclareLaunchArgument("default_edge_id", default_value="edge_0"),
        DeclareLaunchArgument("config", default_value=str(share / "config/pool_cleaning.yaml")),
        DeclareLaunchArgument("bridge_config", default_value=str(bridge / "config/bridge.yaml")),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value=str(localization / "config/localization.rviz")),
        DeclareLaunchArgument("localization_config", default_value=""),
        DeclareLaunchArgument("reference_trajectory_id", default_value="0"),
        DeclareLaunchArgument("lidar_extrinsics_file", default_value=str(mapping / "config/lidar_planar.yaml")),
        include(localization, "localization.launch.py", {
            "map": config("map"),
            "localization_config": config("localization_config"),
            "start_trajectory": "false",
            "reference_trajectory_id": config("reference_trajectory_id"),
            "scan_topic": config("scan_topic"),
            "publish_lidar_tf": config("publish_lidar_tf"),
            "lidar_extrinsics_file": config("lidar_extrinsics_file"),
            "start_rviz": config("start_rviz"),
            "rviz_config": config("rviz_config"),
        }, IfCondition(config("start_localization"))),
        include(bridge, "bridge.launch.py", {
            "config": config("bridge_config"),
            **{name: config("output_enabled") for name in (
                "serial_enabled", "motion_enabled", "direction_confirmed", "allow_fifth_thruster")},
        }, IfCondition(config("start_bridge"))),
        include(share, "pool_cleaning.launch.py", {
            "config": config("config"),
            "map": config("map"),
            "scan_topic": config("scan_topic"),
            "output_enabled": config("output_enabled"),
            "edge_task_enabled": config("edge_task_enabled"),
            "start_policy": config("start_policy"),
            "default_edge_id": config("default_edge_id"),
            "use_map_tf": "true",
            "require_localization_ready": "true",
        }),
        LogInfo(msg="Task remains idle. Start with: ros2 service call "
                    "/pool_cleaning/start std_srvs/srv/Trigger '{}'"),
    ])
