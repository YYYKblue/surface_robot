"""L-task system startup; starts one localization owner and one hardware bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cleaning = Path(get_package_share_directory("f1_l_cleaning"))
    pool = Path(get_package_share_directory("f1_pool_cleaning"))
    bridge = Path(get_package_share_directory("f1_nano_bridge"))
    mapping = Path(get_package_share_directory("f1_mapping_bringup"))

    def include(directory, filename, args, condition=None):
        launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / "launch" / filename)),
            launch_arguments=args.items())
        return GroupAction(actions=[launch], condition=condition)

    arg = LaunchConfiguration
    # One map input: localization state, the open A->B->C route and the pose
    # gate all come from this file.
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="",
                              description="Frozen map: .pbstream or the map .yaml beside it"),
        DeclareLaunchArgument("start_localization", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_bridge", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_cleaning", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_policy", default_value="nearest_reachable", choices=["nearest_reachable", "specified"]),
        DeclareLaunchArgument("default_edge_id", default_value="edge_0"),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("config", default_value=str(cleaning / "config/l_cleaning.yaml")),
        DeclareLaunchArgument("perception_config", default_value=str(cleaning / "config/perception.yaml")),
        DeclareLaunchArgument("reference_trajectory_id", default_value="0"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("bridge_config", default_value=str(bridge / "config/bridge.yaml")),
        DeclareLaunchArgument("lidar_extrinsics_file", default_value=str(mapping / "config/lidar_planar.yaml")),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        include(pool, "pool_localization.launch.py", {
            "map": arg("map"),
            "start_trajectory": "false",
            "reference_trajectory_id": arg("reference_trajectory_id"),
            "scan_topic": arg("scan_topic"),
            "lidar_extrinsics_file": arg("lidar_extrinsics_file"),
            "publish_lidar_tf": arg("publish_lidar_tf"),
            "start_rviz": arg("start_rviz"),
            "rviz_config": str(pool / "config/pool_localization.rviz"),
            "cleaning_prefix": "l_cleaning",
        }, IfCondition(arg("start_localization"))),
        include(bridge, "bridge.launch.py", {
            "config": arg("bridge_config"),
            **{name: arg("output_enabled") for name in
               ("serial_enabled", "motion_enabled", "direction_confirmed", "allow_fifth_thruster")},
        }, IfCondition(arg("start_bridge"))),
        include(cleaning, "l_cleaning.launch.py", {
            "config": arg("config"),
            "perception_config": arg("perception_config"),
            "map": arg("map"),
            "scan_topic": arg("scan_topic"),
            "start_policy": arg("start_policy"),
            "default_edge_id": arg("default_edge_id"),
            "output_enabled": arg("output_enabled"),
        }, IfCondition(arg("start_cleaning"))),
        LogInfo(msg="L task is idle; after localization is ready, start with: "
                    "ros2 service call /l_cleaning/start std_srvs/srv/Trigger '{}'"),
    ])
