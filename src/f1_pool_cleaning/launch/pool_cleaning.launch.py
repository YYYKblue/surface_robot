from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("f1_pool_cleaning"))
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/pool_cleaning.yaml")),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("map_pose_topic", default_value="/pool_cleaning/map_pose"),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_map_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(package="f1_pool_cleaning", executable="pool_perception", name="pool_perception",
             parameters=[str(share / "config/perception.yaml"), {"use_sim_time": sim}],
             remappings=[("/pavo2_scan", LaunchConfiguration("scan_topic"))]),
        Node(package="f1_pool_cleaning", executable="pool_controller", output="screen",
             parameters=[LaunchConfiguration("config"), {"use_sim_time": sim,
                 "output_enabled": ParameterValue(LaunchConfiguration("output_enabled"), value_type=bool),
                 "use_map_tf": ParameterValue(LaunchConfiguration("use_map_tf"), value_type=bool)}],
             remappings=[("/pool_cleaning/map_pose", LaunchConfiguration("map_pose_topic"))]),
    ])
