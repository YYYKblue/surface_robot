"""Start namespaced shared perception/controller; output remains preview-only by default."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from f1_l_cleaning.topics import REMAPPINGS


def generate_launch_description():
    share = Path(get_package_share_directory("f1_l_cleaning"))
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/l_cleaning.yaml")),
        DeclareLaunchArgument("perception_config", default_value=str(share / "config/perception.yaml")),
        DeclareLaunchArgument("map", default_value="",
                              description="Frozen map: .pbstream or the map .yaml beside it"),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(package="f1_pool_cleaning", executable="pool_perception", name="l_perception",
             output="screen", parameters=[LaunchConfiguration("perception_config"),
                                           {"use_sim_time": sim}],
             remappings=[*REMAPPINGS, ("/pavo2_scan", LaunchConfiguration("scan_topic"))]),
        Node(package="f1_l_cleaning", executable="l_controller", name="l_controller",
             output="screen", parameters=[LaunchConfiguration("config"),
                 {"use_sim_time": sim,
                  "map": LaunchConfiguration("map"),
                  "output_enabled": ParameterValue(LaunchConfiguration("output_enabled"), value_type=bool)}]),
    ])
