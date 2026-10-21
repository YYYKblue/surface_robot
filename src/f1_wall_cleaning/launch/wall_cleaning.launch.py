from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("f1_wall_cleaning")) / "config/wall_cleaning.yaml")
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    output = ParameterValue(LaunchConfiguration("output_enabled"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("output_enabled", default_value="false", choices=["true", "false"]),
        Node(package="f1_wall_cleaning", executable="wall_perception", name="wall_perception",
             output="screen", parameters=[LaunchConfiguration("config"), {"use_sim_time": sim}],
             remappings=[("/pavo2_scan", LaunchConfiguration("scan_topic"))]),
        Node(package="f1_wall_cleaning", executable="wall_controller", name="wall_controller",
             output="screen", parameters=[LaunchConfiguration("config"),
                                          {"use_sim_time": sim, "output_enabled": output}]),
    ])
