from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("f1_rectangle_cleaning"))
    sim = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/rectangle_preview.yaml")),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("map_pose_topic", default_value="/rectangle_cleaning/map_pose"),
        DeclareLaunchArgument("local_motion_topic", default_value="/rectangle_cleaning/local_motion"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(package="f1_wall_cleaning", executable="wall_perception", name="rectangle_perception",
             parameters=[str(share / "config/perception.yaml"), {"use_sim_time": sim}],
             remappings=[("/pavo2_scan", LaunchConfiguration("scan_topic")),
                         ("/wall_cleaning/wall_observation", "/rectangle_cleaning/wall_observation"),
                         ("/wall_cleaning/wall_marker", "/rectangle_cleaning/wall_marker"),
                         ("/wall_cleaning/reset_observation", "/rectangle_cleaning/reset_observation")]),
        Node(package="f1_rectangle_cleaning", executable="rectangle_controller", output="screen",
             parameters=[LaunchConfiguration("config"), {"use_sim_time": sim}],
             remappings=[("/rectangle_cleaning/map_pose", LaunchConfiguration("map_pose_topic")),
                         ("/rectangle_cleaning/local_motion", LaunchConfiguration("local_motion_topic"))]),
    ])
