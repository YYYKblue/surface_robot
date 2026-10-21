"""Publish measured sensor mounting transforms independently of SLAM."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from f1_mapping_bringup.configuration import load_transforms, static_tf_arguments


def sensor_nodes(context):
    transforms = load_transforms(LaunchConfiguration("sensors_file").perform(context))
    return [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"base_to_{child}", output="screen",
            arguments=static_tf_arguments(child, values),
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
        for child, values in transforms
    ]


def generate_launch_description():
    share = get_package_share_directory("f1_mapping_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("sensors_file", default_value=f"{share}/config/sensors.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=sensor_nodes),
    ])
