from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("input_timeout_sec", default_value="0.25"),
        Node(package="f1_operator_gateway", executable="f1_operator_gateway",
             name="f1_operator_gateway", output="screen", parameters=[{
                 "bind_address": LaunchConfiguration("bind_address"),
                 "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                 "input_timeout_sec": ParameterValue(LaunchConfiguration("input_timeout_sec"), value_type=float),
             }]),
    ])
