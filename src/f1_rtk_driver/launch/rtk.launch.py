from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_driver(context):
    parameters = [LaunchConfiguration("config")]
    port = LaunchConfiguration("port").perform(context)
    if port:
        parameters.append({"port": ParameterValue(port, value_type=str)})
    return [Node(package="f1_rtk_driver", executable="rtk_driver",
                 parameters=parameters, output="screen")]


def generate_launch_description():
    config = Path(get_package_share_directory("f1_rtk_driver")) / "config" / "rtk.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config)),
        DeclareLaunchArgument("port", default_value="", description="Override the serial port in YAML"),
        OpaqueFunction(function=start_driver),
    ])
