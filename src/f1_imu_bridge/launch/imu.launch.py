from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("f1_imu_bridge")
    return LaunchDescription([
        DeclareLaunchArgument("parameters_file", default_value=f"{share}/config/imu.yaml"),
        Node(package="f1_imu_bridge", executable="imu_bridge", name="f1_imu_bridge",
             output="screen", parameters=[LaunchConfiguration("parameters_file")]),
    ])
