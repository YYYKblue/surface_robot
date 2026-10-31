"""Standalone PAVO2 driver; produces scans without SLAM or mounting TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("f1_lidar_driver"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/pavo2.yaml"),
                              description="PAVO2 ROS parameter YAML; settings apply on restart"),
        Node(package="f1_lidar_driver", executable="pavo2_scan_node", name="pavo2_scan_node",
             output="screen", parameters=[LaunchConfiguration("config")]),
    ])
