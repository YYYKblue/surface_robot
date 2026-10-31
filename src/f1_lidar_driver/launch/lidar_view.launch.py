"""Optional viewer for an already-running driver; never starts another driver."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("f1_lidar_driver"))
    return LaunchDescription([
        Node(package="f1_lidar_driver", executable="pavo2_fov_marker_node",
             name="pavo2_fov_marker_node", output="screen", parameters=[{
                 "frame_id": "pavo2s_frame", "marker_topic": "/pavo2_scan_visualization",
                 "range_m": 100.0, "angle_min_deg": -140.0,
                 "angle_max_deg": 140.0, "arc_step_deg": 1.0,
             }]),
        Node(package="rviz2", executable="rviz2", name="pavo2_rviz", output="screen",
             arguments=["-d", str(share / "rviz/pavo2_scan_only.rviz")]),
    ])
