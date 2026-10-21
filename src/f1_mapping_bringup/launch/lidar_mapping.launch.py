"""Lidar-only SLAM with a planar vehicle-centre pose and lidar mounting TF."""
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from f1_mapping_bringup.configuration import load_lidar_planar_transform, static_tf_arguments


def executable(directory, candidates):
    for name in candidates:
        if (directory / name).is_file() and os.access(directory / name, os.X_OK):
            return name
    raise RuntimeError(f"Cartographer executables missing: {candidates}; check ros2 pkg executables cartographer_ros")


def lidar_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    resolution = float(value("resolution"))
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be a positive finite number in metres")
    if not value("scan_topic").startswith("/") or len(value("scan_topic")) < 2:
        raise ValueError("scan_topic must be an absolute ROS topic name")
    config_dir, basename = value("configuration_directory"), value("configuration_basename")
    if Path(basename).name != basename or not (Path(config_dir) / basename).is_file():
        raise ValueError("configuration_basename must name a Lua file in configuration_directory")
    mounting = None
    if value("publish_lidar_tf") == "true":
        mounting = load_lidar_planar_transform(value("lidar_extrinsics_file"))
    directory = Path(get_package_prefix("cartographer_ros")) / "lib" / "cartographer_ros"
    slam = executable(directory, ("cartographer_node",))
    grid = executable(directory, ("cartographer_occupancy_grid_node", "occupancy_grid_node"))
    parameters = [{"use_sim_time": value("use_sim_time") == "true"}]
    nodes = []
    if mounting is not None:
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_lidar_planar", output="screen", parameters=parameters,
            arguments=static_tf_arguments("pavo2s_frame", mounting),
        ))
    nodes.extend([
        Node(package="cartographer_ros", executable=slam, name="cartographer_node",
             output="screen", parameters=parameters,
             arguments=["-configuration_directory", config_dir, "-configuration_basename", basename],
             remappings=[("scan", value("scan_topic"))]),
        Node(package="cartographer_ros", executable=grid, name="cartographer_occupancy_grid_node",
             output="screen", parameters=parameters,
             arguments=["-resolution", str(resolution), "-publish_period_sec", "1.0"]),
    ])
    if value("use_rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", name="mapping_rviz",
                          output="screen", parameters=parameters,
                          arguments=["-d", value("rviz_config")]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("f1_mapping_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("configuration_directory", default_value=f"{share}/config"),
        DeclareLaunchArgument("configuration_basename", default_value="pavo2_2d.lua"),
        DeclareLaunchArgument("lidar_extrinsics_file", default_value=f"{share}/config/lidar_planar.yaml"),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"],
                             description="Publish planar base_link -> lidar TF; disable if already provided"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("rviz_config", default_value=f"{share}/rviz/mapping.rviz"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=lidar_nodes),
    ])
