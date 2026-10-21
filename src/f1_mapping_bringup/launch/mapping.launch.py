"""PAVO2 + F1 IMU mapping. Drivers and the serial gateway run separately."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from f1_mapping_bringup.configuration import load_transforms


def mapping_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    resolution = float(value("resolution"))
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be a positive finite number in metres")
    config_dir = value("configuration_directory")
    basename = value("configuration_basename")
    if Path(basename).name != basename or not (Path(config_dir) / basename).is_file():
        raise ValueError("configuration_basename must name a Lua file in configuration_directory")
    for topic in ("scan_topic", "imu_topic"):
        if not value(topic).startswith("/") or len(value(topic)) < 2:
            raise ValueError(f"{topic} must be an absolute ROS topic name")

    # Validate missing measurements before starting any process.
    nodes = []
    if value("publish_sensor_tf") == "true":
        load_transforms(value("sensors_file"))
        share = get_package_share_directory("f1_mapping_bringup")
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{share}/launch/sensor_tf.launch.py"),
            launch_arguments={"sensors_file": value("sensors_file"),
                              "use_sim_time": value("use_sim_time")}.items(),
        ))

    # Resolve dependencies only at execution so --show-args also works on a
    # configuration-development PC without Cartographer installed.
    get_package_share_directory("cartographer_ros")
    parameters = [{"use_sim_time": value("use_sim_time") == "true"}]
    nodes.extend([
        Node(package="cartographer_ros", executable="cartographer_node",
             name="cartographer_node", output="screen", parameters=parameters,
             arguments=["-configuration_directory", config_dir,
                        "-configuration_basename", basename],
             remappings=[("scan", value("scan_topic")), ("imu", value("imu_topic"))]),
        Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node",
             name="cartographer_occupancy_grid_node", output="screen", parameters=parameters,
             arguments=["-resolution", str(resolution), "-publish_period_sec", "1.0"]),
    ])
    if value("use_rviz") == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", name="mapping_rviz",
                          output="screen", parameters=parameters,
                          arguments=["-d", value("rviz_config")]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("f1_mapping_bringup")
    arguments = [
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("imu_topic", default_value="/f1/imu/data_raw"),
        DeclareLaunchArgument("configuration_directory", default_value=f"{share}/config"),
        DeclareLaunchArgument("configuration_basename", default_value="pavo2_f1_2d.lua"),
        DeclareLaunchArgument("sensors_file", default_value=f"{share}/config/sensors.yaml"),
        DeclareLaunchArgument("resolution", default_value="0.05", description="Grid cell size in metres"),
        DeclareLaunchArgument("rviz_config", default_value=f"{share}/rviz/mapping.rviz"),
    ]
    for name, default in (("use_sim_time", "false"), ("publish_sensor_tf", "true"),
                          ("use_rviz", "false")):
        arguments.append(DeclareLaunchArgument(name, default_value=default, choices=["true", "false"]))
    return LaunchDescription(arguments + [OpaqueFunction(function=mapping_nodes)])
