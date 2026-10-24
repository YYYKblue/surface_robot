"""Load a frozen pbstream, optionally starting a localization trajectory."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from f1_mapping_bringup.configuration import load_lidar_planar_transform, static_tf_arguments


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    state = Path(value("load_state_filename"))
    if not state.is_file() or state.suffix != ".pbstream":
        raise ValueError("load_state_filename must name the existing reference-map .pbstream")
    share = Path(get_package_share_directory("f1_pool_cleaning"))
    parameters = [{"use_sim_time": value("use_sim_time") == "true"}]
    result = []
    if value("publish_lidar_tf") == "true":
        mounting = load_lidar_planar_transform(value("lidar_extrinsics_file"))
        result.append(Node(package="tf2_ros", executable="static_transform_publisher",
            name="base_to_lidar_planar", parameters=parameters,
            arguments=static_tf_arguments("pavo2s_frame", mounting)))
    result.append(Node(package="cartographer_ros", executable="cartographer_node",
        name="pool_localization", output="screen", parameters=parameters,
        arguments=["-configuration_directory", str(share / "config"),
                   "-configuration_basename", "pool_localization.lua",
                   "-load_state_filename", str(state), "-load_frozen_state=true",
                   "-start_trajectory_with_default_topics=" + value("start_trajectory")],
        remappings=[("scan", value("scan_topic"))]))
    # The manager starts a trajectory only after a valid RViz initial pose. It
    # also publishes the readiness gate consumed by the cleaning controller.
    result.append(Node(package="f1_pool_cleaning", executable="pool_localization",
        name="pool_localization_manager", output="screen", parameters=[
            *parameters,
            {"map_frame": "map", "base_frame": "base_link",
             "configuration_directory": str(share / "config"),
             "configuration_basename": "pool_localization.lua",
             "reference_trajectory_id": int(value("reference_trajectory_id")),
             "scan_topic": value("scan_topic")},
        ]))
    if value("start_rviz") == "true":
        result.append(Node(package="rviz2", executable="rviz2", name="pool_rviz2",
                            output="screen", arguments=["-d", value("rviz_config")]))
    return result


def generate_launch_description():
    mapping = Path(get_package_share_directory("f1_mapping_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("load_state_filename"),
        DeclareLaunchArgument("start_trajectory", default_value="false", choices=["true", "false"],
                              description="Compatibility mode; stage 1 starts after /initialpose"),
        DeclareLaunchArgument("reference_trajectory_id", default_value="0"),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("lidar_extrinsics_file", default_value=str(mapping / "config/lidar_planar.yaml")),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value=str(
            get_package_share_directory("f1_pool_cleaning") + "/config/pool_localization.rviz")),
        OpaqueFunction(function=nodes),
    ])
