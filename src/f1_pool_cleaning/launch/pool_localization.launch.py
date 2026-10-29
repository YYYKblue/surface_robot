"""Load a frozen pbstream and publish its occupancy grid for RViz2."""
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
    raise RuntimeError(
        f"Cartographer occupancy-grid executable missing: {candidates}; "
        "check ros2 pkg executables cartographer_ros")


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    # ``map`` is the one map input: it names either the frozen .pbstream or the
    # map .yaml beside it.  ``load_state_filename`` stays as a direct override
    # for callers that already hold a .pbstream path.
    map_path, state_arg = value("map").strip(), value("load_state_filename").strip()
    if map_path:
        from f1_pool_cleaning.map_profile import MapProfileError, resolve_pbstream
        try:
            state = resolve_pbstream(map_path)
        except MapProfileError as exc:
            raise ValueError(str(exc)) from exc
    elif state_arg:
        state = Path(state_arg)
    else:
        raise ValueError("set map:=<pbstream or map yaml> to load the reference map")
    if not state.is_file() or state.suffix != ".pbstream":
        raise ValueError("map must name the existing reference-map .pbstream")
    share = Path(get_package_share_directory("f1_pool_cleaning"))
    cartographer_lib = Path(get_package_prefix("cartographer_ros")) / "lib" / "cartographer_ros"
    occupancy_grid = executable(
        cartographer_lib, ("cartographer_occupancy_grid_node", "occupancy_grid_node"))
    parameters = [{"use_sim_time": value("use_sim_time") == "true"}]
    localization_config = value("localization_config")
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
    # cartographer_node publishes submaps and TF; this separate node converts
    # the frozen submaps into the /map OccupancyGrid consumed by RViz2.
    result.append(Node(package="cartographer_ros", executable=occupancy_grid,
        name="cartographer_occupancy_grid_node", output="screen", parameters=parameters,
        arguments=["-resolution", value("occupancy_grid_resolution"),
                   "-publish_period_sec", value("occupancy_grid_publish_period_sec")]))
    # The manager starts a trajectory only after a valid RViz initial pose. It
    # also publishes the readiness gate consumed by the cleaning controller.
    manager_parameters = [*parameters, {
        "map_frame": "map", "base_frame": "base_link",
        "configuration_directory": str(share / "config"),
        "configuration_basename": "pool_localization.lua",
        "reference_trajectory_id": int(value("reference_trajectory_id")),
        "scan_topic": value("scan_topic"),
        # The manager derives its pose gate from the same map as the task.  The
        # resolved .pbstream keeps a direct load_state_filename override working.
        "map": map_path or str(state),
    }]
    if localization_config:
        manager_parameters.append(localization_config)
    manager_remappings = []
    if value("cleaning_prefix"):
        prefix = value("cleaning_prefix").strip("/")
        if not prefix or "/" in prefix:
            raise ValueError("cleaning_prefix must be one nonempty ROS name segment")
        manager_remappings = [
            (f"/pool_cleaning/{name}", f"/{prefix}/{name}")
            for name in ("localization_status", "localization_ready",
                         "set_initial_pose", "relocalize")]
    result.append(Node(package="f1_pool_cleaning", executable="pool_localization",
        name="pool_localization_manager", output="screen", parameters=[
            *manager_parameters,
        ], remappings=manager_remappings))
    if value("start_rviz") == "true":
        result.append(Node(package="rviz2", executable="rviz2", name="pool_rviz2",
                            output="screen", arguments=["-d", value("rviz_config")]))
    return result


def generate_launch_description():
    mapping = Path(get_package_share_directory("f1_mapping_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="",
                              description="Frozen reference map: .pbstream or the map .yaml beside it"),
        DeclareLaunchArgument("load_state_filename", default_value="",
                              description="Direct .pbstream override; prefer map:="),
        DeclareLaunchArgument("start_trajectory", default_value="false", choices=["true", "false"],
                              description="Compatibility mode; stage 1 starts after /initialpose"),
        DeclareLaunchArgument("reference_trajectory_id", default_value="0"),
        DeclareLaunchArgument("localization_config", default_value="",
                              description="Optional extra LocalizationConfig YAML overrides"),
        DeclareLaunchArgument("cleaning_prefix", default_value="",
                              description="Remap the localization manager's cleaning gate under this prefix"),
        DeclareLaunchArgument("scan_topic", default_value="/pavo2_scan"),
        DeclareLaunchArgument("occupancy_grid_resolution", default_value="0.05"),
        DeclareLaunchArgument("occupancy_grid_publish_period_sec", default_value="1.0"),
        DeclareLaunchArgument("lidar_extrinsics_file", default_value=str(mapping / "config/lidar_planar.yaml")),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value=str(
            get_package_share_directory("f1_pool_cleaning") + "/config/pool_localization.rviz")),
        OpaqueFunction(function=nodes),
    ])
