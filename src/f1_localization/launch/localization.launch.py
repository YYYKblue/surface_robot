"""Load a frozen pbstream and publish its occupancy grid for RViz2."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from f1_mapping_bringup.configuration import antenna_tf_arguments, load_lidar_planar_transform, static_tf_arguments


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
        from f1_localization.map_io import MapProfileError, resolve_pbstream
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
    share = Path(get_package_share_directory("f1_localization"))
    cartographer_lib = Path(get_package_prefix("cartographer_ros")) / "lib" / "cartographer_ros"
    occupancy_grid = executable(
        cartographer_lib, ("cartographer_occupancy_grid_node", "occupancy_grid_node"))
    parameters = [{"use_sim_time": value("use_sim_time") == "true"}]
    localization_config = value("localization_config")
    use_rtk = value("use_rtk") == "true"
    heading = value("use_wt102_heading") == "true"
    basename = "rtk_localization.lua" if use_rtk else "localization.lua"
    result = []
    if value("publish_lidar_tf") == "true":
        mounting = load_lidar_planar_transform(value("lidar_extrinsics_file"))
        result.append(Node(package="tf2_ros", executable="static_transform_publisher",
            name="base_to_lidar_planar", parameters=parameters,
            arguments=static_tf_arguments("pavo2s_frame", mounting)))
    scan_remap = [("scan", value("scan_topic"))]
    if use_rtk:
        result.append(Node(package="tf2_ros", executable="static_transform_publisher",
                 name="base_to_gnss_antenna", parameters=parameters,
                 arguments=antenna_tf_arguments(value("gnss_config"))))
    if heading and value("use_sim_time") != "true" and value("start_wt102_driver") == "true":
        result.append(Node(package="f1_wt102_driver", executable="wt102_uart", output="screen",
                           parameters=[{"port": value("wt102_port")}]))
    if use_rtk or heading:
        result.append(Node(package="f1_mapping_bringup", executable="gnss_cartographer",
            name="pool_localization", output="screen", remappings=scan_remap,
            parameters=[value("gnss_config"), *parameters, {
                "configuration_directory": str(share / "config"), "configuration_basename": basename,
                "use_rtk": use_rtk, "use_wt102_heading": heading,
                "gnss_state_file": str(state), "start_trajectory": value("start_trajectory") == "true"}]))
    else:
        result.append(Node(package="cartographer_ros", executable="cartographer_node",
            name="pool_localization", output="screen", parameters=parameters, remappings=scan_remap,
            arguments=["-configuration_directory", str(share / "config"), "-configuration_basename", basename,
                       "-load_state_filename", str(state), "-load_frozen_state=true",
                       "-start_trajectory_with_default_topics=" + value("start_trajectory")]))
    # cartographer_node publishes submaps and TF; this separate node converts
    # the frozen submaps into the /map OccupancyGrid consumed by RViz2.
    result.append(Node(package="cartographer_ros", executable=occupancy_grid,
        name="cartographer_occupancy_grid_node", output="screen", parameters=parameters,
        arguments=["-resolution", value("occupancy_grid_resolution"),
                   "-publish_period_sec", value("occupancy_grid_publish_period_sec"),
                   *(["-include_unfrozen_submaps=false"] if use_rtk and heading else [])]))
    # The manager validates the operator seed before starting a trajectory.
    manager_parameters = [*parameters, {
        "map_frame": "map", "base_frame": "base_link",
        "configuration_directory": str(share / "config"),
        "configuration_basename": basename,
        "reference_trajectory_id": int(value("reference_trajectory_id")),
        "scan_topic": value("scan_topic"),
        # The manager derives its pose gate from the same map as the task.  The
        # resolved .pbstream keeps a direct load_state_filename override working.
        "map": map_path or str(state),
    }]
    if localization_config:
        manager_parameters.append(localization_config)
    if use_rtk:
        manager_parameters.append(value("gnss_config"))
    # Keep the manager on the loaded map and the same fused trajectory options.
    fusion_parameters = {"use_rtk": use_rtk, "use_lake_pose": use_rtk and heading, "gnss_state_file": str(state)}
    if use_rtk:
        fusion_parameters.update(configuration_directory=str(share / "config"), configuration_basename=basename)
    manager_parameters.append(fusion_parameters)
    manager_remappings = []
    if value("cleaning_prefix"):
        prefix = value("cleaning_prefix").strip("/")
        if not prefix or "/" in prefix:
            raise ValueError("cleaning_prefix must be one nonempty ROS name segment")
        manager_remappings = [
            (f"/pool_cleaning/{name}", f"/{prefix}/{name}")
            for name in ("localization_status", "localization_ready",
                         "set_initial_pose", "relocalize")]
    result.append(Node(package="f1_localization", executable="localization_manager",
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
        DeclareLaunchArgument("use_rtk", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_wt102_heading", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("wt102_port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("start_wt102_driver", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gnss_config", default_value=str(mapping / "config/gnss_fusion.yaml")),
        DeclareLaunchArgument("start_trajectory", default_value="false", choices=["true", "false"],
                              description="Compatibility mode; default waits for an operator initial pose"),
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
            get_package_share_directory("f1_localization") + "/config/localization.rviz")),
        OpaqueFunction(function=nodes),
    ])
