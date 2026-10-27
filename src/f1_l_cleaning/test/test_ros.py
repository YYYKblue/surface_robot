"""ROS adapter smoke tests; all outputs remain preview-only."""
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from f1_interfaces.srv import SetEdge
from f1_pool_cleaning.mission import Pose
from f1_pool_cleaning.perception import Observation
from f1_pool_cleaning.ros_support import to_observation_message

from f1_l_cleaning.controller_node import LController


@pytest.fixture(autouse=True)
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "181")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_l_controller_owns_two_edges_under_isolated_names():
    controller = None
    peer = None
    try:
        route = [-1.801431, .681948, -10.462431, -.786552,
                 -11.917931, 10.962448]
        controller = LController(parameter_overrides=[
            rclpy.parameter.Parameter("route_xy", value=route),
            rclpy.parameter.Parameter("use_map_tf", value=False),
            rclpy.parameter.Parameter("require_localization_ready", value=False),
        ])
        assert len(controller.pool.edges) == 2
        assert not controller.live
        peer = Node("l_adapter_probe")
        endpoints = peer.get_publisher_names_and_types_by_node("l_controller", "/")
        names = {name for name, _ in endpoints}
        assert "/l_cleaning/status" in names
        assert "/l_cleaning/command_preview" in names
        assert "/pool_cleaning/status" not in names
        assert "/f1/actuator_command" not in names
    finally:
        if peer is not None:
            peer.destroy_node()
        if controller is not None:
            controller.destroy_node()


def test_turn_discards_cached_wall_and_rejects_queued_pre_turn_scans(route_parameters):
    node = LController(parameter_overrides=[Parameter(k, value=v) for k, v in {
        **route_parameters, "use_map_tf": False, "require_localization_ready": False,
    }.items()])
    try:
        node.timer.cancel()
        now = node.get_clock().now().nanoseconds
        node.pose = Pose(now, -10., 0., 0.)
        node.wall = Observation(now, valid=True, distance=.34, angle=0.)
        node._set_selected_edge(node.pool.edges[0])
        generation = node.edge_generation
        node.mission.edge = node.pool.edges[1]
        node._sync_mission_edge()
        assert node.edge_generation == generation + 1
        assert node.selected_edge.id == "edge_1"
        assert not node.wall.valid
        old = to_observation_message(Observation(now, valid=True, distance=.34),
                                     "base_link", "test", 1)
        node._wall(old)
        assert not node.wall.valid
        fresh = to_observation_message(Observation(node._wall_after_turn_ns + 1,
                                                   valid=True, distance=.34),
                                       "base_link", "test", 2)
        node._wall(fresh)
        assert node.wall.valid
    finally:
        node.destroy_node()


def test_cannot_skip_first_wall_or_start_without_localization(route_parameters):
    node = LController(parameter_overrides=[Parameter(k, value=v) for k, v in route_parameters.items()])
    try:
        response = node._set_edge(SetEdge.Request(edge_id="edge_1"), SetEdge.Response())
        assert not response.success and node.selected_edge is None
        response = node._start(Trigger.Request(), Trigger.Response())
        assert not response.success and "localization" in response.message
        assert node.mission.phase == "IDLE" and node.lease is None
    finally:
        node.destroy_node()


def test_localization_launch_applies_large_map_bounds_and_l_topics(tmp_path):
    # Execute the shared launch factory with the L overrides, then instantiate
    # the manager with those exact parameters. No Cartographer or hardware starts.
    import importlib.util
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from f1_pool_cleaning.localization_node import LocalizationManager

    pool = Path(get_package_share_directory("f1_pool_cleaning"))
    spec = importlib.util.spec_from_file_location("pool_localization_launch", pool / "launch/pool_localization.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    # The factory must reject a missing map before creating any nodes.
    context = LaunchContext()
    context.launch_configurations["load_state_filename"] = "/missing/reference.pbstream"
    with pytest.raises(ValueError, match="existing reference-map"):
        launch.nodes(context)
    params_file = Path(__file__).resolve().parents[1] / "config/localization.yaml"
    params = yaml.safe_load(params_file.read_text())["pool_localization_manager"]["ros__parameters"]
    from f1_l_cleaning.topics import REMAPPINGS
    manager = LocalizationManager(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()],
        cli_args=["--ros-args", *[a for old, new in REMAPPINGS for a in ("-r", f"{old}:={new}")]])
    try:
        assert manager.config.origin_x_m == -14.
        assert manager.status_pub.topic_name == "/l_cleaning/localization_status"
        assert manager.ready_pub.topic_name == "/l_cleaning/localization_ready"
    finally:
        manager.destroy_node()
