"""Real ROS manager with synthetic map/TF/scan and mock Cartographer services."""
import json
import time
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from cartographer_ros_msgs.msg import StatusCode, TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from f1_interfaces.srv import SetInitialPose
from f1_localization.localization_node import LocalizationManager


@pytest.mark.parametrize('prefix', ['pool_cleaning', 'l_cleaning'])
def test_independent_manager_initialization_ready_loss_and_relocalize(monkeypatch, tmp_path, reference_map, prefix):
    monkeypatch.setenv('ROS_DOMAIN_ID', '182')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init()
    peer = None
    executor = SingleThreadedExecutor()
    manager = None
    try:
        peer = Node('localization_test_peer')
        share = Path(get_package_share_directory('f1_localization'))
        remaps = [(f'/pool_cleaning/{name}', f'/{prefix}/{name}')
                  for name in ('localization_status', 'localization_ready', 'set_initial_pose', 'relocalize')]
        manager = LocalizationManager(parameter_overrides=[
            Parameter('map', value=str(reference_map)), Parameter('stable_sec', value=.1),
            Parameter('trajectory_query_period_sec', value=.05),
            Parameter('configuration_directory', value=str(share / 'config')),
        ], cli_args=['--ros-args', *[a for old, new in remaps for a in ('-r', f'{old}:={new}')]])
        executor.add_node(peer)
        executor.add_node(manager)
        starts, finishes, statuses = [], [], []
        active = set()

        def states(request, response):
            response.status.code = StatusCode.OK
            response.trajectory_states.trajectory_id = [0] + sorted(active)
            response.trajectory_states.trajectory_state = [TrajectoryStates.FROZEN] + [TrajectoryStates.ACTIVE] * len(active)
            return response

        def start(request, response):
            starts.append(request)
            response.status.code = StatusCode.OK
            response.trajectory_id = len(starts)
            active.add(response.trajectory_id)
            return response

        def finish(request, response):
            finishes.append(request.trajectory_id)
            active.discard(request.trajectory_id)
            response.status.code = StatusCode.OK
            return response

        services = [peer.create_service(GetTrajectoryStates, '/get_trajectory_states', states),
                    peer.create_service(StartTrajectory, '/start_trajectory', start),
                    peer.create_service(FinishTrajectory, '/finish_trajectory', finish)]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = peer.create_subscription(String, f'/{prefix}/localization_status',
                                       lambda msg: statuses.append(json.loads(msg.data)), qos)
        map_pub = peer.create_publisher(OccupancyGrid, '/map', qos)
        scan_pub = peer.create_publisher(LaserScan, '/pavo2_scan', 10)
        pose_pub = peer.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        broadcaster = TransformBroadcaster(peer)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 40
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * 1600
        grid.data[10 * 40 + 20] = 100  # scan endpoint (2, 1)
        map_pub.publish(grid)
        feed_scan = True

        def feed():
            stamp = peer.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
            tf.header.stamp = stamp
            tf.transform.translation.x = tf.transform.translation.y = 1.
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            if feed_scan:
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = 'base_link', stamp
                scan.range_min, scan.range_max = .1, 10.
                scan.ranges = [1.]
                scan_pub.publish(scan)

        timer = peer.create_timer(.03, feed)

        def wait_for(predicate, timeout=5.):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.02)
                if predicate():
                    return
            raise AssertionError(f'condition timed out; latest status: {statuses[-1:]!r}')

        client = peer.create_client(SetInitialPose, f'/{prefix}/set_initial_pose')
        reset = peer.create_client(Trigger, f'/{prefix}/relocalize')
        wait_for(lambda: client.service_is_ready() and reset.service_is_ready() and manager._states_checked and statuses)
        assert not starts and not statuses[-1]['ready']
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = pose.pose.pose.position.y = 1.
        pose.pose.pose.orientation.w = 1.
        future = client.call_async(SetInitialPose.Request(pose=pose))
        wait_for(future.done)
        assert future.result().success
        wait_for(lambda: statuses[-1]['ready'])
        assert len(starts) == 1 and starts[0].use_initial_pose
        assert starts[0].relative_to_trajectory_id == 0
        assert starts[0].configuration_basename == 'localization.lua'
        assert manager.status_pub.topic_name == f'/{prefix}/localization_status'
        assert not peer.get_publishers_info_by_topic('/f1/actuator_command')
        assert not any(name in peer.get_node_names() for name in ('pool_controller', 'l_controller', 'nano_bridge'))

        feed_scan = False
        wait_for(lambda: not statuses[-1]['ready'] and 'scan' in statuses[-1]['reason'])
        feed_scan = True
        future = reset.call_async(Trigger.Request())
        wait_for(future.done)
        assert future.result().success
        wait_for(lambda: not active and manager._finish_future is None)
        assert finishes == [1] and not manager.gate.ready
        pose_pub.publish(pose)
        wait_for(lambda: statuses[-1]['ready'] and len(starts) == 2)

        pose.header.frame_id = 'odom'
        future = client.call_async(SetInitialPose.Request(pose=pose))
        wait_for(future.done)
        assert not future.result().success
        wait_for(lambda: statuses[-1]['state'] == 'PAUSED')
        assert not statuses[-1]['ready']
    finally:
        executor.shutdown()
        if manager is not None:
            manager.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.shutdown()
