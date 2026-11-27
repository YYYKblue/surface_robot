"""ROS node integration with synthetic map-frame localization and TF."""
import json
import time

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from f1_interfaces.srv import SetLaneSpacing
from f1_area_patrol.controller import PatrolNode


@pytest.fixture(autouse=True)
def isolated_ros(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "113")


def test_preview_pause_resume_and_fault(tmp_path):
    raster = tmp_path/'water.pgm'
    raster.write_bytes(b'P5\n100 60\n255\n'+np.full((60, 100), 255, np.uint8).tobytes())
    path = tmp_path/'water.yaml'
    path.write_text('image: water.pgm\nresolution: 0.05\norigin: [0,0,0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    rclpy.init()
    node = PatrolNode(parameter_overrides=[Parameter("map", value=str(path))])
    def update(y=1.):
        now = node.get_clock().now()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.header.stamp = now.to_msg()
        tf.transform.translation.x, tf.transform.translation.y = 1., y
        tf.transform.rotation.w = 1.
        node.tf.set_transform(tf, 'test')
        node._localization(String(data=json.dumps(dict(ready=True, state="LOCALIZATION_READY",
            map_id=node.map_id, trajectory_id=1, initial_pose_stamp_ns=1, stamp_ns=now.nanoseconds))))
        node._tick()
    def action(name):
        return node._action(name, None, Trigger.Response()).success
    try:
        assert node.lease is None and not action('start')
        end = time.monotonic()+3.
        while node.route is None and time.monotonic()<end:
            update()
            time.sleep(.005)
        node._status()
        assert node.route and node.plan_id
        old_plan = node.plan_id
        # Move across the centre between preview and start; the larger side flips.
        update(2.)
        assert action("start")
        assert node.phase == "PLANNING"
        end = time.monotonic()+3.
        while node.phase == "PLANNING" and time.monotonic()<end:
            update(2.12)  # Further drift during planning must not invalidate the job.
            time.sleep(.005)
        assert node.phase == "PATROLLING"
        assert node.route.legs[0].start == (1., 2.) and node.plan_id != old_plan
        assert node.route.region[node.grid.cell(2., 1.)]
        assert not node.route.region[node.grid.cell(2., 2.3)]
        run_id = node.run_id
        update(2.12)
        assert action('pause') and not node._spacing(SetLaneSpacing.Request(spacing_m=.5), SetLaneSpacing.Response()).success
        assert action('resume') and node.resume_sequence == 1 and node.run_id == run_id
        update(2.12)
        assert node.phase == "PATROLLING"
        node._localization(String(data=json.dumps(dict(node.loc, ready=False))))
        assert node.phase == "PAUSED" and node.pause_kind == "fault" and not action("resume")
        assert action("cancel")
        assert node._spacing(SetLaneSpacing.Request(spacing_m=.5), SetLaneSpacing.Response()).success
        assert node.route is None
        update(2.12)
        assert action("start") and node.phase == "PLANNING"
        assert action("cancel")
        update(2.12)
        assert node.phase == "CANCELLED" and node.lease is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_services_hold_and_release_simulated_bridge(tmp_path):
    """Exercise async start, fault stop and the actual actuator topic over local DDS."""
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import SingleThreadedExecutor
    from f1_interfaces.msg import ActuatorCommand, ControlState
    from f1_interfaces.srv import AcquireControl, ReleaseControl

    path = tmp_path / "water.yaml"
    (tmp_path / "water.pgm").write_bytes(b"P5\n100 60\n255\n"+bytes([255])*6000)
    path.write_text("image: water.pgm\nresolution: 0.05\norigin: [0,0,0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    rclpy.init()
    node = PatrolNode(parameter_overrides=[Parameter("map", value=str(path)), Parameter("output_enabled", value=True)])
    peer = Node("patrol_simulated_bridge")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    state = ControlState(connected=True, state="READY", source="NANO", armed=True, result="OK",
                         connection_id="test", epoch=1, allow_fifth_thruster=True)
    commands, releases = [], []
    state_pub = peer.create_publisher(ControlState, "/f1/control_state", 5)
    peer.create_subscription(ActuatorCommand, "/f1/actuator_command", commands.append, qos_profile_sensor_data)
    def acquire(req, res):
        state.owner, state.state, state.motion_authorized = req.owner, "ACTIVE", True
        res.success, res.token, res.epoch = True, "patrol-token", 1
        return res
    def release(req, res):
        releases.append(req.token)
        state.owner, state.state, state.motion_authorized = "", "READY", False
        res.success = True
        return res
    peer.create_service(AcquireControl, "/f1/acquire_control", acquire)
    peer.create_service(ReleaseControl, "/f1/release_control", release)
    start = peer.create_client(Trigger, "/area_patrol/start")
    def feedback():
        now = peer.get_clock().now()
        state.stamp = now.to_msg()
        state_pub.publish(state)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_link"
        tf.header.stamp = now.to_msg()
        tf.transform.translation.x, tf.transform.translation.y = 1., 1.
        tf.transform.rotation.w = 1.
        node.tf.set_transform(tf, "test")
        node._localization(String(data=json.dumps(dict(ready=True, state="LOCALIZATION_READY",
            map_id=node.map_id, trajectory_id=1, initial_pose_stamp_ns=1, stamp_ns=now.nanoseconds))))
    peer.create_timer(.03, feedback)
    def wait_for(condition):
        until = time.monotonic()+6.
        while not condition() and time.monotonic()<until:
            executor.spin_once(timeout_sec=.01)
        assert condition(), (node.phase, node.reason, node._blockers())
    try:
        wait_for(lambda: start.service_is_ready() and not node._blockers())
        reply = start.call_async(Trigger.Request())
        wait_for(reply.done)
        assert reply.result().success
        wait_for(lambda: any(c.speed_percent or c.thrust_enabled for c in commands))
        assert state.owner.startswith("patrol:") and all(c.token=="patrol-token" and not c.brush_enabled for c in commands)
        state.source = "RC"
        wait_for(lambda: bool(releases) and commands[-1].direction==ActuatorCommand.STOP and not commands[-1].thrust_enabled)
        assert releases == ["patrol-token"] and node.pause_kind=="fault" and not node.lease.token
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
