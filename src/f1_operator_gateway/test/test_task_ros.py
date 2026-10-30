"""Real WebSocket -> gateway -> mock ROS services, with no bridge/hardware."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from f1_interfaces.srv import SetInitialPose
import websockets

from operator_gateway.node import OperatorGateway


def test_websocket_initial_pose_task_routing_completion_and_localization_loss():
    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    gateway = OperatorGateway()
    peer = Node("operator_test_robot")
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(peer)
    pool = ThreadPoolExecutor(max_workers=1)
    state = dict(initial=False, pose_stamp=0, publish_loc=True, started=None)
    calls = []
    statuses = {kind: dict(task_type=kind, phase="IDLE", run_id="", can_start=True,
        start_blockers=[], map_id="test-map", output_enabled=False, edge_count=2 if kind == "l" else 4)
        for kind in ("l", "rectangle")}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    loc_pub = peer.create_publisher(String, "/pool_cleaning/localization_status", qos)
    map_pub = peer.create_publisher(OccupancyGrid, "/map", qos)
    pubs = {kind: peer.create_publisher(String, prefix + "/status", 5)
            for kind, prefix in (("l", "/l_cleaning"), ("rectangle", "/pool_cleaning"))}
    mapping = OccupancyGrid()
    mapping.header.frame_id = "map"
    mapping.info.resolution = .1
    mapping.info.width = mapping.info.height = 100
    mapping.info.origin.orientation.w = 1.
    mapping.data = [0] * 10000
    map_pub.publish(mapping)

    def set_pose(request, response):
        calls.append(("pose", request))
        state["initial"] = True
        stamp = request.pose.header.stamp
        state["pose_stamp"] = stamp.sec*10**9+stamp.nanosec
        response.success, response.message = True, "accepted"
        return response
    peer.create_service(SetInitialPose, "/pool_cleaning/set_initial_pose", set_pose)

    def task_callback(kind, action):
        def callback(request, response):
            calls.append((kind, action))
            if action == "start":
                statuses[kind].update(phase="PLANNING", run_id=uuid.uuid4().hex)
                state["started"] = time.monotonic()
            else:
                statuses[kind]["phase"] = "CANCELLED" if action == "cancel" else "PAUSED"
            response.success, response.message = True, "accepted"
            return response
        return callback
    for kind, prefix in (("l", "/l_cleaning"), ("rectangle", "/pool_cleaning")):
        for action in ("start", "pause", "cancel"):
            peer.create_service(Trigger, prefix + "/" + action, task_callback(kind, action))

    def publish():
        now = peer.get_clock().now().nanoseconds
        if state["publish_loc"]:
            loc_pub.publish(String(data=json.dumps(dict(stamp_ns=now, map_id="test-map",
                initial_pose_received=state["initial"], initial_pose_stamp_ns=state["pose_stamp"],
                ready=state["initial"], state="LOCALIZATION_READY" if state["initial"] else "LOCALIZATION_CHECK"))))
        if state["started"] and time.monotonic()-state["started"] > .3:
            statuses["l"]["phase"] = "COMPLETE"
        for kind, pub in pubs.items():
            pub.publish(String(data=json.dumps(dict(statuses[kind], stamp_ns=now))))
    peer.create_timer(.05, publish)
    port = gateway.hub.server.sockets[0].getsockname()[1]

    async def scenario():
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            async def until(predicate):
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    packet = json.loads(await asyncio.wait_for(ws.recv(), 5))
                    if predicate(packet):
                        return packet
                raise AssertionError("expected websocket packet never arrived")
            async def send(packet):
                await ws.send(json.dumps(packet))
            await send(dict(op="hello", client="integration-test"))
            await until(lambda p: p.get("type") == "map")
            first = await until(lambda p: p.get("type") == "telemetry" and p.get("localization", {}).get("fresh"))
            assert not first["tasks"]["l"]["can_start"]
            await send(dict(op="initial_pose", request_id="pose", map_revision=1, x=2., y=3., yaw=1.))
            await until(lambda p: p.get("request_id") == "pose" and p.get("success"))
            await until(lambda p: p.get("type") == "telemetry" and p["tasks"]["l"]["can_start"])
            await send(dict(op="task", request_id="start", task_type="l", action="start"))
            await until(lambda p: p.get("request_id") == "start" and p.get("success"))
            done = await until(lambda p: p.get("type") == "telemetry" and p["tasks"]["l"].get("phase") == "COMPLETE")
            assert done["tasks"]["l"]["run_id"]
            state["publish_loc"] = False
            await asyncio.sleep(.8)
            await until(lambda p: p.get("type") == "telemetry" and not p["localization"]["fresh"])
            await send(dict(op="task", request_id="stale-start", task_type="rectangle", action="start"))
            rejected = await until(lambda p: p.get("request_id") == "stale-start")
            assert not rejected["success"]
    future = pool.submit(asyncio.run, scenario())
    try:
        deadline = time.monotonic()+20
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert future.done()
        future.result()
        assert ("l", "start") in calls
        assert ("rectangle", "start") not in calls
        pose = next(value for kind, value in calls if kind == "pose")
        assert pose.pose.header.frame_id == "map"
    finally:
        gateway.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        pool.shutdown(wait=True)
