"""Real WebSocket -> gateway -> mock ROS services, with no bridge/hardware."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import OccupancyGrid
from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl, SetInitialPose
import websockets

from operator_gateway.node import OperatorGateway


@pytest.mark.parametrize("fault", ["incompatible", "rc", "handshake", "epoch", "connection", "lease_lost"])
def test_continuous_manual_output_requires_live_compatible_firmware_and_never_replays(fault):
    """Exercise the wire contract and ROS output with simulated firmware, no hardware."""
    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    gateway = OperatorGateway()
    peer = Node("operator_manual_test_robot")
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(peer)
    pool = ThreadPoolExecutor(max_workers=1)
    firmware = dict(compatible=True, source="NANO", phase=None, epoch=1,
                    connection_id="serial-session", leased=False)
    commands, acquired, released = [], [], []
    control_pub = peer.create_publisher(ControlState, "/f1/control_state", 10)
    peer.create_subscription(ActuatorCommand, "/f1/actuator_command", commands.append,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def publish_control():
        message = ControlState()
        message.stamp = peer.get_clock().now().to_msg()
        message.connected, message.armed = True, firmware["phase"] != "HANDSHAKE"
        message.state = firmware["phase"] or ("ACTIVE" if firmware["leased"] else "READY")
        message.source, message.result = firmware["source"], "OK"
        message.connection_id, message.epoch = firmware["connection_id"], firmware["epoch"]
        message.motion_authorized = firmware["leased"]
        message.owner = "operator_app" if firmware["leased"] else ""
        message.firmware_protocol_version = 3 if firmware["compatible"] else 2
        message.firmware_protocol_compatible = firmware["compatible"]
        control_pub.publish(message)

    def acquire(request, response):
        acquired.append(request.owner)
        firmware["leased"] = True
        response.success, response.token = True, "manual-test-token"
        return response

    def release(request, response):
        released.append(request.token)
        firmware["leased"] = False
        response.success = True
        return response

    peer.create_timer(.05, publish_control)
    peer.create_service(AcquireControl, "/f1/acquire_control", acquire)
    peer.create_service(ReleaseControl, "/f1/release_control", release)
    port = gateway.hub.server.sockets[0].getsockname()[1]

    async def scenario():
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            async def until(predicate):
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    packet = json.loads(await asyncio.wait_for(ws.recv(), 5))
                    if predicate(packet):
                        return packet
                raise AssertionError("manual contract state not received")

            async def manual(sequence, speed, *, brush=False):
                await ws.send(json.dumps(dict(op="manual_input", seq=sequence,
                    direction="FORWARD", speed_percent=speed, mode="BOTH",
                    brush_enabled=brush, brush_percent=50 if brush else 0,
                    thrust_enabled=False, thrust_percent=0)))

            await ws.send(json.dumps(dict(op="hello", client="continuous-manual-test")))
            hello = await until(lambda p: p.get("type") == "hello")
            assert "continuous_drive_percent_v1" in hello["capabilities"]
            await until(lambda p: p.get("type") == "telemetry" and p["drive_output"]["available"])
            await manual(1, 0)
            await until(lambda p: p.get("type") == "telemetry")
            assert not acquired and not commands
            await manual(2, 37)
            await until(lambda p: any(c.speed_percent == 37 for c in commands))
            assert acquired == ["operator_app"]
            await manual(3, 0, brush=True)
            await until(lambda p: any(c.speed_percent == 0 and c.brush_enabled
                                     and c.brush_percent == 50. for c in commands))
            await manual(4, 0)
            await until(lambda p: bool(commands) and commands[-1].speed_percent == 0
                         and not commands[-1].brush_enabled)
            # Revocation must clear the old lease even if no new input arrives.
            changes = {
                "incompatible": dict(compatible=False, phase="INCOMPATIBLE"),
                "rc": dict(source="RC", phase="RC", epoch=2),
                "handshake": dict(phase="HANDSHAKE"),
                "epoch": dict(epoch=2),
                "connection": dict(connection_id="new-serial-session"),
                "lease_lost": {},
            }[fault]
            firmware.update(changes, leased=False)
            await until(lambda p: p.get("type") == "telemetry" and bool(released))
            assert not gateway.token and gateway.last_input is None
            await manual(5, 99)
            reply = await until(lambda p: p.get("type") == "reply" and p.get("op") == "manual_input")
            assert reply["success"] is False
            if fault in ("incompatible", "rc", "handshake"):
                await manual(6, 0)  # Neutral before authority returns cannot rearm.
                await until(lambda p: p.get("type") == "telemetry")
                assert gateway.manual_rearm_required
            count = len(commands)
            firmware.update(compatible=True, phase=None, source="NANO")
            await until(lambda p: p.get("type") == "telemetry" and p["drive_output"]["available"]
                        and p["state"]["state"] == "READY" and p["state"]["source"] == "NANO")
            await manual(7, 99)  # Continuing the old hold after recovery is still rejected.
            reply = await until(lambda p: p.get("type") == "reply" and p.get("op") == "manual_input")
            assert reply["success"] is False and "中性" in reply["message"]
            assert len(commands) == count and len(acquired) == 1
            await manual(8, 0)
            await manual(9, 99)
            await until(lambda p: len(commands) > count)
            assert commands[-1].speed_percent == 99 and len(acquired) == 2

    future = pool.submit(asyncio.run, scenario())
    try:
        deadline = time.monotonic() + 20
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert future.done()
        future.result()
    finally:
        gateway.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        pool.shutdown(wait=True)


def test_websocket_manual_timeout_history_is_available_after_reconnect():
    # STOP-only input exercises the real watchdog without acquiring control.
    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    gateway = OperatorGateway()
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    pool = ThreadPoolExecutor(max_workers=1)
    port = gateway.hub.server.sockets[0].getsockname()[1]

    async def scenario():
        async def until(ws, predicate):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                packet = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if predicate(packet):
                    return packet
            raise AssertionError("diagnostic telemetry not received")

        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await ws.send(json.dumps(dict(op="hello", client="diagnostics-test")))
            hello = await until(ws, lambda p: p.get("type") == "hello")
            assert "diagnostic_history" in hello["capabilities"]
            await ws.send(json.dumps(dict(op="manual_input", seq=1, direction="STOP",
                speed_percent=25, mode="BOTH", brush_enabled=False, brush_percent=0,
                thrust_enabled=False, thrust_percent=0)))
            packet = await until(ws, lambda p: any(e["reason"] == "input_timeout"
                for e in p.get("diagnostics", {}).get("events", [])))
            history = packet["diagnostics"]
            event = next(e for e in history["events"] if e["reason"] == "input_timeout")
            assert event["details"]["input_age_ms"] >= 250
            assert event["details"]["had_lease"] is False
        async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
            await ws.send(json.dumps(dict(op="hello", client="diagnostics-reconnect")))
            replay = await until(ws, lambda p: bool(p.get("diagnostics", {}).get("events")))
            assert replay["diagnostics"]["session"] == history["session"]
            assert event in replay["diagnostics"]["events"]

    future = pool.submit(asyncio.run, scenario())
    try:
        deadline = time.monotonic() + 15
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert future.done()
        future.result()
    finally:
        gateway.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        pool.shutdown(wait=True)


def test_websocket_initial_pose_task_routing_completion_and_localization_loss():
    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    gateway = OperatorGateway()
    peer = Node("operator_test_robot")
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(peer)
    pool = ThreadPoolExecutor(max_workers=1)
    state = dict(initial=False, pose_stamp=0, publish_loc=True, started=None, auto_complete=True)
    calls = []
    statuses = {kind: dict(task_type=kind, phase="IDLE", run_id="", can_start=True,
        forward_boost_enabled=False, forward_boost_editable=True,
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
                statuses[kind]["phase"] = {"pause": "PAUSED", "stop": "STOPPED", "cancel": "CANCELLED"}[action]
            response.success, response.message = True, "accepted"
            return response
        return callback
    for kind, prefix in (("l", "/l_cleaning"), ("rectangle", "/pool_cleaning")):
        for action in ("start", "pause", "stop", "cancel"):
            peer.create_service(Trigger, prefix + "/" + action, task_callback(kind, action))
        def boost_callback(request, response, k=kind):
            calls.append((k, "set_forward_boost"))
            statuses[k]["forward_boost_enabled"] = request.data
            response.success, response.message = True, "setting accepted"
            return response
        peer.create_service(SetBool, prefix + "/set_forward_boost", boost_callback)

    def publish():
        now = peer.get_clock().now().nanoseconds
        if state["publish_loc"]:
            loc_pub.publish(String(data=json.dumps(dict(stamp_ns=now, map_id="test-map",
                initial_pose_received=state["initial"], initial_pose_stamp_ns=state["pose_stamp"],
                ready=state["initial"], state="LOCALIZATION_READY" if state["initial"] else "LOCALIZATION_CHECK"))))
        if state["auto_complete"] and state["started"] and time.monotonic()-state["started"] > .3:
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
            await until(lambda p: p.get("type") == "telemetry" and p["tasks"]["l"]["can_set_forward_boost"])
            await send(dict(op="task", request_id="boost", task_type="l", action="set_forward_boost", enabled=True))
            await until(lambda p: p.get("request_id") == "boost" and p.get("success") and not p.get("pending"))
            assert statuses["l"]["forward_boost_enabled"] is True
            assert statuses["rectangle"]["forward_boost_enabled"] is False
            await send(dict(op="task", request_id="start", task_type="l", action="start"))
            await until(lambda p: p.get("request_id") == "start" and p.get("success"))
            done = await until(lambda p: p.get("type") == "telemetry" and p["tasks"]["l"].get("phase") == "COMPLETE")
            assert done["tasks"]["l"]["run_id"]
            state["auto_complete"] = False
            await send(dict(op="task", request_id="second-start", task_type="l", action="start"))
            await until(lambda p: p.get("request_id") == "second-start" and p.get("success"))
            await send(dict(op="task", request_id="locked-boost", task_type="l", action="set_forward_boost", enabled=False))
            rejected = await until(lambda p: p.get("request_id") == "locked-boost")
            assert not rejected["success"] and statuses["l"]["forward_boost_enabled"]
            await send(dict(op="task", request_id="stop", task_type="l", action="stop"))
            await until(lambda p: p.get("request_id") == "stop" and p.get("success"))
            stopped = await until(lambda p: p.get("type") == "telemetry" and p["tasks"]["l"].get("phase") == "STOPPED")
            assert stopped["tasks"]["l"]["run_id"] != done["tasks"]["l"]["run_id"]
            await send(dict(op="task", request_id="cancel", task_type="l", action="cancel"))
            await until(lambda p: p.get("request_id") == "cancel" and p.get("success"))
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
        assert ("l", "stop") in calls and ("l", "cancel") in calls
        assert ("rectangle", "start") not in calls
        pose = next(value for kind, value in calls if kind == "pose")
        assert pose.pose.header.frame_id == "map"
    finally:
        gateway.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        pool.shutdown(wait=True)
