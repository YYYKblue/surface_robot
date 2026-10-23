"""Real TCP/ROS adapter checks, with no serial port or hardware outputs."""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
import zmq

from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl
from nano_ros_bridge.node import Bridge
from nano_ros_bridge.ui_gateway import UIGateway


@pytest.fixture(autouse=True)
def isolated_ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "175")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))


class Connection:
    def __init__(self, executor):
        self.executor = executor
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect("tcp://127.0.0.1:5556")
        self.packets = []

    def send(self, op, **fields):
        self.socket.send_json(dict(protocol="f1-ui-v2", op=op, request_id=op, **fields))

    def until(self, predicate):
        deadline = time.monotonic() + 4
        hello_at = 0
        while not predicate() and time.monotonic() < deadline:
            if time.monotonic() - hello_at > .08:
                self.send("hello")
                hello_at = time.monotonic()
            self.executor.spin_once(timeout_sec=.005)
            while self.socket.poll(0):
                self.packets.append(self.socket.recv_json())
        assert predicate(), self.packets[-3:]

    def reply(self, op):
        return next((p for p in reversed(self.packets) if p.get("op") == op), None)

    def close(self):
        self.socket.close(linger=0)
        self.context.term()


def test_native_disabled_bridge_visible_over_tcp_and_acquire_rejected():
    rclpy.init()
    bridge, gateway = Bridge(), UIGateway()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(gateway)
    client = Connection(executor)
    try:
        client.until(lambda: any(p.get("state", {}).get("state") == "DISCONNECTED" for p in client.packets)
                     and gateway.backend.acquire_client.service_is_ready())
        client.send("acquire", owner="test")
        client.until(lambda: client.reply("acquire") is not None)
        assert not client.reply("acquire")["success"]
        assert bridge.worker.thread is None
        assert not gateway.set_parameters([Parameter("use_sim_time", value=True)])[0].successful
    finally:
        gateway.destroy_node()
        bridge.destroy_node()
        client.close()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.parametrize("allow_fifth", [False, True])
def test_tcp_acquire_command_and_token_scoped_stop_through_ros(allow_fifth):
    rclpy.init(args=["--ros-args", "-p", "allow_fifth_thruster:=" + str(allow_fifth).lower()])
    backend, gateway = Node("fake_bridge"), UIGateway()
    executor = SingleThreadedExecutor()
    executor.add_node(backend)
    executor.add_node(gateway)
    commands, released = [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    backend.create_subscription(ActuatorCommand, "/f1/actuator_command", commands.append, qos)
    publisher = backend.create_publisher(ControlState, "/f1/control_state", 1)

    def acquire(request, response):
        response.success, response.token, response.epoch = True, "ros-lease", 7
        response.first_command_timeout_ms = response.max_command_ttl_ms = 150
        return response

    def release(request, response):
        released.append(request.token)
        response.success = True
        return response

    backend.create_service(AcquireControl, "/f1/acquire_control", acquire)
    backend.create_service(ReleaseControl, "/f1/release_control", release)
    state = ControlState(state="READY", connected=True, source="NANO", armed=True,
                         result="OK", epoch=7, connection_id="fake", ctrl_age_sec=.01, ack_age_sec=.01,
                         allow_fifth_thruster=allow_fifth)
    backend.create_timer(.04, lambda: publisher.publish(state))
    client = Connection(executor)
    try:
        client.until(lambda: any(p.get("state", {}).get("state") == "READY" for p in client.packets)
                     and gateway.backend.acquire_client.service_is_ready()
                     and gateway.backend.release_client.service_is_ready()
                     and gateway.backend.publisher.get_subscription_count() == 1)
        client.send("acquire", owner="test")
        client.until(lambda: client.reply("acquire") is not None)
        assert client.reply("acquire")["success"]
        ticket = next(p["ticket"] for p in reversed(client.packets) if p.get("type") == "state")
        client.send("command", token="ros-lease", ticket=ticket, sequence=1, direction=4,
                    speed_percent=25, mode=1, brush_enabled=True, brush_percent=-35,
                    thrust_enabled=allow_fifth, thrust_percent=40 if allow_fifth else 0)
        client.until(lambda: bool(commands))
        command = commands[-1]
        assert command.token == "ros-lease" and command.direction == 4
        assert command.brush_enabled and command.brush_percent == -35.
        assert command.thrust_enabled is allow_fifth
        assert command.thrust_percent == (40. if allow_fifth else 0.)
        ns = lambda stamp: stamp.sec * 1_000_000_000 + stamp.nanosec
        assert ns(command.valid_until) - ns(command.computed_at) == 150_000_000
        assert ns(command.direction_until) == 0
        client.send("stop", token="ros-lease")
        client.until(lambda: client.reply("stop") is not None)
        assert client.reply("stop")["success"] and released == ["ros-lease"]
    finally:
        gateway.destroy_node()
        backend.destroy_node()
        client.close()
        executor.shutdown()
        rclpy.shutdown()
