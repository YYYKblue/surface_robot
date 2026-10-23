"""ZeroMQ ROUTER to the native ROS bridge; never opens the F103 serial port."""
import json
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import zmq

from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl

from .ui_protocol import GatewayCore


class RosBackend:
    def __init__(self, node):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = node.create_publisher(ActuatorCommand, "/f1/actuator_command", qos)
        self.acquire_client = node.create_client(AcquireControl, "/f1/acquire_control")
        self.release_client = node.create_client(ReleaseControl, "/f1/release_control")

    @staticmethod
    def _call(client, request, callback):
        if not client.service_is_ready():
            callback(dict(success=False, message="bridge service unavailable"))
            return

        def complete(future):
            try:
                response = future.result()
                result = {key: getattr(response, key) for key in response.get_fields_and_field_types()}
            except Exception as error:
                result = dict(success=False, message=str(error))
            callback(result)

        client.call_async(request).add_done_callback(complete)

    def acquire(self, owner, callback):
        self._call(self.acquire_client, AcquireControl.Request(owner=owner), callback)

    def release(self, token, callback):
        self._call(self.release_client, ReleaseControl.Request(token=token), callback)

    def publish(self, command):
        message = ActuatorCommand()
        for name in ("token", "sequence", "direction", "speed_percent", "mode",
                     "brush_enabled", "thrust_enabled"):
            setattr(message, name, getattr(command, name))
        message.brush_percent, message.thrust_percent = float(command.brush_percent), float(command.thrust_percent)
        for name, ns in (("computed_at", command.computed_ns), ("valid_until", command.deadline_ns),
                         ("direction_until", command.direction_until_ns)):
            stamp = getattr(message, name)
            stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)
        self.publisher.publish(message)


class UIGateway(Node):
    def __init__(self):
        super().__init__("f1_ui_gateway")
        self.declare_parameter("bind_address", "127.0.0.1", ParameterDescriptor(read_only=True))
        self.declare_parameter("allow_fifth_thruster", False, ParameterDescriptor(read_only=True))
        if self.get_parameter("use_sim_time").value:
            raise ValueError("UI gateway rejects use_sim_time")
        self.add_on_set_parameters_callback(self._validate_parameters)
        address = self.get_parameter("bind_address").value
        if not isinstance(address, str) or not address or any(c in address for c in ":/; \t\n"):
            raise ValueError("bind_address must be an IPv4 address or hostname")
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.ROUTER)
        try:
            for option, value in ((zmq.LINGER, 0), (zmq.SNDHWM, 16), (zmq.RCVHWM, 64),
                                  (zmq.MAXMSGSIZE, 32768), (zmq.ROUTER_MANDATORY, 1)):
                self.socket.setsockopt(option, value)
            self.socket.bind(f"tcp://{address}:5556")
        except Exception:
            self.socket.close(linger=0)
            self.zmq_context.term()
            raise
        self.send_failures = 0
        self.last_send_failure_warn = 0.
        self.backend = RosBackend(self)
        self.core = GatewayCore(self.backend, self._send,
                                allow_fifth_thruster=self.get_parameter("allow_fifth_thruster").value,
                                report_loss=lambda reason: self.get_logger().warning("UI control lost: " + reason))
        self.subscription = self.create_subscription(ControlState, "/f1/control_state", self._state, 1)
        self.timer = self.create_timer(.01, self._tick)
        self.get_logger().info(f"f1-ui-v2 listening on tcp://{address}:5556")

    @staticmethod
    def _validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and parameter.value is not False:
                return SetParametersResult(successful=False, reason="UI gateway requires system time")
        return SetParametersResult(successful=True)

    def _send(self, ident, packet):
        try:
            self.socket.send_multipart([ident, json.dumps(packet, allow_nan=False).encode()], flags=zmq.DONTWAIT)
        except (zmq.ZMQError, ValueError) as error:
            # Backpressure must not block ROS or extend command validity, but a
            # silently dropped state packet can outlast the UI's receive window.
            self.send_failures += 1
            now = time.monotonic()
            if now - self.last_send_failure_warn >= 1.:
                self.last_send_failure_warn = now
                self.get_logger().warning(
                    f"gateway state send failed {self.send_failures} times; last={error}")

    def _state(self, message):
        state = {}
        for name in message.get_fields_and_field_types():
            value = getattr(message, name)
            if name == "stamp":
                value = dict(sec=value.sec, nanosec=value.nanosec)
            elif name == "target_pwm_bl_br_fl_fr":
                value = [int(pwm) for pwm in value]
            state[name] = value
        self.core.update_state(state)

    def _tick(self):
        for _ in range(64):
            try:
                frames = self.socket.recv_multipart(flags=zmq.DONTWAIT)
            except zmq.Again:
                break
            if len(frames) == 2:
                self.core.handle(*frames)
        self.core.tick()

    def destroy_node(self):
        if hasattr(self, "core"):
            self.timer.cancel()
            self.core.close()
            self.socket.close(linger=0)
            self.zmq_context.term()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = UIGateway()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
