"""ROS API for the F103 worker. Command freshness is checked in the worker too."""
import json
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl
from nano_ros_bridge.imu.conversion import ClockDiscontinuity, Converter, to_ros_message

from .protocol import PROFILE
from .session import Command, Session
from .transport import Worker


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Bridge(Node):
    def __init__(self):
        super().__init__("nano_bridge")
        defaults = {
            "serial_enabled": False,
            "port": "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5761014804-if00",
            "motion_enabled": False,
            "firmware_profile": PROFILE,
            "direction_confirmed": False,
            "allow_fifth_thruster": False,
            "imu_frame_id": "imu_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        get = lambda name: self.get_parameter(name).value
        if self.get_parameter("use_sim_time").value:
            raise ValueError("hardware bridge rejects use_sim_time")
        if get("firmware_profile") != PROFILE:
            raise ValueError(f"unsupported firmware profile; expected {PROFILE}")
        if get("motion_enabled") and not (get("serial_enabled") and get("direction_confirmed")):
            raise ValueError("motion requires serial_enabled and direction_confirmed")
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.worker = Worker(get("port"), get("serial_enabled"), get("motion_enabled"),
                             get("allow_fifth_thruster"))
        self.frame_id = get("imu_frame_id")
        self.converter = Converter(required_status_mask=6, max_age_sec=.2, clock_jump_sec=.05)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.command_subscription = self.create_subscription(
            ActuatorCommand, "/f1/actuator_command", self._command, command_qos)
        self.state_pub = self.create_publisher(ControlState, "/f1/control_state", 5)
        self.imu_pub = self.create_publisher(Imu, "/f1/imu/data_raw", qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.acquire_service = self.create_service(AcquireControl, "/f1/acquire_control", self._acquire)
        self.release_service = self.create_service(ReleaseControl, "/f1/release_control", self._release)
        self.stop_service = self.create_service(Trigger, "/f1/stop", self._stop)
        self.imu_timer = self.create_timer(.01, self._imu)
        self.state_timer = self.create_timer(.1, self._state)
        self.worker.start()
        self.get_logger().info(
            f"F103 bridge: serial={get('serial_enabled')}, motion={get('motion_enabled')}, "
            f"profile={PROFILE}; firmware unchanged")

    def _validate_parameters(self, parameters):
        for p in parameters:
            if p.name == "use_sim_time" and (p.type_ != Parameter.Type.BOOL or p.value):
                return SetParametersResult(successful=False, reason="live bridge requires system time")
        return SetParametersResult(successful=True)

    def _command(self, message):
        self.worker.accept(Command(
            message.token, message.sequence, ns(message.computed_at), ns(message.valid_until),
            message.direction, message.speed_percent,
            message.mode, message.brush_enabled, message.brush_percent,
            message.thrust_enabled, message.thrust_percent, ns(message.direction_until)))

    def _acquire(self, request, response):
        token, reason = self.worker.acquire(request.owner)
        response.success, response.token, response.message = bool(token), token, reason
        snapshot = self.worker.snapshot()
        response.epoch = snapshot["ctrl"].epoch if snapshot["ctrl"] else 0
        response.first_command_timeout_ms = round(Session.TTL * 1000)
        response.max_command_ttl_ms = round(Session.TTL * 1000)
        return response

    def _release(self, request, response):
        response.success = self.worker.release(request.token)
        response.message = "lease revoked; neutral queued" if response.success else "token does not own control"
        return response

    def _stop(self, request, response):
        self.worker.stop()
        response.success = True
        response.message = ("Nano lease revoked; neutral sent on next eligible tick. "
                            "This does not confirm physical stop or override RC.")
        return response

    def _imu(self):
        for session, sample, unix_ns, mono_ns in self.worker.take_imu():
            # Reuse the already-tested pure timestamp converter, without a ZMQ
            # socket or the old f1_imu_bridge node.
            packet = dict(protocol="f1-imu-v1", session=session, imu=sample,
                          received_unix_ns=unix_ns, received_monotonic_ns=mono_ns)
            try:
                converted = self.converter.convert(json.dumps(packet).encode(),
                                                   self.get_clock().now().nanoseconds)
            except ClockDiscontinuity as error:
                self.worker.latch_fault(str(error))
                return
            if converted is not None:
                self.imu_pub.publish(to_ros_message(converted, self.frame_id))

    def _state(self):
        snap = self.worker.snapshot()
        state = ControlState()
        state.stamp = self.get_clock().now().to_msg()
        for name, value in snap.items():
            if name != "ctrl":
                setattr(state, name, value)
        ctrl = snap["ctrl"]
        if ctrl:
            state.source, state.epoch = ctrl.source, ctrl.epoch
            state.armed, state.result, state.mode = ctrl.armed, ctrl.result, ctrl.mode
            state.have_sequence, state.ack_sequence = ctrl.have_sequence, ctrl.ack
            state.target_pwm_bl_br_fl_fr = list(ctrl.pwm)
            state.brush_enabled, state.brush_percent = ctrl.brush_enabled, ctrl.brush_percent
            state.thrust_enabled, state.thrust_percent = ctrl.thrust_enabled, ctrl.thrust_percent
            state.speed_percent, state.direction = ctrl.speed_percent, ctrl.direction
        self.state_pub.publish(state)
        array = DiagnosticArray()
        array.header.stamp = state.stamp
        status = DiagnosticStatus()
        status.name, status.hardware_id = "nano_bridge/f103", "F1_IMU_NEW _back2"
        status.level = DiagnosticStatus.OK if state.state in ("READY", "ACTIVE", "MONITOR") else DiagnosticStatus.WARN
        if state.state == "FAULT":
            status.level = DiagnosticStatus.ERROR
        status.message = f"{state.state}: {state.reason}"
        status.values = [KeyValue(key=k, value=str(v)) for k, v in snap.items() if k != "ctrl"]
        array.status.append(status)
        imu_status = DiagnosticStatus()
        imu_status.name, imu_status.hardware_id = "nano_bridge/imu", status.hardware_id
        imu_status.level = DiagnosticStatus.OK if (0 <= state.imu_age_sec < .2
                                                    and state.imu_status & 6 == 6
                                                    and not self.converter.error) else DiagnosticStatus.WARN
        imu_status.message = self.converter.error or "ACC/GYRO valid; orientation not published"
        array.status.append(imu_status)
        self.diag_pub.publish(array)

    def destroy_node(self):
        if hasattr(self, "worker"):
            self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = Bridge()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal SIGINT may be followed by launch forwarding another SIGINT.
        # Do not let the second signal interrupt the serial neutral/close path.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
