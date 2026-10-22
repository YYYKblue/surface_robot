"""Receive gateway IMU telemetry; never opens a serial or control connection."""
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu
import zmq

from .conversion import ClockDiscontinuity, Converter, to_ros_message


class ImuBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("f1_imu_bridge", **kwargs)
        defaults = {
            "endpoint": "tcp://127.0.0.1:5555", "output_topic": "/f1/imu/data_raw",
            "diagnostics_topic": "/f1/imu/diagnostics", "frame_id": "imu_link",
            "acceleration_scale": 1.0, "angular_velocity_scale": 1.0,
            "required_status_mask": 0, "max_age_sec": 0.5, "clock_jump_sec": 0.1,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        values = {name: self.get_parameter(name).value for name in defaults}
        if self.get_parameter("use_sim_time").value:
            raise ValueError("live IMU bridge requires use_sim_time=false; replay IMU from the bag instead")
        frame = values["frame_id"]
        if not isinstance(frame, str) or not frame or "/" in frame or any(c.isspace() for c in frame):
            raise ValueError("frame_id must be a nonempty ROS frame name without slashes or spaces")
        if not values["endpoint"].startswith("tcp://"):
            raise ValueError("endpoint must use tcp://; run bridge and gateway on the same Nano")
        self.converter = Converter(**{name: values[name] for name in (
            "acceleration_scale", "angular_velocity_scale", "required_status_mask",
            "max_age_sec", "clock_jump_sec")})
        self.frame_id = frame
        self.publisher = self.create_publisher(
            Imu, values["output_topic"], QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.diagnostics = self.create_publisher(DiagnosticArray, values["diagnostics_topic"], 10)
        self.context_zmq = zmq.Context()
        self.subscriber = self.context_zmq.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.LINGER, 0)
        self.subscriber.setsockopt(zmq.RCVHWM, 256)
        self.subscriber.setsockopt(zmq.MAXMSGSIZE, 8192)
        self.subscriber.setsockopt(zmq.SUBSCRIBE, b"f1_imu")
        self.subscriber.connect(values["endpoint"])
        self.last_diagnostic = time.monotonic()
        self.last_count = 0
        self.create_timer(0.005, self.receive)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f"Waiting for f1_imu at {values['endpoint']}; publishing {values['output_topic']}")

    def receive(self):
        if self.get_parameter("use_sim_time").value:
            self.converter.clock_fault = True
            self.converter.error = "live bridge cannot switch to simulation time; restart for live acquisition"
            return
        for _ in range(128):
            try:
                parts = self.subscriber.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            if len(parts) != 2 or parts[0] != b"f1_imu":
                self.converter.counts["invalid"] += 1
                continue
            try:
                sample = self.converter.convert(parts[1], self.get_clock().now().nanoseconds)
            except ClockDiscontinuity as error:
                self.get_logger().error(str(error))
                continue
            if sample is not None:
                self.publisher.publish(to_ros_message(sample, self.frame_id))

    def report(self):
        now = self.get_clock().now()
        last_receive = self.converter.last_receive_ns
        age = None if last_receive is None else (now.nanoseconds - last_receive) / 1e9
        status = DiagnosticStatus()
        status.name = "f1_imu_bridge/input"
        status.hardware_id = "F1 via nano-bridge"
        status.level = DiagnosticStatus.OK
        status.message = "receiving IMU"
        if self.converter.clock_fault:
            status.level = DiagnosticStatus.ERROR
            status.message = self.converter.error
        elif age is None or age > self.converter.max_age_ns / 1e9 or self.converter.error:
            status.level = DiagnosticStatus.WARN
            status.message = self.converter.error or "gateway IMU data is stale"
        now_monotonic = time.monotonic()
        count = self.converter.counts["published"]
        rate = (count - self.last_count) / max(1e-9, now_monotonic - self.last_diagnostic)
        values = dict(self.converter.counts)
        values.update(publish_hz=round(rate, 2), age_sec=age,
                      required_status_mask=self.converter.required_status_mask,
                      timestamp_method="approximate board/host receive alignment")
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in sorted(values.items())]
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [status]
        self.diagnostics.publish(message)
        self.last_count, self.last_diagnostic = count, now_monotonic

    def destroy_node(self):
        self.subscriber.close(linger=0)
        self.context_zmq.term()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
