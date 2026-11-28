"""Receive WT102 frames on a dedicated UART. All stamps use the Nano ROS clock."""
import secrets
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial

from f1_interfaces.msg import Wt102Sample
from .protocol import WitParser, decode_yaw


class Wt102Uart(Node):
    def __init__(self):
        super().__init__('wt102_uart')
        self.device = self.declare_parameter(
            'port', '/dev/ttyTHS1', ParameterDescriptor(read_only=True)).value
        self.pub = self.create_publisher(Wt102Sample, '/f1/wt102/sample', qos_profile_sensor_data)
        self.port = None
        self.retry_at = 0.
        self.last_poll = None
        self.last_stamp = None
        self.last_rate = 0.
        self.start_stream()
        self.create_timer(.002, self.poll)

    def start_stream(self):
        self.parser = WitParser()
        self.stream_id = secrets.randbits(64)
        self.last_stamp = None

    def poll(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now < self.retry_at:
                    return
                self.port = serial.Serial(self.device, 115200, timeout=0, exclusive=True)
                self.port.reset_input_buffer()
                self.start_stream()
                self.last_rate = now
                self.get_logger().info(f'Opened {self.device}, 115200 8N1')
            elif now - self.last_poll > .05:
                # Bytes accumulated during a stall have no hardware timestamps.
                self.port.reset_input_buffer()
                self.start_stream()
                self.get_logger().warning('UART polling gap, discarded buffered data',
                                          throttle_duration_sec=5.)
            self.last_poll = now
            chunk = self.port.read(self.port.in_waiting)
        except (OSError, serial.SerialException) as exc:
            self.close_port()
            self.retry_at = now + 2.
            self.get_logger().warning(f'WT102 UART unavailable: {exc}', throttle_duration_sec=5.)
            return

        if now - self.last_rate > 2.:
            self.get_logger().warning('No valid WT102 Z rate in 2 s', throttle_duration_sec=5.)
        if not chunk:
            return
        stamp = self.get_clock().now()
        if self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp:
            self.start_stream()
        self.last_stamp = stamp.nanoseconds
        # Multiple frames in one host read have no independent acquisition times.
        # Publish only the freshest value of each type at this receipt stamp.
        latest = {}
        for frame in self.parser.feed(chunk):
            decoded = decode_yaw(frame)
            if decoded is not None:
                kind, value = decoded
                latest[kind] = value
        for kind, value in latest.items():
            sample = Wt102Sample()
            sample.header.stamp = stamp.to_msg()
            sample.header.frame_id = 'wt102_link'
            sample.stream_id = self.stream_id
            sample.kind = sample.YAW_RATE if kind == 'yaw_rate' else sample.RELATIVE_YAW
            sample.value = value
            self.pub.publish(sample)
            if kind == 'yaw_rate':
                self.last_rate = now

    def close_port(self):
        if self.port is not None:
            try:
                self.port.close()
            except OSError as exc:
                self.get_logger().warning(f'WT102 UART close failed: {exc}')
            finally:
                self.port = None


def main():
    rclpy.init()
    node = Wt102Uart()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_port()
        node.destroy_node()
        rclpy.try_shutdown()
