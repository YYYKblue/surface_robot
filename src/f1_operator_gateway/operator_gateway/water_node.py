"""Poll configured water probes independently of the operator control process."""
from collections import Counter
from math import nan

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from f1_interfaces.msg import WaterQualityReading
from .water_quality import WaterBus, REGISTER_COUNTS


class WaterQualityNode(Node):
    def __init__(self):
        super().__init__("water_quality")
        self.publisher = self.create_publisher(WaterQualityReading, "/f1/water_quality", 8)
        self.ec_k = self.declare_parameter("ec_k", 0).value
        self.turbidity_range = self.declare_parameter("turbidity_range_ntu", 0).value
        self.bus = WaterBus(self.declare_parameter("port", "").value,
                            self.declare_parameter("baudrate", 4800).value)
        self.addresses = {sensor: self.declare_parameter(f"{sensor}_address", 0).value
                          for sensor in REGISTER_COUNTS}
        self.address_counts = Counter(self.addresses.values())
        self.auto_addresses = [addr for addr, count in self.address_counts.items()
                               if count == 1 and 1 <= addr <= 254]
        self.valid_baud = self.bus.baudrate in (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)
        self.empty_rounds = 0
        self.create_timer(1., self.poll)

    def poll(self):
        if self.bus.path == "auto" and self.valid_baud and self.bus.port is None:
            self.bus.discover(self.auto_addresses)
            if self.bus.port is not None:
                self.get_logger().info(f"Verified water bus on {self.bus.port.port}")
        received = False
        for sensor, address in self.addresses.items():
            if not self.bus.path or address == 0:
                status, detail, values = "unconfigured", "", {}
            elif (self.address_counts[address] > 1 or not 1 <= address <= 254 or
                  not self.valid_baud or (self.bus.path == "auto" and len(self.auto_addresses) < 2)):
                status, detail, values = "config_error", "duplicate/invalid address or invalid baudrate", {}
            else:
                status, detail, values = self.bus.poll(sensor, address, self.ec_k, self.turbidity_range)
                received |= status in ("ok", "range_required")
            msg = WaterQualityReading(stamp=self.get_clock().now().to_msg(), sensor=sensor,
                                      status=status, detail=detail, value=nan, temperature_c=nan,
                                      salinity_ppm=nan, tds_ppm=nan)
            for key, value in values.items():
                setattr(msg, key, value)
            self.publisher.publish(msg)
        if self.bus.path == "auto":
            self.empty_rounds = self.empty_rounds + 1 if self.bus.port is not None and not received else 0
            if self.empty_rounds >= 2:
                self.bus.close()
                self.empty_rounds = 0

    def destroy_node(self):
        self.bus.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = WaterQualityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
