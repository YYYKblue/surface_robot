"""Single-owner bridge output. No ROS endpoints are created in preview mode."""
import math
import secrets
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl
from f1_wall_cleaning.ros_support import stamp_ns


class Lease:
    def __init__(self, node):
        self.node = node
        self.token = self.owner = self.connection = ""
        self.epoch = self.granted_ns = self.sequence = self.generation = 0
        self.pending = self.bridge = None
        self.pending_at = self.received = 0.
        self.publisher = node.create_publisher(ActuatorCommand, "/f1/actuator_command",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.acquire = node.create_client(AcquireControl, "/f1/acquire_control")
        self.release = node.create_client(ReleaseControl, "/f1/release_control")
        self.subscription = node.create_subscription(ControlState, "/f1/control_state", self._bridge, 5)

    def _bridge(self, msg):
        self.bridge, self.received = msg, time.monotonic()
        if self.token or self.pending is not None:
            error = self.error(acquiring=self.pending is not None)
            if error:
                self.node._halt(error)

    def error(self, acquiring=False):
        b = self.bridge
        age = time.monotonic() - self.received
        now = self.node.get_clock().now().nanoseconds
        if (b is None or age >= .3 or not b.connected or not 0 <= now - stamp_ns(b.stamp) < 300_000_000
                or not math.isfinite(b.ctrl_age_sec) or not math.isfinite(b.ack_age_sec)
                or not 0 <= b.ctrl_age_sec + age < .4 or not 0 <= b.ack_age_sec + age < .4):
            return "bridge or CTRL/ACK stale"
        if b.source != "NANO" or not b.armed or b.result != "OK":
            return "Nano does not have armed control (possibly RC takeover)"
        if not b.allow_fifth_thruster:
            return "bridge allow_fifth_thruster is false"
        if acquiring:
            ready = b.state == "READY" and not b.motion_authorized
            granted = self.pending is not None and b.motion_authorized and b.owner == self.owner
            return "" if ready or granted else "bridge is not READY"
        if b.connection_id != self.connection or b.epoch != self.epoch:
            return "bridge connection/epoch changed"
        if b.motion_authorized and b.owner != self.owner:
            return "another owner holds bridge control"
        if not b.motion_authorized and stamp_ns(b.stamp) > self.granted_ns:
            return "bridge revoked lease"
        return ""

    def begin(self):
        error = self.error(acquiring=True)
        if error or not self.acquire.service_is_ready():
            return False, error or "bridge acquire service unavailable"
        self.owner = "rectangle:" + secrets.token_hex(12)
        self.generation += 1
        generation = self.generation
        connection, epoch = self.bridge.connection_id, self.bridge.epoch
        self.node.mission.halt("waiting for bridge lease", "ACQUIRING")
        self.pending_at = time.monotonic()
        self.pending = self.acquire.call_async(AcquireControl.Request(owner=self.owner))
        self.pending.add_done_callback(lambda f: self._acquired(f, generation, connection, epoch))
        return True, "acquire requested; inspect status for actual start"

    def _release(self, token):
        if token and self.release.service_is_ready():
            self.release.call_async(ReleaseControl.Request(token=token))

    def _acquired(self, future, generation, connection, epoch):
        try:
            reply = future.result()
        except Exception as error:
            if generation == self.generation:
                self.node._halt(f"acquire failed: {error}")
            return
        if generation != self.generation:
            self._release(reply.token)
            return
        error = self.error(acquiring=True)
        self.pending = None
        if (not reply.success or not reply.token or error or time.monotonic() - self.pending_at >= .10
                or reply.epoch != epoch or self.bridge.connection_id != connection):
            self._release(reply.token)
            self.node._halt(error or reply.message or "acquire delayed or epoch changed")
            return
        now = self.node.get_clock().now().nanoseconds
        error = self.node._stream_error()
        # Revalidate observations after async acquisition; never reuse pre-lease effort.
        self.node.mission.halt("lease acquired", "IDLE")
        ok, reason = (False, error) if error else self.node.mission.start(self.node._feedback(), now)
        if not ok:
            self._release(reply.token)
            self.node._halt(reason)
            return
        self.token, self.connection, self.epoch = reply.token, connection, epoch
        self.granted_ns, self.sequence = now, 0
        self.node._tick()

    def stop(self):
        self.generation += 1
        self.pending = None
        token, self.token = self.token, ""
        self._release(token)

    def tick_error(self):
        if self.pending is not None and time.monotonic() - self.pending_at >= .5:
            return "acquire response timeout"
        if self.token or self.pending is not None:
            return self.error(acquiring=self.pending is not None)
        return ""

    def publish(self, msg):
        if self.token:
            msg.token, msg.sequence = self.token, self.sequence
            self.sequence += 1
            self.publisher.publish(msg)
