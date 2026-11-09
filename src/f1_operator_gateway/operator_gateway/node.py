"""ROS 2 gateway for the thin Qt operator app.

The Windows client sends only current intent. This node owns the ROS service
interaction, command deadline, latest-input watchdog and telemetry shaping.
"""
import math
import time
from dataclasses import replace

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from f1_interfaces.msg import ActuatorCommand, ControlState
from f1_interfaces.srv import AcquireControl, ReleaseControl

from .protocol import ManualInput, decode
from .task_control import TaskControl
from .websocket_hub import WebSocketHub
from .runtime import RuntimeControl
from .diagnostics import DiagnosticHistory
from .map_stream import LatestMapEncoder


class OperatorGateway(RuntimeControl, TaskControl, Node):
    TTL_NS = 250_000_000

    def __init__(self):
        super().__init__("f1_operator_gateway")
        self.declare_parameter("bind_address", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("input_timeout_sec", .25)
        self.declare_parameter("telemetry_hz", 10.0)
        self.declare_parameter("map_update_period_sec", 2.0)
        address = self.get_parameter("bind_address").value
        port = int(self.get_parameter("port").value)
        if not isinstance(address, str) or not address or any(c in address for c in ":/; \t\n"):
            raise ValueError("bind_address must be an IPv4 address or hostname")
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)
        if not .1 <= self.input_timeout <= 1.0:
            raise ValueError("input_timeout_sec must be between .1 and 1.0")
        self.publisher = self.create_publisher(
            ActuatorCommand, "/f1/actuator_command",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self.acquire_client = self.create_client(AcquireControl, "/f1/acquire_control")
        self.release_client = self.create_client(ReleaseControl, "/f1/release_control")
        self._init_tasks()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._init_runtime()
        self.control_state = {}
        self.diagnostic_history = DiagnosticHistory(self._diagnostic_event)
        self.telemetry_sequence = 0
        self._last_tracks = None
        self.control_received = 0.
        self.manual_generation = 0
        self._manual_acquires = 0
        self.manual_releases = []
        self.map_data = None
        self.map_signature = None
        self.map_revision = 0
        self.pose = None
        self.active_client = None
        self.last_input = None
        self.last_input_at = None
        self.last_app_seq = -1
        self.token = ""
        self.pending_acquire = False
        self.bridge_sequence = 0
        self.hub = WebSocketHub(address, port, None)
        self.hub.start()
        self.map_stream = LatestMapEncoder(float(self.get_parameter("map_update_period_sec").value))
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ControlState, "/f1/control_state", self._control, state_qos)
        self.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self._scan, state_qos)
        self.tick_timer = self.create_timer(.02, self._tick)
        self.telemetry_timer = self.create_timer(1.0 / max(.5, float(self.get_parameter("telemetry_hz").value)), self._telemetry)
        self.tracks_timer = self.create_timer(.5, self._publish_tracks)
        self.get_logger().info(f"f1 operator gateway listening on ws://{address}:{port}")

    def _control(self, message):
        state = {}
        for name in message.get_fields_and_field_types():
            value = getattr(message, name)
            if name == "stamp":
                value = {"sec": value.sec, "nanosec": value.nanosec}
            elif name == "target_pwm_bl_br_fl_fr":
                value = [int(item) for item in value]
            state[name] = value
        self.control_state = state
        self.control_received = time.monotonic()
        self.diagnostic_history.bridge(state)

    def _map(self, message):
        if self.runtime_enabled and (not self.runtime_accept_map or
                message.header.stamp.sec * 10**9 + message.header.stamp.nanosec < self.runtime_epoch_ns):
            return
        self.map_stream.offer(message, frozen=(self.runtime_enabled and self.runtime["mode"] == "localization"))

    def _publish_map(self):
        mapping = self.map_stream.take()
        if mapping is None or (self.runtime_enabled and not self.runtime_accept_map):
            return
        self.map_revision += 1
        runtime_session = self.runtime["session_id"] if self.runtime_enabled else ""
        self.map_data = dict(mapping, revision=self.map_revision, runtime_session_id=runtime_session)
        self.hub.broadcast(self._map_packet())

    def _map_packet(self):
        return dict(type="map", map=self.map_data,
                    runtime_session_id=self.map_data.get("runtime_session_id", ""))

    def _scan(self, message):
        self._runtime_scan(message)

    def _tick(self):
        now = time.monotonic()
        previous = getattr(self, "_last_tick_at", now)
        self._tick_gap_ms = max(getattr(self, "_tick_gap_ms", 0.), (now - previous) * 1000)
        self._last_tick_at = now
        for event in self.hub.poll():
            self._handle_event(event)
        self._read_pose()
        self._task_tick()
        self._runtime_tick()
        self._publish_map()
        if self.task_locks and (self.token or self.last_input is not None):
            self._stop_manual("automatic_task_active")
        if self.active_client and self.last_input_at is not None:
            if time.monotonic() - self.last_input_at >= self.input_timeout:
                self._stop_manual("input_timeout")
            elif self.last_input is not None:
                if not self.token:
                    if self._needs_control(self.last_input):
                        self._acquire_if_ready()
                elif self.last_input.sequence > self.last_app_seq:
                    self._publish(self.last_input)
                    self.last_app_seq = self.last_input.sequence

    def _handle_event(self, event):
        kind, client_id = event[0], event[1]
        if kind == "connected":
            return
        if kind == "disconnected":
            if client_id == self.active_client:
                self._stop_manual("app_disconnected")
                self.active_client = None
                pending = self.command_pending
                if pending and pending["stage"] == "handover":
                    self._finish_operation(pending, False, "操作端已断开，未发送操作")
            return
        try:
            packet = decode(event[2])
        except (TypeError, ValueError, UnicodeError, RecursionError) as error:
            self.hub.send(client_id, {"type": "error", "message": str(error)})
            return
        if packet["op"] == "hello":
            self.hub.send(client_id, {"type": "hello", "protocol": "f1-operator-v1", "telemetry_hz": 10,
                "session_id": self.diagnostic_history.session,
                "client_id": client_id, "capabilities": ["task_selection", "initial_pose", "start_conditions", "task_stop", "task_tracks", "diagnostic_history", "forward_boost", "compact_stream_v1", "indexed8_map"]
                    + (["mapping", "map_catalog"] if self.runtime_enabled else [])})
            if self.map_data:
                self.hub.send(client_id, self._map_packet())
            self.hub.send(client_id, {"type": "diagnostics", "diagnostics": self.diagnostic_history.view()})
            self.hub.send(client_id, self._tracks_packet())
            return
        if packet["op"] in ("manual_input", "manual_stop", "task", "initial_pose", "mapping") and not (
                packet["op"] == "mapping" and packet["action"] in ("list", "preview")):
            if self.active_client not in (None, client_id):
                self.hub.send(client_id, {"type": "reply", "op": packet["op"],
                                         "request_id": packet.get("request_id", ""),
                                         "success": False, "message": "已有其他操作端连接"})
                return
            self.active_client = client_id
        if packet["op"] == "manual_stop":
            self._stop_manual(packet["reason"])
            self.hub.send(client_id, {"type": "reply", "op": "manual_stop", "success": True})
        elif packet["op"] == "manual_input":
            self._input(client_id, packet["input"])
        elif packet["op"] in ("task", "initial_pose"):
            self._operation_request(client_id, packet)
        elif packet["op"] == "mapping":
            self._runtime_request(client_id, packet)

    def _input(self, client_id, value: ManualInput):
        if self._tasks_busy():
            if self._needs_control(value):
                self.hub.send(client_id, {"type": "reply", "op": "manual_input", "success": False,
                                           "message": "自动任务运行或操作处理中，手动输入已拒绝"})
            return
        if value.sequence <= self.last_app_seq:
            return
        if value.thrust_enabled and self.control_state.get("allow_fifth_thruster") is not True:
            self.hub.send(client_id, {
                "type": "reply", "op": "manual_input", "success": False,
                "message": "Nano 未确认第五电推已启用（allow_fifth_thruster=true）",
            })
            # Do not forward an unsupported fifth-motor field. Keep the
            # chassis command alive so the rest of the controls still work.
            value = replace(value, thrust_enabled=False, thrust_percent=0)
        self.last_input, self.last_input_at = value, time.monotonic()
        if not self.token and self._needs_control(value):
            self._acquire_if_ready()

    @staticmethod
    def _needs_control(value: ManualInput):
        return value.direction != "STOP" or value.brush_enabled or value.thrust_enabled

    def _fresh_ready(self):
        state = self.control_state
        age = time.monotonic() - self.control_received
        stamp = state.get("stamp", {})
        stamp_ns = stamp.get("sec", 0)*10**9 + stamp.get("nanosec", 0)
        return (age < .3 and 0 <= self.get_clock().now().nanoseconds-stamp_ns < 300_000_000
                and 0 <= state.get("ctrl_age_sec", float("inf")) + age < .4
                and 0 <= state.get("ack_age_sec", float("inf")) + age < .4
                and state.get("connected") is True and state.get("source") == "NANO"
                and state.get("armed") is True and state.get("result") == "OK"
                and state.get("state") in ("READY", "ACTIVE"))

    def _acquire_if_ready(self):
        if self.pending_acquire or self.token or not self._fresh_ready():
            return
        if not self.acquire_client.service_is_ready():
            return
        self.pending_acquire = True
        self._manual_acquires += 1
        generation = self.manual_generation
        request = AcquireControl.Request(owner="operator_app")
        try:
            future = self.acquire_client.call_async(request)
        except Exception as error:
            self._manual_acquires -= 1
            self.pending_acquire = False
            self.get_logger().warning(f"manual acquire could not be sent: {error}")
            return

        def complete(done):
            self._manual_acquires -= 1
            if generation == self.manual_generation:
                self.pending_acquire = False
            try:
                result = done.result()
            except Exception as error:
                self.get_logger().warning(f"automatic manual session failed: {error}")
                return
            if generation == self.manual_generation and not self._tasks_busy() and result.success and self.active_client and self.last_input_at is not None \
                    and time.monotonic() - self.last_input_at < self.input_timeout:
                self.token = result.token
                self.bridge_sequence = 0
                self.last_app_seq = -1
                self._publish(self.last_input)
                self.last_app_seq = self.last_input.sequence
            else:
                if result.token:
                    self._release_token(result.token)

        future.add_done_callback(complete)

    def _publish(self, value: ManualInput):
        if not self.token or value is None:
            return
        now = self.get_clock().now().nanoseconds
        message = ActuatorCommand()
        message.token = self.token
        message.sequence = self.bridge_sequence
        self.bridge_sequence += 1
        message.direction = {"STOP": 0, "FORWARD": 1, "REVERSE": 2, "LEFT": 3, "RIGHT": 4}[value.direction]
        message.speed_percent = value.speed_percent
        message.mode = {"FRONT": 0, "BOTH": 1, "BACK": 2}[value.mode]
        message.brush_enabled, message.brush_percent = value.brush_enabled, value.brush_percent
        message.thrust_enabled, message.thrust_percent = value.thrust_enabled, value.thrust_percent
        for field, stamp_ns in (("computed_at", now), ("valid_until", now + self.TTL_NS)):
            stamp = getattr(message, field)
            stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        self.publisher.publish(message)

    def _stop_manual(self, reason):
        # Capture timing before clearing input, including disconnects whose event
        # can only reach the operator after reconnecting.
        if self.token or self.pending_acquire or self.last_input_at is not None:
            self.diagnostic_history.record(
                "gateway", reason,
                input_age_ms=(None if self.last_input_at is None else
                              round((time.monotonic() - self.last_input_at) * 1000, 1)),
                input_timeout_ms=round(self.input_timeout * 1000, 1),
                had_lease=bool(self.token),
                **DiagnosticHistory.snapshot(self.control_state))
        self.manual_generation += 1
        token = self.token
        self.token = ""
        self.pending_acquire = False
        self.last_input = self.last_input_at = None
        self.last_app_seq = -1
        if token:
            self._release_token(token)
        self.get_logger().info(f"manual output stopped: {reason}")

    def _release_token(self, token):
        if not self.release_client.service_is_ready():
            return
        self.manual_releases = [f for f in self.manual_releases if not f.done()]
        self.manual_releases.append(self.release_client.call_async(ReleaseControl.Request(token=token)))

    def _read_pose(self):
        if self.runtime_enabled and not self.runtime_accept_map:
            self.pose = None
            return
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
            self.pose = None
            return
        stamp_ns = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
        if self.runtime_enabled and stamp_ns < self.runtime_epoch_ns:
            self.pose = None
            return
        if not 0 <= self.get_clock().now().nanoseconds - (transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec) < 500_000_000:
            self.pose = None
            return
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = {"x": float(t.x), "y": float(t.y), "yaw": float(yaw),
                     "stamp": {"sec": transform.header.stamp.sec, "nanosec": transform.header.stamp.nanosec}}

    def _diagnostic_event(self, history):
        self.hub.broadcast({"type": "diagnostics", "diagnostics": history})

    def _tracks_packet(self):
        return dict(type="tracks", session_id=self.diagnostic_history.session,
                    tasks={kind: track.snapshot() for kind, track in self.task_tracks.items()})

    def _publish_tracks(self):
        packet = self._tracks_packet()
        if packet["tasks"] != self._last_tracks:
            self._last_tracks = packet["tasks"]
            self.hub.broadcast(packet)

    def _telemetry(self):
        self.telemetry_sequence += 1
        now = time.monotonic()
        stream = dict(session=self.diagnostic_history.session, sequence=self.telemetry_sequence,
                      generated_unix=time.time(), generated_monotonic=now,
                      tick_gap_ms=round(getattr(self, "_tick_gap_ms", 0.), 2),
                      control_age_ms=round(max(0., now - self.control_received) * 1000, 2))
        self._tick_gap_ms = 0.
        if hasattr(self.hub, "stats"):
            stream["transport"] = self.hub.stats(include_clients=False)
        self.hub.broadcast({"type": "telemetry", "state": self.control_state,
                            "stream": stream,
                            "control_fresh": time.monotonic() - self.control_received < .3,
                            "pose": self.pose,
                            "active_client": self.active_client, "runtime": self._runtime_view(), **self._task_telemetry()})

    def destroy_node(self):
        self._stop_manual("gateway_shutdown")
        self.map_stream.close()
        self._close_runtime()
        self.hub.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OperatorGateway()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
