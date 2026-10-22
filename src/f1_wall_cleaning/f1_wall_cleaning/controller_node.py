import secrets
import time

from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_srvs.srv import Trigger
from f1_wall_interfaces.msg import WallObservation, EffortPreview, CleaningStatus
from f1_bridge_interfaces.msg import ActuatorCommand, ControlState
from f1_bridge_interfaces.srv import AcquireControl, ReleaseControl

from .control import Follower
from .ros_support import configuration, from_observation_message, stamp, stamp_ns


class WallController(Node):
    def __init__(self, **kwargs):
        super().__init__("wall_controller", **kwargs)
        self.config = configuration(self)
        self.declare_parameter("output_enabled", False, ParameterDescriptor(read_only=True))
        self.live = self.get_parameter("output_enabled").value
        if self.live and self.get_parameter("use_sim_time").value:
            raise ValueError("hardware output rejects use_sim_time")
        if self.live and self.config.control_error(True):
            raise ValueError(self.config.control_error(True))
        if self.config.supervised_trial:
            self.get_logger().warning("SUPERVISED TRIAL: brush disabled, non-contact wall following; "
                                      "motion response and stopping/sweep margins are uncalibrated")
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.follower = Follower(self.config)
        self.observation = None
        self.observation_session = ""
        self.observation_sequence = -1
        self.observation_received = 0.
        self.bridge = None
        self.bridge_received = 0.
        self.task_id = ""
        self.token = ""
        self.bound_connection = ""
        self.bound_epoch = 0
        self.generation = 0
        self.pending = None
        self.pending_at = 0.
        self.granted_ns = 0
        self.command_sequence = 0
        self.clock_pair = None
        self.clock_fault = False
        self.preview_pub = self.create_publisher(EffortPreview, "/wall_cleaning/effort_preview", 2)
        self.status_pub = self.create_publisher(CleaningStatus, "/wall_cleaning/status", 5)
        latest_observation = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.observation_sub = self.create_subscription(
            WallObservation, "/wall_cleaning/wall_observation", self._observation, latest_observation)
        # Preview mode has no actuator publisher and no bridge service clients.
        self.command_pub = self.acquire_client = self.release_client = None
        if self.live:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.command_pub = self.create_publisher(ActuatorCommand, "/f1/actuator_command", qos)
            self.acquire_client = self.create_client(AcquireControl, "/f1/acquire_control")
            self.release_client = self.create_client(ReleaseControl, "/f1/release_control")
            self.bridge_sub = self.create_subscription(ControlState, "/f1/control_state", self._bridge, 5)
        self.task_services = [
            self.create_service(Trigger, "/wall_cleaning/start", self._start),
            self.create_service(Trigger, "/wall_cleaning/pause", self._pause),
            self.create_service(Trigger, "/wall_cleaning/resume", self._resume),
            self.create_service(Trigger, "/wall_cleaning/cancel", self._cancel),
        ]
        # Monotonic watchdog still runs while /clock is paused during replay.
        self.tick_timer = self.create_timer(.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status_timer = self.create_timer(.1, self._status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _validate_parameters(self, parameters):
        if self.live and any(p.name == "use_sim_time" and p.value for p in parameters):
            return SetParametersResult(successful=False, reason="hardware output requires system time")
        return SetParametersResult(successful=True)

    def _observation(self, message):
        if message.header.frame_id != self.config.base_frame or not message.session:
            if self.follower.active:
                self._halt("PAUSED_SAFETY", "invalid observation frame/session")
            return
        if self.observation_session and message.session != self.observation_session:
            if self.follower.active or self.pending:
                self._halt("PAUSED_SAFETY", "perception restarted or association reset")
            self.observation_sequence = -1
            self.observation = None
        if message.sequence <= self.observation_sequence:
            return
        observation = from_observation_message(message)
        if self.observation and observation.stamp_ns <= self.observation.stamp_ns:
            return
        self.observation_session = message.session
        self.observation_sequence = message.sequence
        self.observation = observation
        self.observation_received = time.monotonic()
        if not observation.valid and (self.follower.active or self.pending):
            self._halt("PAUSED_SAFETY", observation.reason)
        elif self.follower.active:
            # Use fresh observations immediately. Waiting for the 50 ms timer
            # can consume a whole command period when scan arrival and timer
            # callbacks coincide, including preview mode. The timer still watches for stopped streams;
            # Follower.update never renews a command from a repeated scan.
            self._tick()

    def _bridge(self, message):
        self.bridge, self.bridge_received = message, time.monotonic()
        if (self.token or self.pending) and message.source == "RC":
            self._halt("PAUSED_RC", "RC takeover; explicit resume required after return")
        elif self.token and (message.connection_id != self.bound_connection or message.epoch != self.bound_epoch):
            self._halt("PAUSED_SAFETY", "bridge connection/epoch changed")

    def _bridge_error(self, acquiring=False):
        b = self.bridge
        age = time.monotonic() - self.bridge_received
        now = self.get_clock().now().nanoseconds
        if (b is None or age >= .3 or not b.connected or not 0 <= now - stamp_ns(b.stamp) < 300_000_000
                or not 0 <= b.ctrl_age_sec + age < .4 or not 0 <= b.ack_age_sec + age < .4):
            return "bridge or CTRL/ACK stale"
        if b.source != "NANO" or not b.armed or b.result != "OK":
            return "Nano does not have armed control"
        if acquiring:
            ready = b.state == "READY" and not b.motion_authorized
            granted = self.follower.phase == "ACQUIRING" and b.motion_authorized and b.owner == self.task_id
            return "" if ready or granted else "bridge is not READY"
        if b.connection_id != self.bound_connection or b.epoch != self.bound_epoch:
            return "bridge connection/epoch changed"
        if b.motion_authorized and b.owner != self.task_id:
            return "another owner holds bridge control"
        if not b.motion_authorized and stamp_ns(b.stamp) > self.granted_ns:
            return "bridge revoked lease"
        return ""

    def _release_token(self, token):
        if token and self.release_client is not None and self.release_client.service_is_ready():
            self.release_client.call_async(ReleaseControl.Request(token=token))
        # If service unavailable, no more commands; bridge's original deadline expires.

    def _halt(self, phase, reason):
        self.generation += 1
        self.pending = None
        old, self.token = self.token, ""
        self.follower.halt(phase, reason)
        self._release_token(old)
        self._publish_preview(None)

    def _begin(self, response, resume=False):
        if self.clock_fault:
            response.success, response.message = False, "clock fault latched; restart controller"
            return response
        allowed = self.follower.phase.startswith("PAUSED") if resume else self.follower.phase in ("IDLE", "CANCELLED")
        error = self.config.control_error(self.live) or self.follower.observation_error(
            self.observation, self.get_clock().now().nanoseconds)
        if not allowed or self.pending:
            error = "task state does not permit this start/resume"
        if self.observation_received and time.monotonic() - self.observation_received >= self.config.scan_max_age_sec:
            error = "observation receive stream stale"
        if self.live and not error:
            error = self._bridge_error(acquiring=True)
        if error:
            response.success, response.message = False, error
            return response
        if not resume:
            self.task_id = "wall:" + secrets.token_hex(12)
            self.follower.brush_active_sec = 0.
        if not self.live:
            self.follower.start(self.observation, self.get_clock().now().nanoseconds)
            response.success, response.message = True, "preview started; hardware output disabled"
            return response
        if not self.acquire_client.service_is_ready():
            response.success, response.message = False, "bridge acquire service unavailable"
            return response
        self.generation += 1
        generation = self.generation
        connection, epoch = self.bridge.connection_id, self.bridge.epoch
        self.follower.halt("ACQUIRING", "waiting for fresh bridge lease")
        self.pending_at = time.monotonic()
        self.pending = self.acquire_client.call_async(AcquireControl.Request(owner=self.task_id))
        self.pending.add_done_callback(lambda f: self._acquired(f, generation, connection, epoch))
        response.success, response.message = True, "acquire requested; inspect status for actual start"
        return response

    def _acquired(self, future, generation, connection, epoch):
        try:
            reply = future.result()
        except Exception as error:
            if generation == self.generation:
                self._halt("PAUSED_SAFETY", f"acquire failed: {error}")
            return
        if generation != self.generation:
            self._release_token(reply.token)
            return
        self.pending = None
        if not reply.success:
            self._halt("PAUSED_SAFETY", reply.message)
            return
        error = self._bridge_error(acquiring=True)
        if (time.monotonic() - self.pending_at >= .10 or reply.epoch != epoch
                or self.bridge.connection_id != connection or error):
            self._release_token(reply.token)
            self._halt("PAUSED_SAFETY", error or "acquire delayed or epoch changed")
            return
        now = self.get_clock().now().nanoseconds
        if not self.follower.start(self.observation, now, live=True):
            self._release_token(reply.token)
            self._halt("PAUSED_SAFETY", self.follower.reason)
            return
        self.token, self.bound_connection, self.bound_epoch = reply.token, connection, epoch
        self.granted_ns, self.command_sequence = now, 0
        self._tick()  # Do not spend the first-command deadline waiting for a timer.

    def _start(self, request, response):
        return self._begin(response)

    def _resume(self, request, response):
        return self._begin(response, resume=True)

    def _pause(self, request, response):
        self._halt("PAUSED_USER", "user pause")
        response.success, response.message = True, "paused; lease revoked, no automatic resume"
        return response

    def _cancel(self, request, response):
        self._halt("CANCELLED", "user cancel; no complete-pool coverage claim")
        response.success, response.message = True, "cancelled"
        return response

    def _tick(self):
        now, mono = self.get_clock().now().nanoseconds, time.monotonic()
        if self.clock_pair:
            old_ros, old_mono = self.clock_pair
            if now < old_ros or (self.live and abs((now - old_ros) / 1e9 - (mono - old_mono)) > .05):
                self.clock_fault = self.live
                self._halt("PAUSED_SAFETY", "clock discontinuity; reset task")
        self.clock_pair = now, mono
        if self.pending and mono - self.pending_at >= .5:
            self._halt("PAUSED_SAFETY", "acquire response timeout")
        if not self.follower.active:
            return
        error = ""
        if mono - self.observation_received >= self.config.scan_max_age_sec:
            error = "observation receive stream stopped"
        if self.live:
            error = error or self._bridge_error()
        if error:
            self._halt("PAUSED_RC" if self.bridge and self.bridge.source == "RC" else "PAUSED_SAFETY", error)
            return
        effort = self.follower.update(self.observation, now)
        if not self.follower.active:
            self._halt(self.follower.phase, self.follower.reason)
        elif effort:
            self._publish_preview(effort)
            if self.live and self.token:
                message = ActuatorCommand()
                message.token, message.sequence = self.token, self.command_sequence
                self.command_sequence += 1
                message.computed_at, message.valid_until = stamp(effort.computed_ns), stamp(effort.deadline_ns)
                message.direction_until = stamp(effort.direction_until_ns)
                message.direction, message.speed_percent = effort.direction, effort.speed_percent
                message.mode = ActuatorCommand.BOTH
                message.brush_enabled, message.brush_percent = effort.brush_enabled, effort.brush_percent
                message.thrust_enabled, message.thrust_percent = False, 0.
                self.command_pub.publish(message)

    def _publish_preview(self, effort):
        message = EffortPreview()
        message.header.frame_id = self.config.base_frame
        message.header.stamp = stamp(effort.computed_ns if effort else self.get_clock().now().nanoseconds)
        message.phase, message.reason = self.follower.phase, self.follower.reason
        message.valid, message.hardware_output_enabled = effort is not None, self.live
        if effort:
            message.observation_stamp, message.valid_until = stamp(effort.observation_ns), stamp(effort.deadline_ns)
            message.direction_until = stamp(effort.direction_until_ns)
            message.direction, message.speed_percent = effort.direction, effort.speed_percent
            message.brush_enabled, message.brush_percent = effort.brush_enabled, effort.brush_percent
        self.preview_pub.publish(message)

    def _status(self):
        message = CleaningStatus()
        now = self.get_clock().now().nanoseconds
        message.header.stamp, message.header.frame_id = stamp(now), self.config.base_frame
        message.task_id, message.phase = self.task_id, self.follower.phase
        message.reason = self.config.control_error(self.live) or self.follower.reason
        if self.config.supervised_trial:
            message.reason = "supervised trial; uncalibrated response; brush off: " + message.reason
        message.output_enabled = self.live
        message.configuration_ready = not self.config.control_error(self.live)
        message.lease_held, message.cleaning_side = bool(self.token), "left"
        message.observation_age_sec = -1. if self.observation is None else (now - self.observation.stamp_ns) / 1e9
        message.brush_active_sec = self.follower.brush_active_sec
        message.speed_feedback_available, message.coverage_available = False, False
        self.status_pub.publish(message)

    def destroy_node(self):
        if hasattr(self, "follower"):
            self.generation += 1
            self.pending = None
            token, self.token = self.token, ""
            self.follower.halt("CANCELLED", "controller shutdown")
            if self.context.ok():
                self._release_token(token)
        return super().destroy_node()


def main(args=None):
    from .runner import run
    run(WallController, args)
