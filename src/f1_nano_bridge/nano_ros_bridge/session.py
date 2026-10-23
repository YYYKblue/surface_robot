"""Clock-injected session/lease state machine. Caller serializes access."""
from dataclasses import dataclass, replace
import secrets

from .protocol import U32, channels, encode, forward, neutral


@dataclass(frozen=True)
class Command:
    token: str
    sequence: int
    computed_ns: int
    deadline_ns: int
    direction: int = 0
    speed_percent: int = 25
    mode: int = 1
    brush_enabled: bool = False
    brush_percent: float = 0
    thrust_enabled: bool = False
    thrust_percent: float = 0
    direction_until_ns: int = 0

    def channels(self):
        return channels(self.direction, self.speed_percent, self.mode, self.brush_enabled,
                        self.brush_percent, self.thrust_enabled, self.thrust_percent)


class Session:
    TTL = 0.150
    CTRL_TIMEOUT = 0.400
    ACK_TIMEOUT = 0.400
    MAX_SEND_GAP = 0.080

    def __init__(self, motion_enabled=False, allow_fifth=False):
        self.motion_enabled = motion_enabled
        self.allow_fifth = allow_fifth
        self.connected = False
        self.connection_id = ""
        self.ctrl = None
        self.ctrl_at = None
        self.ack_at = None
        self.last_ack = None
        self.wire_seq = 0
        self.last_sent = None
        self.last_tx_at = None
        self.barrier = None
        self.ready = False
        self.token = ""
        self.owner = ""
        self.command = None
        self.expires = 0.
        self.command_seq = -1
        self.reason = "serial disabled"
        self.last_stop_reason = ""
        self.fault = ""
        self.rejected = 0
        self.tx_frames = 0
        self.max_tx_gap = 0.
        self.reconnects = 0
        self.last_command_at = None
        self.last_command_age_ms = 0.
        self.last_command_remaining_ms = 0.
        self.max_command_gap_ms = 0.
        self.last_deadline_overrun_ms = 0.
        self.last_command_rejection = ""

    def stop(self, reason):
        if self.token:
            self.last_stop_reason = reason
        self.token = self.owner = ""
        self.command = None
        self.command_seq = -1
        self.expires = 0.
        self.ready = False
        self.barrier = None
        self.reason = reason

    def opened(self):
        self.stop("waiting for CTRL")
        self.connected = True
        self.connection_id = secrets.token_hex(16)
        self.ctrl = None
        self.ctrl_at = self.ack_at = self.last_ack = None
        self.last_sent = self.last_tx_at = None
        self.wire_seq = 0
        self.reconnects += 1

    def closed(self, reason):
        self.stop(reason)
        self.connected = False

    def latch_fault(self, reason):
        self.fault = reason
        self.stop(reason)

    def receive(self, ctrl, now):
        if not self.connected:
            return
        old = self.ctrl
        changed = old is None or (ctrl.source, ctrl.epoch) != (old.source, old.epoch)
        reboot = old is not None and ctrl.milliseconds != old.milliseconds and not forward(ctrl.milliseconds, old.milliseconds)
        if old is not None and not changed and not reboot and ctrl.milliseconds == old.milliseconds:
            return  # Duplicate telemetry cannot refresh health.
        if changed or reboot:
            self.stop("source/epoch changed" if not reboot else "device clock reset")
            self.last_ack = None
            self.ack_at = None
            self.last_sent = None
            self.wire_seq = 0
        self.ctrl = ctrl
        self.ctrl_at = now
        if ctrl.source != "NANO":
            self.reason = "RC has control; Nano stop cannot override RC"
            return
        if ctrl.have_sequence and self.last_sent is not None:
            if forward(ctrl.ack, self.last_sent):
                self.stop("ACK ahead of transmitted sequence")
                return
            if self.last_ack is None or forward(ctrl.ack, self.last_ack):
                self.last_ack = ctrl.ack
                self.ack_at = now
        if self.token and (not ctrl.armed or ctrl.result != "OK"):
            self.stop("firmware no longer armed/OK")
        if not self.token and (not ctrl.armed or ctrl.result != "OK" or not ctrl.neutral):
            self.ready = False
        barrier_acked = (self.barrier is not None and ctrl.have_sequence
                         and self.last_ack is not None
                         and (ctrl.ack == self.barrier or forward(ctrl.ack, self.barrier)))
        if (not self.token and barrier_acked and ctrl.armed and ctrl.result == "OK"
                and ctrl.neutral and not self.fault):
            self.ready = True
            self.reason = "neutral acknowledged; waiting for explicit acquire"

    def health(self, now):
        if not self.connected:
            return False
        if self.ctrl_at is None or now - self.ctrl_at >= self.CTRL_TIMEOUT:
            if self.token or self.ready or self.barrier is not None:
                self.stop("CTRL timeout")
            return False
        if self.ctrl.source != "NANO":
            return False
        if self.ack_at is not None and now - self.ack_at >= self.ACK_TIMEOUT:
            if self.token or self.ready:
                self.stop("ACK progress timeout")
        if self.token and now >= self.expires:
            self.last_deadline_overrun_ms = (now - self.expires) * 1000
            self.stop("command deadline expired")
        return True

    def acquire(self, owner, now):
        self.health(now)
        if not self.motion_enabled:
            return "", "motion_enabled is false"
        if (not self.motion_enabled or self.fault or not self.ready or self.token
                or not isinstance(owner, str) or not 1 <= len(owner.strip()) <= 128):
            return "", self.fault or self.reason or "control unavailable"
        self.token = secrets.token_hex(24)
        self.owner = owner
        self.last_stop_reason = ""
        self.last_command_at = None
        self.last_command_age_ms = self.last_command_remaining_ms = 0.
        self.max_command_gap_ms = self.last_deadline_overrun_ms = 0.
        self.last_command_rejection = ""
        self.expires = now + self.TTL
        self.command_seq = -1
        self.reason = "lease acquired; waiting for first command"
        return self.token, "acquired"

    def release(self, token):
        if not token or token != self.token:
            return False
        self.stop("owner released control")
        return True

    def accept(self, cmd, now, unix_ns):
        self.health(now)
        if not self.token or cmd.token != self.token or cmd.sequence <= self.command_seq:
            self.rejected += 1
            self.last_command_rejection = "no matching lease or non-increasing sequence"
            return False
        try:
            cmd.channels()
            if cmd.thrust_enabled and not self.allow_fifth:
                raise ValueError("fifth thruster is disabled by configuration")
            lifetime = cmd.deadline_ns - cmd.computed_ns
            if not (0 < lifetime <= round(self.TTL * 1e9)
                    and cmd.computed_ns <= unix_ns + 5_000_000
                    and cmd.deadline_ns > unix_ns):
                raise ValueError("invalid or expired original command deadline")
            if cmd.direction_until_ns and not (cmd.computed_ns < cmd.direction_until_ns
                    <= cmd.computed_ns + 500_000_000):
                raise ValueError("invalid direction pulse deadline")
        except (ValueError, TypeError, OverflowError) as error:
            self.rejected += 1
            self.last_command_rejection = str(error)
            self.stop(str(error))
            return False
        if self.last_command_at is not None:
            self.max_command_gap_ms = max(self.max_command_gap_ms, (now - self.last_command_at) * 1000)
        self.last_command_at = now
        self.last_command_age_ms = (unix_ns - cmd.computed_ns) / 1e6
        self.last_command_remaining_ms = (cmd.deadline_ns - unix_ns) / 1e6
        self.command = cmd
        self.command_seq = cmd.sequence
        # Conversion happens only once, never on the 50 Hz resend path.
        self.expires = now + min(self.TTL, (cmd.deadline_ns - unix_ns) / 1e9)
        self.action_expires = (now + (cmd.direction_until_ns - unix_ns) / 1e9
                               if cmd.direction_until_ns else self.expires)
        self.reason = "executing leased command"
        return True

    def frame(self, now):
        if self.last_tx_at is not None and now - self.last_tx_at >= self.MAX_SEND_GAP:
            if self.token or self.ready:
                self.stop("transmit scheduling gap")
        if not self.health(now):
            return None
        cmd = self.command if self.token and not self.fault else None
        if cmd and now >= self.action_expires:
            # Pulse end stops all actuators without revoking a still-fresh lease.
            # Its absolute deadline cannot be extended by the 50 Hz resend path.
            cmd = replace(cmd, direction=0, brush_enabled=False, thrust_enabled=False)
        values = cmd.channels() if cmd else neutral()
        seq = self.wire_seq
        self.wire_seq = (seq + 1) & U32
        if not self.token and self.barrier is None:
            self.barrier = seq
        self.last_sent = seq
        return encode(values, self.ctrl.epoch, seq)

    def sent(self, now):
        if self.last_tx_at is not None:
            self.max_tx_gap = max(self.max_tx_gap, now - self.last_tx_at)
        self.last_tx_at = now
        self.tx_frames += 1

    def snapshot(self, now):
        self.health(now)
        state = ("FAULT" if self.fault else "DISCONNECTED" if not self.connected
                 else "RC" if self.ctrl and self.ctrl.source == "RC"
                 else "ACTIVE" if self.token else "READY" if self.ready
                 else "HANDSHAKE")
        if not self.motion_enabled and state == "READY":
            state = "MONITOR"
        return dict(state=state, reason=self.fault or self.reason,
                    last_stop_reason=self.last_stop_reason,
                    allow_fifth_thruster=self.allow_fifth,
                    connected=self.connected, connection_id=self.connection_id,
                    motion_authorized=bool(self.token), owner=self.owner, ctrl=self.ctrl,
                    ctrl_age_sec=-1. if self.ctrl_at is None else now - self.ctrl_at,
                    ack_age_sec=-1. if self.ack_at is None else now - self.ack_at,
                    command_remaining_sec=max(0., self.expires - now),
                    sent_sequence=self.last_sent or 0, rejected_commands=self.rejected,
                    tx_frames=self.tx_frames, reconnects=self.reconnects,
                    max_tx_gap_sec=self.max_tx_gap)

    def diagnostics(self, now):
        return dict(command_receive_age_ms=-1. if self.last_command_at is None
                    else (now - self.last_command_at) * 1000,
                    command_age_at_accept_ms=self.last_command_age_ms,
                    command_remaining_at_accept_ms=self.last_command_remaining_ms,
                    command_max_receive_gap_ms=self.max_command_gap_ms,
                    command_deadline_overrun_ms=self.last_deadline_overrun_ms,
                    command_last_rejection=self.last_command_rejection)
