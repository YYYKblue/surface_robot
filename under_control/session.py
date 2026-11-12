"""Monotonic-clock safety state. The serial worker owns its lock and all I/O."""
import uuid

from .protocol import U32, encode

INPUT_TTL = .250
REPORT_TTL = .250
REPLY_TIMEOUT = .150
PERIOD = .050
BUS_QUIET = .003


class Session:
    def __init__(self, motion_enabled=False):
        self.motion_enabled = motion_enabled
        self.connected = False
        self.connection_id = ""
        self.generation = 1
        self.epoch = 0
        self.sequence = 0
        self.report = None
        self.report_at = None
        self.report_generation = None
        self.pending = None
        self.ready = False
        self.owner = None
        self.input_at = None
        self.intent = ("STOP", 0)
        self.reason = self.last_stop_reason = "not connected"

    def stop(self, reason, client=None):
        if client is not None and self.owner not in (None, client):
            return False
        self.generation += 1
        self.owner = self.input_at = None
        self.intent = ("STOP", 0)
        self.last_stop_reason = reason
        self.ready = False
        self.reason = reason
        return True

    def opened(self):
        self.stop("serial connected; new input required")
        self.connected = True
        self.connection_id = uuid.uuid4().hex
        self.epoch = self.sequence = 0
        self.report = self.report_at = self.report_generation = self.pending = None
        self.ready = False
        self.reason = "discovering H7"

    def disconnected(self, reason):
        self.stop(reason)
        self.connected = self.ready = False
        self.pending = None
        self.reason = reason

    def tick(self, now):
        if self.input_at is not None and now - self.input_at >= INPUT_TTL:
            self.stop("app input timeout")
        if self.ready and (self.report_at is None or now - self.report_at >= REPORT_TTL):
            self.stop("H7 feedback timeout")
            self.ready = False
            self.reason = "H7 feedback timeout"

    def submit(self, client, packet, now):
        self.tick(now)
        if self.owner not in (None, client):
            return False, "another operator owns underwater control"
        if packet["control_generation"] != self.generation:
            return False, "control session changed; release and press again"
        direction, speed = packet["direction"], packet["speed_percent"]
        if direction == "STOP" or speed == 0:
            direction, speed = "STOP", 0
        if direction != "STOP" and not (self.ready and self.motion_enabled):
            return False, "H7 not ready or motion disabled"
        # Neutral observers cannot renew someone else's lease (checked above).
        self.input_at = now
        self.intent = direction, speed
        self.owner = client if direction != "STOP" else None
        return True, "accepted"

    def request(self, now):
        self.tick(now)
        if not self.connected or self.pending is not None:
            return None
        self.sequence = (self.sequence + 1) & U32
        direction, speed = self.intent if self.ready and self.motion_enabled else ("STOP", 0)
        self.pending = (self.epoch, self.sequence, direction, speed, now, self.generation)
        return encode(self.epoch, self.sequence, direction, speed)

    def receive(self, report, now):
        if self.pending is None:
            raise ValueError("unsolicited H7 reply")
        epoch, sequence, direction, speed, sent_at, generation = self.pending
        if now - sent_at >= REPLY_TIMEOUT:
            raise ValueError("late H7 reply")
        previous = self.report
        if previous is not None:
            advance = (report.ms - previous.ms) & U32
            # Natural uint32 wrap is valid. Resetting to epoch=1 on boot is not
            # sufficient to identify a reboot, so check board uptime as well.
            if advance == 0 or advance >= 0x80000000:
                raise ValueError("H7 uptime repeated or reset")
        changed = (report.epoch != self.epoch or
                   (previous is not None and previous.source != report.source))
        if changed:
            self.stop("H7 control identity changed")
            self.ready = False
        if report.source == "NANO" and epoch == report.epoch:
            if not report.have_seq or report.ack_seq != sequence:
                raise ValueError("H7 ACK does not match request")
        self.pending = None
        self.report, self.report_at, self.epoch = report, now, report.epoch
        self.report_generation = generation
        if report.source == "RC":
            self.ready = False
            self.reason = "handheld remote owns control"
        elif report.result != "OK" or not report.armed:
            if self.ready:
                self.stop("H7 disarmed")
            self.ready = False
            self.reason = report.result
        elif epoch == report.epoch and not changed and generation == self.generation:
            if self.ready or (direction == "STOP" and speed == 0 and report.neutral):
                self.ready = True
                self.reason = "ready" if self.motion_enabled else "motion disabled"

    def snapshot(self, now):
        self.tick(now)
        report = self.report
        age = None if self.report_at is None else max(0., now - self.report_at)
        fresh = self.connected and age is not None and age < REPORT_TTL
        source = report.source if fresh else "UNKNOWN"
        available = bool(self.ready and fresh and self.motion_enabled and source == "NANO")
        return dict(connected=self.connected, connection_id=self.connection_id,
                    control_generation=self.generation, epoch=self.epoch,
                    state=("RC" if source == "RC" else "ACTIVE" if available and self.owner
                           else "READY" if available else "WAITING" if self.connected else "OFFLINE"),
                    available=available, source=source, owner=self.owner,
                    armed=bool(fresh and report.armed), result=report.result if fresh else "UNKNOWN",
                    reason=self.reason, last_stop_reason=self.last_stop_reason,
                    report_age_sec=age, control_fresh=fresh,
                    input_age_sec=None if self.input_at is None else max(0., now - self.input_at),
                    motion_enabled=self.motion_enabled, firmware_protocol_version=6,
                    direction=report.direction if fresh else None,
                    speed_percent=report.speed_percent if fresh else None,
                    compatibility_values=list(report.current) if fresh else None,
                    compatibility_goals=list(report.goal) if fresh else None,
                    ack_seq=report.ack_seq if fresh else None,
                    stop_confirmed=bool(fresh and source == "NANO" and report.neutral
                                        and self.report_generation == self.generation))
