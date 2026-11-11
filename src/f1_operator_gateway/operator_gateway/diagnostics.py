"""Bounded diagnostic history; contains no control tokens or executable intent."""
from collections import deque
import time
import uuid


class DiagnosticHistory:
    def __init__(self, on_record=None):
        self.session = uuid.uuid4().hex
        self.events = deque(maxlen=50)
        self.sequence = 0
        self.bridge_signature = None
        self.on_record = on_record

    def record(self, origin, reason, **details):
        self.sequence += 1
        event = dict(id=self.sequence, time_unix=time.time(),
                     source=origin, reason=reason, details=details)
        self.events.append(event)
        if self.on_record is not None:
            self.on_record(dict(session=self.session, events=[event]))

    def bridge(self, state):
        keys = ("connection_id", "state", "source", "owner", "motion_authorized",
                "reason", "last_stop_reason", "firmware_protocol_version",
                "firmware_protocol_compatible")
        signature = tuple(state.get(key) for key in keys)
        if signature != self.bridge_signature:
            self.bridge_signature = signature
            self.record("bridge", state.get("reason", ""), **self.snapshot(state))

    @staticmethod
    def snapshot(state):
        return {key: state[key] for key in (
            "state", "source", "owner", "motion_authorized", "last_stop_reason",
            "ctrl_age_sec", "ack_age_sec", "command_remaining_sec", "reconnects",
            "max_tx_gap_sec", "rejected_commands", "firmware_protocol_version",
            "firmware_protocol_compatible") if key in state}

    def view(self):
        return dict(session=self.session, events=list(self.events))
