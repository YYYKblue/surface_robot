"""Virtual-time UI -> gateway -> Session -> unmodified C firmware regressions.

Network and ROS delays are independent of the serial worker. Integer millisecond
events avoid accumulated float clock error; only production clock APIs see floats.
The C fixture compiles into pytest's temporary directory, never into firmware/.
"""
import ctypes as c
import heapq
import json

import pytest

from nano_ros_bridge.protocol import channels, decode, encode
from nano_ros_bridge.session import Session
from nano_ros_bridge.ui_protocol import GatewayCore, PROTOCOL


NS_PER_MS = 1_000_000
TTL_NS = 250_000_000


class Chain:
    def __init__(self, firmware, *, down_ms=0, up_ms=0, ui_wait_ms=0, ros_ms=0):
        self.firmware = firmware
        self.ms = 0
        self.events = []
        self.event_order = 0
        self.sequence = 0
        self.down_ms, self.up_ms = down_ms, up_ms
        self.ui_wait_ms, self.ros_ms = ui_wait_ms, ros_ms
        self.streaming = False
        self.sample_ms = None  # None means a new GUI sample for every command.
        self.corrupt_next_frame = False
        self.published, self.accepted, self.replies, self.released = [], [], [], []
        self.acquires = 0
        self.session = Session(motion_enabled=True)
        self.session.opened()
        self.session.receive(self.report(), self.monotonic())
        self.gateway = GatewayCore(self, self.outbound,
                                   clock=self.monotonic, unix_ns=self.unix_ns)
        self.advance(560)  # Real firmware's >=300 ms neutral handshake + CTRL ACK.
        assert self.session.ready
        self.sync_state()
        self.send("hello")
        self.send("acquire", owner="timing-test")
        self.token = self.replies[-1]["token"]
        assert self.token == self.session.token
        # Seed one fresh command before imposing steady-state network delay.
        self.gateway.tick()
        self.send_command(self.latest_ticket, input_age_ns=0)
        self.streaming = True
        self.gateway.tick()

    def monotonic(self):
        return self.ms / 1000

    def unix_ns(self):
        return 10_000_000_000 + self.ms * NS_PER_MS

    def schedule(self, delay_ms, callback):
        self.event_order += 1
        heapq.heappush(self.events, (self.ms + delay_ms, self.event_order, callback))

    def report(self):
        buffer = c.create_string_buffer(256)
        size = self.firmware.test_report(self.ms, buffer, len(buffer))
        assert size > 0
        return decode(buffer.raw[:size])

    def sync_state(self):
        state = self.session.snapshot(self.monotonic())
        ctrl = state.pop("ctrl")
        state.update(source=ctrl.source, epoch=ctrl.epoch,
                     armed=ctrl.armed, result=ctrl.result)
        self.gateway.update_state(state)

    def send(self, op, **fields):
        self.gateway.handle(b"ui", json.dumps(dict(
            protocol=PROTOCOL, op=op, request_id=f"{op}-{self.sequence}",
            **fields)).encode())

    def command_packet(self, ticket, input_age_ns):
        self.sequence += 1
        return dict(token=self.token, ticket=ticket, sequence=self.sequence,
                    input_age_ns=input_age_ns, direction=1, speed_percent=25,
                    mode=1, brush_enabled=False, brush_percent=0,
                    thrust_enabled=False, thrust_percent=0)

    def send_command(self, ticket, input_age_ns):
        self.send("command", **self.command_packet(ticket, input_age_ns))

    def outbound(self, ident, packet):
        assert ident == b"ui"
        if packet["type"] == "reply":
            self.replies.append(packet)
            return
        self.latest_ticket = packet["ticket"]
        if self.streaming:
            def sample_and_send():
                sampled = self.ms if self.sample_ms is None else self.sample_ms
                fields = self.command_packet(packet["ticket"],
                                             (self.ms - sampled) * NS_PER_MS)
                self.schedule(self.up_ms, lambda: self.send("command", **fields))
            self.schedule(self.down_ms + self.ui_wait_ms, sample_and_send)

    # Gateway backend: ROS services run immediately; topic delivery is delayed.
    def acquire(self, owner, callback):
        self.acquires += 1
        token, reason = self.session.acquire(owner, self.monotonic())
        callback(dict(success=bool(token), token=token, message=reason,
                      epoch=self.session.ctrl.epoch, max_command_ttl_ms=250))

    def release(self, token, callback):
        self.released.append(token)
        callback(dict(success=self.session.release(token)))

    def publish(self, command):
        self.published.append(command)
        def deliver():
            accepted = self.session.accept(command, self.monotonic(), self.unix_ns())
            self.accepted.append((self.ms, command, accepted))
        self.schedule(self.ros_ms, deliver)

    def advance(self, duration_ms):
        for _ in range(duration_ms):
            self.ms += 1
            while self.events and self.events[0][0] <= self.ms:
                _, _, callback = heapq.heappop(self.events)
                callback()
            if self.ms % 20 == 0:
                frame = self.session.frame(self.monotonic())
                if frame is not None:
                    if self.corrupt_next_frame:
                        frame = frame[:-1] + bytes([frame[-1] ^ 1])
                        self.corrupt_next_frame = False
                    self.firmware.test_host(frame, len(frame), self.ms)
                    self.session.sent(self.monotonic())
            self.firmware.test_tick(self.ms)
            if self.ms % 100 == 0:
                self.session.receive(self.report(), self.monotonic())
                self.sync_state()
                if b"ui" in self.gateway.clients:
                    self.send("hello")
            if self.ms % 20 == 0:
                self.gateway.tick()

    def assert_stopped(self):
        assert not self.session.token and self.session.command is None
        assert not self.gateway.clients[b"ui"].token
        assert self.report().neutral
        assert self.acquires == 1  # Fresh tickets/hellos never reacquire a lease.


@pytest.mark.parametrize("network_round_trip_ms", [80, 100])
def test_network_round_trip_and_ros_jitter_preserve_live_control(firmware, network_round_trip_ms):
    """50 Hz input with 80/100 ms RTT plus 20 ms UI/ROS scheduling budget."""
    half = network_round_trip_ms // 2
    chain = Chain(firmware, down_ms=half, up_ms=half, ui_wait_ms=5, ros_ms=15)
    chain.advance(2000)
    assert chain.session.token == chain.token == chain.gateway.clients[b"ui"].token
    assert len(chain.accepted) > 90 and all(ok for _, _, ok in chain.accepted)
    assert chain.report().direction == "FORWARD"
    assert chain.report().armed and chain.report().result == "OK"
    assert not chain.released and chain.acquires == 1


@pytest.mark.parametrize("network_round_trip_ms", [80, 100])
def test_previous_100_ms_budget_reproduces_deadline_loss(firmware, monkeypatch,
                                                       network_round_trip_ms):
    """Keep this reproduction to show why a healthy serial link used to lose control."""
    monkeypatch.setattr(GatewayCore, "TICKET_TTL_NS", 100_000_000)
    half = network_round_trip_ms // 2
    chain = Chain(firmware, down_ms=half, up_ms=half, ui_wait_ms=5, ros_ms=15)
    chain.advance(600)
    chain.assert_stopped()
    assert chain.session.last_stop_reason == "command deadline expired"
    assert chain.session.snapshot(chain.monotonic())["ctrl_age_sec"] < .1


def test_latency_beyond_budget_does_not_execute_or_reacquire(firmware):
    chain = Chain(firmware, down_ms=120, up_ms=120, ui_wait_ms=20, ros_ms=15)
    chain.advance(600)
    chain.assert_stopped()
    # Only the seed command can execute; queued commands cannot renew its lease.
    assert sum(ok for _, _, ok in chain.accepted) == 1
    # Either layer may notice first: the gateway releases on its own input
    # timeout, the bridge on the original deadline.
    assert chain.session.last_stop_reason in ("command deadline expired",
                                              "owner released control")


def test_new_tickets_cannot_extend_one_old_gui_snapshot(firmware):
    chain = Chain(firmware, down_ms=5, up_ms=5)
    chain.sample_ms = chain.ms
    sampled_unix_ns = chain.unix_ns()
    chain.advance(100)
    assert chain.session.token == chain.token
    aged = chain.published[1:]
    assert len(aged) >= 4
    # Subtracting GUI snapshot age from ticket time is conservative: downlink
    # delay reduces the deadline instead of making the old snapshot look newer.
    assert all(command.deadline_ns <= sampled_unix_ns + TTL_NS for command in aged)
    assert len({command.deadline_ns for command in aged}) == 1
    chain.advance(300)
    chain.assert_stopped()
    assert all(command.deadline_ns <= sampled_unix_ns + TTL_NS
               for command in chain.published)


def test_no_input_stops_motion_despite_hellos_and_new_tickets(firmware):
    chain = Chain(firmware, down_ms=10, up_ms=10)
    chain.advance(300)
    assert chain.session.token == chain.token
    chain.streaming = False
    chain.advance(40)  # Drain already sampled/in-flight commands, then stop input.
    last_deadline = max(command.deadline_ns for command in chain.published)
    chain.advance(400)
    assert chain.unix_ns() > last_deadline
    chain.assert_stopped()
    # A late sender holding the former token cannot revive motion with a new ticket.
    chain.send_command(chain.latest_ticket, input_age_ns=0)
    chain.advance(40)
    chain.assert_stopped()
    assert chain.session.last_stop_reason == "command deadline expired"


@pytest.mark.parametrize("fault", ["epoch", "rc_takeover"])
def test_firmware_identity_changes_still_revoke_delayed_stream(firmware, fault):
    chain = Chain(firmware, down_ms=10, up_ms=10)
    chain.advance(300)
    assert chain.session.token == chain.token
    if fault == "epoch":
        chain.corrupt_next_frame = True  # Actual C CRC fault advances firmware epoch.
    else:
        rc = encode(channels(1), 1, 0)[12:37]
        firmware.test_remote(rc, chain.ms)  # Actual C remote-activity arbitration.
    chain.advance(500)
    chain.assert_stopped()
    assert chain.session.last_stop_reason == "source/epoch changed"
    assert chain.report().epoch != 1
