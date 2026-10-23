"""Deterministic lease/replay/deadline checks without ROS or hardware."""
import json

import pytest

from nano_ros_bridge.ui_protocol import GatewayCore, PROTOCOL


class Clock:
    now = 10.
    unix = 1_800_000_000_000_000_000

    def advance(self, seconds):
        self.now += seconds
        self.unix += round(seconds * 1e9)


class Backend:
    def __init__(self):
        self.commands, self.released, self.pending = [], [], []

    def acquire(self, owner, callback):
        self.pending.append(callback)

    def grant(self, token="lease"):
        self.pending.pop(0)(dict(success=True, token=token, epoch=1, max_command_ttl_ms=150))

    def release(self, token, callback):
        self.released.append(token)
        callback(dict(success=True))

    def publish(self, command):
        self.commands.append(command)


def ready(**fields):
    return dict(dict(state="READY", connected=True, source="NANO", epoch=1, armed=True,
                     result="OK", ctrl_age_sec=.01, ack_age_sec=.01, connection_id="board",
                     motion_authorized=False), **fields)


@pytest.fixture
def harness():
    clock, backend, packets = Clock(), Backend(), []
    core = GatewayCore(backend, lambda ident, packet: packets.append((ident, packet)),
                       clock=lambda: clock.now, unix_ns=lambda: clock.unix)
    core.update_state(ready())
    return core, clock, backend, packets


def send(core, op, ident=b"ui", **fields):
    core.handle(ident, json.dumps(dict(protocol=PROTOCOL, op=op, request_id="request", **fields)).encode())


def acquire(harness):
    core, _, backend, _ = harness
    send(core, "hello")
    core.tick()
    ticket = next(reversed(core.clients[b"ui"].tickets))
    send(core, "acquire", owner="F1-Qt")
    backend.grant()
    return ticket


def command(core, issued_ticket, **changes):
    fields = dict(token="lease", ticket=issued_ticket, sequence=1, direction=1, speed_percent=25,
                  mode=1, brush_enabled=False, brush_percent=0, thrust_enabled=False, thrust_percent=0)
    fields.update(changes)
    send(core, "command", **fields)


def test_original_ticket_time_survives_forwarding(harness):
    core, clock, backend, packets = harness
    ticket = acquire(harness)
    original_ns = clock.unix
    clock.advance(.04)
    command(core, ticket)
    result = backend.commands[-1]
    assert result.computed_ns == original_ns
    assert result.deadline_ns == original_ns + 150_000_000
    assert result.direction == 1 and packets[-1][1]["success"]


def test_consumed_ticket_cannot_be_replayed(harness):
    core, _, backend, packets = harness
    ticket = acquire(harness)
    command(core, ticket)
    command(core, ticket, sequence=2)
    assert len(backend.commands) == 1
    assert backend.released == ["lease"]
    assert not packets[-1][1]["success"]


def test_expired_ticket_cannot_be_refreshed(harness):
    core, clock, backend, _ = harness
    ticket = acquire(harness)
    clock.advance(.151)
    command(core, ticket)
    assert not backend.commands and backend.released == ["lease"]


@pytest.mark.parametrize("elapsed", [.099, .110, .149])
def test_ticket_round_trip_uses_existing_bridge_ttl(harness, elapsed):
    core, clock, backend, _ = harness
    ticket = acquire(harness)
    created_ns = clock.unix
    clock.advance(elapsed)
    command(core, ticket)
    assert not backend.released
    assert backend.commands[-1].deadline_ns == created_ns + 150_000_000


def test_input_snapshot_age_is_deducted_without_clock_synchronization(harness):
    core, clock, backend, _ = harness
    ticket = acquire(harness)
    created_ns = clock.unix
    clock.advance(.06)
    command(core, ticket, input_age_ns=30_000_000)
    assert backend.commands[-1].computed_ns == created_ns - 30_000_000
    assert backend.commands[-1].deadline_ns == created_ns + 120_000_000


@pytest.mark.parametrize("age", [-1, True, 1.5, "1", 150_000_000])
def test_invalid_snapshot_age_releases_control(harness, age):
    core, _, backend, _ = harness
    command(core, acquire(harness), input_age_ns=age)
    assert not backend.commands and backend.released == ["lease"]


def test_fresh_ticket_cannot_renew_expired_gui_input(harness):
    core, clock, backend, _ = harness
    ticket = acquire(harness)
    clock.advance(.06)
    command(core, ticket, input_age_ns=100_000_000)
    assert not backend.commands and backend.released == ["lease"]


@pytest.mark.parametrize("changes", [
    {"direction": True}, {"direction": 5}, {"speed_percent": 100}, {"speed_percent": 25.},
    {"mode": -1}, {"brush_enabled": 1}, {"brush_percent": float("nan")},
    {"brush_percent": float("inf")}, {"brush_percent": 101}, {"thrust_percent": "50"},
    {"sequence": True}, {"sequence": -1}, {"sequence": 2**64}, {"thrust_enabled": True},
    {"ticket": []}, {"computed_at": 0},
])
def test_invalid_owned_command_revokes_lease(harness, changes):
    core, _, backend, _ = harness
    command(core, acquire(harness), **changes)
    assert not backend.commands and backend.released == ["lease"]


def test_fifth_thruster_requires_explicit_gateway_configuration(harness):
    core, _, backend, _ = harness
    core.allow_fifth = True
    core.update_state(ready(allow_fifth_thruster=True))
    command(core, acquire(harness), thrust_enabled=True, thrust_percent=-20)
    assert backend.commands[-1].thrust_enabled
    assert backend.commands[-1].thrust_percent == -20


def test_mode_change_preserves_lease_and_forwards_new_group(harness):
    core, _, backend, _ = harness
    command(core, acquire(harness))
    core.tick()
    ticket = next(reversed(core.clients[b"ui"].tickets))
    command(core, ticket, sequence=2, mode=2)
    assert len(backend.commands) == 2 and not backend.released
    assert core.clients[b"ui"].token == "lease" and backend.commands[-1].mode == 2


@pytest.mark.parametrize("gateway,bridge", [(False, False), (False, True), (True, False), (True, True)])
def test_fifth_capability_requires_both_nodes(harness, gateway, bridge):
    core, _, backend, packets = harness
    core.allow_fifth = gateway
    core.update_state(ready(allow_fifth_thruster=bridge))
    ticket = acquire(harness)
    core.tick()
    assert packets[-1][1]["capabilities"]["allow_fifth_thruster"] is (gateway and bridge)
    command(core, ticket, thrust_enabled=True, thrust_percent=25)
    assert bool(backend.commands) is (gateway and bridge)


def test_repeated_sequence_with_new_ticket_is_rejected(harness):
    core, _, backend, _ = harness
    command(core, acquire(harness))
    core.tick()
    command(core, next(reversed(core.clients[b"ui"].tickets)))
    assert len(backend.commands) == 1 and backend.released == ["lease"]


def test_hello_cannot_keep_motion_alive(harness):
    core, clock, backend, _ = harness
    command(core, acquire(harness))
    for _ in range(3):
        clock.advance(.1)
        core.update_state(ready())
        send(core, "hello")
    assert backend.released == ["lease"]
    assert not core.clients[b"ui"].token


@pytest.mark.parametrize("change", [dict(source="RC"), dict(epoch=2), dict(connection_id="new"),
                                     dict(armed=False), dict(connected=False), dict(ack_age_sec=.4)])
def test_bridge_changes_and_unhealthy_state_revoke(harness, change):
    core, _, backend, _ = harness
    acquire(harness)
    core.update_state(ready(**change))
    assert backend.released == ["lease"]


def test_bridge_staleness_revokes_even_with_client_hello(harness):
    core, clock, backend, _ = harness
    acquire(harness)
    for _ in range(4):
        clock.advance(.08)
        send(core, "hello")
    assert backend.released == ["lease"]
    send(core, "acquire", owner="F1-Qt")
    assert not backend.pending


def test_active_lease_loss_is_not_automatically_reacquired(harness):
    core, _, backend, _ = harness
    acquire(harness)
    core.update_state(ready(state="ACTIVE", motion_authorized=True))
    core.update_state(ready())
    assert backend.released == ["lease"] and not backend.pending


def test_client_timeout_and_late_acquire_response(harness):
    core, clock, backend, _ = harness
    send(core, "hello")
    send(core, "acquire", owner="F1-Qt")
    clock.advance(.251)
    core.tick()
    backend.grant("late")
    assert backend.released == ["late"] and not core.clients


def test_pending_stop_cancels_and_releases_late_token(harness):
    core, _, backend, _ = harness
    send(core, "hello")
    send(core, "acquire", owner="F1-Qt")
    send(core, "stop", token="")
    backend.grant("late")
    assert backend.released == ["late"] and not core.clients[b"ui"].token


def test_old_acquire_cannot_overwrite_a_new_request(harness):
    core, _, backend, _ = harness
    send(core, "hello")
    send(core, "acquire", owner="first")
    send(core, "release", token="")
    send(core, "acquire", owner="second")
    backend.grant("old")
    backend.grant("new")
    assert backend.released == ["old"] and core.clients[b"ui"].token == "new"


@pytest.mark.parametrize("op", ["stop", "release"])
def test_other_connection_cannot_release_token_even_if_it_knows_it(harness, op):
    core, _, backend, _ = harness
    acquire(harness)
    send(core, "hello", ident=b"other")
    send(core, op, ident=b"other", token="lease")
    assert not backend.released and core.clients[b"ui"].token == "lease"
    send(core, op, token="lease")
    assert backend.released == ["lease"]


def test_ticket_is_bound_to_connection(harness):
    core, _, backend, _ = harness
    ticket = acquire(harness)
    send(core, "hello", ident=b"other")
    send(core, "acquire", ident=b"other", owner="other")
    backend.grant("other-lease")
    send(core, "command", ident=b"other", token="other-lease", ticket=ticket, sequence=1,
         direction=1, speed_percent=25, mode=1, brush_enabled=False, brush_percent=0,
         thrust_enabled=False, thrust_percent=0)
    assert backend.released == ["other-lease"] and not backend.commands
    assert core.clients[b"ui"].token == "lease"


@pytest.mark.parametrize("payload", [b"[]", b"null", b"{", b"\xff", b"x" * 32769,
    b'{"protocol":"f1-ui-v2","op":"hello","op":"acquire","request_id":"r"}',
    b'{"protocol":"f1-ui-v2","op":[],"request_id":"r"}'])
def test_malformed_input_is_ignored(harness, payload):
    core, _, backend, _ = harness
    core.handle(b"ui", payload)
    assert not core.clients and not backend.commands


def test_clock_jump_latches_fault_and_releases(harness):
    core, clock, backend, _ = harness
    acquire(harness)
    clock.unix += 100_000_000
    core.tick()
    assert core.clock_fault and backend.released == ["lease"]
    core.update_state(ready())
    send(core, "acquire", owner="F1-Qt")
    assert not backend.pending


def test_deschedule_between_clock_reads_does_not_revoke_control(harness):
    core, clock, backend, _ = harness
    acquire(harness)
    called = False

    def slow_clock_read():
        nonlocal called
        if not called:
            clock.advance(.06)
            called = True
        return clock.unix

    core.unix_ns = slow_clock_read
    core.tick()
    core.tick()  # A skipped sample must not poison the next clock comparison.
    assert not core.clock_fault and not backend.released
    assert core.clients[b"ui"].token == "lease"


def test_clients_and_tickets_remain_bounded(harness):
    core, _, _, _ = harness
    for index in range(100):
        send(core, "hello", ident=str(index).encode())
    for _ in range(100):
        core.tick()
    assert len(core.clients) == core.MAX_CLIENTS
    assert all(len(client.tickets) <= 8 for client in core.clients.values())


def test_close_releases_current_and_pending_control(harness):
    core, _, backend, _ = harness
    acquire(harness)
    send(core, "hello", ident=b"other")
    send(core, "acquire", ident=b"other", owner="other")
    core.close()
    backend.grant("late")
    assert backend.released == ["lease", "late"] and not core.clients
