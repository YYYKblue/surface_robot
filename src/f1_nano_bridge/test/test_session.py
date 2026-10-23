from dataclasses import replace

import pytest

from nano_ros_bridge.protocol import Control
from nano_ros_bridge.session import Command, Session


def report(ms=100, ack=0, **kw):
    return replace(Control(ms, "NANO", 1, True, ack, "OK", "BOTH", True,
                           False, 0, False, 0, (1500,) * 4), **kw)


def ready():
    core = Session(motion_enabled=True)
    core.opened()
    core.receive(report(0, have_sequence=False, armed=False, result="WAIT_NANO"), 0.)
    core.frame(0.)
    core.sent(0.)
    core.receive(report(20), .02)
    assert core.ready
    return core


def command(token, sequence=0, computed_ns=1_000_000_000, deadline_ns=1_150_000_000, **kw):
    return Command(token, sequence, computed_ns, deadline_ns, 1, **kw)


def test_startup_disabled_and_no_lease_without_neutral_ack():
    core = Session()
    core.opened()
    core.receive(report(), 0.)
    assert not core.acquire("wall", 0.)[0]
    assert not core.accept(command("fake"), 0., 1_000_000_000)
    assert core.frame(0.) is not None
    assert not core.token


def test_deadline_is_not_renewed_by_resends_or_duplicates():
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert core.accept(command(token), .02, 1_050_000_000)
    for now in (.04, .06, .08, .10):
        assert core.frame(now) is not None
        core.sent(now)
    assert not core.accept(command(token), .10, 1_130_000_000)
    assert core.token == token
    core.frame(.121)
    assert not core.token and core.command is None
    assert "deadline" in core.reason
    assert not core.acquire("wall", .121)[0]  # Stop must first be acknowledged.


@pytest.mark.parametrize("computed,deadline", [
    (1_000_000_000, 1_250_000_001), (1_100_000_000, 1_350_000_000),
    (900_000_000, 1_000_000_000), (1_000_000_000, 999_000_000)])
def test_invalid_times_revoke_lease(computed, deadline):
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert not core.accept(command(token, computed_ns=computed, deadline_ns=deadline), .02, 1_000_000_000)
    assert not core.token


def test_stop_barrier_prevents_replay_and_requires_ack():
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert core.accept(command(token), .02, 1_000_000_000)
    core.frame(.04)
    core.sent(.04)
    core.stop("user stop")
    assert not core.accept(command(token, 1), .05, 1_010_000_000)
    assert not core.acquire("wall", .05)[0]
    core.frame(.06)
    core.sent(.06)
    core.receive(report(60, ack=1), .06)  # Old ack predates the neutral barrier.
    assert not core.ready
    core.receive(report(80, ack=2), .08)
    token2, _ = core.acquire("wall", .08)
    assert token2 and token2 != token
    assert not core.accept(command(token, 100), .08, 1_020_000_000)
    assert core.token == token2


def test_rc_epoch_reconnect_and_device_reset_all_revoke():
    for incoming in (report(40, source="RC", epoch=2), report(40, epoch=2), report(1)):
        core = ready()
        token, _ = core.acquire("wall", .02)
        core.accept(command(token), .02, 1_000_000_000)
        core.receive(incoming, .04)
        assert not core.token and not core.ready and core.command is None
    core = ready()
    core.acquire("wall", .02)
    old_connection = core.connection_id
    core.closed("unplugged")
    core.opened()
    assert not core.token and core.ctrl is None and core.connection_id != old_connection


def test_ctrl_duplicates_and_imu_do_not_keep_control_alive():
    core = ready()
    core.receive(report(20), .41)
    assert core.ctrl_at == .02
    assert core.frame(.43) is None
    assert not core.ready


def test_ack_stall_and_send_stall_revoke():
    core = ready()
    core.acquire("wall", .02)
    core.frame(.101)
    assert not core.token and "scheduling" in core.reason
    core = ready()
    for i in range(2, 23):
        now = i * .02
        core.frame(now)
        core.sent(now)
        core.receive(report(i * 20), now)  # New CTRL timestamps but no ACK advance.
    assert not core.acquire("wall", .44)[0]
    assert "ACK" in core.reason


def test_fifth_disabled_and_clock_fault_latches_across_reopen():
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert not core.accept(command(token, thrust_enabled=True), .02, 1_000_000_000)
    core.latch_fault("clock")
    core.closed("disconnect")
    core.opened()
    assert core.fault == "clock"
    assert not core.acquire("wall", 0.)[0]


def test_unclaimed_lease_expires_and_other_owner_cannot_release():
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert not core.acquire("manual", .03)[0]
    assert not core.release("wrong")
    core.health(.28)
    assert not core.release(token) and not core.token


def test_mode_change_preserves_lease_and_auxiliaries():
    core = ready()
    token, _ = core.acquire("wall", .02)
    for sequence, mode in enumerate((1, 0, 2, 1)):
        assert core.accept(command(token, sequence, mode=mode, brush_enabled=True, brush_percent=-25),
                           .02 + sequence * .01, 1_000_000_000 + sequence * 10_000_000)
        assert core.token == token and core.command.mode == mode
        assert core.command.brush_enabled and core.command.brush_percent == -25


def test_non_neutral_report_cannot_enable_ready_and_loss_reason_survives_handshake():
    core = ready()
    token, _ = core.acquire("wall", .02)
    assert core.accept(command(token), .02, 1_000_000_000)
    core.stop("command deadline expired")
    core.frame(.04)
    core.sent(.04)
    core.receive(report(40, ack=1, pwm=(1580, 1500, 1500, 1500)), .04)
    assert not core.ready
    core.receive(report(60, ack=1), .06)
    assert core.ready
    assert core.snapshot(.06)["last_stop_reason"] == "command deadline expired"
    core.receive(report(80, ack=1, armed=False, result="WAIT_NEUTRAL"), .08)
    assert not core.ready


def test_absolute_pulse_end_stops_aux_and_motion_without_renewal_by_resend():
    core = ready()
    token, _ = core.acquire("wall", .02)
    cmd = command(token, direction_until_ns=1_050_000_000,
                  brush_enabled=True, brush_percent=20)
    assert core.accept(cmd, .02, 1_000_000_000)
    def decoded_channels(now):
        packet = core.frame(now)
        core.sent(now)
        bits = int.from_bytes(packet[13:35], "little")
        return [(((bits >> (i * 11)) & 2047) * 5 + 6994) // 8 for i in range(16)]
    assert decoded_channels(.04)[2:4] == [1950, 1500]
    assert decoded_channels(.08)[2:4] == [1500, 1500]
    assert decoded_channels(.10)[9:11] == [1050, 1050]
    assert core.token == token  # Fresh lease survives expected pulse completion.
    # A newly computed continuation retains the ORIGINAL pulse end.
    assert core.accept(command(token, 1, computed_ns=1_040_000_000,
                               deadline_ns=1_140_000_000, direction_until_ns=1_050_000_000),
                       .10, 1_060_000_000)
    assert decoded_channels(.12)[2:4] == [1500, 1500]


def test_conflict_revokes_lease_and_requires_neutral_barrier():
    core = ready()
    token, _ = core.acquire("wall", .02)
    core.receive(report(40, result="DIR_CONFLICT"), .04)
    assert not core.token and not core.ready
    assert not core.acquire("wall", .04)[0]
