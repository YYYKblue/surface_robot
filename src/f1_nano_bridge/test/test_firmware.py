import ctypes as c
import json
import os
from pathlib import Path
import random
import subprocess

from nano_ros_bridge.protocol import channels, decode, encode, neutral
from nano_ros_bridge.session import Command, Session
from nano_ros_bridge.ui_protocol import GatewayCore, PROTOCOL


def report(lib, milliseconds):
    buf = c.create_string_buffer(256)
    size = lib.test_report(milliseconds, buf, len(buf))
    assert size > 0
    return decode(buf.raw[:size])


def feed(lib, packet, ms):
    lib.test_host(packet, len(packet), ms)
    lib.test_tick(ms)


def test_real_c_decodes_random_channels_and_new_polarity(firmware):
    lib = firmware
    rng = random.Random(103)
    for seq in range(100):
        values = [rng.randint(1050, 1950) for _ in range(16)]
        packet = encode(values, 1, seq)
        # Parser-only checks include all channels, independent of actuator rules.
        lib.test_host(packet[:17], 17, seq * 20)
        lib.test_host(packet[17:], 22, seq * 20 + 1)
        assert [lib.test_get(32 + i) for i in range(16)] == values
    lib.test_init()
    for seq in range(17):
        feed(lib, encode(neutral(), 1, seq), seq * 20)
    assert lib.test_get(2) == 1
    feed(lib, encode(channels(1, brush_enabled=True, brush_percent=-25), 1, 17), 340)
    state = report(lib, 340)
    assert state.pwm == (1625, 1625, 1375, 1375)
    assert state.brush_enabled and state.brush_percent == -25


def test_real_c_session_handshake_rc_return_and_no_resume(firmware):
    lib = firmware
    core = Session(motion_enabled=True)
    core.opened()
    core.receive(report(lib, 0), 0.)

    def step(ms, rc_values=None, send=True):
        if rc_values is not None:
            lib.test_remote(encode(rc_values, 1, 0)[12:37], ms)
        if send:
            packet = core.frame(ms / 1000)
            if packet:
                lib.test_host(packet, len(packet), ms)
                core.sent(ms / 1000)
        lib.test_tick(ms)
        if ms % 100 == 0:
            core.receive(report(lib, ms), ms / 1000)

    for ms in range(20, 421, 20):
        step(ms)
    assert core.ready and report(lib, 420).armed
    token, _ = core.acquire("wall-clean", .42)
    assert core.accept(Command(token, 0, 1_000_000_000, 1_150_000_000, 1,
                               brush_enabled=True, brush_percent=30), .42, 1_000_000_000)
    step(440)
    assert report(lib, 440).pwm == (1625, 1625, 1375, 1375)
    remote = neutral()
    remote[2] = 1950
    for ms in range(460, 521, 20):
        step(ms, remote)
    assert core.ctrl.source == "RC" and not core.token
    # RC goes quiet; firmware requires 5 s, then a new epoch and neutral handshake.
    for ms in range(540, 6101, 20):
        step(ms, neutral())
    assert core.ctrl.source == "NANO" and core.ready
    assert core.command is None and not core.token
    assert report(lib, 6100).neutral
    assert not core.accept(Command(token, 1, 7_000_000_000, 7_150_000_000, 1), 6.1, 7_000_000_000)


def test_real_c_timeout_crc_fault_and_wire_wrap(firmware):
    lib = firmware
    for i in range(17):
        feed(lib, encode(neutral(), 1, (0xfffffff0 + i) & 0xffffffff), i * 20)
    assert report(lib, 320).armed and report(lib, 320).ack == 0
    feed(lib, encode(channels(1), 1, 1), 340)
    lib.test_tick(440)
    assert lib.test_get(1) == 2 and report(lib, 440).neutral
    for i in range(17):
        feed(lib, encode(neutral(), 2, i), 460 + i * 20)
    bad = bytearray(encode(channels(1), 2, 17))
    bad[15] ^= 1
    feed(lib, bytes(bad), 800)
    assert lib.test_get(1) == 3 and report(lib, 800).neutral


def test_existing_firmware_regressions(firmware_library):
    root = Path(os.environ.get("F1_FIRMWARE_DIR", Path(__file__).resolve().parents[4] / "firmware"))
    result = subprocess.run(
        ["/usr/bin/python3", "-B", "-m", "unittest", "discover", "-s", str(root / "tests"),
         "-p", "test_control_mux.py", "-v"],
        env=dict(os.environ, CONTROL_TEST_LIBRARY=str(firmware_library), PYTHONDONTWRITEBYTECODE="1"),
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Ran 26 tests" in result.stderr
    assert "skipped" not in result.stderr.lower()


def test_v2_direction_gear_group_matrix_matches_firmware(firmware):
    lib = firmware
    signs = [(0, 0, 0, 0), (1, 1, -1, -1), (-1, -1, 1, 1),
             (-1, 1, 1, -1), (1, -1, -1, 1)]
    for seq in range(17):
        feed(lib, encode(neutral(), 1, seq), seq * 20)
    seq = 17
    for mode in range(3):
        for gear in (25, 50, 75):
            for direction in range(5):
                ms = seq * 20
                feed(lib, encode(channels(direction, gear, mode), 1, seq), ms)
                state = report(lib, ms)
                expected = tuple(1500 if (mode == 0 and i < 2 or mode == 2 and i >= 2)
                                 else 1500 + sign * gear * 5 for i, sign in enumerate(signs[direction]))
                assert state.pwm == expected
                assert state.direction == ("STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT")[direction]
                assert state.speed_percent == gear and state.result == "OK"
                seq += 1


def test_v1_rejected_and_conflict_is_reported_with_independent_aux(firmware):
    import binascii
    import struct
    lib = firmware
    for seq in range(17):
        feed(lib, encode(neutral(), 1, seq), seq * 20)
    values = channels(1, brush_enabled=True, brush_percent=20)
    values[3] = 1950  # Deliberately conflicting raw frame, never produced by channels().
    feed(lib, encode(values, 1, 17), 340)
    state = report(lib, 340)
    assert state.result == "DIR_CONFLICT" and state.direction == "STOP"
    assert state.pwm == (1500,) * 4 and state.brush_enabled
    old = bytearray(encode(channels(1), 1, 18))
    old[2] = 1
    old[-2:] = struct.pack('<H', binascii.crc_hqx(old[2:37], 0xffff))
    feed(lib, bytes(old), 360)
    state = report(lib, 360)
    assert state.epoch != 1 and state.neutral


def test_ui_gateway_session_and_real_firmware_keep_lease_across_groups_and_auxiliaries(firmware):
    """Exercise the UI wire contract all the way to the compiled C control mux."""
    lib = firmware
    session = Session(motion_enabled=True, allow_fifth=True)
    session.opened()
    session.receive(report(lib, 0), 0.)
    now_ms = 0

    def step():
        nonlocal now_ms
        now_ms += 20
        now = now_ms / 1000
        frame = session.frame(now)
        assert frame is not None
        feed(lib, frame, now_ms)
        session.sent(now)
        state = report(lib, now_ms)
        session.receive(state, now)
        return state

    for _ in range(21):
        step()
    assert session.ready

    class Backend:
        def acquire(self, owner, callback):
            token, reason = session.acquire(owner, now_ms / 1000)
            callback(dict(success=bool(token), token=token, message=reason,
                          epoch=1, max_command_ttl_ms=150))

        def release(self, token, callback):
            callback(dict(success=session.release(token)))

        def publish(self, command):
            assert session.accept(command, now_ms / 1000, unix_ns())

    unix_ns = lambda: 10_000_000_000 + now_ms * 1_000_000
    replies = []
    gateway = GatewayCore(Backend(), lambda ident, packet: replies.append(packet),
                          allow_fifth_thruster=True, clock=lambda: now_ms / 1000, unix_ns=unix_ns)

    def update_gateway():
        state = session.snapshot(now_ms / 1000)
        ctrl = state.pop("ctrl")
        state.update(source=ctrl.source, epoch=ctrl.epoch, armed=ctrl.armed, result=ctrl.result)
        gateway.update_state(state)

    def send(op, **fields):
        gateway.handle(b"ui", json.dumps(dict(protocol=PROTOCOL, op=op, request_id=op, **fields)).encode())

    update_gateway()
    send("hello")
    send("acquire", owner="Qt-chain-test")
    token = replies[-1]["token"]
    assert token == session.token
    for sequence, (mode, brush, thrust) in enumerate(((1, 25, -35), (0, -50, 40), (2, 75, -20)), 1):
        update_gateway()
        gateway.tick()
        send("command", token=token, ticket=replies[-1]["ticket"], sequence=sequence,
             direction=1, speed_percent=25, mode=mode,
             brush_enabled=True, brush_percent=brush, thrust_enabled=True, thrust_percent=thrust)
        assert replies[-1]["success"]
        state = step()
        expected = tuple(1500 if mode == 0 and i < 2 or mode == 2 and i >= 2
                         else pwm for i, pwm in enumerate((1625, 1625, 1375, 1375)))
        assert state.pwm == expected and state.result == "OK"
        assert state.brush_enabled and state.brush_percent == brush
        assert state.thrust_enabled and state.thrust_percent == thrust
        assert session.token == token and gateway.clients[b"ui"].token == token
    # With no new UI computation, the original deadline still stops every output.
    for _ in range(6):
        state = step()
    assert state.neutral and not session.token
    assert session.last_stop_reason == "command deadline expired"
    gateway.close()
