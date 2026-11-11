import ctypes as c
import os
from pathlib import Path
import random
import re
import subprocess

import pytest

from nano_ros_bridge.protocol import Remote, channels, decode, encode, neutral
from nano_ros_bridge.session import Command, Session


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
    assert state.pwm == (1600, 1600, 1400, 1600)
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
    assert report(lib, 440).pwm == (1600, 1600, 1400, 1600)
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
    root = Path(os.environ.get("F1_FIRMWARE_DIR",
                               Path(__file__).resolve().parents[3] / "under" / "F103firmware"))
    result = subprocess.run(
        ["/usr/bin/python3", "-B", "-m", "unittest", "discover", "-s", str(root / "tests"),
         "-p", "test_control_mux.py", "-v"],
        env=dict(os.environ, CONTROL_TEST_LIBRARY=str(firmware_library), PYTHONDONTWRITEBYTECODE="1"),
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    count = re.search(r"Ran (\d+) tests?", result.stderr)
    assert count and int(count.group(1)) > 0, result.stderr
    assert "skipped" not in result.stderr.lower()


def test_v3_every_percent_direction_group_matches_firmware(firmware):
    lib = firmware
    signs = [(0, 0, 0, 0), (1, 1, -1, 1), (-1, -1, 1, -1),
             (-1, 1, 1, 1), (1, -1, -1, -1)]
    for seq in range(17):
        feed(lib, encode(neutral(), 1, seq), seq * 20)
    seq = 17
    for mode in range(3):
        for speed in range(101):
            for direction in range(5):
                ms = seq * 20
                feed(lib, encode(channels(direction, speed, mode), 1, seq), ms)
                state = report(lib, ms)
                expected = tuple(1500 if (mode == 0 and i < 2 or mode == 2 and i >= 2)
                                 else 1500 + sign * speed * 4 for i, sign in enumerate(signs[direction]))
                assert state.pwm == expected
                assert state.direction == ("STOP", "FORWARD", "REVERSE", "LEFT", "RIGHT")[direction if speed else 0]
                assert state.speed_percent == speed and state.result == "OK"
                assert state.protocol_version == 3
                seq += 1


@pytest.mark.parametrize("old_version", [1, 2])
def test_old_versions_rejected_and_conflict_keeps_independent_aux(firmware, old_version):
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
    old[2] = old_version
    old[-2:] = struct.pack('<H', binascii.crc_hqx(old[2:37], 0xffff))
    feed(lib, bytes(old), 360)
    state = report(lib, 360)
    assert state.epoch != 1 and state.neutral


def test_zero_percent_stops_main_motors_and_preserves_auxiliary_commands(firmware):
    for seq in range(17):
        feed(firmware, encode(neutral(), 1, seq), seq * 20)
    feed(firmware, encode(channels(1, 37, brush_enabled=True, brush_percent=50,
                                   thrust_enabled=True, thrust_percent=-20), 1, 17), 340)
    assert report(firmware, 340).pwm == (1648, 1648, 1352, 1648)
    feed(firmware, encode(channels(1, 0, brush_enabled=True, brush_percent=50,
                                   thrust_enabled=True, thrust_percent=-20), 1, 18), 360)
    state = report(firmware, 360)
    assert state.direction == "STOP" and state.speed_percent == 0
    assert state.pwm == (1500,) * 4
    assert state.brush_enabled and state.brush_percent == 50
    assert state.thrust_enabled and state.thrust_percent == -20


def test_real_c_remote_report_decodes_all_fourteen_channels_and_expires(firmware):
    # The physical remote still uses its original three-position CH6.
    values = neutral()
    values[5] = 1500
    values[11:14] = [1050, 1500, 1950]
    packet = encode(values, 1, 0)[12:37]
    firmware.test_remote(packet, 100)
    buffer = c.create_string_buffer(256)
    length = firmware.test_rc_report(100, buffer, len(buffer))
    sample = decode(buffer.raw[:length])
    assert isinstance(sample, Remote) and sample.valid
    assert tuple(sample.channels) == tuple(values[:14])
    length = firmware.test_rc_report(200, buffer, len(buffer))
    stale = decode(buffer.raw[:length])
    assert not stale.valid and tuple(stale.channels) == (1500,) * 14
