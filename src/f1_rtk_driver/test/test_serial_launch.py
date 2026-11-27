from contextlib import ExitStack, contextmanager
from functools import reduce
import math
import json
from operator import xor
import os
import pty
import signal
import subprocess
import time
from types import SimpleNamespace

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus

from f1_interfaces.msg import RawSentence, RtkStatus


GGA = (b"$GNGGA,023541.00,2304.76261531,N,11415.24485118,E,"
       b"1,18,1.5,44.5917,M,-3.1322,M,,*59\r\n")
HPR = b"$GNHPR,023541.00,101.6490,000.2235,000.0000,4,19,0.00,0999*54\r\n"


def at_epoch(line, utc):
    fields = line[1:line.index(b"*")].split(b",")
    fields[1] = utc
    body = b",".join(fields)
    return b"$" + body + f"*{reduce(xor, body, 0):02X}\r\n".encode()


def attach_receiver(link):
    master, slave = pty.openpty()
    try:
        link.symlink_to(os.ttyname(slave))
    finally:
        os.close(slave)
    return os.fdopen(master, "wb", buffering=0)


def stop_launch(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def observed_hz(messages):
    span = Time.from_msg(messages[-1].header.stamp) - Time.from_msg(messages[0].header.stamp)
    return (len(messages) - 1) * 1e9 / span.nanoseconds


@contextmanager
def launch_driver(tmp_path, monkeypatch, config_text=None):
    # Keep injected PTY samples outside the vessel's live domain 33.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(40 + os.getpid() % 40))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.delenv("F1_RTK_NTRIP_CONFIG", raising=False)
    port = tmp_path / "receiver"
    command = ["ros2", "launch", "f1_rtk_driver", "rtk.launch.py", f"port:={port}"]
    if config_text is not None:
        config = tmp_path / "rtk.yaml"
        config.write_text(config_text)
        command.append(f"config:={config}")
    with ExitStack() as stack:
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        probe = rclpy.create_node("rtk_test_probe")
        stack.callback(probe.destroy_node)
        types = {"fix": NavSatFix, "status": RtkStatus, "raw": RawSentence}
        received = {name: [] for name in types}
        for name, kind in types.items():
            probe.create_subscription(kind, f"/f1/rtk/{name}", received[name].append, 100)
        logfile = tmp_path / "launch.log"
        with logfile.open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        stack.callback(stop_launch, process)

        def until(predicate, repeat=None):
            deadline = time.monotonic() + 8
            next_write = 0
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.03)
                if predicate():
                    return
                assert process.poll() is None, logfile.read_text()
                if repeat and time.monotonic() >= next_write:
                    repeat()
                    next_write = time.monotonic() + 0.1
            pytest.fail(f"Timed out waiting for driver. Launch log:\n{logfile.read_text()}")

        def status_matches(**values):
            return bool(received["status"]) and all(
                getattr(received["status"][-1], key) == value for key, value in values.items())

        until(lambda: all(probe.count_publishers(f"/f1/rtk/{name}") == 1 for name in types)
              and status_matches(connected=False, fix_quality=-1))

        def connect():
            master = stack.enter_context(attach_receiver(port))
            until(lambda: status_matches(connected=True))
            return master

        yield probe, received, until, status_matches, connect


def test_installed_launch_pty_and_ros_topics(tmp_path, monkeypatch):
    config = ("/f1/rtk/rtk_driver:\n  ros__parameters:\n"
              "    stale_timeout_sec: 0.8\n    reconnect_sec: 0.2\n    status_rate_hz: 5.0\n"
              "    antenna_frame: test_ant1\n")
    with launch_driver(tmp_path, monkeypatch, config) as (probe, received, until, matches, connect):
        until(lambda: len(received["status"]) >= 7)
        assert observed_hz(received["status"]) == pytest.approx(5.0, rel=.15)
        master = connect()
        master.write(GGA[:20])
        time.sleep(.06)
        master.write(GGA[20:] + HPR)
        until(lambda: matches(fix_valid=True, heading_valid=False, heading_quality=-1)
              and len(received["raw"]) == 2 and received["fix"])
        fix = received["fix"][-1]
        assert fix.header.frame_id == "test_ant1" and fix.status.status == NavSatStatus.STATUS_FIX
        assert fix.altitude == pytest.approx(41.4595)
        assert matches(heading_age_sec=-1.)
        for topic in ("heading", "bow_heading", "base_fix"):
            assert probe.count_publishers("/f1/rtk/" + topic) == 0
        master.write(GGA.replace(b"*59", b"*00"))
        until(lambda: matches(rejected_lines=1))
        until(lambda: matches(fix_valid=False) and received["fix"][-1].status.status == -1)
        assert math.isnan(received["fix"][-1].latitude)
        master.close()
        until(lambda: matches(connected=False, fix_quality=-1))
        (tmp_path / "receiver").unlink()
        second = connect()
        second.write(GGA)
        until(lambda: matches(fix_valid=True, fix_quality=1))


def test_twenty_hz_receiver_stream(tmp_path, monkeypatch):
    with launch_driver(tmp_path, monkeypatch) as (probe, received, until, matches, connect):
        master = connect()
        master.write(GGA)
        until(lambda: matches(fix_valid=True) and all(received.values()))
        for messages in received.values():
            messages.clear()
        epochs = [f"1200{i // 20:02d}.{(i % 20) * 5:02d}" for i in range(200)]
        start = time.monotonic()
        for i, utc in enumerate(epochs):
            deadline = start + i * .05
            while (remaining := deadline - time.monotonic()) > 0:
                rclpy.spin_once(probe, timeout_sec=remaining)
            # GGA only, then good and corrupt HPR alongside identical positions.
            extra = b"" if i < 100 else (HPR if i % 2 else HPR.replace(b"*54", b"*00"))
            master.write(at_epoch(GGA, utc.encode()) + extra)
        until(lambda: len(received["fix"]) == len(epochs) and matches(fix_utc=epochs[-1]))
        assert all(f.latitude == received["fix"][0].latitude and f.longitude == received["fix"][0].longitude
                   and f.status.status == NavSatStatus.STATUS_FIX for f in received["fix"])
        # The simulated receiver and the status timer run on steady clocks.
        # A host wall-clock correction changes ROS header stamps during a run.
        elapsed = time.monotonic() - start
        assert (len(received["fix"]) - 1) / elapsed == pytest.approx(20, rel=.1)
        assert len(received["status"]) / elapsed == pytest.approx(20, rel=.1)
        assert matches(heading_valid=False, rejected_lines=0)


def test_ntrip_writes_only_rtcm_while_publishing_twenty_hz(tmp_path, monkeypatch):
    from test_ntrip import Caster, rtcm

    with Caster() as caster:
        private = tmp_path / "ntrip.json"
        private.write_text(json.dumps(dict(host="127.0.0.1", port=caster.port, mountpoint="TEST",
                                          username="test-user", password="test-secret",
                                          gga_interval_sec=0.2, reconnect_sec=0.2)))
        private.chmod(0o600)
        config = ("/f1/rtk/rtk_driver:\n  ros__parameters:\n"
                  f"    ntrip_config_file: '{private}'\n")
        with launch_driver(tmp_path, monkeypatch, config) as (probe, received, until, matches, connect):
            diagnostics = []
            probe.create_subscription(DiagnosticArray, "/f1/rtk/corrections", diagnostics.append, 10)
            master = connect()
            os.set_blocking(master.fileno(), False)
            wire = bytearray()

            def feed(utc):
                master.write(at_epoch(GGA.replace(b",1,18,", b",4,18,"), utc))
                try:
                    wire.extend(os.read(master.fileno(), 65536))
                except BlockingIOError:
                    pass

            until(lambda: caster.ggas and wire and matches(fix_quality=4), lambda: feed(b"120000.00"))
            for messages in received.values():
                messages.clear()
            start = time.monotonic()
            epochs = [f"1200{1 + i // 20:02d}.{i % 20 * 5:02d}" for i in range(100)]
            for i, epoch in enumerate(epochs):
                deadline = start + i * 0.05
                while (remaining := deadline - time.monotonic()) > 0:
                    rclpy.spin_once(probe, timeout_sec=remaining)
                feed(epoch.encode())

            def measured(name):
                # DDS may deliver a warm-up sample after the counters are cleared.
                # Select the exact 100 measured GGA epochs via their shared receipt stamp.
                stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in received["raw"]
                          if m.sentence.startswith("$GNGGA,") and m.sentence.split(",")[1] in epochs}
                return [m for m in received[name] if (m.header.stamp.sec, m.header.stamp.nanosec) in stamps]

            until(lambda: len(measured("fix")) >= 100
                  and diagnostics and diagnostics[-1].status[0].message == "streaming")
            assert len(measured("fix")) == 100
            assert observed_hz(measured("fix")) == pytest.approx(20, rel=0.1)
            assert matches(fix_quality=4, heading_quality=-1, rejected_lines=0)
            assert wire and len(wire) % len(rtcm()) == 0
            assert bytes(wire) == rtcm() * (len(wire) // len(rtcm()))
            values = {entry.key: entry.value for entry in diagnostics[-1].status[0].values}
            assert int(values["written_bytes"]) > 0 and values["fix_quality"] == "4"
            assert "test-secret" not in str(diagnostics)

            # Disconnect the network while continuing receiver input, then recover.
            caster.drop.set()
            until(lambda: len(caster.headers) >= 2 and matches(fix_quality=4), lambda: feed(b"120006.00"))
            # Invalid GGA closes the stream and no old location is resent.
            invalid = GGA.replace(b",1,18,", b",0,18,")
            master.write(at_epoch(invalid, b"120007.00") + at_epoch(HPR, b"120007.00"))
            until(lambda: matches(fix_valid=False) and diagnostics[-1].status[0].message == "waiting_gga")


def test_bad_private_config_does_not_stop_acquisition(tmp_path, monkeypatch):
    private = tmp_path / "bad.json"
    private.write_text('{"password":"test-secret"}')
    private.chmod(0o600)
    config = f"/f1/rtk/rtk_driver:\n  ros__parameters:\n    ntrip_config_file: '{private}'\n"
    with launch_driver(tmp_path, monkeypatch, config) as (probe, received, until, matches, connect):
        diagnostics = []
        probe.create_subscription(DiagnosticArray, "/f1/rtk/corrections", diagnostics.append, 10)
        master = connect()
        until(lambda: matches(fix_valid=True, heading_valid=False) and diagnostics,
              lambda: master.write(GGA + HPR))
        assert diagnostics[-1].status[0].message == "config_error"
        assert "test-secret" not in (tmp_path / "launch.log").read_text()


def test_partial_serial_writes_resume_and_old_session_tail_is_discarded(monkeypatch):
    from test_ntrip import GGA as VALID_GGA, rtcm
    from wit_rtk.node import RtkDriver
    from wit_rtk.ntrip import Config, NtripClient

    client = NtripClient(Config("localhost", 8002, "TEST", "user", "password"))
    client.set_receiver(True)
    now = time.monotonic()
    client.update_gga(VALID_GGA, now)
    client.queue.append((now, rtcm()))
    client.queued_bytes = len(rtcm())
    driver = SimpleNamespace(ntrip=client, pending_rtcm=None, last_rtcm_write=None,
                             device=SimpleNamespace(fileno=lambda: 42))
    output = bytearray()
    calls = 0

    def write(_, data):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise BlockingIOError()
        size = min(2, len(data))
        output.extend(data[:size])
        return size

    monkeypatch.setattr(os, "write", write)
    RtkDriver.write_corrections(driver, now)
    assert bytes(output) == rtcm()[:2] and driver.pending_rtcm is not None
    RtkDriver.write_corrections(driver, now)
    assert bytes(output) == rtcm() and driver.pending_rtcm is None
    assert client.snapshot()["written_bytes"] == len(rtcm())

    # A pending tail from before a receiver disconnect must not enter the new session.
    driver.pending_rtcm = (client.generation, now, rtcm()[2:])
    client.set_receiver(False)
    client.set_receiver(True)
    client.update_gga(VALID_GGA, now)
    RtkDriver.write_corrections(driver, now)
    assert bytes(output) == rtcm() and driver.pending_rtcm is None
