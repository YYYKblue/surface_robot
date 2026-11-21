from contextlib import ExitStack
from functools import reduce
import math
from operator import xor
import os
import pty
import signal
import subprocess
import time

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from f1_rtk_driver.msg import Heading, RawSentence, RtkStatus


GGA = (b"$GNGGA,023541.00,2304.76261531,N,11415.24485118,E,"
       b"1,18,1.5,44.5917,M,-3.1322,M,,*59\r\n")
HPR = b"$GNHPR,023541.00,101.6490,000.2235,000.0000,4,19,0.00,0999*54\r\n"


def at_epoch(line, utc, quality=None):
    fields = line[1:line.index(b"*")].split(b",")
    fields[1] = utc
    if quality is not None:
        fields[5] = quality  # HPR solution code
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


def test_installed_launch_pty_and_ros_topics(tmp_path, monkeypatch):
    # Keep injected PTY samples outside the vessel's live domain 33.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(40 + os.getpid() % 40))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    port = tmp_path / "receiver"
    config = tmp_path / "rtk.yaml"
    config.write_text("/f1/rtk/rtk_driver:\n  ros__parameters:\n"
                      "    stale_timeout_sec: 0.8\n    reconnect_sec: 0.2\n"
                      "    antenna_frame: test_ant1\n    base_frame: test_base\n")
    with ExitStack() as stack:
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        probe = rclpy.create_node("rtk_test_probe")
        stack.callback(probe.destroy_node)
        types = {"fix": NavSatFix, "heading": Heading, "status": RtkStatus, "raw": RawSentence,
                 "base_fix": NavSatFix, "bow_heading": Heading}
        received = {name: [] for name in types}
        for name, kind in types.items():
            probe.create_subscription(kind, f"/f1/rtk/{name}", received[name].append, 100)
        logfile = tmp_path / "launch.log"
        with logfile.open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "f1_rtk_driver", "rtk.launch.py", f"config:={config}", f"port:={port}"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
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
        master = stack.enter_context(attach_receiver(port))
        until(lambda: status_matches(connected=True))
        # A sentence can span reads; multiple sentences can arrive in one read.
        master.write(GGA[:20])
        time.sleep(0.06)
        master.write(GGA[20:] + HPR)
        until(lambda: status_matches(fix_valid=True, heading_valid=True)
              and all(received.values()) and len(received["raw"]) == 2)
        fix = received["fix"][-1]
        heading = received["heading"][-1]
        assert fix.header.frame_id == "test_ant1" and fix.status.status == NavSatStatus.STATUS_FIX
        assert fix.altitude == pytest.approx(41.4595)
        assert heading.valid and heading.quality == 4 and heading.heading_deg == pytest.approx(101.649)
        assert heading.utc == "023541.00" and heading.header.stamp.sec > 0
        bow = received["bow_heading"][-1]
        center = received["base_fix"][-1]
        assert bow.header.frame_id == center.header.frame_id == "test_base"
        assert bow.header.stamp == heading.header.stamp and center.header.stamp == fix.header.stamp
        assert bow.valid and bow.quality == 4 and bow.utc == heading.utc
        assert bow.heading_deg == pytest.approx(59.649)
        assert center.status == fix.status and math.isnan(center.altitude)
        assert (center.latitude, center.longitude) == pytest.approx(
            (23.079376133573, 114.254083512029), abs=1e-10, rel=0)
        assert [msg.sentence for msg in received["raw"]] == [line.decode().strip() for line in (GGA, HPR)]
        assert received["status"][-1].fix_quality == 1 and received["status"][-1].heading_quality == 4

        # A corrupt HPR must reach raw capture without refreshing the heading.
        bad = HPR.replace(b"*54", b"*55")
        master.write(bad + b"x" * 1100 + b"\r\n")
        until(lambda: status_matches(fix_valid=True, heading_valid=False, rejected_lines=2)
              and not received["heading"][-1].valid and not received["bow_heading"][-1].valid
              and received["base_fix"][-1].status.status == NavSatStatus.STATUS_NO_FIX,
              lambda: master.write(GGA))
        assert math.isnan(received["base_fix"][-1].longitude)
        assert received["heading"][-1].quality == -1 and math.isnan(received["heading"][-1].heading_deg)
        assert bad.decode().strip() in [msg.sentence for msg in received["raw"]]

        until(lambda: status_matches(fix_valid=False, heading_valid=True)
              and received["fix"][-1].status.status == NavSatStatus.STATUS_NO_FIX, lambda: master.write(HPR))
        assert math.isnan(received["fix"][-1].latitude)

        master.write(at_epoch(GGA, b"023542") + at_epoch(HPR, b"023542.000"))
        until(lambda: status_matches(fix_valid=True, heading_valid=True)
              and received["base_fix"][-1].status.status == NavSatStatus.STATUS_FIX)
        # Leave a complete unpaired fix and part of the next line in the old session.
        master.write(at_epoch(GGA, b"023543") + GGA[:20])
        until(lambda: status_matches(fix_utc="023543"))
        master.close()
        port.unlink()
        until(lambda: status_matches(connected=False, fix_valid=False, heading_valid=False,
                                     fix_quality=-1, heading_quality=-1)
              and not received["bow_heading"][-1].valid
              and received["base_fix"][-1].status.status == NavSatStatus.STATUS_NO_FIX)
        second = stack.enter_context(attach_receiver(port))
        until(lambda: status_matches(connected=True, fix_valid=False, heading_valid=False))
        before = len(received["base_fix"])
        second.write(at_epoch(HPR, b"023543.00") + at_epoch(GGA, b"023544.00"))
        until(lambda: status_matches(fix_utc="023544.00", fix_valid=True, heading_valid=True))
        second.write(at_epoch(HPR, b"023544"))
        until(lambda: len(received["base_fix"]) > before)
        assert len(received["base_fix"]) == before + 1
        assert received["base_fix"][-1].header.stamp == received["fix"][-1].header.stamp

        # Fresh but mismatched epochs must still expire the last aligned fix.
        mismatch = at_epoch(HPR, b"023545.2") + at_epoch(GGA, b"023545.4")
        until(lambda: status_matches(fix_valid=True, heading_valid=True)
              and received["base_fix"][-1].status.status == NavSatStatus.STATUS_NO_FIX,
              lambda: second.write(mismatch))
        second.write(at_epoch(HPR, b"023546.0") + at_epoch(GGA, b"023546.00"))
        until(lambda: received["base_fix"][-1].status.status == NavSatStatus.STATUS_FIX)
        second.write(at_epoch(HPR, b"023546.2", quality=b"5"))
        until(lambda: status_matches(fix_valid=True, heading_valid=False, heading_quality=5)
              and received["base_fix"][-1].status.status == NavSatStatus.STATUS_NO_FIX
              and not received["bow_heading"][-1].valid)
