"""Exercise GNSS constraints through both installed Cartographer launch entry points."""
from contextlib import contextmanager
from functools import reduce
import json
import math
from operator import xor
import os
import re
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from cartographer_ros_msgs.msg import StatusCode
from cartographer_ros_msgs.srv import WriteState
from f1_interfaces.msg import RawSentence
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from f1_localization.gnss_reference import read_reference
from f1_mapping_bringup.gnss_positions import PlanarPose


@contextmanager
def launch(tmp_path, package, filename, *args):
    with (tmp_path / (package + ".log")).open("w") as log:
        process = subprocess.Popen(["ros2", "launch", package, filename, *args],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    assert "process has died" not in (tmp_path / (package + ".log")).read_text()


def room_scan(east, yaw):
    # Four walls observed from the configured rear-facing, aft-mounted lidar.
    sx, sy = east - .2143195 * math.cos(yaw), -.2143195 * math.sin(yaw)
    ranges = []
    for i in range(360):
        angle = yaw + math.radians(i)  # ray zero: mounting yaw pi + scan angle -pi
        c, s = math.cos(angle), math.sin(angle)
        dx = ((5 if c > 0 else -3) - sx) / c if abs(c) > 1e-8 else math.inf
        dy = ((4 if s > 0 else -2) - sy) / s if abs(s) > 1e-8 else math.inf
        ranges.append(min(dx, dy))
    return LaserScan(angle_min=-math.pi, angle_max=math.pi - math.tau / 360,
                     angle_increment=math.tau / 360, range_min=.1, range_max=15.,
                     scan_time=.05, time_increment=.05 / 360, ranges=ranges)


def test_mapping_save_reload_manual_seed_and_position_outages(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "163")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init()
    peer = rclpy.create_node("gnss_survey_test")
    try:
        raw = peer.create_publisher(RawSentence, "/f1/rtk/raw", 10)
        scans = peer.create_publisher(LaserScan, "/pavo2_scan", qos_profile_sensor_data)
        writer = peer.create_client(WriteState, "/write_state")
        markers, constraints, states = [], [], []
        grids = []
        peer.create_subscription(OccupancyGrid, "/map", grids.append, 1)
        peer.create_subscription(MarkerArray, "/landmark_poses_list", markers.append, 10)
        peer.create_subscription(PointStamped, "/f1/rtk/position_constraint", constraints.append, 10)
        peer.create_subscription(String, "/pool_cleaning/localization_status",
                                 lambda m: states.append(json.loads(m.data)), 10)
        survey = tmp_path / "survey.pbstream"
        east, yaw = 0., 0.
        scan = room_scan(east, yaw)
        tick = 0
        send_gnss = send_lidar = True
        surveying = True

        def feed():
            nonlocal tick, yaw, scan, east
            tick += 1
            now = peer.get_clock().now()
            stamp = now.to_msg()
            if surveying:
                yaw = .35 * math.sin(tick / 35.)
                east = 2.3 * math.sin(tick / 80.)
                scan = room_scan(east, yaw)
            if send_lidar:
                scan.header = Header(stamp=(now - Duration(seconds=.05)).to_msg(), frame_id="pavo2s_frame")
                scans.publish(scan)
            if send_gnss:
                utc = f"12{(tick // 1200) % 60:02}{(tick // 20) % 60:02}.{(tick % 20) * 5:02}"
                # Known mounting and ENU transform; the GGA measures the rotating antenna.
                x = east - .191*math.cos(yaw) - .213*math.sin(yaw)
                y = -.191*math.sin(yaw) + .213*math.cos(yaw)
                lon = 114.25 + x / 102523.
                lat = 23.25 + y / 110750.
                body = (f"GNGGA,{utc},23{(lat-23)*60:09.6f},N,114{(lon-114)*60:09.6f},E,"
                        "1,12,1.1,0,M,0,M,,")
                raw.publish(RawSentence(header=Header(stamp=stamp, frame_id="gnss_ant1"),
                                        sentence=f"${body}*{reduce(xor, body.encode()):02X}"))

        peer.create_timer(.05, feed)

        def until(predicate, timeout=15):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(peer, timeout_sec=.04)
                if predicate():
                    return
            pytest.fail(f"Timed out; constraints={len(constraints)}, recent state={states[-1:]}")

        with launch(tmp_path, "f1_mapping_bringup", "lidar_mapping.launch.py"):
            # Wait for the survey to populate free space at the map exporter's threshold.
            def survey_ready():
                return (markers and markers[-1].markers and grids and
                        sum(0 <= c < 18 for c in grids[-1].data) > 1000 and writer.service_is_ready())
            until(survey_ready, 60)
            assert len(constraints) >= 3
            future = writer.call_async(WriteState.Request(filename=str(survey), include_unfinished_submaps=True))
            until(future.done)
            assert future.result().status.code == StatusCode.OK
        origin, map_from_enu, _ = read_reference(survey, 0)
        assert abs(origin.latitude - 23.25) < .0001
        assert abs(origin.longitude - 114.25) < .0001
        subprocess.run(["ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                        "-pbstream_filename", str(survey), "-map_filestem", str(survey.with_suffix("")),
                        "-resolution", "0.05"], check=True, timeout=15)

        # Reload the map at a different position and compass heading using a manual seed.
        surveying = False
        east, yaw = .7, math.pi / 2
        scan = room_scan(east, yaw)
        with launch(tmp_path, "f1_localization", "localization.launch.py", f"map:={survey}"):
            tf = Buffer()
            listener = TransformListener(tf, peer)
            pose_pub = peer.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            until(lambda: states and pose_pub.get_subscription_count() and len(states) >= 10)
            assert not states[-1]["initial_pose_received"] and not states[-1]["ready"]
            x, y = origin.project(23.25, 114.25 + east / 102523.)
            expected = map_from_enu.compose(PlanarPose(x, y, yaw))
            seed = PoseWithCovarianceStamped()
            seed.header = Header(stamp=peer.get_clock().now().to_msg(), frame_id="map")
            seed.pose.pose = expected.message()
            pose_pub.publish(seed)
            until(lambda: states[-1]["ready"], 25)
            assert states[-1]["initial_pose_source"] == "rviz"
            actual = tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            assert math.hypot(actual.translation.x - expected.x, actual.translation.y - expected.y) < .35
            actual_yaw = 2 * math.atan2(actual.rotation.z, actual.rotation.w)
            assert abs(math.remainder(actual_yaw - expected.yaw, math.tau)) < .15
            # Allow one fixed-map optimization while both inputs are available.
            count = len(constraints)
            until(lambda: len(constraints) >= count + 5 and constraints[-1].header.frame_id == "map")
            send_gnss = False
            last_status = states[-1]["stamp_ns"]
            end = time.monotonic() + 1.2
            until(lambda: time.monotonic() >= end)
            assert states[-1]["ready"]
            assert states[-1]["stamp_ns"] > last_status
            count = len(constraints)
            send_lidar = False
            until(lambda: not states[-1]["ready"])
            assert len(constraints) == count
            send_gnss = True
            until(lambda: len(constraints) > count)
            assert not states[-1]["ready"]
            snapshot = tmp_path / "localization.pbstream"
            future = writer.call_async(WriteState.Request(filename=str(snapshot), include_unfinished_submaps=True))
            until(future.done)
            assert future.result().status.code == StatusCode.OK
            info = subprocess.run(["cartographer_pbstream", "--all_debug_strings", "info", str(snapshot)],
                                  capture_output=True, text=True, timeout=10, check=True)
            state_text = info.stderr + info.stdout
            active_id = states[-1]["trajectory_id"]
            assert re.search(rf"Serialized data: fixed_frame_pose_data \{{\s+trajectory_id: {active_id}\s+"
                             r"fixed_frame_pose_data \{\s+timestamp: \d+\s+pose \{", state_text)
            assert "fixed_frame_origin_in_map {" in state_text

            # Retiring the GNSS trajectory must leave the backend usable for the next manual seed.
            reset = peer.create_client(Trigger, "/pool_cleaning/relocalize")
            until(reset.service_is_ready)
            future = reset.call_async(Trigger.Request())
            until(future.done)
            assert future.result().success
            until(lambda: states[-1]["trajectory_id"] == -1 and not states[-1]["initial_pose_received"])
            send_lidar = True
            pose_pub = peer.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            until(lambda: pose_pub.get_subscription_count() > 0)
            seed = PoseWithCovarianceStamped()
            seed.header = Header(stamp=peer.get_clock().now().to_msg(), frame_id="map")
            seed.pose.pose = expected.message()
            pose_pub.publish(seed)
            until(lambda: states[-1]["ready"] and states[-1]["initial_pose_source"] == "rviz")
    finally:
        peer.destroy_node()
        rclpy.try_shutdown()
