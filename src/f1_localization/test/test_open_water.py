"""No-return beams and independent position fixes cannot clear unknown water."""
import math
import subprocess
import time
from functools import reduce
from operator import xor

import rclpy
from cartographer_ros_msgs.msg import StatusCode
from cartographer_ros_msgs.srv import WriteState
from f1_interfaces.msg import RawSentence
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from test_gnss_slam import launch, room_scan


def cell(grid, x, y):
    ix = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    iy = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    assert 0 <= ix < grid.info.width and 0 <= iy < grid.info.height
    return grid.data[iy * grid.info.width + ix]


def test_no_echo_remains_unknown(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "167")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setenv("F1_MAPPING_BACKEND", "open_water")
    rclpy.init()
    peer = rclpy.create_node("water_scan_test")
    try:
        scans = peer.create_publisher(LaserScan, "/pavo2_scan", qos_profile_sensor_data)
        receiver = peer.create_publisher(RawSentence, "/f1/rtk/raw", 10)
        positions = []
        peer.create_subscription(PointStamped, "/f1/rtk/position_constraint", positions.append, 10)
        grids = []
        peer.create_subscription(OccupancyGrid, "/map", grids.append, 1)
        writer = peer.create_client(WriteState, "/write_state")
        scan = room_scan(0., 0.)
        scan.header.frame_id = "pavo2s_frame"
        room = list(scan.ranges)
        # Alternate all three common encodings of missing range. A later
        # mixed scan leaves the entire east-facing quadrant unobserved.
        no_echo = [0., math.inf, math.nan] * 120
        scan.ranges = no_echo
        tick = 0

        def publish():
            nonlocal tick
            scan.header.stamp = peer.get_clock().now().to_msg()
            scans.publish(scan)
            tick += 1
            utc = f"12{tick // 1200 % 60:02}{tick // 20 % 60:02}.{tick % 20 * 5:02}"
            body = f"GNGGA,{utc},2300.000,N,11400.000,E,4,12,1.0,0,M,0,M,,"
            receiver.publish(RawSentence(header=Header(stamp=scan.header.stamp, frame_id="gnss_ant1"),
                                         sentence=f"${body}*{reduce(xor, body.encode()):02X}"))
        timer = peer.create_timer(.055, publish)

        def spin(seconds, done=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not done():
                rclpy.spin_once(peer, timeout_sec=.04)

        with launch(tmp_path, "f1_mapping_bringup", "lidar_mapping.launch.py"):
            spin(12, writer.service_is_ready)
            assert writer.service_is_ready()
            spin(8, lambda: len(positions) >= 3)
            assert len(positions) >= 3
            assert all(all(c < 0 for c in g.data) for g in grids)
            scan.ranges = [no_echo[i] if i < 45 or i > 315 else r for i, r in enumerate(room)]
            # Accumulate enough real returns for the exported free-space threshold (0.196).
            spin(40, lambda: grids and 0 <= cell(grids[-1], -1., 0.) < 18)
            assert grids
            grid = grids[-1]
            assert max(grid.data) >= 60
            # Tracking starts at ANT1; the scanner faces aft. Sample well inside
            # the missing sector, away from neighboring real rays.
            assert all(cell(grid, x, 0.) == -1 for x in (.5, 1., 2.))
            assert cell(grid, -1., 0.) in range(18)
            scan.ranges = no_echo
            spin(2.)
            assert all(cell(grids[-1], x, 0.) == -1 for x in (.5, 1., 2.))
            survey = tmp_path / "no_echo.pbstream"
            saved = writer.call_async(WriteState.Request(filename=str(survey), include_unfinished_submaps=True))
            spin(10, saved.done)
            assert saved.done() and saved.result().status.code == StatusCode.OK
        timer.cancel()
        grids.clear()
        # Load just the saved submaps, without starting a localization trajectory.
        subprocess.run(["ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                        "-pbstream_filename", str(survey), "-map_filestem", str(survey.with_suffix("")),
                        "-resolution", "0.05"], check=True, timeout=15)
        with launch(tmp_path, "f1_localization", "localization.launch.py", f"map:={survey}"):
            spin(15, lambda: bool(grids))
            assert grids
            assert all(cell(grids[-1], x, 0.) == -1 for x in (.5, 1., 2.))
            assert cell(grids[-1], -1., 0.) in range(50)
    finally:
        peer.destroy_node()
        rclpy.try_shutdown()
