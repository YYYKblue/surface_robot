"""Exercise a lake-centre survey and reload the actual saved Cartographer state."""
from functools import reduce
import math
from operator import xor
import subprocess
import time

import cv2
import pytest
import rclpy
from cartographer_ros_msgs.srv import WriteState
from f1_interfaces.msg import Heading, RawSentence
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Header
import yaml

from f1_localization.gnss_reference import read_reference
from test_gnss_slam import launch


def test_no_echo_fixed_rtk_survey(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "167")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    rclpy.init()
    peer = rclpy.create_node("open_water_test")
    try:
        water = peer.create_publisher(LaserScan, "/pavo2_scan/water", qos_profile_sensor_data)
        raw = peer.create_publisher(RawSentence, "/f1/rtk/raw", 10)
        center = peer.create_publisher(NavSatFix, "/f1/rtk/base_fix", 10)
        heading = peer.create_publisher(Heading, "/f1/rtk/bow_heading", 10)
        grids = []
        peer.create_subscription(OccupancyGrid, "/map", grids.append, 1)
        writer = peer.create_client(WriteState, "/write_state")
        east, quality, tick, hit, send_scan = 0., 1, 0, False, True

        beam_count = 4500  # deployed full-circle PAVO2 scan
        no_echo = math.inf

        def feed():
            nonlocal tick
            tick += 1
            stamp = peer.get_clock().now().to_msg()
            ranges = [no_echo] * beam_count
            if hit:
                ranges[0] = 1.5  # ray zero points east for this mounting and heading
            if send_scan:
                water.publish(LaserScan(header=Header(stamp=stamp, frame_id="pavo2s_frame"),
                    angle_min=-math.pi, angle_max=math.pi-math.tau/beam_count, angle_increment=math.tau/beam_count,
                    range_min=.1, range_max=100., ranges=ranges))
            utc = f"1200{tick//20:02d}.{tick%20*5:02d}"
            lon = 114.+east/102523.
            body = f"GNGGA,{utc},2300.000000,N,114{(lon-114)*60:09.6f},E,{quality},12,1.,0,M,0,M,,"
            header = Header(stamp=stamp, frame_id="base_link")
            raw.publish(RawSentence(header=header, sentence=f"${body}*{reduce(xor,body.encode()):02X}"))
            center.publish(NavSatFix(header=header, latitude=23., longitude=lon))
            heading.publish(Heading(header=header, utc=utc, quality=4, valid=True, heading_deg=90.))

        def wait(predicate, timeout=12):
            stop = time.monotonic()+timeout
            while time.monotonic()<stop:
                rclpy.spin_once(peer, timeout_sec=.04)
                if predicate():
                    return
            latest = [(g.info.width, g.info.height, min(g.data), max(g.data)) for g in grids[-1:]]
            pytest.fail(f"Timed out with {len(grids)} grids, latest dimensions and occupancy {latest}")

        def dwell(seconds):
            end = time.monotonic()+seconds
            wait(lambda: time.monotonic()>=end)

        def cell(grid, x, y):
            meta = grid.info
            ix = math.floor((x-meta.origin.position.x)/meta.resolution)
            iy = math.floor((y-meta.origin.position.y)/meta.resolution)
            if not (0 <= ix < meta.width and 0 <= iy < meta.height):
                return -1
            return grid.data[iy*meta.width+ix]

        peer.create_timer(.05, feed)
        state = tmp_path / "lake.pbstream"
        with launch(tmp_path,"f1_mapping_bringup","lidar_mapping.launch.py","mapping_backend:=open_water"):
            wait(writer.service_is_ready)
            dwell(.8)
            assert not grids  # only single-point GNSS
            no_echo = math.nan  # invalid measurements in otherwise live scans
            quality = 4
            dwell(1.3)
            assert not grids
            no_echo = math.inf
            wait(lambda: grids and cell(grids[-1], 1., 0.)==0)
            assert cell(grids[-1], 4., 0.)==-1
            # Fresh laser frames alone cannot expand the map after a loss of fixed position.
            quality = 5
            dwell(1.3)
            count = len(grids)
            east = 1.
            dwell(1.2)
            assert len(grids) == count
            quality = 4
            # Move the boat while every beam has no return.
            for i in range(1,9):
                east = i*.5
                dwell(.25)
            wait(lambda: cell(grids[-1], 6., 0.)==0)
            assert cell(grids[-1], 8., 0.)==-1 and max(grids[-1].data)==0
            hit = True
            wait(lambda: max(grids[-1].data)==100)
            hit = False
            dwell(1.2)
            assert max(grids[-1].data)==100
            # Grow the grid eastwards and retain the wall at its original coordinates.
            def occupied_near_wall(grid):
                return any(cell(grid, 5.28 + dx, dy) == 100
                           for dx in (-.05, 0., .05) for dy in (-.05, 0., .05))
            for i in range(9, 17):
                east = i * .5
                dwell(.25)
            wait(lambda: cell(grids[-1], 10., 0.) == 0)
            assert occupied_near_wall(grids[-1])
            assert writer.service_is_ready()
            bad = writer.call_async(WriteState.Request(filename=str(tmp_path / "missing" / "lake.pbstream")))
            wait(bad.done)
            assert bad.result().status.code != 0
            send_scan = False
            dwell(1.2)
            count = len(grids)
            east = 10.
            dwell(1.2)
            assert len(grids)==count
            future = writer.call_async(WriteState.Request(filename=str(state), include_unfinished_submaps=True))
            wait(future.done)
            assert future.result().status.code==0
        origin, _, _ = read_reference(state,0)
        assert origin.latitude==pytest.approx(23.)
        subprocess.run(["ros2","run","cartographer_ros","cartographer_pbstream_to_ros_map",
            "-pbstream_filename",str(state),"-map_filestem",str(state.with_suffix("")),"-resolution","0.05"],
            check=True,timeout=20)
        meta = yaml.safe_load(state.with_suffix(".yaml").read_text())
        pixels = cv2.imread(str(state.with_suffix(".pgm")),cv2.IMREAD_GRAYSCALE)
        def pixel(x,y):
            return pixels[pixels.shape[0]-1-math.floor((y-meta["origin"][1])/meta["resolution"]),
                          math.floor((x-meta["origin"][0])/meta["resolution"])]
        assert pixel(10, 0) > 220 and pixels.min() < 65
        grids.clear()
        with launch(tmp_path,"f1_localization","localization.launch.py",f"map:={state}"):
            wait(lambda: grids and cell(grids[-1], 10., 0.) in range(20),25)
    finally:
        peer.destroy_node()
        rclpy.shutdown()
