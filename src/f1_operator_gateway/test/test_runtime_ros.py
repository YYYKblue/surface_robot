"""Real SLAM -> write_state -> raster export -> app map selection, synthetic scans only."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import websockets

from operator_gateway.node import OperatorGateway


def test_real_mapping_save_reconnect_and_load_without_hardware(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'port:=0', '-p', 'managed_runtime:=true',
                     '-p', 'workspace_root:=' + str(tmp_path), '-p', 'localization_prefix:=/f1_localization'])
    gateway = OperatorGateway()
    peer = Node('operator_synthetic_lidar')
    publisher = peer.create_publisher(LaserScan, '/pavo2_scan', qos_profile_sensor_data)
    scan = LaserScan()
    scan.header.frame_id = 'pavo2s_frame'
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 360
    scan.angle_max = scan.angle_min + 359 * scan.angle_increment
    scan.range_min, scan.range_max, scan.scan_time = .1, 15., .02
    started = time.monotonic()
    def publish():
        # Rotate the sensor in a fixed rectangular room. Identical stationary
        # scans are intentionally rejected by Cartographer's motion filter.
        yaw = (time.monotonic() - started) * .5
        scan.ranges = [min(3/max(abs(math.cos(scan.angle_min+i*scan.angle_increment+yaw)), 1e-6),
                           2/max(abs(math.sin(scan.angle_min+i*scan.angle_increment+yaw)), 1e-6)) for i in range(360)]
        scan.header.stamp = peer.get_clock().now().to_msg()
        publisher.publish(scan)
    peer.create_timer(.02, publish)
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(peer)
    pool = ThreadPoolExecutor(max_workers=1)
    port = gateway.hub.server.sockets[0].getsockname()[1]

    async def scenario():
        async def until(ws, predicate, seconds=45):
            deadline = time.monotonic()+seconds
            last = None
            while time.monotonic() < deadline:
                last = json.loads(await asyncio.wait_for(ws.recv(), 10))
                if predicate(last):
                    return last
            logs = {p.name: p.read_text(errors='replace')[-4000:] for p in (tmp_path/'log/operator_runtime').glob('*.log')}
            raise AssertionError((last, logs))
        async def send(ws, packet):
            await ws.send(json.dumps(packet))
        async with websockets.connect(f'ws://127.0.0.1:{port}', max_size=16*1024*1024) as ws:
            await send(ws, dict(op='hello', client='mapping-test'))
            hello = await until(ws, lambda p: p.get('type') == 'hello')
            assert 'mapping' in hello['capabilities']
            await until(ws, lambda p: p.get('runtime', {}).get('scan_fresh'))
            await send(ws, dict(op='mapping', action='start', request_id='start'))
            reply = await until(ws, lambda p: p.get('request_id') == 'start')
            assert reply['success'], reply
            ready = await until(ws, lambda p: p.get('runtime', {}).get('map_available') and p.get('pose'))
            session = ready['runtime']['session_id']
            assert not ready['can_set_initial_pose']
            # First grid can contain only weak/unknown probabilities. Accumulate
            # repeated rays before expecting an export usable for localization.
            await until(ws, lambda p: p.get('type') == 'map' and
                        sum(0 <= v < 20 for v in (p.get('map') or {}).get('data', [])) > 100)
            await send(ws, dict(op='task', action='start', task_type='rectangle', request_id='blocked'))
            assert not (await until(ws, lambda p: p.get('request_id') == 'blocked'))['success']
        # Disconnect stops manual intent but keeps the SLAM session alive.
        await asyncio.sleep(.3)
        async with websockets.connect(f'ws://127.0.0.1:{port}', max_size=16*1024*1024) as ws:
            await send(ws, dict(op='hello', client='mapping-reconnect'))
            reconnected = await until(ws, lambda p: p.get('runtime', {}).get('session_id') == session)
            assert reconnected['runtime']['mode'] == 'mapping'
            await send(ws, dict(op='mapping', action='save_end', name='测试池', request_id='save'))
            saved = await until(ws, lambda p: p.get('request_id') == 'save' and not p.get('pending'), seconds=60)
            assert saved['success'], saved
            assert all(Path(path).is_file() for path in saved['saved']['paths'])
            await send(ws, dict(op='mapping', action='list', request_id='list'))
            listing = await until(ws, lambda p: p.get('request_id') == 'list')
            assert listing['maps'][0]['name'] == '测试池' and listing['maps'][0]['valid']
            await send(ws, dict(op='mapping', action='load', name='测试池', request_id='load'))
            loaded = await until(ws, lambda p: p.get('request_id') == 'load', seconds=45)
            assert loaded['success'], loaded
            status = await until(ws, lambda p: p.get('runtime', {}).get('phase') == 'LOCALIZATION')
            assert status['runtime']['current_map'] == '测试池'
            assert not status['localization']['ready'] and status['can_set_initial_pose']
            assert status['runtime']['session_id'] != session
            assert all(not t['can_start'] for t in status['tasks'].values())
            # Re-enter SLAM from a loaded map; retire old TF/controllers first.
            await send(ws, dict(op='mapping', action='start', request_id='second-start'))
            restarted = await until(ws, lambda p: p.get('request_id') == 'second-start')
            assert restarted['success'], restarted
            fresh = await until(ws, lambda p: p.get('runtime', {}).get('phase') == 'MAPPING')
            assert fresh['runtime']['session_id'] not in (session, status['runtime']['session_id'])
            assert not fresh['localization']['ready'] and not fresh['can_set_initial_pose']
            await send(ws, dict(op='mapping', action='discard', confirm=True, request_id='discard'))
            assert (await until(ws, lambda p: p.get('request_id') == 'discard'))['success']
            idle = await until(ws, lambda p: p.get('runtime', {}).get('phase') == 'IDLE')
            assert not idle['runtime']['map_available'] and not idle['pose']
    future = pool.submit(asyncio.run, scenario())
    try:
        deadline = time.monotonic()+155
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert future.done(), gateway.runtime
        future.result()
        assert peer.get_node_names().count('f1_operator_gateway') == 1
    finally:
        gateway.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        pool.shutdown(wait=True)
