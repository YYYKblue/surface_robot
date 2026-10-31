from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

from operator_gateway.runtime import RuntimeControl
from operator_gateway.map_store import MapStore
from test_task_control import Gateway
from test_map_store import triplet


class RuntimeGateway(RuntimeControl, Gateway):
    def __init__(self, tmp_path):
        super().__init__()
        self.runtime_enabled = True
        self.runtime = dict(mode='idle', phase='IDLE', current_map='', session_id='', last_saved=None)
        self.runtime_pending = self.runtime_process = None
        self.runtime_accept_map = False
        self.runtime_epoch_ns = 0
        self.runtime_scan_received = time.monotonic()
        self.runtime_scan_valid = True
        self.runtime_pool = ThreadPoolExecutor(max_workers=1)
        self.map_store = MapStore(tmp_path)
        self.runtime_log = tmp_path / 'log'
        self.params.update(mapping_resolution=.05, scan_topic='/pavo2_scan', publish_lidar_tf=True,
                           output_enabled=False, rectangle_config='', l_config='', localization_config='')
        self.pose = dict(x=0, y=0, yaw=0)
        self.map_signature = None
        self.tf_buffer = SimpleNamespace(clear=lambda: None)
        self.hub.broadcast = lambda packet: self.sent.append(packet)
        self.names = []
        self.get_node_names = lambda: self.names
        self.count_publishers = lambda topic: 0
        self.write_state_client = self.create_client(None, '/write_state')
        self.write_state_type = SimpleNamespace(Request=lambda **kwargs: SimpleNamespace(**kwargs))
        self.manual_stops = []
        self._stop_manual = self.manual_stops.append

    def close(self):
        self.runtime_pool.shutdown()


def request(action, **fields):
    return dict(dict(op='mapping', action=action, request_id=action), **fields)


def tick_until(node, predicate, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        node._runtime_tick()
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError((node.runtime, node.runtime_pending, node.sent))


@pytest.fixture
def node(tmp_path):
    value = RuntimeGateway(tmp_path)
    yield value
    value.close()


def test_busy_and_stale_scan_prevent_mapping_and_manual_is_interlocked(node):
    node.task_locks.add('l')
    node._runtime_request('app', request('start'))
    assert not node.sent[-1]['success'] and node.runtime_pending is None
    node.task_locks.clear()
    node.runtime_scan_valid = False
    node._runtime_request('app', request('start', request_id='start2'))
    assert '雷达' in node.sent[-1]['message']
    node.runtime_scan_valid = True
    node._runtime_request('app', request('start', request_id='start3'))
    assert node._tasks_busy() and not node._runtime_task_allowed()
    node.active_client = None
    node._runtime_tick()
    assert node.runtime_pending is None and not node.sent[-1]['success']


def test_external_cartographer_conflict_never_starts_or_kills_process(node, monkeypatch):
    monkeypatch.setattr('operator_gateway.runtime.ManagedProcess', lambda *args: pytest.fail('must not launch'))
    node.names = ['cartographer_node']
    node._runtime_request('app', request('start'))
    node._runtime_tick()
    node._runtime_tick()
    node.runtime_pending['since'] -= 9
    node._runtime_tick()
    assert node.runtime_pending is None
    assert not node.sent[-1]['success'] and '重复' in node.sent[-1]['message']


def test_duplicate_start_only_spawns_once_and_reset_clears_old_map(node, monkeypatch):
    processes = []
    def spawn(command, log):
        processes.append(command)
        return SimpleNamespace(alive=lambda: True, stop=lambda: None)
    monkeypatch.setattr('operator_gateway.runtime.ManagedProcess', spawn)
    packet = request('start')
    node._runtime_request('app', packet)
    node._runtime_request('app', packet)
    assert node.sent[-1]['pending']
    tick_until(node, lambda: node.runtime_pending is None)
    assert node.runtime['mode'] == 'mapping' and node.runtime['phase'] == 'MAPPING'
    assert node.map_data is None and not node.tasks and not node.localization
    node._runtime_request('app', packet)
    assert len(processes) == 1
    assert 'mode:=mapping' in processes[0]


def test_save_failure_preserves_mapping_and_snapshot_then_retry_succeeds(node, monkeypatch):
    node.runtime.update(mode='mapping', phase='MAPPING')
    node.runtime_process = SimpleNamespace(alive=lambda: True, stop=lambda: None)
    node._runtime_request('app', request('save_end', name='新场地'))
    tick_until(node, lambda: node.runtime_pending['stage'] == 'writing')
    write, future = node.write_state_client.calls[0]
    Path(write.filename).write_bytes(b'cartographer-state')
    def failure(*args):
        raise RuntimeError('export failed')
    monkeypatch.setattr('operator_gateway.runtime.export_map', failure)
    future.set_result(SimpleNamespace(status=SimpleNamespace(code=0, message='OK')))
    tick_until(node, lambda: node.runtime_pending is None)
    assert node.runtime['mode'] == 'mapping' and node.runtime['phase'] == 'MAPPING'
    assert not node.sent[-1]['success']
    assert not node.map_store.catalog() and len(node.map_store.recovered()) == 1
    def export(stem, *args):
        triplet(Path(stem).parent, Path(stem).name)
    monkeypatch.setattr('operator_gateway.runtime.export_map', export)
    node._runtime_request('app', request('retry_export', name='新场地'))
    tick_until(node, lambda: node.runtime_pending is None)
    assert node.sent[-1]['success']
    assert node.map_store.catalog()[0]['valid']
    assert node.runtime['mode'] == 'mapping'  # Retry is a snapshot, not an implicit exit.
    assert len(node.write_state_client.calls) == 1


def test_save_inflight_timeout_remains_uncertain_and_never_reissues(node):
    node.runtime.update(mode='mapping', phase='MAPPING')
    node._runtime_request('app', request('save', name='pending'))
    tick_until(node, lambda: node.runtime_pending['stage'] == 'writing')
    node.runtime_pending['since'] -= 121
    node._runtime_tick()
    node._runtime_request('app', request('save', name='pending'))
    assert node.runtime_pending['stage'] == 'writing'
    assert len(node.write_state_client.calls) == 1
    assert '待确认' in node.runtime['message']


def test_load_invalid_map_keeps_existing_mode_and_discard_requires_confirmation(node):
    node.runtime.update(mode='localization', phase='LOCALIZATION', current_map='existing')
    node._runtime_request('app', request('load', name='missing'))
    tick_until(node, lambda: node.runtime_pending is None)
    assert node.runtime['current_map'] == 'existing' and node.runtime['mode'] == 'localization'
    node.runtime.update(mode='mapping', phase='MAPPING')
    node._runtime_request('app', request('discard'))
    assert not node.sent[-1]['success']
    node._runtime_request('app', request('load', name='missing', request_id='load2'))
    assert not node.sent[-1]['success'] and node.runtime['mode'] == 'mapping'
