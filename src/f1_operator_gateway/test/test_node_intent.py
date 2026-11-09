from operator_gateway.node import OperatorGateway
from operator_gateway.protocol import ManualInput
from operator_gateway.diagnostics import DiagnosticHistory


class FakeHub:
    def __init__(self):
        self.sent = []

    def send(self, client_id, packet):
        self.sent.append((client_id, packet))

    def broadcast(self, packet):
        self.sent.append((None, packet))


def gateway(allow_fifth):
    node = OperatorGateway.__new__(OperatorGateway)
    node.command_pending = None
    node.task_locks = set()
    node.last_app_seq = -1
    node.control_state = {"allow_fifth_thruster": allow_fifth}
    node.diagnostic_history = DiagnosticHistory()
    node.hub = FakeHub()
    node.token = ""
    node.last_input = None
    node.last_input_at = None
    node._acquire_if_ready = lambda: None
    return node


def command(thrust_enabled=True):
    return ManualInput(1, "FORWARD", 25, "BOTH", False, 0,
                       thrust_enabled, -80)


def test_disabled_fifth_is_stripped_without_breaking_chassis_input():
    node = gateway(False)
    node._input("operator", command())
    assert node.last_input.direction == "FORWARD"
    assert not node.last_input.thrust_enabled
    assert node.last_input.thrust_percent == 0
    assert node.hub.sent[0][1]["success"] is False


def test_enabled_fifth_value_reaches_the_gateway_intent():
    node = gateway(True)
    node._input("operator", command())
    assert node.last_input.thrust_enabled
    assert node.last_input.thrust_percent == -80
    assert node.hub.sent == []


def test_running_task_rejects_manual_intent():
    node = gateway(True)
    node.task_locks = {"l"}
    node._input("operator", command())
    assert node.last_input is None
    assert not node.hub.sent[-1][1]["success"]


def test_scan_only_updates_runtime_health_without_building_app_point_cloud():
    node = gateway(True)
    seen = []
    node._runtime_scan = seen.append
    message = object()
    node._scan(message)
    assert seen == [message]
    assert not hasattr(node, "scan")


def test_late_manual_acquire_is_released_after_switch_to_task():
    from concurrent.futures import Future
    from types import SimpleNamespace
    import time
    node = gateway(True)
    node.pending_acquire = False
    node._manual_acquires = 0
    node.manual_generation = 0
    node._fresh_ready = lambda: True
    node.active_client = "operator"
    node.input_timeout = .25
    node.last_input = command()
    node.last_input_at = time.monotonic()
    pending = Future()
    node.acquire_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: pending)
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None, warning=lambda message: None)
    released, published = [], []
    node._release_token = released.append
    node._publish = published.append
    # Use the real method; the fixture normally stubs acquisition out.
    OperatorGateway._acquire_if_ready(node)
    assert node._manual_acquires == 1
    node._stop_manual("task_start")
    node.command_pending = {"stage": "handover"}
    pending.set_result(SimpleNamespace(success=True, token="late-manual-token"))
    assert released == ["late-manual-token"]
    assert not published and not node.token and node._manual_acquires == 0


def test_manual_timeout_history_survives_release_without_exposing_token():
    import time
    from types import SimpleNamespace
    node = gateway(True)
    node.pending_acquire = False
    node.manual_generation = 0
    node.input_timeout = .25
    node.token = "private-control-token"
    node.control_state.update(source="NANO", state="ACTIVE", ctrl_age_sec=.02)
    node.last_input = command()
    node.last_input_at = time.monotonic() - .3
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    released = []
    node._release_token = released.append
    node._stop_manual("input_timeout")
    event = node.diagnostic_history.view()["events"][-1]
    assert event["reason"] == "input_timeout"
    assert event["details"]["input_age_ms"] >= 300
    assert event["details"]["input_timeout_ms"] == 250
    assert event["details"]["had_lease"] is True
    assert event["details"]["source"] == "NANO"
    assert "private-control-token" not in str(event)
    assert released == ["private-control-token"] and node.last_input_at is None


def test_bridge_history_preserves_short_lived_stop_and_is_bounded():
    history = DiagnosticHistory()
    history.bridge(dict(state="ACTIVE", source="NANO", reason="executing leased command"))
    for age in range(100):
        history.bridge(dict(state="ACTIVE", source="NANO", reason="executing leased command", ctrl_age_sec=age / 1000))
    assert len(history.events) == 1
    history.bridge(dict(state="HANDSHAKE", reason="command deadline expired",
                        last_stop_reason="command deadline expired"))
    history.bridge(dict(state="READY", reason="neutral acknowledged; waiting for explicit acquire"))
    assert history.view()["events"][1]["reason"] == "command deadline expired"
    for _ in range(100):
        history.record("gateway", "input_timeout")
    assert len(history.events) == 50 and history.sequence == 103


def stream_gateway():
    import time
    from operator_gateway.tracks import TaskTrack
    node = gateway(True)
    node.runtime_enabled = False
    node.diagnostic_history = DiagnosticHistory(node._diagnostic_event)
    node.telemetry_sequence = 0
    node.control_received = time.monotonic()
    node.pose = dict(x=1., y=2., yaw=0.)
    node.active_client = None
    node._task_telemetry = lambda: {"tasks": {"l": {"phase": "IDLE"}}}
    node.task_tracks = {"l": TaskTrack()}
    node._last_tracks = None
    node.map_data = dict(revision=7, encoding="indexed8_zlib", data_zlib_base64="encoded",
                         runtime_session_id="runtime-map-session")
    return node


def test_core_telemetry_omits_visual_bulk_and_has_monotonic_stream_sequence():
    node = stream_gateway()
    node._telemetry()
    node._telemetry()
    packets = [packet for _, packet in node.hub.sent]
    assert [packet["stream"]["sequence"] for packet in packets] == [1, 2]
    for packet in packets:
        assert not {"scan", "task", "diagnostics", "map"} & packet.keys()
        assert packet["stream"]["session"] == node.diagnostic_history.session
        assert packet["stream"]["generated_unix"] > 0
        assert packet["stream"]["generated_monotonic"] > 0


def test_diagnostics_send_each_new_event_and_hello_replays_history_map_and_tracks():
    import json
    node = stream_gateway()
    node.diagnostic_history.record("gateway", "first")
    node.diagnostic_history.record("gateway", "second")
    emitted = [packet for _, packet in node.hub.sent]
    assert [len(p["diagnostics"]["events"]) for p in emitted] == [1, 1]
    assert [p["diagnostics"]["events"][0]["id"] for p in emitted] == [1, 2]
    node.hub.sent.clear()
    node._handle_event(("packet", "new-client", json.dumps(dict(op="hello", client="test"))))
    packets = {packet["type"]: packet for _, packet in node.hub.sent}
    assert packets["map"]["map"] is node.map_data
    assert packets["map"]["runtime_session_id"] == "runtime-map-session"
    assert {"compact_stream_v1", "indexed8_map"} <= set(packets["hello"]["capabilities"])
    assert [e["id"] for e in packets["diagnostics"]["diagnostics"]["events"]] == [1, 2]
    assert packets["tracks"]["session_id"] == packets["hello"]["session_id"] == node.diagnostic_history.session


def test_tracks_only_broadcast_changed_snapshots():
    node = stream_gateway()
    node._publish_tracks()
    node._publish_tracks()
    assert len(node.hub.sent) == 1
    node.task_tracks["l"].update("map", "run", dict(x=1., y=2., stamp=dict(sec=1)), recording=True)
    node._publish_tracks()
    assert len(node.hub.sent) == 2
    assert node.hub.sent[-1][1]["tasks"]["l"]["point_count"] == 1


def test_map_callback_defers_all_grid_work_and_commits_revision_with_ready_result():
    from types import SimpleNamespace
    node = stream_gateway()
    node.runtime_enabled = True
    node.runtime = {"mode": "localization", "session_id": "runtime-new-session"}
    node.runtime_accept_map = True
    node.runtime_epoch_ns = 1_000_000_000
    node.map_revision = 7
    offered = []
    result = dict(width=2, height=1, encoding="indexed8_zlib", data_zlib_base64="encoded")
    pending = [None, result]
    node.map_stream = SimpleNamespace(offer=lambda message, **kw: offered.append((message, kw)),
                                     take=lambda: pending.pop(0))
    # No info/data attributes: the callback must only queue a reference.
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)))
    previous = node.map_data
    node._map(message)
    assert offered == [(message, {"frozen": True})]
    node._publish_map()
    assert node.map_data is previous and node.map_revision == 7
    node._publish_map()
    assert node.map_revision == node.map_data["revision"] == 8
    assert node.hub.sent[-1][1]["map"] is node.map_data
    assert node.hub.sent[-1][1]["runtime_session_id"] == node.map_data["runtime_session_id"] == "runtime-new-session"
    node.map_stream.take = lambda: result
    node.runtime_accept_map = False
    node._publish_map()
    assert node.map_revision == 8  # Never commit a late result while stopping.
