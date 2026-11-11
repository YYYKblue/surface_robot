from dataclasses import replace
from types import SimpleNamespace
import time

import pytest

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
    node.control_state = dict(allow_fifth_thruster=allow_fifth,
                             firmware_protocol_version=3, firmware_protocol_compatible=True,
                             connected=True, stamp=dict(sec=100, nanosec=0),
                             ctrl_age_sec=.02, ack_age_sec=.02,
                             connection_id="serial-session", epoch=1,
                             source="NANO", armed=True, result="OK", state="READY")
    node.control_received = time.monotonic()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None, warning=lambda message: None)
    node.diagnostic_history = DiagnosticHistory()
    node.hub = FakeHub()
    node.token = ""
    node.pending_acquire = False
    node.manual_started = False
    node.manual_rearm_required = False
    node.manual_lease_acquired_ns = None
    node.manual_generation = 0
    node.input_timeout = .25
    node.last_input = None
    node.last_input_at = None
    node._acquire_if_ready = lambda: None
    return node


def command(thrust_enabled=True):
    return ManualInput(1, "FORWARD", 25, "BOTH", False, 0.,
                       thrust_enabled, -80.)


def control_update(node, **changes):
    state = dict(node.control_state, **changes)
    state["stamp"] = SimpleNamespace(**state.get("stamp", dict(sec=100, nanosec=0)))
    message = SimpleNamespace(**state, get_fields_and_field_types=lambda: tuple(state))
    node._control(message)


def test_disabled_fifth_is_stripped_without_breaking_chassis_input():
    node = gateway(False)
    node._input("operator", command())
    assert node.last_input.direction == "FORWARD"
    assert not node.last_input.thrust_enabled
    assert node.last_input.thrust_percent == 0
    assert node.hub.sent[0][1]["success"] is False
    published = []
    node.publisher = SimpleNamespace(publish=published.append)
    node.token, node.bridge_sequence = "manual-token", 0
    assert node._publish(node.last_input)
    assert not published[0].thrust_enabled and published[0].thrust_percent == 0.


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


@pytest.mark.parametrize("fields, expected", [
    (dict(speed_percent=0), False),
    (dict(direction="STOP", speed_percent=100), False),
    (dict(speed_percent=1), True),
    (dict(speed_percent=0, brush_enabled=True, brush_percent=0), False),
    (dict(speed_percent=0, brush_enabled=True, brush_percent=-30), True),
    (dict(speed_percent=0, brush_enabled=False, brush_percent=30), False),
    (dict(speed_percent=0, thrust_enabled=True, thrust_percent=0), False),
    (dict(speed_percent=0, thrust_enabled=True, thrust_percent=1), True),
])
def test_control_is_needed_only_for_effective_output(fields, expected):
    assert OperatorGateway._needs_control(replace(command(False), **fields)) is expected


def test_zero_drive_does_not_acquire_but_keeps_auxiliary_control_independent():
    node = gateway(True)
    acquired = []
    node._acquire_if_ready = lambda: acquired.append(True)
    node._input("operator", replace(command(False), speed_percent=0))
    assert node.last_input.speed_percent == 0 and not acquired
    node._input("operator", replace(command(False), speed_percent=0,
                                    brush_enabled=True, brush_percent=50))
    assert node.last_input.brush_percent == 50 and acquired == [True]


@pytest.mark.parametrize("fields", [
    dict(firmware_protocol_version=0), dict(firmware_protocol_version=2),
    dict(firmware_protocol_compatible=False), dict(connected=False),
    dict(ctrl_age_sec=.4), dict(ctrl_age_sec=-.1),
    dict(stamp=dict(sec=99, nanosec=700_000_000)),
    dict(stamp=dict(sec=101, nanosec=0)),
])
def test_drive_availability_rejects_unknown_incompatible_or_stale_firmware(fields):
    node = gateway(True)
    assert node._drive_output_available()
    node.control_state.update(fields)
    assert not node._drive_output_available()
    node._input("operator", command(False))
    assert node.last_input is None
    assert not node.hub.sent[-1][1]["success"]
    assert "F103 v3" in node.hub.sent[-1][1]["message"]


def test_drive_availability_requires_fresh_ros_state_and_is_not_motion_authorization():
    node = gateway(True)
    assert node._fresh_ready()
    node.control_state.update(source="RC", armed=False)
    assert node._drive_output_available() and not node._fresh_ready()
    node.control_received -= .3
    assert not node._drive_output_available()


@pytest.mark.parametrize("field", ["ctrl_age_sec", "ack_age_sec"])
def test_ros_queue_age_counts_toward_ctrl_and_ack_deadlines(field):
    node = gateway(True)
    node.control_state.update(stamp=dict(sec=99, nanosec=800_000_000), **{field: .25})
    assert node._control_age_sec() == pytest.approx(.2)
    assert not node._fresh_ready()
    if field == "ctrl_age_sec":
        assert not node._drive_output_available()
    node._input("operator", command(False))
    assert node.last_input is None and node.manual_rearm_required


def test_missing_firmware_fields_reject_output_and_neutral_cannot_rearm():
    node = gateway(True)
    node.control_state = {}
    node._input("operator", command(False))
    assert node.last_input is None
    node._input("operator", replace(command(False), direction="STOP"))
    assert node.last_input is None and node.manual_rearm_required


@pytest.mark.parametrize("changes", [
    dict(connection_id="new-serial-session"), dict(epoch=2),
    dict(source="RC", state="RC"), dict(state="HANDSHAKE"),
    dict(state="DISCONNECTED", connected=False), dict(state="INCOMPATIBLE"),
    dict(state="FAULT"), dict(state="MONITOR"), dict(armed=False),
    dict(result="NEUTRAL_REQUIRED"), dict(ack_age_sec=.4),
])
def test_control_session_or_authority_loss_revokes_manual_state(changes):
    node = gateway(True)
    node._input("operator", command(False))
    node.token, node.pending_acquire = "old-token", True
    released = []
    node._release_token = released.append
    control_update(node, **changes)
    assert released == ["old-token"]
    assert not node.token and not node.pending_acquire and node.last_input is None
    assert node.manual_rearm_required


@pytest.mark.parametrize("changes", [dict(source="RC", state="RC"), dict(state="HANDSHAKE")])
def test_held_input_is_not_cached_until_ready_and_neutral_must_arrive_after_ready(changes):
    node = gateway(True)
    ready = dict(node.control_state)
    control_update(node, **changes)
    node._input("operator", command(False))
    assert node.last_input is None and node.manual_rearm_required
    node._input("operator", replace(command(False), speed_percent=0))
    assert node.manual_rearm_required  # Neutral under RC/handshake cannot rearm.
    control_update(node, **ready)
    node._input("operator", replace(command(False), sequence=2))
    assert node.last_input is None and node.manual_rearm_required
    node._input("operator", replace(command(False), sequence=3, speed_percent=0))
    assert not node.manual_rearm_required
    node._input("operator", replace(command(False), sequence=4))
    assert node.last_input.sequence == 4


def test_initial_connection_handshake_does_not_block_first_explicit_ready_input():
    node = gateway(True)
    ready = dict(node.control_state)
    node.control_state = {}
    control_update(node, **dict(ready, state="HANDSHAKE"))
    control_update(node, **ready)
    node._input("operator", command(False))
    assert not node.manual_rearm_required and node.last_input is not None


def test_bridge_lease_loss_clears_token_even_if_handshake_sample_was_missed():
    node = gateway(True)
    node.token = "old-token"
    node.manual_started = True
    node.manual_lease_acquired_ns = 99_999_999_999
    node.control_state.update(state="ACTIVE", motion_authorized=True, owner="operator_app")
    released = []
    node._release_token = released.append
    control_update(node, state="READY", motion_authorized=False, owner="")
    assert released == ["old-token"] and not node.token and node.manual_rearm_required


def test_ready_status_generated_before_acquire_does_not_revoke_new_token():
    node = gateway(True)
    node.token = "new-token"
    node.manual_started = True
    node.manual_lease_acquired_ns = 100_000_000_001
    node._release_token = lambda token: pytest.fail("old READY status revoked the new lease")
    control_update(node, state="READY", motion_authorized=False, owner="")
    assert node.token == "new-token" and not node.manual_rearm_required


@pytest.mark.parametrize("fields", [dict(brush_enabled=True, brush_percent=50),
                                   dict(thrust_enabled=True, thrust_percent=-80)])
def test_auxiliary_output_also_requires_compatible_firmware(fields):
    node = gateway(True)
    node.control_state["firmware_protocol_compatible"] = False
    node._input("operator", replace(command(False), direction="STOP", speed_percent=0, **fields))
    assert node.last_input is None and not node.hub.sent[-1][1]["success"]


def test_firmware_loss_releases_active_control_and_discards_previous_intent():
    node = gateway(True)
    released = []
    node._release_token = released.append
    node.token = "old-token"
    node.last_input = command(False)
    node.last_input_at = time.monotonic()
    node.control_state["firmware_protocol_compatible"] = False
    node._input("operator", replace(command(False), sequence=2))
    assert released == ["old-token"]
    assert node.last_input is None and not node.token
    assert node.diagnostic_history.view()["events"][-1]["reason"] == "firmware protocol unavailable"


def test_firmware_loss_revokes_the_stored_intent_without_another_app_packet():
    node = gateway(True)
    node.active_client = "operator"
    node.token = "old-token"
    node.last_input = command(False)
    node.last_input_at = time.monotonic()
    node.control_state["firmware_protocol_version"] = 2
    node.hub.poll = lambda: []
    node._read_pose = node._task_tick = node._runtime_tick = node._publish_map = lambda: None
    released = []
    node._release_token = released.append
    node._tick()
    assert released == ["old-token"] and node.last_input is None and not node.token


def test_publish_rechecks_firmware_and_preserves_percent_without_changing_ttl():
    node = gateway(True)
    published, released = [], []
    node.publisher = SimpleNamespace(publish=published.append)
    node._release_token = released.append
    node.token = "current-token"
    node.bridge_sequence = 0
    assert node._publish(replace(command(False), speed_percent=37))
    message = published[0]
    assert message.speed_percent == 37
    assert message.valid_until.nanosec - message.computed_at.nanosec == 250_000_000
    node.control_state["firmware_protocol_version"] = 2
    assert not node._publish(command(False))
    assert len(published) == 1 and released == ["current-token"]


def test_late_manual_acquire_is_released_when_firmware_becomes_incompatible():
    from concurrent.futures import Future
    node = gateway(True)
    node._manual_acquires = 0
    node.active_client = "operator"
    node.last_input = command(False)
    node.last_input_at = time.monotonic()
    pending = Future()
    node.acquire_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: pending)
    released, published = [], []
    node._release_token = released.append
    node._publish = published.append
    OperatorGateway._acquire_if_ready(node)
    node.control_state["firmware_protocol_version"] = 2
    pending.set_result(SimpleNamespace(success=True, token="late-token"))
    assert released == ["late-token"] and not published
    assert node.last_input is None and not node.token and node._manual_acquires == 0


def test_rc_channel_array_is_converted_for_json_telemetry():
    from array import array
    import json
    node = gateway(True)
    message = SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0),
        rc_channels=array("H", [1024] * 14), target_pwm_bl_br_fl_fr=array("H", [1500] * 4),
        get_fields_and_field_types=lambda: ["stamp", "rc_channels", "target_pwm_bl_br_fl_fr"])
    node._control(message)
    assert json.loads(json.dumps(node.control_state))["rc_channels"] == [1024] * 14


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
        assert packet["drive_output"] == dict(min=0, max=100, step=1,
                                               firmware_protocol_version=3, available=True)
    node.control_state["firmware_protocol_version"] = 2
    node._telemetry()
    assert node.hub.sent[-1][1]["drive_output"]["available"] is False


def test_telemetry_reports_effective_ros_queue_age_and_rejects_future_source_stamp():
    node = stream_gateway()
    node.control_state["stamp"] = dict(sec=99, nanosec=800_000_000)
    node._telemetry()
    packet = node.hub.sent[-1][1]
    assert packet["stream"]["control_age_ms"] == 200.
    node.control_state["stamp"] = dict(sec=101, nanosec=0)
    node._telemetry()
    packet = node.hub.sent[-1][1]
    assert packet["stream"]["control_age_ms"] is None
    assert not packet["control_fresh"] and not packet["drive_output"]["available"]


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
    assert {"compact_stream_v1", "indexed8_map", "continuous_drive_percent_v1"} <= set(packets["hello"]["capabilities"])
    assert packets["hello"]["drive_output"] == node._drive_output()
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
