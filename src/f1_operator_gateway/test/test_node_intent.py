from operator_gateway.node import OperatorGateway
from operator_gateway.protocol import ManualInput


class FakeHub:
    def __init__(self):
        self.sent = []

    def send(self, client_id, packet):
        self.sent.append((client_id, packet))


def gateway(allow_fifth):
    node = OperatorGateway.__new__(OperatorGateway)
    node.command_pending = None
    node.task_locks = set()
    node.last_app_seq = -1
    node.control_state = {"allow_fifth_thruster": allow_fifth}
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


def test_scan_transform_uses_sensor_frame_timestamp_translation_and_rotation():
    import math
    from types import SimpleNamespace
    from geometry_msgs.msg import TransformStamped
    node = gateway(True)
    node.scan = dict(frame_id="laser", stamp=dict(sec=1, nanosec=0),
                     angle_min=0., angle_increment=.1, range_min=.05, range_max=10., ranges=[1.])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_100_000_000))
    transform = TransformStamped()
    transform.transform.translation.x = 2.
    transform.transform.translation.y = 3.
    transform.transform.rotation.z = math.sin(math.pi/4)
    transform.transform.rotation.w = math.cos(math.pi/4)
    looked_up = []
    def lookup(target, frame, stamp):
        looked_up.append((target, frame, stamp.nanoseconds))
        return transform
    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    result = node._map_scan()
    assert looked_up == [("map", "laser", 1_000_000_000)]
    import pytest
    assert result["points"][0] == pytest.approx([2., 4.])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_000_000_000))
    assert node._map_scan() is None


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
