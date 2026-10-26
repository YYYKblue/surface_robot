from operator_gateway.node import OperatorGateway
from operator_gateway.protocol import ManualInput


class FakeHub:
    def __init__(self):
        self.sent = []

    def send(self, client_id, packet):
        self.sent.append((client_id, packet))


def gateway(allow_fifth):
    node = OperatorGateway.__new__(OperatorGateway)
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
