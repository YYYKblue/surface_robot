"""Synthetic ZMQ -> ROS integration, isolated from production IMU topics."""
import json
import time
import uuid

import pytest


def test_waits_without_sensor_then_converts_gateway_packet(tmp_path, monkeypatch):
    zmq = pytest.importorskip("zmq", reason="python3-zmq required for real transport integration")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Imu
    from f1_imu_bridge.node import ImuBridge

    token = uuid.uuid4().hex
    topic = f"/f1_imu_bridge_test_{token}/imu"
    diagnostics_topic = f"/f1_imu_bridge_test_{token}/diagnostics"
    zmq_context = zmq.Context()
    publisher = zmq_context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    port = publisher.bind_to_random_port("tcp://127.0.0.1")
    context = Context()
    rclpy.init(args=[], context=context)
    bridge = listener = executor = None
    try:
        bridge = ImuBridge(context=context, namespace=f"test_{token}", parameter_overrides=[
            Parameter("endpoint", value=f"tcp://127.0.0.1:{port}"),
            Parameter("output_topic", value=topic),
            Parameter("diagnostics_topic", value=diagnostics_topic),
        ])
        listener = Node(f"imu_test_listener_{token}", context=context)
        messages, diagnostics = [], []
        listener.create_subscription(Imu, topic, messages.append, 10)
        listener.create_subscription(DiagnosticArray, diagnostics_topic, diagnostics.append, 10)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(bridge)
        executor.add_node(listener)
        deadline = time.monotonic() + 5
        while not diagnostics and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert diagnostics, "no diagnostics while waiting for sensor data"
        assert diagnostics[-1].status[0].level == DiagnosticStatus.WARN
        assert not messages, "bridge must not invent IMU samples without input"

        deadline = time.monotonic() + 5
        sequence = 0
        while not messages and time.monotonic() < deadline:
            sequence += 1
            document = {"protocol": "f1-imu-v1", "session": "synthetic-integration:1",
                        "received_unix_ns": time.time_ns(),
                        "received_monotonic_ns": time.monotonic_ns(),
                        "imu": {"kind": "IMU", "sequence": sequence, "stm32_ms": sequence * 20,
                                "status": 31, "acceleration": [0, 0, 9.81],
                                "angular_velocity": [0, 0, 0.1]}}
            publisher.send_multipart([b"f1_imu", json.dumps(document).encode()])
            for _ in range(5):
                executor.spin_once(timeout_sec=0.005)
        assert messages, "no ROS IMU received from synthetic gateway packet"
        assert messages[-1].header.frame_id == "imu_link"
        assert messages[-1].linear_acceleration.z == pytest.approx(9.81)
        assert messages[-1].angular_velocity.z == pytest.approx(0.1)
        assert messages[-1].orientation_covariance[0] == -1
    finally:
        if executor is not None:
            executor.shutdown()
        if bridge is not None:
            bridge.destroy_node()
        if listener is not None:
            listener.destroy_node()
        if context.ok():
            context.shutdown()
        publisher.close()
        zmq_context.term()
