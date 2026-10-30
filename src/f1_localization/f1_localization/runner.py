import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def run(node_type, args=None):
    # A terminal group SIGINT plus launch's forwarded SIGINT can race with
    # rclpy's asynchronous context shutdown. Keep teardown on this thread.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = node_type()
        # Bound the wait so Python signal handlers also run for nodes without
        # timers or incoming data (there is no rclpy signal guard condition).
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
