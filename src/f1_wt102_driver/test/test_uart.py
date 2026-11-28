import math
import os
import pty
import select
from unittest.mock import Mock

import pytest
import rclpy
from f1_interfaces.msg import Wt102Sample
from f1_wt102_driver.uart import Wt102Uart


def test_timestamped_uart_and_reconnect(tmp_path):
    link = tmp_path / 'sensor'

    def attach():
        writer, reader = pty.openpty()
        link.symlink_to(os.ttyname(reader))
        os.close(reader)
        return writer

    writer = attach()
    rclpy.init(args=['--ros-args', '-p', f'port:={link}'])
    node = Wt102Uart()
    node.pub = Mock()

    def poll_readable():
        # The PTY line discipline may deliver a master's write asynchronously.
        assert select.select([node.port.fileno()], [], [], 1.)[0]
        node.poll()

    try:
        node.poll()
        rate = bytes.fromhex('55 52 00 00 00 20 00 f0 00 00 b7')
        yaw = bytes.fromhex('55 53 00 00 00 00 00 40 00 00 e8')
        os.write(writer, b'noise' + rate[:5])
        poll_readable()
        node.pub.publish.assert_not_called()
        os.write(writer, rate[5:] + yaw)
        poll_readable()
        samples = [c.args[0] for c in node.pub.publish.call_args_list]
        assert [s.kind for s in samples] == [Wt102Sample.YAW_RATE, Wt102Sample.RELATIVE_YAW]
        assert [s.value for s in samples] == pytest.approx([math.radians(-250), math.pi / 2])
        assert all(s.header.frame_id == 'wt102_link' and s.header.stamp.sec > 0 for s in samples)
        epoch = samples[0].stream_id
        os.close(writer)
        writer = None
        node.poll()
        assert node.port is None
        link.unlink()
        writer = attach()
        node.retry_at = 0
        node.poll()
        os.write(writer, rate)
        poll_readable()
        resumed = node.pub.publish.call_args.args[0]
        assert resumed.kind == Wt102Sample.YAW_RATE and resumed.stream_id != epoch
    finally:
        if writer is not None:
            os.close(writer)
        node.close_port()
        node.destroy_node()
        rclpy.shutdown()
