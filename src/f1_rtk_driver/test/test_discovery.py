from contextlib import ExitStack
import os
import pty
from types import SimpleNamespace

from wit_rtk.discovery import ReceiverDiscovery


def adapter(stack, pid):
    master, slave = pty.openpty()
    stack.callback(os.close, master)
    stack.callback(os.close, slave)
    return master, SimpleNamespace(device=os.ttyname(slave), vid=0x1a86, pid=pid)


def test_only_identify_checksummed_gga_and_hpr_on_same_ch340(monkeypatch):
    with ExitStack() as stack:
        _, f103 = adapter(stack, 0x55d4)
        a, first = adapter(stack, 0x7523)
        b, second = adapter(stack, 0x7523)
        devices = [f103, first]
        monkeypatch.setattr("wit_rtk.discovery.list_ports.comports", lambda: devices)
        scan = ReceiverDiscovery(115200, 0.5)
        stack.callback(scan.close)
        assert scan.poll(0) is None and scan.handle.port == first.device
        # The receiver can report no solution indoors. Both valid sentence
        # types are still required; neither a GGA alone nor a bad HPR suffices.
        gga = b"$GNGGA,,,,,,0,00,,,,,,,*78\r\n"
        hpr = b"$GNHPR,,,,,,,,*43\r\n"
        os.write(a, gga + hpr.replace(b"*43", b"*00"))
        assert scan.poll(.2) is None
        assert scan.poll(1.6) is None and scan.handle is None
        devices[:] = [f103, second]
        assert scan.poll(2.2) is None and scan.handle.port == second.device
        os.write(b, hpr + gga[:6])
        assert scan.poll(2.3) is None
        os.write(b, gga[6:])
        handle = scan.poll(2.4)
        assert handle is not None and handle.port == second.device
        handle.close()
        devices[:] = [first]
        assert scan.poll(3.0) is None
        os.write(a, gga + hpr)
        handle = scan.poll(3.1)
        assert handle is not None and handle.port == first.device
        handle.close()
