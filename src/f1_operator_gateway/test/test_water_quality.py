import io

import pytest

from operator_gateway.water_quality import measurements, read_registers


class Port(io.BytesIO):
    def __init__(self, response):
        super().__init__(response)
        self.request = None

    def reset_input_buffer(self):
        pass

    def write(self, request):
        self.request = request
        return len(request)


@pytest.mark.parametrize("frame,count,words,query", [
    ("01030403160109da25", 2, (790, 265), "010300000002c40b"),
    ("0103020316397a", 1, (790,), "010300000001840a"),
    ("0103042710010930d4", 2, (10000, 265), "010300000002c40b"),
    ("0103040d2e00dbd8cd", 2, (3374, 219), "010300000002c40b"),
])
def test_manual_read_frames(frame, count, words, query):
    port = Port(bytes.fromhex(frame))
    assert read_registers(port, 1, count) == words
    assert port.request.hex() == query


@pytest.mark.parametrize("frame,address,count,error", [
    ("", 1, 1, TimeoutError),
    ("0103020316", 1, 1, ValueError),
    ("010302031639ff", 1, 1, ValueError),
    ("01030403160109b8be", 1, 2, ValueError),  # CRC printed in the pH manual
    ("018302c0f1", 1, 1, ValueError),
    ("0103020316397a", 2, 1, ValueError),
    ("0103020316397a", 1, 2, ValueError),
])
def test_invalid_responses(frame, address, count, error):
    with pytest.raises(error):
        read_registers(Port(bytes.fromhex(frame)), address, count)


@pytest.mark.parametrize("k,expected", [(1, 1000.), (10, 10000.), (0, None)])
def test_ec_variants(k, expected):
    values = measurements("ec", [10000, 265, 490, 550], k, 0)
    assert values.pop("value", None) == expected
    assert values == dict(temperature_c=26.5, salinity_ppm=490., tds_ppm=550.)


@pytest.mark.parametrize("maximum,expected", [(0, None), (50, 33.74), (200, 337.4),
                                               (1000, 337.4), (4000, 337.4)])
def test_turbidity_variants(maximum, expected):
    values = measurements("turbidity", [3374, 0xffe7], 0, maximum)
    assert values.pop("value", None) == expected
    assert values == dict(temperature_c=-2.5)


def test_auto_discovery_skips_busy_streaming_and_one_slave_then_follows_bus(monkeypatch):
    import struct
    from types import SimpleNamespace
    from operator_gateway import water_quality as module

    ports = ["busy", "rtk", "one", "water"]
    handles = []

    class Candidate:
        def __init__(self, name, baudrate, **kwargs):
            assert kwargs["exclusive"]
            if name == "busy":
                raise OSError("locked")
            self.port, self.baudrate = name, baudrate
            self.addresses = []
            self.closed = False
            handles.append(self)

        def reset_input_buffer(self):
            pass

        def write(self, request):
            assert self.baudrate == 4800
            addr, function, start, count = struct.unpack(">BBHH", request[:-2])
            assert (function, start, count) == (3, 2000, 1)
            self.addresses.append(addr)
            return len(request)

        def read(self, size):
            addr = self.addresses[-1]
            if self.port == "one" and addr != 1:
                return b""
            data = bytes((addr, 3, 2, 0, addr))
            return data + module.crc_bytes(data)

        def close(self):
            self.closed = True

    monkeypatch.setattr(module.list_ports, "comports", lambda: [
        SimpleNamespace(vid=0x1a86, pid=0x7523, device=p) for p in ports])
    monkeypatch.setattr(module.serial, "Serial", Candidate)
    monkeypatch.setattr(module.select, "select", lambda r, w, x, t: (
        r if r[0].port == "rtk" else [], [], []))
    bus = module.WaterBus("auto", 4800)
    for _ in ports:
        bus.discover([1, 2, 3, 4])
    assert bus.port.port == "water"
    assert all(h.closed for h in handles if h.port != "water")
    assert next(h for h in handles if h.port == "rtk").addresses == []
    assert bus.port.addresses == [1, 2]
    bus.close()
    ports[:] = ["changed_usb_number"]
    bus.next_scan = 0
    bus.discover([1, 2, 3, 4])
    assert bus.port.port == "changed_usb_number"
    bus.close()
