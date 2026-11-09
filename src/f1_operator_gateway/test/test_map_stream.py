import base64
from array import array
from threading import Event
import time
from types import SimpleNamespace as NS
import zlib

from operator_gateway.map_stream import LatestMapEncoder, encode_map


def grid(values, width=None):
    return NS(header=NS(frame_id="map"), data=array("b", values),
              info=NS(width=width or len(values), height=1, resolution=.05,
                      origin=NS(position=NS(x=1., y=2.), orientation=NS(x=0., y=0., z=0., w=1.))))


def take(stream, timeout=2.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = stream.take()
        if result is not None:
            return result
        time.sleep(.002)
    raise AssertionError("map encoder did not produce a result")


def test_palette_encoding_keeps_ros_grid_and_row_order():
    message = grid([-1, 0, 50, 51, 100])
    _, packet = encode_map(message)
    assert packet["encoding"] == "indexed8_zlib"
    assert zlib.decompress(base64.b64decode(packet["data_zlib_base64"])) == bytes([0, 1, 1, 2, 2])
    assert list(message.data) == [-1, 0, 50, 51, 100]
    assert "data" not in packet and packet["origin"] == dict(x=1., y=2., yaw=0.)


def test_localization_freezes_first_map_until_session_reset():
    stream = LatestMapEncoder(.03)
    try:
        assert stream.offer(grid([0]), frozen=True)
        assert not stream.offer(grid([100]), frozen=True)
        first = take(stream)
        assert not stream.offer(grid([100]), frozen=True)
        stream.reset()
        assert stream.offer(grid([100]), frozen=True)
        assert take(stream) != first
    finally:
        stream.close()


def test_mapping_rate_limit_keeps_only_latest_pending_grid():
    encoded = []
    def encode(message):
        encoded.append((time.monotonic(), list(message.data)))
        return encode_map(message)
    stream = LatestMapEncoder(.15, encoder=encode)
    try:
        stream.offer(grid([0]))
        take(stream)
        stream.offer(grid([10]))
        stream.offer(grid([30]))
        stream.offer(grid([100]))
        result = take(stream)
        assert [values for _, values in encoded] == [[0], [100]]
        assert encoded[1][0] - encoded[0][0] >= .15
        assert zlib.decompress(base64.b64decode(result["data_zlib_base64"])) == bytes([2])
    finally:
        stream.close()


def test_reset_discards_an_inflight_map_and_releases_old_output():
    started, finish = Event(), Event()
    def encode(message):
        if list(message.data) == [0]:
            started.set()
            assert finish.wait(2)
        return encode_map(message)
    stream = LatestMapEncoder(.02, encoder=encode)
    try:
        stream.offer(grid([0]), frozen=True)
        assert started.wait(1)
        stream.reset()
        stream.offer(grid([100]), frozen=True)
        finish.set()
        result = take(stream)
        assert zlib.decompress(base64.b64decode(result["data_zlib_base64"])) == bytes([2])
        stream.reset()
        assert stream.take() is None
    finally:
        finish.set()
        stream.close()
