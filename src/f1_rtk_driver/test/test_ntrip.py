"""Local fake caster coverage; no purchased account or external network is used."""

import base64
from contextlib import contextmanager
import json
import socket
import threading
import time

import pytest

from wit_rtk.ntrip import Config, NtripClient, RtcmFrames, crc24q, load_config, read_response


GGA = b"$GNGGA,023541.00,2304.76261531,N,11415.24485118,E,1,18,1.5,44.5917,M,-3.1322,M,,*59"


def rtcm(payload=b"\x3e\xd0\x00\x01\xd3\x02"):
    body = bytes((0xD3, len(payload) >> 8, len(payload) & 255)) + payload
    return body + crc24q(body).to_bytes(3, "big")


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    pytest.fail("condition did not become true")


class Caster:
    def __init__(self, response=b"ICY 200 OK\r\n", data=None):
        self.response = response
        self.data = rtcm() if data is None else data
        self.headers = []
        self.ggas = []
        self.closed = threading.Event()
        self.drop = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.thread = threading.Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.closed.set()
        self.thread.join(timeout=3)
        self.listener.close()
        assert not self.thread.is_alive()

    def run(self):
        while not self.closed.is_set():
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(0.1)
                buf = b""
                try:
                    while b"\r\n\r\n" not in buf and not self.closed.is_set():
                        try:
                            data = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        buf += data
                    if b"\r\n\r\n" not in buf:
                        continue
                    head, buf = buf.split(b"\r\n\r\n", 1)
                    self.headers.append(head)
                    conn.sendall(self.response[:9])
                    time.sleep(0.01)
                    conn.sendall(self.response[9:] + self.data)
                    next_frame = time.monotonic() + 0.1
                    while not self.closed.is_set():
                        if self.drop.is_set():
                            self.drop.clear()
                            break
                        try:
                            data = conn.recv(4096)
                            if not data:
                                break
                            buf += data
                            while b"\r\n" in buf:
                                line, buf = buf.split(b"\r\n", 1)
                                self.ggas.append(line)
                        except socket.timeout:
                            pass
                        if time.monotonic() >= next_frame:
                            conn.sendall(self.data)
                            next_frame = time.monotonic() + 0.1
                except OSError:
                    pass

    def config(self, **overrides):
        return Config(host="127.0.0.1", port=self.port, mountpoint="TEST", username="test-user",
                      password="test-secret", gga_interval_sec=0.1, reconnect_sec=0.1,
                      **overrides)


@contextmanager
def client_for(caster, **overrides):
    client = NtripClient(caster.config(**overrides), gga_max_age=5)
    client.start()
    try:
        yield client
    finally:
        client.close()
        assert not client.thread.is_alive()


def test_config_is_private_and_not_represented(tmp_path):
    path = tmp_path / "config.json"
    values = dict(host="localhost", port=8002, mountpoint="TEST", username="test-user", password="test-secret")
    path.write_text(json.dumps(values))
    path.chmod(0o644)
    with pytest.raises(ValueError, match="private"):
        load_config(path)
    path.chmod(0o600)
    cfg = load_config(path)
    assert cfg.port == 8002 and "test-secret" not in repr(cfg) and "test-user" not in repr(cfg)
    for key, value in (("host", "a\r\nInjected: x"), ("mountpoint", "a b"),
                       ("port", 0), ("username", "a:b"), ("gga_interval_sec", float("inf"))):
        with pytest.raises(ValueError):
            Config(**dict(values, **{key: value}))


def test_private_configuration_writer_and_replacement(tmp_path):
    from wit_rtk.configure_ntrip import save_private

    path = tmp_path / "private" / "ntrip.json"
    cfg = Config("localhost", 8002, "TEST", "account", "password")
    assert save_private(path, cfg) == path
    assert path.stat().st_mode & 0o777 == 0o600
    assert load_config(path) == cfg
    save_private(path, Config("localhost", 8002, "TEST", "new-account", "new-password"))
    assert load_config(path).username == "new-account"
    assert list(path.parent.iterdir()) == [path]


def test_rtcm_crc_fragmentation_resync_and_bounded_partial():
    # CRC-24Q published check value for the ASCII test vector.
    assert crc24q(b"123456789") == 0xCDE703
    valid = rtcm()
    damaged = valid[:-1] + bytes((valid[-1] ^ 1,))
    parser = RtcmFrames()
    result = []
    for byte in b"HTTP error\r\n" + damaged + valid + valid:
        result += parser.feed(bytes((byte,)))
    # False preambles inside damaged frames can delay resync up to a full frame.
    result += parser.feed(b"\x00" * 1029)
    assert result == [valid, valid]
    assert parser.rejected >= 1 and len(parser.buffer) <= 1028


@pytest.mark.parametrize("response,accepted", [
    (b"ICY 200 OK\r\n", True),
    (b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n", True),
    (b"HTTP/1.0 401 Unauthorized\r\n\r\n", False),
    (b"SOURCETABLE 200 OK\r\n", False),
    (b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n", False),
    (b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n", False),
])
def test_handshake(response, accepted):
    reader, writer = socket.socketpair()
    with reader, writer:
        reader.settimeout(0.1)
        writer.sendall(response + rtcm())
        if accepted:
            assert read_response(reader) == rtcm()
        else:
            with pytest.raises(OSError):
                read_response(reader)


def test_gga_auth_stream_reconnect_and_receiver_disconnect():
    with Caster() as caster, client_for(caster) as client:
        time.sleep(0.1)
        assert not caster.headers
        client.set_receiver(True)
        client.update_gga(GGA, time.monotonic())
        wait_for(lambda: client.snapshot()["received_frames"] >= 2 and len(caster.ggas) >= 2)
        header = caster.headers[0]
        assert b"GET /TEST HTTP/1.0" in header
        assert b"Authorization: Basic " + base64.b64encode(b"test-user:test-secret") in header
        assert all(line == GGA for line in caster.ggas)
        generation, _, frame = client.take_frame(time.monotonic())
        assert frame == rtcm()
        caster.drop.set()
        wait_for(lambda: len(caster.headers) >= 2 and client.snapshot()["received_frames"] >= 3)
        assert client.observation()[0] != generation
        client.set_receiver(False)
        assert client.take_frame(time.monotonic()) is None
        wait_for(lambda: client.snapshot()["state"] == "waiting_receiver")
        count = len(caster.headers)
        time.sleep(0.3)
        assert len(caster.headers) == count


def test_bad_auth_and_no_rtcm_never_queue_data():
    for response, data in ((b"HTTP/1.0 401 Unauthorized\r\n\r\n", rtcm()),
                           (b"ICY 200 OK\r\n", b"not RTCM")):
        with Caster(response, data) as caster, client_for(caster, data_timeout_sec=0.3) as client:
            client.set_receiver(True)
            client.update_gga(GGA, time.monotonic())
            wait_for(lambda: client.snapshot()["reconnects"] >= 1)
            assert client.snapshot()["received_bytes"] == 0
            assert client.take_frame(time.monotonic()) is None
            assert "test-secret" not in str(client.snapshot())


def test_stale_gga_and_stale_queue_are_not_used():
    with Caster() as caster, client_for(caster) as client:
        client.set_receiver(True)
        client.update_gga(GGA, time.monotonic() - 10)
        wait_for(lambda: client.snapshot()["state"] == "waiting_gga")
        assert not caster.headers
        client.update_gga(GGA, time.monotonic())
        wait_for(lambda: client.snapshot()["received_frames"] > 0)
        assert client.take_frame(time.monotonic() + 10) is None
        client.update_gga(None, time.monotonic())
        wait_for(lambda: client.snapshot()["state"] == "waiting_gga")
        assert client.take_frame(time.monotonic()) is None


def test_full_queue_reconnects_instead_of_unbounded_backlog():
    with Caster(data=rtcm() * 20) as caster, client_for(caster) as client:
        client.MAX_QUEUE_BYTES = 100
        client.set_receiver(True)
        client.update_gga(GGA, time.monotonic())
        wait_for(lambda: client.snapshot()["reconnects"] > 0)
        assert client.snapshot()["queued_bytes"] <= 100
