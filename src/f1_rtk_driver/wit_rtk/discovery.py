"""Passively identify a WTRTK-982 among USB CH340 adapters."""

import serial
from serial.tools import list_ports

from wit_rtk.nmea import Fix, Heading, parse_sentence


class ReceiverDiscovery:
    def __init__(self, baudrate, retry):
        self.baudrate = baudrate
        self.retry = retry
        self.candidates = []
        self.handle = None
        self.next_scan = 0.0
        self.reason = "Waiting for a CH340 with checksummed GGA and HPR"

    def close(self):
        if self.handle is not None:
            self.handle.close()
            self.handle = None

    def poll(self, now):
        if self.handle is None:
            if not self.candidates:
                if now < self.next_scan:
                    return None
                self.candidates = sorted(p.device for p in list_ports.comports()
                                         if (p.vid, p.pid) == (0x1a86, 0x7523))
                self.next_scan = now + self.retry
            if not self.candidates:
                return None
            port = self.candidates.pop(0)
            try:
                self.handle = serial.Serial(port, self.baudrate, timeout=0, exclusive=True)
            except (OSError, serial.SerialException):
                return None
            self.deadline = now + 1.5
            self.partial = b""
            self.seen = set()
        try:
            lines = (self.partial + self.handle.read(4096)).split(b"\n")
        except (OSError, serial.SerialException):
            self.close()
            return None
        self.partial = lines.pop()[-1024:]
        for line in lines:
            try:
                sample = parse_sentence(line.rstrip(b"\r"))
            except ValueError:
                continue
            if sample is not None:
                self.seen.add(type(sample))
        if self.seen == {Fix, Heading}:
            # Transfer the verified, still-locked descriptor to the ROS reader.
            handle, self.handle = self.handle, None
            self.candidates.clear()
            return handle
        if now >= self.deadline:
            self.close()
            if not self.candidates:
                self.next_scan = now + self.retry
        return None
