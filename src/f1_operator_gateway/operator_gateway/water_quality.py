"""Read-only Modbus RTU transactions for the four probes in docs/water."""
import select
import struct
import time

import serial
from serial.tools import list_ports


REGISTER_COUNTS = {"ph": 2, "ec": 4, "chlorine": 1, "turbidity": 2}


def crc_bytes(payload):
    crc = 0xffff
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return struct.pack("<H", crc)


def read_registers(port, address, count, start=0):
    request = struct.pack(">BBHH", address, 3, start, count)
    request += crc_bytes(request)
    port.reset_input_buffer()
    if port.write(request) != len(request):
        raise OSError("incomplete Modbus request")
    frame = port.read(5 + 2 * count)
    if not frame:
        raise TimeoutError("no Modbus response")
    if len(frame) < 5 or crc_bytes(frame[:-2]) != frame[-2:]:
        raise ValueError("incomplete response or CRC mismatch")
    if frame[0] != address:
        raise ValueError("unexpected Modbus address")
    if frame[1] == 0x83 and len(frame) == 5:
        raise ValueError(f"Modbus exception {frame[2]:02x}")
    if frame[1:3] != bytes((3, 2 * count)) or len(frame) != 5 + 2 * count:
        raise ValueError("unexpected function or register count")
    return struct.unpack(f">{count}H", frame[3:-2])


def measurements(sensor, words, ec_k, turbidity_range):
    if sensor == "chlorine":
        return {"value": words[0] / 100}
    signed_temperature = words[1] if words[1] < 32768 else words[1] - 65536
    result = {"temperature_c": signed_temperature / 10}
    if sensor == "ec":
        divisor = {1: 10, 10: 1}.get(ec_k)
        result.update(salinity_ppm=float(words[2]), tds_ppm=float(words[3]))
    elif sensor == "turbidity":
        # Register 0x0000 has a different definition from offset register 0x0050.
        divisor = {50: 100, 200: 10, 1000: 10, 4000: 10}.get(turbidity_range)
    else:  # pH
        divisor = 100
    if divisor is not None:
        result["value"] = words[0] / divisor
    return result


class WaterBus:
    def __init__(self, path, baudrate):
        self.path = path
        self.baudrate = baudrate
        self.port = None
        self.last_transaction = 0.
        self.candidates = []
        self.next_scan = 0.

    def discover(self, addresses):
        """Try one free CH340 per call. Keep the verified descriptor without reopening."""
        if self.port is not None or len(addresses) < 2:
            return
        if not self.candidates:
            if time.monotonic() < self.next_scan:
                return
            self.candidates = sorted(p.device for p in list_ports.comports()
                                     if (p.vid, p.pid) == (0x1a86, 0x7523))
            self.next_scan = time.monotonic() + 5.
        if not self.candidates:
            return
        port = None
        try:
            port = serial.Serial(self.candidates.pop(0), 115200, timeout=.25,
                                 write_timeout=.25, exclusive=True)
            # RTK streams NMEA at 115200. Leave streaming ports before transmitting.
            if select.select([port], [], [], 1.5)[0]:
                return
            port.baudrate = self.baudrate
            matches = 0
            for address in addresses:
                time.sleep(40 / self.baudrate)
                try:
                    words = read_registers(port, address, 1, start=0x07d0)
                except (TimeoutError, ValueError):
                    continue
                if words == (address,):
                    matches += 1
                if matches == 2:
                    self.port, port = port, None
                    self.last_transaction = time.monotonic()
                    self.candidates.clear()
                    return
        except (OSError, ValueError):
            pass  # Busy, removed, or failed to configure. Try the next candidate.
        finally:
            if port is not None:
                port.close()
            if not self.candidates:
                self.next_scan = time.monotonic() + 5.

    def poll(self, sensor, address, ec_k, turbidity_range):
        try:
            if self.port is None:
                if self.path == "auto":
                    return "offline", "searching for water bus", {}
                self.port = serial.Serial(self.path, self.baudrate, timeout=.25,
                                          write_timeout=.25, exclusive=True)
            # The manuals require at least four silent 8N1 characters between frames.
            remaining = self.last_transaction + 40 / self.baudrate - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            words = read_registers(self.port, address, REGISTER_COUNTS[sensor])
        except TimeoutError as exc:
            return "offline", str(exc), {}
        except ValueError as exc:
            return "protocol_error", str(exc), {}
        except OSError as exc:
            self.close()
            return "offline", str(exc), {}
        finally:
            self.last_transaction = time.monotonic()
        values = measurements(sensor, words, ec_k, turbidity_range)
        return ("ok" if "value" in values else "range_required"), "", values

    def close(self):
        if self.port is not None:
            self.port.close()
            self.port = None
