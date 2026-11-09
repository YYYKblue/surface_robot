"""Prepare immutable map images outside the GUI and socket receive threads."""
import base64
import threading
import time
import zlib

try:
    from PySide6.QtGui import QColor, QImage
except ImportError:
    from PyQt5.QtGui import QColor, QImage


def prepare_map(packet, key="map"):
    mapping = packet.get(key)
    if mapping is None:
        return packet
    width, height = int(mapping.get("width", 0)), int(mapping.get("height", 0))
    size = width * height
    if width <= 0 or height <= 0 or size > 64 * 1024 * 1024:
        raise ValueError("地图尺寸无效或过大")
    if mapping.get("encoding") == "indexed8_zlib":
        compressed = base64.b64decode(mapping["data_zlib_base64"], validate=True)
        decoder = zlib.decompressobj()
        pixels = decoder.decompress(compressed, size + 1)
        if len(pixels) != size or not decoder.eof or decoder.unused_data:
            raise ValueError("地图压缩数据与尺寸不一致")
        if pixels.translate(bytes(0 if i < 3 else 1 for i in range(256))).count(1):
            raise ValueError("地图包含无效颜色索引")
    elif "data" in mapping:
        data = mapping["data"]
        if len(data) != size:
            raise ValueError("地图数据与尺寸不一致")
        pixels = bytes(2 if v > 50 else 1 if v >= 0 else 0 for v in data)
    else:
        raise ValueError("不支持的地图编码")
    image = QImage(pixels, width, height, width, QImage.Format_Indexed8).copy()
    image.setColorTable([QColor(color).rgb() for color in ("#eef2f3", "#ffffff", "#536875")])
    result = dict(packet)
    result[key] = {k: v for k, v in mapping.items() if k not in ("data", "data_zlib_base64")}
    result["_map_image"] = image.mirrored(False, True)
    return result


class MapDecoder:
    """One running decode and one replaceable pending map; resets reject old results."""
    def __init__(self, deliver, report_error):
        self.deliver, self.report_error = deliver, report_error
        self.condition = threading.Condition()
        self.pending = None
        self.generation = 0
        self.closed = False
        self.thread = None

    def submit(self, packet, order):
        with self.condition:
            if self.closed:
                return
            # Replacement affects only the pending slot. Invalidating every
            # running decode would starve display when arrivals outrun decoding.
            self.pending = (self.generation, packet, order)
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, name="operator-map-image", daemon=True)
                self.thread.start()
            self.condition.notify()

    def reset(self):
        with self.condition:
            self.generation += 1
            self.pending = None

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                generation, packet, order = self.pending
                self.pending = None
            start = time.monotonic()
            try:
                result = prepare_map(packet)
                result["_map_decode_ms"] = round((time.monotonic() - start) * 1000, 2)
            except Exception as error:
                with self.condition:
                    if generation == self.generation and not self.closed:
                        self.report_error("地图解码失败：" + str(error))
                continue
            with self.condition:
                if generation == self.generation and not self.closed:
                    self.deliver(result, order)

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=1)
