"""Map, pose and laser visualization widget."""
import math

try:
    from PySide6.QtCore import QPointF, Qt
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
    from PySide6.QtWidgets import QWidget
except ImportError:
    from PyQt5.QtCore import QPointF, Qt
    from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
    from PyQt5.QtWidgets import QWidget


class MapCanvas(QWidget):
    # The current vehicle installation exposes the sensor/base heading with a
    # 180 degree difference.  Keep the correction at the display boundary so
    # the map pose and the scan overlay use the same visible hull heading.
    DISPLAY_HEADING_OFFSET_RAD = math.pi

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(680, 500)
        self.telemetry = {}
        self.map_image = None
        self.map_revision = object()
        self.setToolTip("滚轮缩放，地图叠加实时雷达和机器人位置")

    def set_telemetry(self, packet):
        mapping = packet.get("map")
        # Older gateways sent map packets as {type, width, height, ...}; the
        # app originally only accepted the newer nested form.  Normalize both
        # forms here so an app can still display a map during a rolling update.
        if mapping is None and packet.get("type") == "map" and "width" in packet:
            mapping = {key: packet[key] for key in
                       ("resolution", "width", "height", "origin", "data", "revision")
                       if key in packet}
        if mapping is not None:
            revision = mapping.get("revision")
            if revision is None:
                revision = hash((mapping.get("resolution"), mapping.get("width"),
                                 mapping.get("height"), mapping.get("origin", {}).get("x"),
                                 mapping.get("origin", {}).get("y"),
                                 tuple(mapping.get("data", []))))
            if revision != self.map_revision:
                self.map_image = self._make_map_image(mapping)
                self.map_revision = revision
            self.telemetry["map"] = mapping
        elif "map" in packet:
            self.map_image = None
            self.map_revision = object()
            self.telemetry.pop("map", None)
        self.telemetry.update({key: value for key, value in packet.items()
                               if value is not None and key != "map"})
        self.update()

    @staticmethod
    def _make_map_image(mapping):
        width, height = int(mapping.get("width", 0)), int(mapping.get("height", 0))
        if not width or not height:
            return None
        data = mapping.get("data", [])
        image = QImage(width, height, QImage.Format_RGB32)
        for row in range(height):
            for col in range(width):
                index = row * width + col
                value = int(data[index]) if index < len(data) else -1
                image.setPixelColor(col, row, QColor("#536875") if value > 50 else
                                    QColor("#ffffff") if value >= 0 else QColor("#eef2f3"))
        # OccupancyGrid row zero is the bottom row in map coordinates, while a
        # QImage row zero is at the top of the widget.
        return image.mirrored(False, True)

    @classmethod
    def _display_yaw(cls, pose):
        return float(pose.get("yaw", 0.0)) + cls.DISPLAY_HEADING_OFFSET_RAD

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#e9eef1"))
        margin = 28
        area = self.rect().adjusted(margin, margin, -margin, -margin)
        p.setPen(QPen(QColor("#dce5e9"), 1))
        for x in range(area.left(), area.right(), 28):
            p.drawLine(x, area.top(), x, area.bottom())
        for y in range(area.top(), area.bottom(), 28):
            p.drawLine(area.left(), y, area.right(), y)
        mapping = self.telemetry.get("map") or {}
        width, height = int(mapping.get("width", 0)), int(mapping.get("height", 0))
        resolution = float(mapping.get("resolution", .02) or .02)
        if width and height:
            scale = min(area.width() / (width * resolution), area.height() / (height * resolution))
            ox = area.center().x() - width * resolution * scale / 2
            oy = area.center().y() + height * resolution * scale / 2
            if self.map_image is not None:
                target = self.map_image.scaled(round(width * scale * resolution),
                                               round(height * scale * resolution),
                                               Qt.IgnoreAspectRatio, Qt.FastTransformation)
                p.drawImage(round(ox), round(oy - height * resolution * scale), target)
        else:
            scale, ox, oy = 120., area.center().x(), area.center().y()
        pose = self.telemetry.get("pose") or {}
        if pose:
            px = ox + (float(pose.get("x", 0)) - float(mapping.get("origin", {}).get("x", 0))) * scale
            py = oy - (float(pose.get("y", 0)) - float(mapping.get("origin", {}).get("y", 0))) * scale
            scan = self.telemetry.get("scan") or {}
            increment, angle_min = float(scan.get("angle_increment", 0)), float(scan.get("angle_min", 0))
            p.setPen(QPen(QColor("#df8b35"), 2))
            for i, distance in enumerate(scan.get("ranges", [])):
                if distance is None:
                    continue
                distance = float(distance)
                if not math.isfinite(distance) or not .05 < distance < 20:
                    continue
                angle = self._display_yaw(pose) + angle_min + i * increment
                p.drawPoint(QPointF(px + math.cos(angle) * distance * scale,
                                    py - math.sin(angle) * distance * scale))
            p.setBrush(QColor("#00877e"))
            p.setPen(QPen(QColor("#006b64"), 2))
            yaw = self._display_yaw(pose)
            points = QPolygonF([QPointF(px + 25 * math.cos(yaw),
                                        py - 25 * math.sin(yaw)),
                                QPointF(px - 16 * math.cos(yaw + .8),
                                        py + 16 * math.sin(yaw + .8)),
                                QPointF(px - 16 * math.cos(yaw - .8),
                                        py + 16 * math.sin(yaw - .8))])
            p.drawPolygon(points)
        p.setPen(QPen(QColor("#637783"), 1))
        p.drawText(area.left() + 10, area.top() + 22, "地图 / map    ·    橙色：实时雷达    ·    箭头：机器人朝向")
