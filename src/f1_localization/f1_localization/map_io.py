"""Saved-map paths, occupancy masks and free-space bounds shared by consumers."""
from dataclasses import dataclass
import math
from pathlib import Path


class MapProfileError(ValueError):
    """The map cannot be turned into task geometry without guessing."""


def resolve_map_files(path):
    """Return ``(yaml_path, image_hint)`` for a .pbstream or map .yaml path."""
    path = Path(path).expanduser().resolve()
    if path.suffix == ".pbstream":
        yaml_path = path.with_suffix(".yaml")
        if not yaml_path.is_file():
            raise MapProfileError(
                f"{yaml_path.name} is missing next to {path.name}; export it with "
                "cartographer_pbstream_to_ros_map, then launch with map:=<that path>")
        image = path.with_suffix(".pgm")
        return yaml_path, image if image.is_file() else None
    if path.suffix in (".yaml", ".yml"):
        if not path.is_file():
            raise MapProfileError(f"map yaml not found: {path}")
        return path, None
    raise MapProfileError("map must name a .pbstream or a map .yaml file")


def resolve_pbstream(path):
    """Return the frozen ``.pbstream`` that pairs with ``path``.

    Localization needs Cartographer state, not the raster export; the two are
    saved with the same stem, so either one names the same map.
    """
    path = Path(path).expanduser().resolve()
    if path.suffix == ".pbstream":
        if not path.is_file():
            raise MapProfileError(f"map pbstream not found: {path}")
        return path
    if path.suffix in (".yaml", ".yml", ".pgm"):
        pbstream = path.with_suffix(".pbstream")
        if not pbstream.is_file():
            raise MapProfileError(
                f"{pbstream.name} is missing next to {path.name}; the raster map cannot "
                "replace the frozen Cartographer state")
        return pbstream
    raise MapProfileError("map must name a .pbstream or a map .yaml file")


def bounds_of(points):
    """Axis-aligned map bounds of ``[(x, y), ...]``."""
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs), min(ys), max(xs), max(ys))


def _meta(yaml_path):
    import yaml
    try:
        meta = yaml.safe_load(yaml_path.read_text())
    except (OSError, ValueError) as exc:
        raise MapProfileError(f"cannot read map yaml {yaml_path}: {exc}") from exc
    if not isinstance(meta, dict):
        raise MapProfileError(f"{yaml_path.name} is not a map yaml")
    return meta


def _image_path(yaml_path, meta, hint):
    """Prefer the sibling image so a map copied off the Nano still loads."""
    candidates = []
    if hint is not None:
        candidates.append(Path(hint))
    declared = Path(str(meta.get("image", "")))
    candidates.extend((yaml_path.parent / declared.name, declared))
    for candidate in candidates:
        if candidate.name and candidate.is_file():
            return candidate
    raise MapProfileError(
        f"map image for {yaml_path.name} not found; looked beside the yaml and at "
        f"{meta.get('image')!r}")


@dataclass(frozen=True)
class Occupancy:
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int
    free: object
    occupied: object
    region: object  # largest free connected component

    def xy(self, column, row):
        """Grid index to map metres; image row 0 is the top of the canvas."""
        return (self.origin_x_m + (column + .5) * self.resolution_m,
                self.origin_y_m + (self.height - row - .5) * self.resolution_m)

    def cell(self, x, y):
        """``'free'``/``'occupied'``/``'unknown'``/``None`` for a map point."""
        column = int(math.floor((x - self.origin_x_m) / self.resolution_m))
        # The bottom map row is image row height - 1; map y increases upward.
        row = self.height - 1 - int(math.floor((y - self.origin_y_m) / self.resolution_m))
        if not (0 <= row < self.height and 0 <= column < self.width):
            return None
        if self.occupied[row, column]:
            return "occupied"
        return "free" if self.free[row, column] else "unknown"


def load_occupancy(path):
    """Read the map image as free/occupied masks in map coordinates."""
    import cv2
    import numpy as np

    yaml_path, hint = resolve_map_files(path)
    meta = _meta(yaml_path)
    for key in ("resolution", "origin", "occupied_thresh", "free_thresh"):
        if key not in meta:
            raise MapProfileError(f"{yaml_path.name} is missing {key!r}")
    resolution = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]
    if not math.isfinite(resolution) or resolution <= 0:
        raise MapProfileError("map resolution must be positive")
    if len(origin) >= 3 and abs(origin[2]) > 1e-6:
        raise MapProfileError("rotated map origins are unsupported; re-export the map")
    if len(origin) < 2 or not all(math.isfinite(v) for v in origin[:2]):
        raise MapProfileError(f"{yaml_path.name} has an invalid origin")
    pixels = cv2.imread(str(_image_path(yaml_path, meta, hint)), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise MapProfileError(f"cannot decode the map image for {yaml_path.name}")
    # map_server convention: occupancy probability is 1 - pixel/255 unless the
    # map is negated, so bright pixels are free space.
    probability = 1. - pixels.astype(np.float64) / 255.
    if int(meta.get("negate", 0)):
        probability = 1. - probability
    free = (probability < float(meta["free_thresh"])).astype(np.uint8)
    occupied = (probability > float(meta["occupied_thresh"])).astype(np.uint8)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(free, 8)
    if count < 2:
        raise MapProfileError("map has no free region")
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    region = (labels == index).astype(np.uint8)
    height, width = pixels.shape
    return Occupancy(resolution, origin[0], origin[1], width, height,
                     free, occupied, region)


def occupancy_points(occupancy, mask):
    import numpy as np

    rows, columns = np.nonzero(mask)
    if not len(rows):
        return np.empty((0, 2))
    return np.column_stack(occupancy.xy(columns, rows)).astype(np.float64)


def free_bounds(occupancy):
    points = occupancy_points(occupancy, occupancy.region)
    return (float(points[:, 0].min()), float(points[:, 1].min()),
            float(points[:, 0].max()), float(points[:, 1].max()))
