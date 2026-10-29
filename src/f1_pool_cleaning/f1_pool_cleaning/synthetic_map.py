"""Synthetic PGM/YAML maps for offline examples and tests.

These write small maps with the same conventions as the deployed
Cartographer exports (map_server thresholds, P5 raster, wall bands around free
water), so map derivation can be exercised without the git-ignored ``maps/``
directory.  Nothing here runs at robot runtime.
"""
from pathlib import Path

import numpy as np

RESOLUTION = .05
# map_server: occupancy probability is 1 - pixel/255.
FREE_PIXEL, WALL_PIXEL, UNKNOWN_PIXEL = 254, 0, 205


def write_map(directory, name, free, occupied, origin=(0., 0.), resolution=RESOLUTION):
    """Write a PGM/YAML pair; ``free``/``occupied`` are boolean row masks."""
    directory = Path(directory)
    height, width = free.shape
    pixels = np.full((height, width), UNKNOWN_PIXEL, dtype=np.uint8)
    pixels[free] = FREE_PIXEL
    pixels[occupied] = WALL_PIXEL
    pgm = directory / f"{name}.pgm"
    with pgm.open("wb") as handle:
        handle.write(b"P5\n%d %d\n255\n" % (width, height))
        handle.write(pixels.tobytes())
    yaml_path = directory / f"{name}.yaml"
    yaml_path.write_text(
        f"image: {pgm}\nresolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return yaml_path


def grid(free_fn, bounds, resolution=RESOLUTION):
    """Rasterise ``free_fn(x, y)`` over map-space ``bounds``."""
    x_min, y_min, x_max, y_max = bounds
    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))
    columns = x_min + (np.arange(width) + .5) * resolution
    rows = y_max - (np.arange(height) + .5) * resolution
    mesh_x, mesh_y = np.meshgrid(columns, rows)
    return free_fn(mesh_x, mesh_y), (x_min, y_min)


def wall_band(free, cells=2):
    """Occupied cells outside the free region, ``cells`` deep."""
    import cv2

    grown = cv2.dilate(free.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=cells)
    return grown.astype(bool) & ~free


def rectangle_map(directory, resolution=RESOLUTION):
    """A closed 3.60 x 1.40 m basin: free water from x=1..4.6, y=1..2.4."""
    free, origin = grid(lambda x, y: (x >= 1.) & (x <= 4.6) & (y >= 1.) & (y <= 2.4),
                        (0., 0., 5.6, 3.4), resolution)
    return write_map(directory, "rectangle", free, wall_band(free), origin, resolution)


def l_map(directory, resolution=RESOLUTION):
    """An open L: a west-running channel joined to a north-running channel.

    Wall-on-left order is west along the south wall, then a right turn north
    along the west wall -- the same handedness as the deployed L task.
    """
    def water(x, y):
        horizontal = (x >= -11.8) & (x <= -.2) & (y >= .2) & (y <= 2.2)
        vertical = (x >= -11.8) & (x <= -9.8) & (y >= .2) & (y <= 11.8)
        return horizontal | vertical
    free, origin = grid(water, (-12.6, -0.6, 0.6, 12.6), resolution)
    return write_map(directory, "l_shape", free, wall_band(free), origin, resolution)
