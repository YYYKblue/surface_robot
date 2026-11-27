"""Export the same boat-centre route used by the ROS controller."""
import argparse
import json
import cv2
import numpy as np

from f1_localization.map_io import load_occupancy
from .mission import Settings
from .planner import WaterGrid, plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map")
    parser.add_argument("--start", type=float, nargs=2, required=True, metavar=("X", "Y"))
    parser.add_argument("--spacing", type=float, default=Settings.lane_spacing_m)
    parser.add_argument("--polygon", type=float, nargs="+", default=[])
    parser.add_argument("--png", required=True)
    args = parser.parse_args()
    grid = WaterGrid(load_occupancy(args.map), args.polygon)
    route = plan(grid, tuple(args.start), args.spacing)
    img = np.full((*grid.free.shape, 3), 100, np.uint8)
    img[grid.map.occupied > 0] = (30, 30, 30)
    img[grid.free > 0] = (190, 170, 140)
    img[grid.labels == grid.label(args.start)] = (220, 210, 180)
    img[route.region] = (235, 255, 235)
    # Upsample small pools so individual raster cells and short transfers are legible.
    scale = max(1, min(8, 800//max(img.shape[:2])))
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    def point(xy):
        r, c = grid.cell(*xy)
        return c*scale+scale//2, r*scale+scale//2
    for leg in route.legs:
        cv2.line(img, point(leg.start), point(leg.end),
                 (200, 70, 30) if leg.kind == "lane" else (30, 145, 210), 1)
    cv2.circle(img, point(args.start), 3, (20, 20, 220), -1)
    if not cv2.imwrite(args.png, img):
        parser.error("无法写入预览图片")
    print(json.dumps(dict(legs=len(route.legs), reachable_m2=route.region.sum()*grid.res**2,
                          disconnected_m2=route.disconnected_m2, skipped_m2=route.skipped_m2,
                          uncovered_cells=int((route.region & ~route.band).sum())), indent=2))


if __name__ == "__main__":
    main()
