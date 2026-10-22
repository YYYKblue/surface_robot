#!/usr/bin/env python3
"""Offline rectangular-pool route illustration; never publishes ROS commands."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--end-inset", type=float, default=.80,
                        help="Illustrative centre-to-short-wall distance, not calibrated stopping distance")
    parser.add_argument("--wall-offset", type=float, default=.32,
                        help="Illustrative centre-to-long-wall target; refine with live lidar")
    args = parser.parse_args()
    meta = yaml.safe_load(args.map_yaml.read_text())
    # Backups preserve Nano absolute image paths. Prefer their local sibling.
    image_path = args.map_yaml.parent / Path(meta["image"]).name
    if not image_path.is_file():
        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = args.map_yaml.parent / image_path
    pixels = np.array(Image.open(image_path).convert("L"))
    resolution = float(meta["resolution"])
    ox, oy, yaw = map(float, meta["origin"])
    if yaw != 0 or meta.get("negate", 0) != 0:
        raise ValueError("This inspection tool currently expects zero map yaw and negate=0")
    probability = 1. - pixels.astype(float) / 255.
    free = probability < float(meta["free_thresh"])
    occupied = probability > float(meta["occupied_thresh"])
    count, labels, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), 8)
    if count < 2:
        raise ValueError("No connected free region")
    free_region = labels == (1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))

    def points(mask):
        rows, cols = np.where(mask)
        return np.column_stack((ox + (cols + .5) * resolution,
                                oy + (pixels.shape[0] - rows - .5) * resolution))

    free_points = points(free_region)
    box = cv2.boxPoints(cv2.minAreaRect(free_points.astype(np.float32)))
    edges = np.roll(box, -1, axis=0) - box
    axis = edges[np.argmax(np.linalg.norm(edges, axis=1))].astype(float)
    axis /= np.linalg.norm(axis)
    if axis[0] < 0:
        axis = -axis
    normal = np.array([-axis[1], axis[0]])
    fu, fv = free_points @ axis, free_points @ normal
    occupied_points = points(occupied)
    ou, ov = occupied_points @ axis, occupied_points @ normal

    def median(values, mask):
        if np.count_nonzero(mask) < 8:
            raise ValueError("Insufficient occupied cells to estimate four rectangular walls")
        return float(np.median(values[mask]))

    # Median occupied bands, excluding corners. This assumes one empty rectangle;
    # it is a visual planning aid, not a general obstacle-aware wall extractor.
    middle_v = (ov > fv.min() + .20) & (ov < fv.max() - .20)
    middle_u = (ou > fu.min() + .20) & (ou < fu.max() - .20)
    u0 = median(ou, middle_v & (ou < fu.min() + .10))
    u1 = median(ou, middle_v & (ou > fu.max() - .10))
    v0 = median(ov, middle_u & (ov < fv.min() + .10))
    v1 = median(ov, middle_u & (ov > fv.max() - .10))
    if not 0 < args.end_inset < (u1 - u0) / 2:
        raise ValueError("End inset leaves no usable cleaning segment")
    if not 0 < args.wall_offset < (v1 - v0) / 2:
        raise ValueError("Wall offset leaves no separation between lanes")
    vm = (v0 + v1) / 2
    start_u, end_u = u0 + args.end_inset, u1 - args.end_inset
    route_uv = [(start_u, v1 - args.wall_offset), (end_u, v1 - args.wall_offset),
                (end_u, vm), (end_u, v0 + args.wall_offset),
                (start_u, v0 + args.wall_offset)]
    route = np.array([axis * u + normal * v for u, v in route_uv])
    heading = math.atan2(axis[1], axis[0])
    # Current symmetric rigid envelope plus 3 cm margin, expanded by an explicit
    # illustrative 10 cm localization reserve. Neither includes unknown dynamics.
    turn_radius = math.hypot(.56 / 2 + .03, .56 / 2 + .03)
    pose_reserve = .10
    reserved_radius = turn_radius + pose_reserve
    rows, cols = np.indices(pixels.shape)
    gx = ox + (cols + .5) * resolution
    gy = oy + (pixels.shape[0] - rows - .5) * resolution
    centre = route[2]
    # Half-cell diagonal padding includes every cell intersecting the disk.
    disk = (gx - centre[0]) ** 2 + (gy - centre[1]) ** 2 <= (
        reserved_radius + resolution / math.sqrt(2)) ** 2
    within_image = (ox <= centre[0] - reserved_radius
                    and centre[0] + reserved_radius <= ox + pixels.shape[1] * resolution
                    and oy <= centre[1] - reserved_radius
                    and centre[1] + reserved_radius <= oy + pixels.shape[0] * resolution)
    turn_disk_free = bool(within_image and np.all(free[disk]))
    report = {
        "execution_ready": False,
        "purpose": "offline candidate illustration, not executable waypoints",
        "map_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "resolution_m": resolution,
        "canvas_size_m": [pixels.shape[1] * resolution, pixels.shape[0] * resolution],
        "largest_free_area_m2": float(free_region.sum() * resolution ** 2),
        "free_cell_centres_min_rectangle_m": [float(np.ptp(fu)), float(np.ptp(fv))],
        "approx_wall_centre_spacing_m": [u1 - u0, v1 - v0],
        "long_axis_yaw_rad": heading,
        "wall_offset_m": args.wall_offset,
        "illustrative_centre_end_inset_m": args.end_inset,
        "rigid_turn_radius_with_margin_m": turn_radius,
        "illustrative_pose_reserve_m": pose_reserve,
        "turn_disk_in_known_free_cells": turn_disk_free,
        "candidate_map_xy_m": dict(zip(
            ["A_start", "A_stop", "centre_turn_180", "B_start", "B_stop"], route.tolist())),
        "unresolved": ["live localization and starting-wall identity",
                       "fifth-thruster wire polarity and outward-motion confirmation",
                       "swept brush geometry and dynamic stopping/turn drift",
                       "live obstacle coverage during 180-degree turn",
                       "actual contact/cleaning performance"],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "map_analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.imshow(pixels, cmap="gray", vmin=0, vmax=255,
              extent=[ox, ox + pixels.shape[1] * resolution,
                      oy, oy + pixels.shape[0] * resolution], origin="upper")
    for i, j, color, style in [(0, 1, "#0876c9", "-"), (1, 2, "#d47700", "--"),
                                (2, 3, "#d47700", "--"), (3, 4, "#0876c9", "-")]:
        ax.annotate("", xy=route[j], xytext=route[i],
                    arrowprops=dict(arrowstyle="->", color=color, lw=2.5, linestyle=style))
    for point, label in zip(route, ["A start", "A stop", "180 deg turn", "B start", "B stop"]):
        ax.plot(*point, "o", color="#8b145c", ms=5)
        ax.annotate(label, point, xytext=(7, 8), textcoords="offset points", fontsize=10,
                    bbox=dict(facecolor="white", alpha=.8, edgecolor="none", pad=1))
    ax.add_patch(Circle(centre, turn_radius, fill=False, color="#8b145c", lw=1.5))
    ax.add_patch(Circle(centre, reserved_radius, fill=False, color="#8b145c", lw=1, linestyle=":"))
    ax.set_title("Two long-wall passes: offline concept, not an executable route")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal")
    fig.text(.5, .025, "Blue: forward cleaning | Orange: lateral transfer, brush off | "
             "Circle: rigid turn envelope + dotted pose reserve", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .055, 1, 1))
    fig.savefig(args.output_dir / "long_wall_route.png", dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
