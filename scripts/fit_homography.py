#!/usr/bin/env python3
"""
Fit a ground-plane homography from matched cone pairs.

Input is the JSON written by extract_cone_correspondences.py: a list of
{"image_point": [u, v], "world_point": [x, y, z]} entries, where x is
forward and y is left in metres. Fits H (image pixel -> ground x/y) with
RANSAC and writes a YAML file the launch files can load directly.
"""

import argparse
import json
import os

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', default='cone_correspondences.json',
                        help='Correspondences JSON (default: %(default)s)')
    parser.add_argument('--output', default='scripts/data/homography_stereo_lidar.yaml',
                        help='Output YAML path (default: %(default)s)')
    parser.add_argument('--max-distance', type=float, default=15.0,
                        help='Ignore cones further than this, metres (default: %(default)s)')
    parser.add_argument('--ransac-threshold', type=float, default=5.0,
                        help='RANSAC reprojection threshold, metres (default: %(default)s)')
    args = parser.parse_args()

    if not os.path.exists(args.input):
        parser.error(f"input file not found: {args.input}")

    print(f"Loading data from {args.input}...")
    with open(args.input) as f:
        data = json.load(f)

    image_pts, world_pts = [], []
    for entry in data:
        wp = entry['world_point']
        # Far cones are less accurate, so leave them out of the fit
        if np.hypot(wp[0], wp[1]) > args.max_distance:
            continue
        image_pts.append(entry['image_point'])
        world_pts.append([wp[0], wp[1]])  # ground plane: x forward, y left

    if len(image_pts) < 4:
        print(f"Need at least 4 points within {args.max_distance} m, got {len(image_pts)}.")
        return

    image_pts = np.array(image_pts, dtype=np.float32)
    world_pts = np.array(world_pts, dtype=np.float32)

    print(f"Computing homography with {len(image_pts)} points...")
    H, status = cv2.findHomography(image_pts, world_pts, cv2.RANSAC, args.ransac_threshold)
    if H is None:
        print("Homography computation failed!")
        return

    print(f"RANSAC inliers: {int(np.sum(status))}/{len(image_pts)}")

    reprojected = cv2.perspectiveTransform(image_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    mean_error = float(np.mean(np.linalg.norm(reprojected - world_pts, axis=1)))
    print(f"Mean reprojection error: {mean_error:.4f} m")

    H = H / H[2, 2]
    output = {
        'homography_matrix': [[float(v) for v in row] for row in H],
        'units_to_metres': 1.0,  # world points are already in metres
        'swap_xy': False,        # row 0 = forward x, row 1 = lateral y
        'frame_origin': {'x': 0.0, 'y': 0.0},
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        f.write(f"# Fitted by fit_homography.py from {len(image_pts)} correspondences, "
                f"mean reprojection error {mean_error:.3f} m.\n")
        yaml.safe_dump(output, f, default_flow_style=None, sort_keys=False)
    print(f"Saved homography to {args.output}")


if __name__ == '__main__':
    main()
