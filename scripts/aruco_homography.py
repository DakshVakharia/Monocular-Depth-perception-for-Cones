#!/usr/bin/env python3
"""
Compute static ground-plane homography from ArUco markers on the ground.

Markers are placed flat on the ground with top edge (+x forward) and left edge (+y)
facing the car. The homography maps image pixel (u, v) -> ground (x, y) in metres.

Usage:
  python3 aruco_homography.py --image img.jpg --markers markers.yaml --show
  python3 aruco_homography.py --camera 0 --markers markers.yaml
"""

import cv2
import yaml
import numpy as np
import argparse
from pathlib import Path
import sys


def load_marker_positions(yaml_path):
    """Load marker IDs and ground positions (centre) from YAML."""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    dictionary_name = data.get('dictionary', 'DICT_4X4_50')
    markers = data.get('markers', {})
    return dictionary_name, {int(k): v for k, v in markers.items()}


def get_aruco_dict(name):
    """Get ArUco dictionary by name."""
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_markers(image, aruco_dict):
    """Detect ArUco markers; return (ids, corners)."""
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, _ = detector.detectMarkers(image)
    ids = ids.flatten() if ids is not None else []
    return ids, corners


def get_marker_center(corners):
    """Get pixel centre of marker from 4 corners."""
    return corners.mean(axis=0)


def get_marker_corners_world(pos, marker_size):
    """Get world coords of 4 corners (TL, TR, BR, BL in OpenCV order).

    Marker placed flat: top (+x forward), left (+y left).
    Corners in marker frame: (centre +/- M/2).
    """
    x, y = pos
    d = marker_size / 2
    # OpenCV order: TL, TR, BR, BL
    return np.array([
        [x + d, y + d],  # TL: +x (forward), +y (left)
        [x + d, y - d],  # TR: +x (forward), -y (right)
        [x - d, y - d],  # BR: -x (back), -y (right)
        [x - d, y + d],  # BL: -x (back), +y (left)
    ], dtype=np.float32)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--image', type=str, help='Path to still image')
    group.add_argument('--camera', type=int, help='Camera device index')
    parser.add_argument('--markers', type=str, required=True,
                        help='YAML file with marker positions')
    parser.add_argument('--output', type=str, default='scripts/data/homography_aruco.yaml',
                        help='Output YAML path (default: scripts/data/homography_aruco.yaml)')
    parser.add_argument('--frame-origin-x', type=float, default=0.0,
                        help='Frame origin x offset (default: 0.0)')
    parser.add_argument('--frame-origin-y', type=float, default=0.0,
                        help='Frame origin y offset (default: 0.0)')
    parser.add_argument('--marker-size', type=float,
                        help='Marker side length (metres); uses 4 corners if set')
    parser.add_argument('--show', action='store_true', help='Display detection overlay')
    args = parser.parse_args()

    # Load marker config
    dict_name, marker_pos = load_marker_positions(args.markers)
    required_ids = set(marker_pos.keys())
    aruco_dict = get_aruco_dict(dict_name)

    # Get image
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            print(f"Error: Could not read {args.image}")
            sys.exit(1)
    else:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"Error: Could not open camera {args.camera}")
            sys.exit(1)
        while True:
            ret, image = cap.read()
            if not ret:
                print("Error: Could not read from camera")
                sys.exit(1)
            cv2.imshow("Camera (SPACE=capture, ESC=quit)", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                cv2.destroyAllWindows()
                break
            elif key == 27:
                cap.release()
                cv2.destroyAllWindows()
                sys.exit(0)
        cap.release()

    # Detect markers
    detected_ids, corners = detect_markers(image, aruco_dict)
    detected_set = set(detected_ids)
    missing = required_ids - detected_set
    if missing:
        print(f"Error: Missing marker IDs: {sorted(missing)}")
        sys.exit(1)

    # Build correspondence points
    img_pts = []
    world_pts = []
    for mid, corner_list in zip(detected_ids, corners):
        if mid not in required_ids:
            continue
        if args.marker_size:
            # Use all 4 corners
            world_corners = get_marker_corners_world(marker_pos[mid], args.marker_size)
            for i, corner in enumerate(corner_list[0]):
                img_pts.append(corner)
                world_pts.append(world_corners[i])
        else:
            # Use centre only
            img_pts.append(get_marker_center(corner_list[0]))
            world_pts.append(marker_pos[mid])

    img_pts = np.array(img_pts, dtype=np.float32)
    world_pts = np.array(world_pts, dtype=np.float32)

    # Compute homography
    if len(img_pts) == 4:
        H, _ = cv2.findHomography(img_pts, world_pts, 0)
    else:
        H, _ = cv2.findHomography(img_pts, world_pts, cv2.RANSAC, 0.05)

    # Normalize H[2,2] = 1
    H = H / H[2, 2]

    # Compute reprojection error
    ones = np.ones((len(img_pts), 1), dtype=np.float32)
    img_homo = np.hstack([img_pts, ones])
    reproj_homo = (H @ img_homo.T).T
    reproj_world = reproj_homo[:, :2] / reproj_homo[:, 2:3]
    errors = np.linalg.norm(reproj_world - world_pts, axis=1)
    print(f"Reprojection errors (m): {errors}")
    print(f"Mean error: {errors.mean():.6f} m")

    # Display overlay
    if args.show:
        overlay = image.copy()
        for mid, corner_list in zip(detected_ids, corners):
            if mid in required_ids:
                pts = corner_list[0].astype(int)
                cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)
                center = pts.mean(axis=0).astype(int)
                cv2.circle(overlay, tuple(center), 5, (0, 0, 255), -1)
        cv2.imshow("Detected markers", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Write output YAML with full precision
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    h_matrix = [[float(f'{H[i, j]:.10e}') for j in range(3)] for i in range(3)]

    output_data = {
        'homography_matrix': h_matrix,
        'units_to_metres': 1.0,
        'swap_xy': False,
        'frame_origin': {
            'x': args.frame_origin_x,
            'y': args.frame_origin_y,
        }
    }

    with open(output_path, 'w') as f:
        f.write("# Homography from ArUco ground markers (scripts/aruco_homography.py)\n")
        f.write("# Maps image pixel (u,v) -> ground plane (x forward, y left).\n")
        yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

    print(f"Homography saved to {output_path}")


if __name__ == '__main__':
    main()
