#!/usr/bin/env python3
"""
Extract matched (image pixel, 3D ground position) cone pairs from a rosbag.

Reads 2D cone bounding boxes (foxglove_msgs/ImageMarkerArray) and 3D cone
positions from a stereo or LiDAR pipeline (any message with a `points` list
of x/y/z, e.g. sensor_msgs/PointCloud), synchronises them by bag time and
pairs each box with its 3D point. The output JSON is the input for
fit_homography.py.

Assumes the 3D points and 2D boxes in a frame are published in the same
order (true when both come from the same detector). Frames with differing
counts are skipped.
"""

import argparse
import json
import math
import os
import random

import rosbag


def cone_base_pixel(marker):
    """Bottom-centre of a bounding box, in image pixels."""
    if marker.position.x == 0.0 and marker.position.y == 0.0 and len(marker.points) > 0:
        # Polygon/rectangle: the base is at the maximum Y coordinate
        avg_x = sum(p.x for p in marker.points) / len(marker.points)
        max_y = max(p.y for p in marker.points)
        return [avg_x, max_y]
    # Position is the centre: add half the scale to reach the bottom (Y grows downwards)
    return [marker.position.x, marker.position.y + (marker.scale / 2.0)]


def read_bag(bag_path, bbox_topic, cones_3d_topic):
    cones_3d_buffer = {}  # timestamp -> points
    cones_2d_buffer = {}  # timestamp -> markers

    print(f"Opening bag: {bag_path}")
    with rosbag.Bag(bag_path) as bag:
        count = 0
        for topic, msg, t in bag.read_messages(topics=[cones_3d_topic, bbox_topic]):
            count += 1
            if count % 1000 == 0:
                print(f"Processed {count} messages...")

            # Bag receive time is the synchronisation key
            timestamp = t.to_sec()
            if topic == cones_3d_topic:
                cones_3d_buffer[timestamp] = msg.points
            elif hasattr(msg, 'markers'):
                cones_2d_buffer[timestamp] = msg.markers
            else:
                print(f"Warning: message on {topic} has no 'markers' attribute")

    print(f"Read {len(cones_3d_buffer)} 3D messages and {len(cones_2d_buffer)} 2D messages.")
    return cones_3d_buffer, cones_2d_buffer


def match_frames(cones_3d_buffer, cones_2d_buffer, tolerance):
    ts_3d = sorted(cones_3d_buffer.keys())
    ts_2d = sorted(cones_2d_buffer.keys())
    i_3d = i_2d = 0
    matched = []
    while i_3d < len(ts_3d) and i_2d < len(ts_2d):
        t3, t2 = ts_3d[i_3d], ts_2d[i_2d]
        if abs(t3 - t2) < tolerance:
            matched.append((t3, t2))
            i_3d += 1
            i_2d += 1
        elif t3 < t2:
            i_3d += 1
        else:
            i_2d += 1
    print(f"Found {len(matched)} synchronised frames with {tolerance * 1000:.0f} ms tolerance.")
    return matched


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bag', required=True, help='Path to the rosbag')
    parser.add_argument('--bbox-topic', required=True,
                        help='2D cone bounding boxes (foxglove_msgs/ImageMarkerArray)')
    parser.add_argument('--cones-3d-topic', required=True,
                        help='3D cone positions from stereo/LiDAR (message with a `points` list)')
    parser.add_argument('--output', default='cone_correspondences.json',
                        help='Output JSON path (default: %(default)s)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='Max time difference to pair frames, seconds (default: %(default)s)')
    parser.add_argument('--samples', type=int, default=500,
                        help='Randomly keep at most this many pairs; 0 keeps all (default: %(default)s)')
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        parser.error(f"bag file not found: {args.bag}")

    cones_3d_buffer, cones_2d_buffer = read_bag(args.bag, args.bbox_topic, args.cones_3d_topic)
    matched = match_frames(cones_3d_buffer, cones_2d_buffer, args.tolerance)

    all_pairs = []
    for t3, t2 in matched:
        points_3d = cones_3d_buffer[t3]
        markers_2d = cones_2d_buffer[t2]
        if len(points_3d) != len(markers_2d):
            continue  # counts differ, pairing would be unreliable
        for p3d, m2d in zip(points_3d, markers_2d):
            if math.isnan(p3d.x) or math.isnan(p3d.y) or math.isnan(p3d.z):
                continue
            all_pairs.append({
                "image_point": cone_base_pixel(m2d),
                "world_point": [p3d.x, p3d.y, p3d.z],
                "timestamp_3d": t3,
                "timestamp_2d": t2,
            })

    if not all_pairs:
        print("No cone pairs extracted. Check the topic names and that both are in the bag.")
        return

    print(f"Extracted {len(all_pairs)} total cone pairs.")
    if args.samples > 0 and len(all_pairs) > args.samples:
        all_pairs = random.sample(all_pairs, args.samples)

    out_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(all_pairs, f, indent=4)
    print(f"Saved {len(all_pairs)} pairs to {args.output}")


if __name__ == "__main__":
    main()
