# Homographic Monocular Camera Depth Perception

Estimates ground position and distance of Formula Student cones from monocular camera bounding boxes using a pre-calibrated ground-plane homography matrix. The 5-node ROS pipeline projects 2D cone detections to a bird's-eye-view ground frame in real time, enabling distance-based perception for autonomous vehicle control.

## Overview

This ROS Noetic package estimates the ground-plane position of cones from a monocular camera by leveraging a homography matrix. Rather than relying on stereo disparity or LiDAR, it assumes the camera is rigidly mounted above a flat ground plane and pre-calibrated via offline stereo/LiDAR fitting or online ArUco markers.

**Pipeline at a glance:**
1. Cone bounding boxes arrive from a perception module (e.g. YOLOv8)
2. Extract cone base points from image coordinates
3. Load or compute ground-plane homography matrix (with optional IMU tilt correction)
4. Project 2D base pixels to 3D ground coordinates (forward/left, metres)
5. Visualize in bird's-eye-view and optionally record video

## System Architecture

```
/cone_detections
        |
        v
[cone_base_detector] → /cone_bases (FrameData)
                           |
                           v
/camera/imu → [h_matrix_publisher] → /homography_matrix (HMatrix, latched)
                           |
                           v
                    [cone_ground_projector] → /projected_cones (FrameBevData)
                           |
         +-------+---------+--------+
         |       |         |        |
         v       v         v        v
[bev_cone_    [standalone_  [track_map_     (optional)
 visualizer]   cone_        visualizer]
             visualizer]    /odom
                          /camera/image/
                          compressed
```

## Requirements

- **Ubuntu 20.04** with ROS Noetic
- **OpenCV 4** with contrib modules (cv2.aruco for calibration)
- **yaml-cpp** development libraries
- **ROS packages**: `cv_bridge`, `foxglove_msgs`, `rosbag`
- **Python 3** with OpenCV contrib and PyYAML (for the calibration scripts only)

Install dependencies:
```bash
sudo apt install ros-noetic-foxglove-msgs ros-noetic-cv-bridge libyaml-cpp-dev
pip3 install opencv-contrib-python pyyaml
```

## Build

Clone into your catkin workspace, source ROS, and build:
```bash
cd ~/catkin_ws/src
git clone https://github.com/DakshVakharia/Monocular-Depth-perception-for-Cones.git homographic_mono_cam_depth_perception
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps homographic_mono_cam_depth_perception
source devel/setup.bash
```

## Input Topics

| Topic | Type | Used by | Description |
|---|---|---|---|
| `/cone_detections` | `foxglove_msgs/ImageMarkerArray` | `cone_base_detector` | Bounding boxes around detected cones (colour, x, y, width, height) |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `standalone_cone_visualizer` | Rectified left camera image (optional, only if showing image overlay) |
| `/camera/imu` | `sensor_msgs/Imu` | `h_matrix_publisher` | IMU quaternion; used only if `tilt_correction:=true` |
| `/odom` | `nav_msgs/Odometry` | `track_map_visualizer` | Vehicle odometry; used only if `show_track_map:=true` |

## Output & Internal Topics

| Topic | Type | Description |
|---|---|---|
| `/cone_bases` | `homographic_mono_cam_depth_perception/FrameData` | **Fields**: `float64 timestamp`, `ConeData[] cones` where each `ConeData` contains `string cone_id`, `uint8 color` (ORANGE=1, BLUE=2, YELLOW=3, GREEN=4), `float32 x` (image u-pixel), `float32 y` (image v-pixel). Published by `cone_base_detector` on each bounding box frame. |
| `/homography_matrix` | `homographic_mono_cam_depth_perception/HMatrix` | **Fields**: `float64[9] data` (row-major 3×3 matrix, image pixel → ground plane). **Latched**: late subscribers still receive the most recent value. Published by `h_matrix_publisher` on startup and whenever IMU tilt exceeds `change_threshold_deg`. |
| `/projected_cones` | `homographic_mono_cam_depth_perception/FrameBevData` | **Fields**: `float64 timestamp`, `ConeBevData[] cones` where each `ConeBevData` contains `string cone_id`, `uint8 color`, `float32 x`/`y` (image), `float32 ground_x`/`ground_y` (metres, forward/left in camera frame), `float32 distance` (Euclidean distance). Published by `cone_ground_projector` on each detector frame. |

## Nodes

| Node | Executable | Subscribes | Publishes | Key Parameters |
|---|---|---|---|---|
| **cone_base_detector** | `cone_base_detector_node` | `/cone_detections` | `/cone_bases` | `y_offset_pixels` (default: 5.0 px, shift base detection downward) |
| **h_matrix_publisher** | `h_matrix_publisher_node` | `/camera/imu` (if `enable_tilt_correction=true`) | `/homography_matrix` (latched) | `h_matrix_path`, `enable_tilt_correction` (default: false), `fx`/`fy`/`cx`/`cy` (required if tilt correction is on), `change_threshold_deg` (default: 0.2 deg) |
| **cone_ground_projector** | `cone_ground_projector_node` | `/cone_bases`, `/homography_matrix` | `/projected_cones` | `max_distance` (default: 40.0 m, cones beyond this are dropped) |
| **bev_cone_visualizer** | `bev_cone_visualizer_node` | `/projected_cones` | (visualization window & optional MP4 via OpenCV) | `canvas_width`/`height` (default: 800×800 px), `pixels_per_meter` (default: 40 px/m), `record_path` |
| **standalone_cone_visualizer** | `standalone_cone_visualizer_node` | `/camera/image/compressed`, `/cone_detections`, `/projected_cones` | (visualization window & optional MP4) | `image_width`/`height` (default: 960×540), `bev_size` (default: 540), `record_path`, `record_seconds` (default: 30 s) |
| **track_map_visualizer** | `track_map_visualizer_node` | `/projected_cones`, `/odom` | (visualization window) | `canvas_width`/`height` (default: 1200×1200), `merge_radius` (default: 1.5 m), `min_range`/`max_range` (default: 1.0–12.0 m), `min_hits` (default: 3 detections) |

## Launch

Two main entry points are provided, one per calibration method. Both need a homography YAML that you generate for your own camera mount first; no pre-fitted matrix is shipped.

| Method | Needs | Generate H with | Launch file |
|---|---|---|---|
| Stereo/LiDAR fit (validated) | A rosbag with 2D cone boxes **and** 3D cone positions from a stereo camera or LiDAR | `extract_cone_correspondences.py` + `fit_homography.py` ([how](#calibration-stereolidar-homography)) | `stereo_lidar_homography.launch` |
| ArUco markers (proof-of-concept) | 4 printed ArUco markers and one photo from the mounted camera | `aruco_homography.py` ([how](#calibration-aruco-homography)) | `aruco_homography.launch` |

### 1. **stereo_lidar_homography.launch** (validated)
Uses a homography matrix fitted offline from stereo/LiDAR cone positions matched to their pixel positions in your own recordings. Most accurate option. Generate the YAML first with [Calibration: Stereo/LiDAR Homography](#calibration-stereolidar-homography).

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch homographic_mono_cam_depth_perception stereo_lidar_homography.launch \
  bag_file:=path/to/rosbag.bag \
  bag_start:=20
```

Loads homography from `scripts/data/homography_stereo_lidar.yaml` by default; pass `h_yaml:=path/to/other.yaml` to use a different file.

### 2. **aruco_homography.launch** (proof-of-concept)
Uses a homography matrix calibrated from 4 ArUco markers placed on the ground. **Not yet field-tested**; useful for quick offline calibration.

**Before launching**, generate the homography YAML using the ArUco script:
```bash
source ~/catkin_ws/devel/setup.bash
python3 scripts/aruco_homography.py \
  --image calibration_image.jpg \
  --markers scripts/data/aruco_markers_example.yaml \
  --marker-size 0.3 \
  --output scripts/data/homography_aruco.yaml \
  --show
```

Then launch:
```bash
roslaunch homographic_mono_cam_depth_perception aruco_homography.launch \
  play_bag:=false
```

Loads homography from `scripts/data/homography_aruco.yaml`.

### Shared Launch Arguments

Both launch files accept these arguments:

| Argument | Default | Description |
|---|---|---|
| `play_bag` | `true` | Play a recorded rosbag; set to `false` for live camera |
| `bag_file` | `path/to/your.bag` | Path to rosbag file (used if `play_bag:=true`) |
| `bag_start` | `0` | Start playback at this timestamp (seconds) |
| `bag_rate` | `1.0` | Playback rate (1.0 = real-time, 2.0 = 2× speed) |
| `tilt_correction` | `false` | Enable IMU-based camera tilt correction (requires an IMU and `fx`/`fy`/`cx`/`cy`) |
| `fx`, `fy`, `cx`, `cy` | `0` | Camera intrinsics from your own calibration; required when `tilt_correction:=true` |
| `image_topic` | `/camera/image/compressed` | Camera image (compressed) |
| `imu_topic` | `/camera/imu` | Camera IMU |
| `bbox_topic` | `/cone_detections` | Cone bounding boxes (`foxglove_msgs/ImageMarkerArray`) |
| `odom_topic` | `/odom` | Vehicle odometry for the track map |
| `show_image_overlay` | `true` | Enable side-by-side camera + BEV visualizer window |
| `show_track_map` | `true` | Enable global track-map visualizer window (integrates odometry) |
| `record_bev` | `""` | Path to save BEV video (MP4); empty = no recording |
| `record_overlay` | `""` | Path to save image overlay video (MP4); empty = no recording |
| `record_seconds` | `30` | Duration of image overlay recording (seconds) |

### Example: Live camera with recording

```bash
roslaunch homographic_mono_cam_depth_perception stereo_lidar_homography.launch \
  play_bag:=false \
  record_overlay:=/tmp/demo.mp4 \
  record_seconds:=40
```

### Example: Playback a rosbag, skip first 20 seconds

```bash
roslaunch homographic_mono_cam_depth_perception stereo_lidar_homography.launch \
  bag_file:=~/rosbags/test.bag \
  bag_start:=20 \
  record_bev:=/tmp/bev.mp4
```

## Calibration: Stereo/LiDAR Homography

Fits H from many real cone detections: each cone's pixel position (from your 2D detector) is paired with its ground position measured by a stereo camera or LiDAR, then H is fitted with RANSAC. More correspondences across the whole image give a more accurate H than 4 markers.

### Requirements

A rosbag recorded with the camera in its final mounting position, containing:

| Topic | Type | Content |
|---|---|---|
| 2D cone boxes | `foxglove_msgs/ImageMarkerArray` | One marker per cone in the image |
| 3D cone positions | any message with a `points` list of x/y/z (e.g. `sensor_msgs/PointCloud`) | One point per cone, metres, **x forward, y left**, in the camera's ground frame |

The 3D points and 2D boxes of a frame must be in the **same order** (true when both come from the same detector). Frames where the counts differ are skipped.

### Calibration Workflow

1. Extract matched cone pairs from the bag:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   python3 scripts/extract_cone_correspondences.py \
     --bag path/to/calibration.bag \
     --bbox-topic /your/cone_bounding_boxes \
     --cones-3d-topic /your/cone_positions_3d \
     --output cone_correspondences.json
   ```
   - `--tolerance`: max time between a 2D and 3D message to pair them (default 0.05 s)
   - `--samples`: randomly keep at most this many pairs; `0` keeps all (default 500)

2. Fit the homography:
   ```bash
   python3 scripts/fit_homography.py \
     --input cone_correspondences.json \
     --output scripts/data/homography_stereo_lidar.yaml
   ```
   - `--max-distance`: ignore cones further than this (default 15 m; far cones are less accurate)
   - `--ransac-threshold`: RANSAC inlier threshold in metres (default 5.0)

   The script prints the inlier count and mean reprojection error, and writes a YAML in the [format the launch files expect](#homography-matrix-format).

3. Launch:
   ```bash
   roslaunch homographic_mono_cam_depth_perception stereo_lidar_homography.launch \
     bag_file:=path/to/test.bag
   ```

Tip: fit on one recording and check accuracy on a different one, so you measure how well H generalises rather than how well it fits.

## Calibration: ArUco Homography

The ArUco calibration script automatically computes homography from 4 ground markers.

### Setup

1. Print or display 4 ArUco markers (dictionary: `DICT_4X4_50`, IDs 0–3)
2. Place them flat on the ground, evenly spaced, with:
   - Top edge (+X) pointing forward (away from car)
   - Left edge (+Y) pointing left (towards driver's side)
3. Record their **ground positions** in a YAML file:

```yaml
# scripts/data/aruco_markers_example.yaml
dictionary: DICT_4X4_50
markers:
  0: [3.0,  1.5]   # ID 0 at (x=3.0 m forward, y=1.5 m left)
  1: [3.0, -1.5]
  2: [6.0,  1.5]
  3: [6.0, -1.5]
```

### Calibration Workflow

1. Take a photo of the markers from your mounted camera:
   ```bash
   # If you have OpenCV GUI available, or save from live camera
   python3 -c "
   import cv2
   cap = cv2.VideoCapture(0)
   ret, frame = cap.read()
   cv2.imwrite('calib_image.jpg', frame)
   cap.release()
   "
   ```

2. Run the calibration script:
   ```bash
   python3 scripts/aruco_homography.py \
     --image calib_image.jpg \
     --markers scripts/data/aruco_markers_example.yaml \
     --marker-size 0.3 \
     --show
   ```
   
   - `--marker-size`: Side length of each marker in metres (0.3 m = 30 cm typical)
   - `--show`: Display detection overlay
   - Output: `scripts/data/homography_aruco.yaml` (default)

3. (Optional) If the camera is offset from the car origin, add frame offset:
   ```bash
   python3 scripts/aruco_homography.py \
     --image calib_image.jpg \
     --markers scripts/data/aruco_markers_example.yaml \
     --marker-size 0.3 \
     --frame-origin-x 0.2 \
     --frame-origin-y -0.1 \
     --output scripts/data/homography_aruco.yaml
   ```

4. Launch with the generated homography:
   ```bash
   roslaunch homographic_mono_cam_depth_perception aruco_homography.launch
   ```

## Homography Matrix Format

Store homography matrices as YAML files in `scripts/data/`:

```yaml
# 3×3 homography matrix (maps image pixel [u, v, 1] → ground [x, y, 1])
homography_matrix:
  - [H00, H01, H02]
  - [H10, H11, H12]
  - [H20, H21, H22]

# Scale factor to convert H output to metres
# Use 1.0 if H is already in metres; 0.001 if output is in millimetres
units_to_metres: 1.0

# Coordinate convention of H output
# false: row 0 = forward (x), row 1 = lateral (y)
# true:  row 0 = lateral (y), row 1 = forward (x) — use if H was fitted with swapped axes
swap_xy: false

# Camera-to-vehicle frame offset (only used for track map overlay)
frame_origin:
  x: 0.0    # Forward offset from camera to car origin (metres)
  y: 0.0    # Lateral offset from camera to car origin (metres)
```

Example (stereo/LiDAR fitted):
```yaml
homography_matrix:
  - [ -2.0349399304e-05, 1.6377215420e-03, -3.8116535982e+00 ]
  - [ 6.2113136240e-03, -3.8722229214e-04, -3.9330251415e+00 ]
  - [ -7.5059054419e-06, -5.1168202770e-03, 1.0000000000e+00 ]
units_to_metres: 1.0
swap_xy: false
frame_origin:
  x: 0.0
  y: 0.0
```

## Tilt Correction

When `tilt_correction:=true`, the `h_matrix_publisher` compensates for camera roll and pitch relative to startup orientation using the camera's IMU. You must pass your camera's intrinsics:

```bash
roslaunch homographic_mono_cam_depth_perception stereo_lidar_homography.launch \
  tilt_correction:=true fx:=<fx> fy:=<fy> cx:=<cx> cy:=<cy>
```

**Formula:**
```
H_corrected = H_base · K · R_cam · K⁻¹
```

Where:
- **H_base**: The static homography matrix from YAML
- **K**: Camera intrinsic matrix built from the `fx`, `fy`, `cx`, `cy` parameters (no defaults; take these from your camera's `camera_info`)
- **R_cam**: 3×3 rotation matrix from IMU quaternion (relative to home orientation)
- **K⁻¹**: Inverse of intrinsics

**How it works:**
1. On first IMU message, the node records the vehicle's baseline roll and pitch
2. Subsequent IMU messages compute delta roll (Δφ) and delta pitch (Δθ)
3. If delta exceeds `change_threshold_deg` (default: 0.2°), compute R_cam and republish corrected H
4. This allows accurate projection even when the vehicle tilts on bumps or slopes

**Disable tilt correction** for flat-ground-only scenarios or if IMU is not available:
```bash
roslaunch ... tilt_correction:=false
```

## Repository Layout

```
src/
├── detection/               # Cone base detector node
│   └── cone_base_detector.cpp
├── projection/              # Homography projection & H matrix publisher
│   ├── h_matrix_publisher.cpp
│   └── cone_ground_projector.cpp
└── visualization/           # BEV, image overlay, and track map visualizers
    ├── bev_cone_visualizer.cpp
    ├── standalone_cone_visualizer.cpp
    └── track_map_visualizer.cpp

include/
├── cone_base_detector.h
├── cone_ground_projector.h
├── h_matrix_publisher.h
├── bev_cone_visualizer.h
├── standalone_cone_visualizer.h
└── track_map_visualizer.h

msg/
├── ConeData.msg             # Individual cone (id, color, image x/y)
├── ConeBevData.msg          # Cone with ground projection (x, y, distance)
├── FrameData.msg            # Frame of FrameData (timestamp + cones)
├── FrameBevData.msg         # Frame of ConeBevData (timestamp + cones)
└── HMatrix.msg              # 3×3 homography matrix

scripts/
├── extract_cone_correspondences.py  # Stereo/LiDAR calibration step 1: bag → matched cone pairs
├── fit_homography.py                # Stereo/LiDAR calibration step 2: cone pairs → H (YAML)
├── aruco_homography.py              # ArUco calibration (4 ground markers → H)
└── data/
    ├── homography_stereo_lidar.yaml    # Your stereo/LiDAR-fitted H (not shipped, create it)
    ├── homography_aruco.yaml           # Generated by the ArUco script (not shipped)
    └── aruco_markers_example.yaml      # Example marker positions for calibration

launch/
├── stereo_lidar_homography.launch   # Validated entry point (pre-fitted H)
├── aruco_homography.launch          # Proof-of-concept (4 markers)
└── pipeline.launch                  # Shared node configuration (included by above)

CMakeLists.txt
package.xml
README.md
```

## Limitations & Future Work

- **Flat-ground assumption:** Homography assumes a planar ground and breaks on slopes, bumps, or ramps
- **Static calibration:** H is pre-fitted offline; dynamic recalibration during runtime not yet supported
- **Error growth:** Projection error increases with distance and camera tilt; tilt correction mitigates but does not eliminate
- **ArUco path not field-tested:** Stereo/LiDAR fitting is validated; ArUco is a convenient offline calibration tool but lacks real-world validation
- **No unit tests:** Pipeline not yet covered by automated test suite; manual validation on rosbags only

## Author

**Daksh Vakharia** · daksh.vakharia@gmail.com

---

**License:** MIT
