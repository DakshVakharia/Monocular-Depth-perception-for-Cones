#include "track_map_visualizer.h"
#include <cmath>

TrackMapVisualizer::TrackMapVisualizer(ros::NodeHandle &nh,
                                       ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private),
      car_x_(0.0), car_y_(0.0), car_yaw_(0.0), pose_received_(false) {

  nh_private_.param<int>("canvas_width",      canvas_width_,        1200);
  nh_private_.param<int>("canvas_height",     canvas_height_,       1200);
  nh_private_.param<float>("pixels_per_meter", pixels_per_meter_,    20.0f);
  nh_private_.param<float>("merge_radius",     merge_radius_,        1.5f);
  nh_private_.param<float>("max_range",        max_range_,           12.0f);
  nh_private_.param<float>("min_range",        min_range_,           1.0f);
  nh_private_.param<int>("min_hits",           min_hits_,            3);
  nh_private_.param<float>("camera_offset_x",  camera_offset_x_,     0.0f);
  nh_private_.param<float>("camera_offset_y",  camera_offset_y_,     0.0f);
  nh_private_.param<std::string>("cone_topic", cone_topic_, "/projected_cones");
  nh_private_.param<std::string>("odom_topic", odom_topic_, "/odom");

  cone_sub_ = nh_.subscribe(cone_topic_, 10,
                             &TrackMapVisualizer::coneCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 10,
                             &TrackMapVisualizer::odomCallback, this);

  ROS_INFO("Track Map Visualizer initialized");
  ROS_INFO("  Cones  : %s", cone_topic_.c_str());
  ROS_INFO("  Odom   : %s", odom_topic_.c_str());
  ROS_INFO("  Scale  : %.1f px/m | Merge radius: %.2f m",
           pixels_per_meter_, merge_radius_);
  ROS_INFO("  Range  : %.1f - %.1f m | Min hits: %d | Camera offset: (%.2f, %.2f) m",
           min_range_, max_range_, min_hits_, camera_offset_x_, camera_offset_y_);
}

TrackMapVisualizer::~TrackMapVisualizer() {
  cv::destroyWindow("Track Map");
}

// ── callbacks ────────────────────────────────────────────────────────────────

void TrackMapVisualizer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  car_x_ = msg->pose.pose.position.x;
  car_y_ = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  const auto &q = msg->pose.pose.orientation;
  car_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  car_path_.emplace_back(static_cast<float>(car_x_),
                         static_cast<float>(car_y_));
  pose_received_ = true;

  // Store pose in buffer (keep last 200)
  odom_buffer_.push_back({msg->header.stamp.toSec(), car_x_, car_y_, car_yaw_});
  if (odom_buffer_.size() > 200)
    odom_buffer_.pop_front();
}

void TrackMapVisualizer::coneCallback(
    const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg) {

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!pose_received_ || odom_buffer_.empty())
    return;

  // Find pose closest to message timestamp
  double target_stamp = msg->timestamp;
  OdomPose best_pose = odom_buffer_.front();
  double min_time_diff = std::abs(best_pose.stamp - target_stamp);

  // Find two neighbouring poses for potential interpolation
  const OdomPose *pose_before = nullptr;
  const OdomPose *pose_after = nullptr;

  for (size_t i = 0; i < odom_buffer_.size(); ++i) {
    double diff = std::abs(odom_buffer_[i].stamp - target_stamp);
    if (diff < min_time_diff) {
      min_time_diff = diff;
      best_pose = odom_buffer_[i];
    }
    if (odom_buffer_[i].stamp <= target_stamp)
      pose_before = &odom_buffer_[i];
    if (odom_buffer_[i].stamp >= target_stamp && pose_after == nullptr)
      pose_after = &odom_buffer_[i];
  }

  // Skip if best pose is too far in time
  if (min_time_diff > 0.2)
    return;

  // Interpolate if we have both before and after poses
  double use_x = best_pose.x;
  double use_y = best_pose.y;
  double use_yaw = best_pose.yaw;

  if (pose_before && pose_after && pose_before != pose_after) {
    double time_diff = pose_after->stamp - pose_before->stamp;
    if (time_diff > 1e-6) {
      double alpha = (target_stamp - pose_before->stamp) / time_diff;
      alpha = std::max(0.0, std::min(1.0, alpha));
      use_x = pose_before->x + alpha * (pose_after->x - pose_before->x);
      use_y = pose_before->y + alpha * (pose_after->y - pose_before->y);
      // Interpolate yaw with wrap handling
      double yaw_diff = pose_after->yaw - pose_before->yaw;
      if (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
      if (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
      use_yaw = pose_before->yaw + alpha * yaw_diff;
    }
  }

  for (const auto &cone : msg->cones) {
    // Check range filters
    float dist = std::sqrt(cone.ground_x * cone.ground_x + cone.ground_y * cone.ground_y);
    if (dist > max_range_ || dist < min_range_)
      continue;

    // Apply camera offset in car frame, then rotate/translate to world frame
    float cx = cone.ground_x + camera_offset_x_;
    float cy = cone.ground_y + camera_offset_y_;

    float gx = static_cast<float>(use_x)
                + std::cos(use_yaw) * cx - std::sin(use_yaw) * cy;
    float gy = static_cast<float>(use_y)
                + std::sin(use_yaw) * cx + std::cos(use_yaw) * cy;

    tryAddCone(gx, gy, cone.color);
  }
}

// ── cone deduplication ────────────────────────────────────────────────────────

bool TrackMapVisualizer::tryAddCone(float x, float y, uint8_t color) {
  // Find nearest existing cone within merge_radius
  int nearest_idx = -1;
  float nearest_dist = merge_radius_;

  for (size_t i = 0; i < track_cones_.size(); ++i) {
    const auto &existing = track_cones_[i];
    // Match if same color or either is UNKNOWN (0)
    if (existing.color != 0 && color != 0 && existing.color != color)
      continue;

    float dx = existing.x - x;
    float dy = existing.y - y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_idx = static_cast<int>(i);
    }
  }

  if (nearest_idx >= 0) {
    // Update existing cone with weighted mean
    auto &existing = track_cones_[nearest_idx];
    float old_weight = static_cast<float>(existing.hits);
    float new_weight = 1.0f;
    float total_weight = old_weight + new_weight;

    existing.x = (existing.x * old_weight + x * new_weight) / total_weight;
    existing.y = (existing.y * old_weight + y * new_weight) / total_weight;
    existing.hits++;
    return true;
  }

  // Add new cone
  track_cones_.push_back({x, y, color, 1});
  return true;
}

// ── coordinate conversion ────────────────────────────────────────────────────

cv::Point2f TrackMapVisualizer::worldToCanvas(float wx, float wy) const {
  // World origin maps to canvas center.
  // X (forward in world) → up on canvas (canvas_y decreases).
  // Y (left in world)    → left on canvas (canvas_x decreases).
  float cx = canvas_width_  / 2.0f - wy * pixels_per_meter_;
  float cy = canvas_height_ / 2.0f - wx * pixels_per_meter_;
  return {cx, cy};
}

cv::Scalar TrackMapVisualizer::colorFromEnum(uint8_t color_enum) const {
  switch (color_enum) {
  case homographic_mono_cam_depth_perception::ConeBevData::ORANGE: return cv::Scalar(0, 165, 255);
  case homographic_mono_cam_depth_perception::ConeBevData::BLUE:   return cv::Scalar(255, 0,   0);
  case homographic_mono_cam_depth_perception::ConeBevData::YELLOW: return cv::Scalar(0, 255, 255);
  case homographic_mono_cam_depth_perception::ConeBevData::GREEN:  return cv::Scalar(0, 255,   0);
  default:                                    return cv::Scalar(128, 128, 128);
  }
}

// ── drawing ──────────────────────────────────────────────────────────────────

void TrackMapVisualizer::drawGrid(cv::Mat &canvas) const {
  cv::Point2f origin = worldToCanvas(0.0f, 0.0f);

  // Distance rings every 10 m from world origin
  for (int m = 10; m <= 200; m += 10) {
    float radius = m * pixels_per_meter_;
    cv::circle(canvas, origin, static_cast<int>(radius),
               cv::Scalar(40, 40, 40), 1);
  }

  // Axis lines through world origin
  cv::line(canvas, cv::Point(origin.x, 0),
           cv::Point(origin.x, canvas_height_), cv::Scalar(60, 60, 60), 1);
  cv::line(canvas, cv::Point(0, origin.y),
           cv::Point(canvas_width_, origin.y), cv::Scalar(60, 60, 60), 1);
}

void TrackMapVisualizer::drawPath(cv::Mat &canvas) const {
  if (car_path_.size() < 2)
    return;

  for (size_t i = 1; i < car_path_.size(); ++i) {
    cv::Point2f p1 = worldToCanvas(car_path_[i - 1].x, car_path_[i - 1].y);
    cv::Point2f p2 = worldToCanvas(car_path_[i].x,     car_path_[i].y);
    cv::line(canvas, p1, p2, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  }
}

void TrackMapVisualizer::drawCones(cv::Mat &canvas) const {
  for (const auto &cone : track_cones_) {
    cv::Point2f pos = worldToCanvas(cone.x, cone.y);

    if (pos.x < 0 || pos.x >= canvas_width_ ||
        pos.y < 0 || pos.y >= canvas_height_)
      continue;

    if (cone.hits >= min_hits_) {
      // Draw confirmed cone with radius 5
      cv::Scalar color = colorFromEnum(cone.color);
      cv::circle(canvas, pos, 5, color, -1, cv::LINE_AA);
      cv::circle(canvas, pos, 5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    } else {
      // Draw unconfirmed as tiny 1px grey dot
      cv::circle(canvas, pos, 1, cv::Scalar(64, 64, 64), -1);
    }
  }
}

void TrackMapVisualizer::drawCar(cv::Mat &canvas) const {
  if (!pose_received_)
    return;

  cv::Point2f pos = worldToCanvas(static_cast<float>(car_x_),
                                   static_cast<float>(car_y_));

  // Triangle pointing in the direction of car_yaw_
  // yaw=0 → forward = up on canvas
  float heading = static_cast<float>(-car_yaw_);  // canvas Y is flipped
  float size = 10.0f;

  std::vector<cv::Point> pts = {
    cv::Point(pos.x + size * std::sin(heading),
              pos.y - size * std::cos(heading)),
    cv::Point(pos.x + size * 0.5f * std::sin(heading + 2.5f),
              pos.y - size * 0.5f * std::cos(heading + 2.5f)),
    cv::Point(pos.x + size * 0.5f * std::sin(heading - 2.5f),
              pos.y - size * 0.5f * std::cos(heading - 2.5f))
  };

  cv::fillConvexPoly(canvas, pts, cv::Scalar(0, 0, 255));

  // Count confirmed vs total cones
  int confirmed = 0;
  for (const auto &cone : track_cones_) {
    if (cone.hits >= min_hits_)
      confirmed++;
  }

  // Stats overlay
  char info[128];
  snprintf(info, sizeof(info),
           "Confirmed: %d | Candidates: %zu | Pos: (%.1f, %.1f)",
           confirmed, track_cones_.size(), car_x_, car_y_);
  cv::putText(canvas, info, cv::Point(10, 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

// ── main loop ────────────────────────────────────────────────────────────────

void TrackMapVisualizer::run() {
  cv::namedWindow("Track Map", cv::WINDOW_AUTOSIZE);

  ros::Rate rate(30);
  while (ros::ok()) {
    cv::Mat canvas = cv::Mat::zeros(canvas_height_, canvas_width_, CV_8UC3);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      drawGrid(canvas);
      drawPath(canvas);
      drawCones(canvas);
      drawCar(canvas);
    }

    cv::imshow("Track Map", canvas);
    if (cv::waitKey(1) == 27)  // ESC to quit
      break;

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "track_map_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  TrackMapVisualizer visualizer(nh, nh_private);
  visualizer.run();

  return 0;
}
