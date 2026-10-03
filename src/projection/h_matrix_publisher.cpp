#include "h_matrix_publisher.h"
#include <yaml-cpp/yaml.h>
#include <cmath>

HMatrixPublisher::HMatrixPublisher(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private),
      home_set_(false), home_roll_(0.0), home_pitch_(0.0),
      last_dr_(0.0), last_dp_(0.0) {

  // --- Parameters ---
  std::string yaml_path;
  nh_private_.param<std::string>("h_matrix_path", yaml_path, "");
  nh_private_.param<std::string>("imu_topic",     imu_topic_,     "/camera/imu");
  nh_private_.param<std::string>("publish_topic", publish_topic_, "/homography_matrix");
  nh_private_.param<double>("change_threshold_deg", threshold_deg_, 0.2);
  nh_private_.param<bool>("enable_tilt_correction", tilt_correction_, true);

  // --- Camera intrinsics (used for tilt correction) ---
  // No defaults: these must come from your own camera's calibration.
  double fx, fy, cx, cy;
  nh_private_.param<double>("fx", fx, 0.0);
  nh_private_.param<double>("fy", fy, 0.0);
  nh_private_.param<double>("cx", cx, 0.0);
  nh_private_.param<double>("cy", cy, 0.0);

  if (tilt_correction_ && (fx <= 0.0 || fy <= 0.0 || cx <= 0.0 || cy <= 0.0)) {
    ROS_FATAL("Tilt correction needs camera intrinsics. Set ~fx, ~fy, ~cx, ~cy "
              "or set ~enable_tilt_correction to false.");
    ros::shutdown();
    return;
  }
  if (!tilt_correction_) {
    fx = fy = 1.0;  // K is unused without tilt correction; keep it invertible
  }

  K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  K_inv_ = K_.inv();

  // --- Load base H from YAML ---
  if (yaml_path.empty() || !loadYaml(yaml_path)) {
    ROS_FATAL("H matrix YAML path not set or failed to load. Set ~h_matrix_path.");
    ros::shutdown();
    return;
  }

  // --- Publisher (latched so late subscribers still get the latest H) ---
  hmatrix_pub_ = nh_.advertise<homographic_mono_cam_depth_perception::HMatrix>(
      publish_topic_, 1, /*latch=*/true);

  // Publish base H immediately so the rest of the pipeline starts working
  publish(H_base_);
  ROS_INFO("H matrix publisher ready. Base H published to %s", publish_topic_.c_str());
  ROS_INFO("  IMU topic     : %s", imu_topic_.c_str());
  ROS_INFO("  Tilt threshold: %.2f deg", threshold_deg_);

  if (tilt_correction_)
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &HMatrixPublisher::imuCallback, this);
  else
    ROS_INFO("  Tilt correction disabled: publishing static H only");
}

HMatrixPublisher::~HMatrixPublisher() {}

bool HMatrixPublisher::loadYaml(const std::string &path) {
  try {
    YAML::Node config = YAML::LoadFile(path);
    auto rows = config["homography_matrix"];
    H_base_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        H_base_.at<double>(i, j) = rows[i][j].as<double>();
    ROS_INFO("Loaded H base from %s", path.c_str());
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to load YAML: %s", e.what());
    return false;
  }
}

void HMatrixPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // Convert quaternion to roll and pitch (degrees)
  const auto &q = msg->orientation;
  double sinr = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  double roll  = std::atan2(sinr, cosr) * 180.0 / M_PI;

  double sinp  = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp))) * 180.0 / M_PI;

  // Record home orientation on the first valid reading
  if (!home_set_) {
    home_roll_  = roll;
    home_pitch_ = pitch;
    home_set_   = true;
    ROS_INFO("Home orientation set: roll=%.3f deg  pitch=%.3f deg", roll, pitch);
    return;
  }

  double dr = roll  - home_roll_;
  double dp = pitch - home_pitch_;

  // Skip if change is below threshold
  if (std::max(std::abs(dr - last_dr_), std::abs(dp - last_dp_)) < threshold_deg_)
    return;

  last_dr_ = dr;
  last_dp_ = dp;

  computeAndPublish(dr, dp);
}

void HMatrixPublisher::computeAndPublish(double delta_roll_deg, double delta_pitch_deg) {
  double dr = delta_roll_deg  * M_PI / 180.0;
  double dp = delta_pitch_deg * M_PI / 180.0;

  // Roll/pitch are IMU body-frame angles (x fwd, y left, z up), with the home
  // attitude as reference: R_delta = R_tilt(home)^T * R_tilt(current).
  auto tilt = [](double r, double p) {
    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
      1, 0, 0,
      0, std::cos(r), -std::sin(r),
      0, std::sin(r),  std::cos(r));
    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
       std::cos(p), 0, std::sin(p),
       0,           1, 0,
      -std::sin(p), 0, std::cos(p));
    return cv::Mat(Ry * Rx);
  };
  double hr = home_roll_ * M_PI / 180.0;
  double hp = home_pitch_ * M_PI / 180.0;
  cv::Mat R_body = tilt(hr, hp).t() * tilt(hr + dr, hp + dp);

  // Body -> camera optical frame (x right, y down, z forward)
  cv::Mat M = (cv::Mat_<double>(3, 3) <<
    0, -1,  0,
    0,  0, -1,
    1,  0,  0);
  cv::Mat R_cam = M * R_body * M.t();

  // A pixel seen by the tilted camera maps back to the calibration pose via
  // p_base = K * R_cam * K^-1 * p_now, so H_new = H_base * K * R_cam * K^-1
  cv::Mat H_new = H_base_ * K_ * R_cam * K_inv_;
  H_new /= H_new.at<double>(2, 2);

  publish(H_new);

  ROS_INFO_THROTTLE(1.0, "H updated: delta_roll=%.3f deg  delta_pitch=%.3f deg",
                    delta_roll_deg, delta_pitch_deg);
}

void HMatrixPublisher::publish(const cv::Mat &H) {
  homographic_mono_cam_depth_perception::HMatrix msg;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      msg.data[i * 3 + j] = H.at<double>(i, j);
  hmatrix_pub_.publish(msg);
}

void HMatrixPublisher::run() { ros::spin(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "h_matrix_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  HMatrixPublisher publisher(nh, nh_private);
  publisher.run();

  return 0;
}
