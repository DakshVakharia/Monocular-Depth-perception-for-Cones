#include "cone_ground_projector.h"
#include <yaml-cpp/yaml.h>
#include <cmath>

ConeGroundProjector::ConeGroundProjector(ros::NodeHandle &nh,
                                         ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private),
      frame_origin_x_(0.0), frame_origin_y_(0.0),
      units_to_metres_(1.0), swap_xy_(false) {

  nh_private_.param<std::string>("hmatrix_topic", hmatrix_topic_,
                                 "/homography_matrix");
  nh_private_.param<std::string>("subscribe_topic", subscribe_topic_,
                                 "/cone_bases");
  nh_private_.param<std::string>("publish_topic", publish_topic_,
                                 "/projected_cones");
  nh_private_.param<std::string>("yaml_path", yaml_path_, "");
  nh_private_.param<double>("max_distance", max_distance_, 40.0);

  if (yaml_path_.empty()) {
    ROS_WARN("~yaml_path not set — assuming H outputs metres (x fwd, y left) "
             "and frame_origin (0, 0).");
  } else {
    loadFrameOrigin();
  }

  hmatrix_sub_ = nh_.subscribe(hmatrix_topic_, 1,
                               &ConeGroundProjector::hMatrixCallback, this);
  frame_data_sub_ = nh_.subscribe(
      subscribe_topic_, 10, &ConeGroundProjector::frameDataCallback, this);
  frame_bev_pub_ =
      nh_.advertise<homographic_mono_cam_depth_perception::FrameBevData>(publish_topic_, 10);

  ROS_INFO("Cone Ground Projector initialized");
  ROS_INFO("  Waiting for H matrix on: %s", hmatrix_topic_.c_str());
  ROS_INFO("  Subscribe: %s | Publish: %s", subscribe_topic_.c_str(),
           publish_topic_.c_str());
  ROS_INFO("  Frame origin (car frame): x=%.3f  y=%.3f",
           frame_origin_x_, frame_origin_y_);
}

ConeGroundProjector::~ConeGroundProjector() {
  ROS_INFO("Cone Ground Projector shutting down");
}

bool ConeGroundProjector::loadFrameOrigin() {
  try {
    YAML::Node config = YAML::LoadFile(yaml_path_);
    // Output convention differs per H source: the stereo/LiDAR and ArUco-script
    // H give metres with row 0 = forward; the legacy screenshot H gives mm with
    // rows swapped.
    if (config["units_to_metres"])
      units_to_metres_ = config["units_to_metres"].as<double>();
    if (config["swap_xy"])
      swap_xy_ = config["swap_xy"].as<bool>();
    if (config["frame_origin"]) {
      frame_origin_x_ = config["frame_origin"]["x"].as<double>();
      frame_origin_y_ = config["frame_origin"]["y"].as<double>();
    }
    ROS_INFO("Loaded %s: units_to_metres=%g swap_xy=%d frame_origin=(%.3f, %.3f)",
             yaml_path_.c_str(), units_to_metres_, swap_xy_, frame_origin_x_,
             frame_origin_y_);
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to load frame_origin from YAML: %s — using (0, 0)", e.what());
    return false;
  }
}

void ConeGroundProjector::hMatrixCallback(
    const homographic_mono_cam_depth_perception::HMatrix::ConstPtr &msg) {

  H_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      H_.at<double>(i, j) = msg->data[i * 3 + j];

  ROS_INFO_THROTTLE(2.0, "Homography matrix received and updated");
}

void ConeGroundProjector::frameDataCallback(
    const homographic_mono_cam_depth_perception::FrameData::ConstPtr &msg) {

  if (H_.empty()) {
    ROS_WARN_THROTTLE(
        5.0, "Cannot project cones: waiting for homography matrix on %s",
        hmatrix_topic_.c_str());
    return;
  }

  homographic_mono_cam_depth_perception::FrameBevData output_frame;
  output_frame.timestamp = msg->timestamp;

  for (const auto &input_cone : msg->cones) {
    cv::Mat image_point =
        (cv::Mat_<double>(3, 1) << input_cone.x, input_cone.y, 1.0);
    cv::Mat ground_point = H_ * image_point;

    double w = ground_point.at<double>(2);
    if (std::abs(w) < 1e-6) {
      ROS_WARN("Cone %s: Division by near-zero in homography",
               input_cone.cone_id.c_str());
      continue;
    }

    homographic_mono_cam_depth_perception::ConeBevData bev_cone;
    bev_cone.cone_id = input_cone.cone_id;
    bev_cone.color   = input_cone.color;
    bev_cone.x       = input_cone.x;
    bev_cone.y       = input_cone.y;

    // Project pixel -> H frame (metres), then shift to car frame
    // using frame_origin loaded from the YAML.
    double a = ground_point.at<double>(0) / w * units_to_metres_;
    double b = ground_point.at<double>(1) / w * units_to_metres_;
    double fwd = swap_xy_ ? b : a;
    double lat = swap_xy_ ? a : b;

    // Points at/above the horizon project behind the camera or to infinity
    if (fwd <= 0.0 || fwd > max_distance_)
      continue;

    bev_cone.ground_x = fwd + frame_origin_x_;
    bev_cone.ground_y = lat + frame_origin_y_;
    bev_cone.distance = std::sqrt(bev_cone.ground_x * bev_cone.ground_x +
                                  bev_cone.ground_y * bev_cone.ground_y);

    output_frame.cones.push_back(bev_cone);

    ROS_INFO_THROTTLE(5.0, "Cone %s: ground=(%.2f, %.2f), dist=%.2fm",
                      bev_cone.cone_id.c_str(), bev_cone.ground_x,
                      bev_cone.ground_y, bev_cone.distance);
  }

  frame_bev_pub_.publish(output_frame);
  ROS_INFO_THROTTLE(2.0, "Processed frame with %zu cones at time %.6f",
                    output_frame.cones.size(), output_frame.timestamp);
}

void ConeGroundProjector::run() { ros::spin(); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "cone_ground_projector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ConeGroundProjector projector(nh, nh_private);
  projector.run();

  return 0;
}
