#ifndef TRACK_MAP_VISUALIZER_H
#define TRACK_MAP_VISUALIZER_H

#include <homographic_mono_cam_depth_perception/ConeBevData.h>
#include <homographic_mono_cam_depth_perception/FrameBevData.h>
#include <deque>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

struct OdomPose {
  double stamp;
  double x;
  double y;
  double yaw;
};

struct TrackedCone {
  float x;
  float y;
  uint8_t color;
  int hits;
};

class TrackMapVisualizer {
public:
  TrackMapVisualizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private); // sets up subscribers and reads canvas parameters
  ~TrackMapVisualizer();
  void run(); // enters the ROS spin loop so cone and odometry callbacks begin firing

private:
  void coneCallback(const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg); // building a global map requires transforming car-relative detections into the world frame
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg); // the car's changing pose is needed to correctly transform each new detection
  bool tryAddCone(float x, float y, uint8_t color); // prevents duplicate cone entries when the car passes the same cone multiple times
  cv::Point2f worldToCanvas(float wx, float wy) const; // the map is stored in metres but must be drawn in pixels
  cv::Scalar colorFromEnum(uint8_t color_enum) const; // maps numeric color enum to BGR values needed for OpenCV rendering
  void drawPath(cv::Mat &canvas) const; // shows where the car has been so engineers can verify track coverage
  void drawCones(cv::Mat &canvas) const; // plots each tracked cone on the accumulated global map
  void drawCar(cv::Mat &canvas) const; // shows the current car position relative to the map for spatial reference
  void drawGrid(cv::Mat &canvas) const; // provides a metric scale reference so distances can be read directly off the canvas

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber cone_sub_;
  ros::Subscriber odom_sub_;

  int   canvas_width_;
  int   canvas_height_;
  float pixels_per_meter_;

  float merge_radius_;
  float max_range_;
  float min_range_;
  int   min_hits_;
  float camera_offset_x_;
  float camera_offset_y_;

  double car_x_;
  double car_y_;
  double car_yaw_;
  bool   pose_received_;

  std::vector<cv::Point2f> car_path_;
  std::vector<TrackedCone> track_cones_;
  std::deque<OdomPose> odom_buffer_;

  mutable std::mutex data_mutex_;

  std::string cone_topic_;
  std::string odom_topic_;
};

#endif // TRACK_MAP_VISUALIZER_H
