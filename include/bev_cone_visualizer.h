#ifndef BEV_CONE_VISUALIZER_H
#define BEV_CONE_VISUALIZER_H

#include <homographic_mono_cam_depth_perception/FrameBevData.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

class BevConeVisualizer {
public:
  BevConeVisualizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private); // sets up the subscriber and reads canvas parameters
  ~BevConeVisualizer();
  void run(); // enters the ROS spin loop so cone data callbacks begin firing

private:
  void frameDataCallback(const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg); // caches the latest projected cone positions for the next render cycle
  void drawCones(cv::Mat &canvas); // renders cone positions in bird's-eye view for real-time distance validation
  void drawGrid(cv::Mat &canvas); // provides a metric scale reference so distances can be read directly off the canvas
  void drawInfoOverlay(cv::Mat &canvas, int cone_count); // surfaces per-frame stats without needing a separate debug tool
  cv::Scalar getColorFromEnum(uint8_t color_enum); // converts numeric color enum to BGR values needed for OpenCV rendering
  cv::Point2f groundToCanvas(float ground_x, float ground_y); // real-world metre coordinates must be scaled and flipped to fit the pixel canvas

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber frame_data_sub_;

  int canvas_width_;
  int canvas_height_;
  float pixels_per_meter_;
  std::string subscribe_topic_;
  std::string record_path_;  // empty = no recording
  cv::VideoWriter writer_;

  std::vector<homographic_mono_cam_depth_perception::ConeBevData> current_cones_;
  std::mutex data_mutex_;
  double last_frame_time_;
};

#endif // BEV_CONE_VISUALIZER_H
