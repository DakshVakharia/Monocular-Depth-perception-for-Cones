#ifndef STANDALONE_CONE_VISUALIZER_H
#define STANDALONE_CONE_VISUALIZER_H

#include <homographic_mono_cam_depth_perception/FrameBevData.h>
#include <deque>
#include <foxglove_msgs/ImageMarkerArray.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <foxglove_msgs/CompressedImage.h>
#include <string>
#include <visualization_msgs/ImageMarker.h>

struct CachedImage {
  cv::Mat image;
  double timestamp;
};

struct CachedBBoxes {
  std::vector<visualization_msgs::ImageMarker> markers;
  double timestamp;
};

class StandaloneConeVisualizer {
public:
  StandaloneConeVisualizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~StandaloneConeVisualizer();
  void run();

private:
  void imageCallback(const foxglove_msgs::CompressedImageConstPtr &msg);
  void bboxCallback(const foxglove_msgs::ImageMarkerArrayConstPtr &msg);
  void frameDataCallback(const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg);

  // Rendering: left panel (image + boxes)
  void drawImagePanel(cv::Mat &panel, const cv::Mat &image,
                      const std::vector<visualization_msgs::ImageMarker> &markers,
                      int cone_count, double frame_time);

  // Rendering: right panel (BEV)
  void drawBevPanel(cv::Mat &bev_canvas);
  void drawGrid(cv::Mat &canvas);
  cv::Point2f groundToCanvas(float ground_x, float ground_y);
  void drawCones(cv::Mat &canvas);
  void drawCarAndInfo(cv::Mat &canvas, int cone_count, double frame_time);

  cv::Scalar getColorFromEnum(uint8_t color_enum);
  double findClosestTimestamp(double target_time, const std::deque<CachedImage> &images,
                              const std::deque<CachedBBoxes> &bboxes,
                              CachedImage &best_image, CachedBBoxes &best_bboxes);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber image_sub_;
  ros::Subscriber bbox_sub_;
  ros::Subscriber frame_data_sub_;

  std::deque<CachedImage> image_cache_;
  std::deque<CachedBBoxes> bbox_cache_;
  std::vector<homographic_mono_cam_depth_perception::ConeBevData> current_cones_;
  std::mutex data_mutex_;

  // Parameters
  std::string image_topic_;
  std::string bbox_topic_;
  std::string subscribe_topic_;
  int image_width_;
  int image_height_;
  int bev_size_;
  float pixels_per_meter_;
  std::string record_path_;

  // Video writer
  cv::VideoWriter writer_;
  double record_seconds_;    // 0 = record until shutdown
  int recorded_frames_ = 0;
  int source_image_width_;  // Original image dimensions for coordinate scaling
  int source_image_height_;

  double last_frame_time_;
};

#endif // STANDALONE_CONE_VISUALIZER_H
