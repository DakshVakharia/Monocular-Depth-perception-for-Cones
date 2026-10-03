#ifndef CONE_BASE_DETECTOR_H
#define CONE_BASE_DETECTOR_H

#include <algorithm>
#include <foxglove_msgs/ImageMarkerArray.h>
#include <homographic_mono_cam_depth_perception/FrameData.h>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <string>
#include <vector>
#include <visualization_msgs/ImageMarker.h>

class ConeBaseDetector {
public:
  ConeBaseDetector(ros::NodeHandle &nh, ros::NodeHandle &nh_private); // sets up subscribers, publishers, and reads parameters
  ~ConeBaseDetector();
  void run(); // starts spinning so incoming marker messages get processed

private:
  void imageMarkerCallback(const foxglove_msgs::ImageMarkerArray::ConstPtr &msg); // bounding boxes alone don't give a ground contact point; this extracts it
  uint8_t determineColorFromRGBA(const std_msgs::ColorRGBA &color); // cone color tells the pipeline which side of the track the cone belongs to
  std::string colorEnumToString(uint8_t color_enum); // downstream nodes use string color names in FrameData messages
  std::string generateConeId(); // unique IDs allow downstream nodes to track or deduplicate individual cones

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher cone_base_pub_;
  ros::Subscriber marker_sub_;

  double y_offset_pixels_;
  std::string subscribe_topic_;
  std::string publish_topic_;

  int cone_counter_;
};

#endif // CONE_BASE_DETECTOR_H
