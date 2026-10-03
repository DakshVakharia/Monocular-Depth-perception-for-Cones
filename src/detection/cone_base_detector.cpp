#include "cone_base_detector.h"
#include <cmath>

ConeBaseDetector::ConeBaseDetector(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), cone_counter_(0) {
  // Load parameters
  nh_private_.param("y_offset_pixels", y_offset_pixels_, 5.0);
  nh_private_.param<std::string>("subscribe_topic", subscribe_topic_,
                                 "/cone_detections");
                                 //read the bounding boxes
  nh_private_.param<std::string>("publish_topic", publish_topic_,
                                 "/cone_bases");
                                 //publish the bases

  ROS_INFO("Cone Base Detector Parameters:");
  ROS_INFO("  Y-offset: %.2f pixels", y_offset_pixels_);
  ROS_INFO("  Subscribe topic: %s", subscribe_topic_.c_str());
  ROS_INFO("  Publish topic: %s", publish_topic_.c_str());

  // Create publisher
  cone_base_pub_ =
      nh_.advertise<homographic_mono_cam_depth_perception::FrameData>(publish_topic_, 10);

  // Create subscriber
  marker_sub_ = nh_.subscribe(subscribe_topic_, 10,
                              &ConeBaseDetector::imageMarkerCallback, this);

  ROS_INFO("Cone Base Detector initialized successfully");
}

ConeBaseDetector::~ConeBaseDetector() {
  ROS_INFO("Cone Base Detector shutting down");
}

void ConeBaseDetector::run() { ros::spin(); }

uint8_t
ConeBaseDetector::determineColorFromRGBA(const std_msgs::ColorRGBA &color) {
  // Extract RGB values (0.0 to 1.0 range)
  float r = color.r;
  float g = color.g;
  float b = color.b;

  // Determine dominant color based on RGB values
  // ORANGE: high R, low G, low B
  if (r > 0.5 && g < 0.3 && b < 0.3) {
    return homographic_mono_cam_depth_perception::ConeData::ORANGE;
  }
  // BLUE: low R, low G, high B
  else if (r < 0.3 && g < 0.3 && b > 0.5) {
    return homographic_mono_cam_depth_perception::ConeData::BLUE;
  }
  // YELLOW: high R, high G, low B
  else if (r > 0.5 && g > 0.5 && b < 0.3) {
    return homographic_mono_cam_depth_perception::ConeData::YELLOW;
  }
  // GREEN: low R, high G, low B
  else if (r < 0.3 && g > 0.5 && b < 0.3) {
    return homographic_mono_cam_depth_perception::ConeData::GREEN;
  }

  return homographic_mono_cam_depth_perception::ConeData::UNKNOWN;
}

std::string ConeBaseDetector::colorEnumToString(uint8_t color_enum) {
  switch (color_enum) {
  case homographic_mono_cam_depth_perception::ConeData::ORANGE:
    return "ORANGE";
  case homographic_mono_cam_depth_perception::ConeData::BLUE:
    return "BLUE";
  case homographic_mono_cam_depth_perception::ConeData::YELLOW:
    return "YELLOW";
  case homographic_mono_cam_depth_perception::ConeData::GREEN:
    return "GREEN";
  default:
    return "UNKNOWN";
  }
}

std::string ConeBaseDetector::generateConeId() {
  return "cone_" + std::to_string(cone_counter_++);
}

void ConeBaseDetector::imageMarkerCallback(
    const foxglove_msgs::ImageMarkerArray::ConstPtr &msg) {
  if (msg->markers.empty()) {
    return;
  }

  // Create FrameData message
  homographic_mono_cam_depth_perception::FrameData frame_msg;
  frame_msg.timestamp = msg->markers[0].header.stamp.toSec();

  // // Check if all markers have the same timestamp
  // bool same_timestamp = true;
  // for (const auto &marker : msg->markers) {
  //   if (marker.header.stamp != msg->markers[0].header.stamp) {
  //     same_timestamp = false;
  //     ROS_WARN("Cone markers in same frame have DIFFERENT timestamps! First:
  //     "
  //              "%.6f, This: %.6f",
  //              msg->markers[0].header.stamp.toSec(),
  //              marker.header.stamp.toSec());
  //   }
  // }
  // if (same_timestamp) {
  //   ROS_INFO_THROTTLE(5.0, "✓ All %zu cones have SAME timestamp: %.6f",
  //                     msg->markers.size(), frame_msg.timestamp);
  // }

  // Process each marker (each marker represents a cone bounding box)
  for (const auto &marker : msg->markers) {
    if (marker.points.empty()) {
      continue;
    }

    // Find the maximum y-coordinate (bottom edge of bbox = cone base in image coords)
    // Image Y increases downward, so max_y is the bottom of the bounding box.
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto &point : marker.points) {
      if (point.y > max_y) {
        max_y = point.y;
      }
    }

    // Find all points at maximum y and calculate average x
    double sum_x = 0.0;
    int count = 0;
    for (const auto &point : marker.points) {
      if (std::abs(point.y - max_y) < 0.001) {
        sum_x += point.x;
        count++;
      }
    }

    if (count == 0) {
      continue;
    }

    double avg_x = sum_x / count;

    // Shift slightly up from the bottom edge (subtract in image coords = move up)
    double final_y = max_y - y_offset_pixels_;

    // Determine cone color from outline_color
    uint8_t cone_color = determineColorFromRGBA(marker.outline_color);

    // Generate unique cone ID (rosbag IDs are not unique - all are 0)
    std::string cone_id = generateConeId();

    // Create ConeData and add to frame
    homographic_mono_cam_depth_perception::ConeData cone_data;
    cone_data.cone_id = cone_id;
    cone_data.color = cone_color;
    cone_data.x = static_cast<float>(avg_x);
    cone_data.y = static_cast<float>(final_y);

    frame_msg.cones.push_back(cone_data);

    // Log the detected cone base
    ROS_DEBUG("Cone [%s] %s: x=%.2f, y=%.2f", cone_id.c_str(),
             colorEnumToString(cone_color).c_str(), avg_x, final_y);
  }

  // Publish the entire frame at once
  if (!frame_msg.cones.empty()) {
    cone_base_pub_.publish(frame_msg);
    ROS_INFO_THROTTLE(2.0, "Published frame with %zu cones at time %.6f",
             frame_msg.cones.size(), frame_msg.timestamp);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cone_base_detector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create detector instance
  ConeBaseDetector detector(nh, nh_private);

  // Run the detector
  detector.run();

  return 0;
}
