#ifndef CONE_GROUND_PROJECTOR_H
#define CONE_GROUND_PROJECTOR_H

#include <homographic_mono_cam_depth_perception/HMatrix.h>
#include <homographic_mono_cam_depth_perception/FrameBevData.h>
#include <homographic_mono_cam_depth_perception/FrameData.h>
#include <map>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

class ConeGroundProjector {
public:
  ConeGroundProjector(ros::NodeHandle &nh, ros::NodeHandle &nh_private); // sets up subscribers, publishers, and loads frame origin from YAML
  ~ConeGroundProjector();
  void run(); // enters the ROS spin loop so cone and H-matrix callbacks begin firing

private:
  void frameDataCallback(const homographic_mono_cam_depth_perception::FrameData::ConstPtr &msg); // cone pixel positions need converting to real-world distances for navigation
  void hMatrixCallback(const homographic_mono_cam_depth_perception::HMatrix::ConstPtr &msg); // H changes whenever the camera tilts, so the projector must always use the latest matrix
  bool loadFrameOrigin(); // H projects into the ArUco frame; this shifts results into the car frame so navigation gets correct coordinates

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber frame_data_sub_;
  ros::Subscriber hmatrix_sub_;
  ros::Publisher frame_bev_pub_;

  cv::Mat H_;
  std::string hmatrix_topic_;
  std::string subscribe_topic_;
  std::string publish_topic_;
  std::string yaml_path_;

  double frame_origin_x_;
  double frame_origin_y_;
  double units_to_metres_; // scale from H output units to metres
  bool swap_xy_;           // true if H row 0 is lateral and row 1 forward
  double max_distance_;    // detections beyond this (near horizon) are dropped
};

#endif // CONE_GROUND_PROJECTOR_H
