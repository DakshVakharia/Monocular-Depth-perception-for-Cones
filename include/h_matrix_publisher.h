#ifndef H_MATRIX_PUBLISHER_H
#define H_MATRIX_PUBLISHER_H

#include <homographic_mono_cam_depth_perception/HMatrix.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

class HMatrixPublisher {
public:
  HMatrixPublisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private); // sets up subscribers, publishers, and loads calibration from YAML
  ~HMatrixPublisher();
  void run(); // enters the ROS spin loop so IMU callbacks begin firing

private:
  bool loadYaml(const std::string &path); // without the base homography loaded, tilt corrections have nothing to update
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg); // IMU is the only sensor that detects camera tilt changes at runtime
  void computeAndPublish(double delta_roll_deg, double delta_pitch_deg); // tilt changes the ground-plane relationship so H must be recomputed before each projection
  void publish(const cv::Mat &H); // separates serialization from computation so computeAndPublish stays focused on the maths

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber imu_sub_;
  ros::Publisher hmatrix_pub_;

  cv::Mat H_base_;
  cv::Mat K_;
  cv::Mat K_inv_;

  bool home_set_;
  double home_roll_;
  double home_pitch_;
  double last_dr_;
  double last_dp_;

  double threshold_deg_;
  bool tilt_correction_;
  std::string imu_topic_;
  std::string publish_topic_;
};

#endif // H_MATRIX_PUBLISHER_H
