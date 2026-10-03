#include "bev_cone_visualizer.h"

BevConeVisualizer::BevConeVisualizer(ros::NodeHandle &nh,
                                     ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), last_frame_time_(0.0) {

  nh_private_.param<int>("canvas_width", canvas_width_, 800);
  nh_private_.param<int>("canvas_height", canvas_height_, 800);
  nh_private_.param<float>("pixels_per_meter", pixels_per_meter_, 40.0f);
  nh_private_.param<std::string>("subscribe_topic", subscribe_topic_,
                                 "/projected_cones");
  nh_private_.param<std::string>("record_path", record_path_, "");

  frame_data_sub_ = nh_.subscribe(subscribe_topic_, 10,
                                  &BevConeVisualizer::frameDataCallback, this);

  ROS_INFO("BEV Cone Visualizer initialized");
  ROS_INFO("Canvas: %dx%d | Scale: %.1f px/m", canvas_width_, canvas_height_,
           pixels_per_meter_);
}

BevConeVisualizer::~BevConeVisualizer() {
  cv::destroyWindow("BEV Cone Visualizer");
}

void BevConeVisualizer::frameDataCallback(
    const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_frame_time_ = msg->timestamp;
  current_cones_ = msg->cones;
}

cv::Point2f BevConeVisualizer::groundToCanvas(float ground_x, float ground_y) {
  // Car is at (0,0) in ground coordinates.
  // In BEV, ground_x is forward, ground_y is left/right.
  // Canvas: (0,0) is top-left.
  // We want car at bottom center of canvas.

  float canvas_x = canvas_width_ / 2.0f - (ground_y * pixels_per_meter_);
  float canvas_y = canvas_height_ - (ground_x * pixels_per_meter_);

  return cv::Point2f(canvas_x, canvas_y);
}

cv::Scalar BevConeVisualizer::getColorFromEnum(uint8_t color_enum) {
  switch (color_enum) {
  case homographic_mono_cam_depth_perception::ConeBevData::ORANGE:
    return cv::Scalar(0, 165, 255);
  case homographic_mono_cam_depth_perception::ConeBevData::BLUE:
    return cv::Scalar(255, 0, 0);
  case homographic_mono_cam_depth_perception::ConeBevData::YELLOW:
    return cv::Scalar(0, 255, 255);
  case homographic_mono_cam_depth_perception::ConeBevData::GREEN:
    return cv::Scalar(0, 255, 0);
  default:
    return cv::Scalar(128, 128, 128);
  }
}

void BevConeVisualizer::drawGrid(cv::Mat &canvas) {
  // Draw distance circles every 5 meters
  for (int m = 5; m <= 50; m += 5) {
    float radius = m * pixels_per_meter_;
    cv::Point2f center(canvas_width_ / 2.0f, canvas_height_);
    if (radius > canvas_height_)
      continue; // circle entirely off canvas
    cv::circle(canvas, center, radius, cv::Scalar(50, 50, 50), 1);

    float label_y = center.y - radius - 5;
    if (label_y >= 10) { // only draw label when it fits inside the canvas
      char label[10];
      snprintf(label, sizeof(label), "%dm", m);
      cv::putText(canvas, label, cv::Point(center.x + 5, label_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1);
    }
  }

  // Draw lateral lines every 2 meters
  for (int m = -10; m <= 10; m += 2) {
    if (m == 0)
      continue;
    float x_offset = m * pixels_per_meter_;
    cv::line(canvas, cv::Point(canvas_width_ / 2.0f - x_offset, 0),
             cv::Point(canvas_width_ / 2.0f - x_offset, canvas_height_),
             cv::Scalar(30, 30, 30), 1);
  }

  // Center line (forward axis)
  cv::line(canvas, cv::Point(canvas_width_ / 2.0f, 0),
           cv::Point(canvas_width_ / 2.0f, canvas_height_),
           cv::Scalar(80, 80, 80), 1);
}

void BevConeVisualizer::drawCones(cv::Mat &canvas) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  for (const auto &cone : current_cones_) {
    cv::Scalar color = getColorFromEnum(cone.color);
    cv::Point2f pos = groundToCanvas(cone.ground_x, cone.ground_y);

    // Skip if outside canvas
    if (pos.x < 0 || pos.x >= canvas_width_ || pos.y < 0 ||
        pos.y >= canvas_height_) {
      continue;
    }

    // Draw cone
    cv::circle(canvas, pos, 8, color, -1, cv::LINE_AA);
    cv::circle(canvas, pos, 8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // Draw distance label
    char dist_text[20];
    snprintf(dist_text, sizeof(dist_text), "%.1fm", cone.distance);
    cv::putText(canvas, dist_text, cv::Point(pos.x + 10, pos.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
}

void BevConeVisualizer::drawInfoOverlay(cv::Mat &canvas, int cone_count) {
  char info_str[100];
  snprintf(info_str, sizeof(info_str), "BEV View | Frame: %.3f | Cones: %d",
           last_frame_time_, cone_count);
  cv::putText(canvas, info_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
              0.7, cv::Scalar(0, 255, 0), 2);

  // Draw car representation (triangle at bottom center)
  cv::Point car_pos(canvas_width_ / 2, canvas_height_ - 10);
  std::vector<cv::Point> car_shape = {cv::Point(car_pos.x - 10, car_pos.y),
                                      cv::Point(car_pos.x + 10, car_pos.y),
                                      cv::Point(car_pos.x, car_pos.y - 20)};
  cv::fillConvexPoly(canvas, car_shape, cv::Scalar(0, 0, 255));
}

void BevConeVisualizer::run() {
  cv::namedWindow("BEV Cone Visualizer", cv::WINDOW_AUTOSIZE);

  ros::Rate rate(30);
  while (ros::ok()) {
    cv::Mat canvas = cv::Mat::zeros(canvas_height_, canvas_width_, CV_8UC3);

    drawGrid(canvas);

    int cone_count = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cone_count = current_cones_.size();
    }

    drawCones(canvas);
    drawInfoOverlay(canvas, cone_count);

    cv::imshow("BEV Cone Visualizer", canvas);

    // Optional recording of exactly what the window shows
    if (!record_path_.empty()) {
      if (!writer_.isOpened()) {
        writer_.open(record_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                     30.0, canvas.size());
        ROS_INFO("Recording BEV view to %s", record_path_.c_str());
      }
      writer_.write(canvas);
    }
    if (cv::waitKey(1) == 27) // ESC
      break;

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bev_cone_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  BevConeVisualizer visualizer(nh, nh_private);
  visualizer.run();

  return 0;
}
