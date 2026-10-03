#include "standalone_cone_visualizer.h"
#include <cv_bridge/cv_bridge.h>
#include <foxglove_msgs/ImageMarkerArray.h>
#include <foxglove_msgs/CompressedImage.h>

StandaloneConeVisualizer::StandaloneConeVisualizer(ros::NodeHandle &nh,
                                                   ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), last_frame_time_(0.0),
      source_image_width_(1280), source_image_height_(720) {

  // Get parameters
  nh_private_.param<std::string>("image_topic", image_topic_,
                                  "/camera/image/compressed");
  nh_private_.param<std::string>("bbox_topic", bbox_topic_,
                                  "/cone_detections");
  nh_private_.param<std::string>("subscribe_topic", subscribe_topic_,
                                  "/projected_cones");
  nh_private_.param<int>("image_width", image_width_, 960);
  nh_private_.param<int>("image_height", image_height_, 540);
  nh_private_.param<int>("bev_size", bev_size_, 540);
  nh_private_.param<float>("pixels_per_meter", pixels_per_meter_, 20.0f);
  nh_private_.param<std::string>("record_path", record_path_, "");
  nh_private_.param<double>("record_seconds", record_seconds_, 0.0);

  // Set up subscribers
  image_sub_ = nh_.subscribe(image_topic_, 10,
                             &StandaloneConeVisualizer::imageCallback, this);
  bbox_sub_ = nh_.subscribe(bbox_topic_, 10,
                            &StandaloneConeVisualizer::bboxCallback, this);
  frame_data_sub_ = nh_.subscribe(subscribe_topic_, 10,
                                  &StandaloneConeVisualizer::frameDataCallback, this);

  ROS_INFO("Starting Standalone Cone Visualizer [SIDE-BY-SIDE]");
  ROS_INFO("Image panel: %dx%d | BEV panel: %dx%d", image_width_, image_height_,
           bev_size_, bev_size_);
  ROS_INFO("Pixels per meter: %.1f", pixels_per_meter_);
  ROS_INFO("Image topic: %s", image_topic_.c_str());
  ROS_INFO("BBox topic: %s", bbox_topic_.c_str());
  ROS_INFO("Cone topic: %s", subscribe_topic_.c_str());
  if (!record_path_.empty()) {
    ROS_INFO("Recording to: %s", record_path_.c_str());
  }
}

StandaloneConeVisualizer::~StandaloneConeVisualizer() {
  if (writer_.isOpened()) {
    writer_.release();
  }
  cv::destroyAllWindows();
}

void StandaloneConeVisualizer::imageCallback(
    const foxglove_msgs::CompressedImageConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  cv::Mat image;
  try {
    image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (image.empty()) {
      ROS_WARN("Failed to decode image");
      return;
    }
  } catch (const std::exception &e) {
    ROS_WARN("Image decode exception: %s", e.what());
    return;
  }

  source_image_width_ = image.cols;
  source_image_height_ = image.rows;

  CachedImage cached;
  cached.image = image;
  cached.timestamp = msg->timestamp.toSec();

  image_cache_.push_back(cached);
  if (image_cache_.size() > 30) {
    image_cache_.pop_front();
  }
}

void StandaloneConeVisualizer::bboxCallback(
    const foxglove_msgs::ImageMarkerArrayConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  CachedBBoxes cached;
  cached.markers = msg->markers;

  // Get timestamp from first marker if available, otherwise use 0
  if (!msg->markers.empty() && msg->markers[0].header.stamp.sec > 0) {
    cached.timestamp = msg->markers[0].header.stamp.toSec();
  } else {
    cached.timestamp = ros::Time::now().toSec();
  }

  bbox_cache_.push_back(cached);
  if (bbox_cache_.size() > 30) {
    bbox_cache_.pop_front();
  }
}

void StandaloneConeVisualizer::frameDataCallback(
    const homographic_mono_cam_depth_perception::FrameBevData::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  last_frame_time_ = msg->timestamp;
  current_cones_ = msg->cones;

  ROS_INFO_THROTTLE(2.0,
                    "Visualizer: Received frame with %zu cones at time %.6f",
                    current_cones_.size(), last_frame_time_);
}

double StandaloneConeVisualizer::findClosestTimestamp(
    double target_time, const std::deque<CachedImage> &images,
    const std::deque<CachedBBoxes> &bboxes, CachedImage &best_image,
    CachedBBoxes &best_bboxes) {
  double best_diff = 1e10;

  if (!images.empty()) {
    for (const auto &img : images) {
      double diff = std::abs(img.timestamp - target_time);
      if (diff < best_diff) {
        best_diff = diff;
        best_image = img;
      }
    }
  }

  // Images and boxes are matched independently to the cone frame time
  double best_box_diff = 1e10;
  if (!bboxes.empty()) {
    for (const auto &box : bboxes) {
      double diff = std::abs(box.timestamp - target_time);
      if (diff < best_box_diff) {
        best_box_diff = diff;
        best_bboxes = box;
      }
    }
  }

  return best_diff;
}

cv::Scalar StandaloneConeVisualizer::getColorFromEnum(uint8_t color_enum) {
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

void StandaloneConeVisualizer::drawImagePanel(
    cv::Mat &panel, const cv::Mat &image,
    const std::vector<visualization_msgs::ImageMarker> &markers,
    int cone_count, double frame_time) {
  // Resize image to panel dimensions
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(image_width_, image_height_));

  // Copy to panel
  resized_image.copyTo(panel);

  // Draw bounding boxes
  float scale_x = (float)image_width_ / source_image_width_;
  float scale_y = (float)image_height_ / source_image_height_;

  for (const auto &marker : markers) {
    if (marker.points.size() < 2)
      continue;

    cv::Scalar color(
        (int)(marker.outline_color.b * 255), (int)(marker.outline_color.g * 255),
        (int)(marker.outline_color.r * 255)); // RGBA to BGR

    // Draw polygon (closed polyline)
    std::vector<cv::Point2i> scaled_points;
    for (const auto &pt : marker.points) {
      scaled_points.push_back(cv::Point2i((int)(pt.x * scale_x), (int)(pt.y * scale_y)));
    }
    scaled_points.push_back(scaled_points[0]); // close the polygon
    cv::polylines(panel, scaled_points, false, color, 2, cv::LINE_AA);
  }

  // Cone base point and projected distance for each cone
  std::vector<homographic_mono_cam_depth_perception::ConeBevData> cones;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cones = current_cones_;
  }
  for (const auto &cone : cones) {
    cv::Point base(cone.x * scale_x, cone.y * scale_y);
    cv::circle(panel, base, 4, getColorFromEnum(cone.color), -1, cv::LINE_AA);
    cv::circle(panel, base, 4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    char dist_text[20];
    snprintf(dist_text, sizeof(dist_text), "%.1fm", cone.distance);
    int baseline = 0;
    cv::Size ts = cv::getTextSize(dist_text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1,
                                  &baseline);
    cv::Point org(base.x - ts.width / 2, base.y + ts.height + 8);
    cv::rectangle(panel, org + cv::Point(-2, baseline),
                  org + cv::Point(ts.width + 2, -ts.height - 2),
                  cv::Scalar(0, 0, 0), -1);
    cv::putText(panel, dist_text, org, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  // Draw info overlay (frame time and cone count)
  char info_str[200];
  snprintf(info_str, sizeof(info_str), "Frame: %.3f | Cones: %d", frame_time,
           cone_count);

  cv::rectangle(panel, cv::Point(10, 10), cv::Point(350, 50),
                cv::Scalar(0, 0, 0), -1);
  cv::rectangle(panel, cv::Point(10, 10), cv::Point(350, 50),
                cv::Scalar(100, 100, 100), 2);

  cv::putText(panel, info_str, cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

cv::Point2f StandaloneConeVisualizer::groundToCanvas(float ground_x,
                                                     float ground_y) {
  float canvas_x = bev_size_ / 2.0f - (ground_y * pixels_per_meter_);
  float canvas_y = bev_size_ - (ground_x * pixels_per_meter_);
  return cv::Point2f(canvas_x, canvas_y);
}

void StandaloneConeVisualizer::drawGrid(cv::Mat &canvas) {
  // Draw distance circles every 5 meters
  for (int m = 5; m <= 50; m += 5) {
    float radius = m * pixels_per_meter_;
    cv::Point2f center(bev_size_ / 2.0f, bev_size_);
    if (radius > bev_size_)
      continue;
    cv::circle(canvas, center, radius, cv::Scalar(50, 50, 50), 1);

    float label_y = center.y - radius - 5;
    if (label_y >= 10) {
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
    cv::line(canvas, cv::Point(bev_size_ / 2.0f - x_offset, 0),
             cv::Point(bev_size_ / 2.0f - x_offset, bev_size_),
             cv::Scalar(30, 30, 30), 1);
  }

  // Center line (forward axis)
  cv::line(canvas, cv::Point(bev_size_ / 2.0f, 0),
           cv::Point(bev_size_ / 2.0f, bev_size_), cv::Scalar(80, 80, 80), 1);
}

void StandaloneConeVisualizer::drawCones(cv::Mat &canvas) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  for (const auto &cone : current_cones_) {
    cv::Scalar color = getColorFromEnum(cone.color);
    cv::Point2f pos = groundToCanvas(cone.ground_x, cone.ground_y);

    // Skip if outside canvas
    if (pos.x < 0 || pos.x >= bev_size_ || pos.y < 0 || pos.y >= bev_size_) {
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

void StandaloneConeVisualizer::drawCarAndInfo(cv::Mat &canvas, int cone_count,
                                               double frame_time) {
  // Car representation (triangle at bottom center)
  cv::Point car_pos(bev_size_ / 2, bev_size_ - 10);
  std::vector<cv::Point> car_shape = {cv::Point(car_pos.x - 10, car_pos.y),
                                      cv::Point(car_pos.x + 10, car_pos.y),
                                      cv::Point(car_pos.x, car_pos.y - 20)};
  cv::fillConvexPoly(canvas, car_shape, cv::Scalar(0, 0, 255));

  // Info overlay
  char info_str[100];
  snprintf(info_str, sizeof(info_str), "BEV | Frame: %.3f | Cones: %d",
           frame_time, cone_count);
  cv::putText(canvas, info_str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 255, 0), 2);
}

void StandaloneConeVisualizer::drawBevPanel(cv::Mat &bev_canvas) {
  drawGrid(bev_canvas);

  int cone_count = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cone_count = current_cones_.size();
  }

  drawCones(bev_canvas);
  drawCarAndInfo(bev_canvas, cone_count, last_frame_time_);
}

void StandaloneConeVisualizer::run() {
  cv::namedWindow("Cone Distance Overlay", cv::WINDOW_NORMAL);
  int total_width = image_width_ + bev_size_;
  cv::resizeWindow("Cone Distance Overlay", total_width, image_height_);

  ros::Rate rate(30);

  while (ros::ok()) {
    ros::spinOnce();

    // Find the best matching image and bbox for the latest cone frame
    CachedImage best_image;
    CachedBBoxes best_bboxes;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (last_frame_time_ > 0) {
        findClosestTimestamp(last_frame_time_, image_cache_, bbox_cache_,
                             best_image, best_bboxes);
      }

      // If no best match yet, use the latest available
      if (best_image.image.empty() && !image_cache_.empty()) {
        best_image = image_cache_.back();
      }
    }

    // Create left panel: camera image with bounding boxes and distance labels
    cv::Mat left_panel = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);

    if (!best_image.image.empty()) {
      int cone_count = 0;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        cone_count = current_cones_.size();
      }
      drawImagePanel(left_panel, best_image.image, best_bboxes.markers,
                     cone_count, last_frame_time_);
    }

    // Create right panel: BEV
    cv::Mat right_panel = cv::Mat::zeros(image_height_, bev_size_, CV_8UC3);
    drawBevPanel(right_panel);

    // Compose left and right panels
    cv::Mat composite;
    cv::hconcat(left_panel, right_panel, composite);

    // Display
    cv::imshow("Cone Distance Overlay", composite);

    // Record if requested
    if (!record_path_.empty()) {
      if (!writer_.isOpened()) {
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        writer_.open(record_path_, fourcc, 30.0, composite.size());
        if (!writer_.isOpened()) {
          ROS_WARN("Failed to open video writer at %s", record_path_.c_str());
        } else {
          ROS_INFO("Recording to %s", record_path_.c_str());
        }
      }
      if (writer_.isOpened()) {
        writer_.write(composite);
        // Finalise the file after record_seconds so it is playable even if
        // the node is later killed without a clean shutdown
        if (record_seconds_ > 0 && ++recorded_frames_ >= record_seconds_ * 30.0) {
          writer_.release();
          record_path_.clear();
          ROS_INFO("Recording finished (%d frames)", recorded_frames_);
        }
      }
    }

    if (cv::waitKey(1) == 27) // ESC
      break;

    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "standalone_cone_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  StandaloneConeVisualizer visualizer(nh, nh_private);
  visualizer.run();

  return 0;
}
