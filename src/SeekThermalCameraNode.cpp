#include "SeekThermalCameraNode.h"
#include <functional>
#include <memory>
#include <opencv2/core/version.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace francor {

using namespace std::chrono_literals;

SeekThermalCameraNode::SeekThermalCameraNode()
  : rclcpp::Node("seek_node"),
    _cam(),
    _pub_colored_image(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, "image_raw", rclcpp::QoS{2}.get_rmw_qos_profile())))
    // _color_image(std::make_shared<sensor_msgs::msg::Image>())
{
  RCLCPP_INFO(get_logger(), "Initialize Seek thermal camera.");

  if (!_cam.open()) {
    RCLCPP_ERROR(get_logger(), "Can't open Seek thermal camera.");
    return;
  }

  _timer_process_camera_data = create_wall_timer(
    0.1s,
    std::bind(&SeekThermalCameraNode::processCameraData, this)
  );
}

void SeekThermalCameraNode::processCameraData()
{
  RCLCPP_INFO(get_logger(), "Process thermal image data.");
  cv::Mat frame;

  if (!_cam.read(frame)) {
    RCLCPP_ERROR(get_logger(), "Can't read image from thermal cam \"seek\".");
    return;
  }

  cv::normalize(frame, frame, 0, 65535, cv::NORM_MINMAX);
  frame.convertTo(frame, CV_8UC1);

  cv::Mat color_frame;
  const int rotate = 0;

  // Rotate image
  if (rotate == 90) {
    cv::transpose(frame, frame);
    cv::flip(frame, frame, 1);
  } else if (rotate == 180) {
    cv::flip(frame, frame, -1);
  } else if (rotate == 270) {
    cv::transpose(frame, frame);
    cv::flip(frame, frame, 0);
  }

  cv::applyColorMap(frame, color_frame, cv::COLORMAP_SPRING);
  cv_bridge::CvImage cv_image{_color_image->header, sensor_msgs::image_encodings::BGR8, color_frame};
  _color_image = cv_image.toImageMsg();
  _pub_colored_image->publish(*_color_image, sensor_msgs::msg::CameraInfo()); // \todo fix camera info
}

} // end namespace francor