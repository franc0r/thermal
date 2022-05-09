#include "SeekThermalCameraNode.h"
#include <functional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <chrono>

#include <cv_bridge/cv_bridge.h>

namespace francor {

using namespace std::chrono_literals;

SeekThermalCameraNode::SeekThermalCameraNode()
  : rclcpp::Node("seek_node"),
    _cam()
{
  _cam.open();

  create_wall_timer(
    0.1s,
    std::bind(&SeekThermalCameraNode::processCameraData, this)
  );
}

void SeekThermalCameraNode::processCameraData()
{
  cv::Mat frame;

  if (!_cam.read(frame)) {
    RCLCPP_ERROR(get_logger(), "Can't read image from thermal cam \"seek\".");
    return;
  }

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

  
}



} // end namespace francor