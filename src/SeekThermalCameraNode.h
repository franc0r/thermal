#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>

#include <seek/seek.h>

namespace francor {

class SeekThermalCameraNode : public rclcpp::Node
{
public:
  SeekThermalCameraNode();

private:
  void processCameraData();

  LibSeek::SeekThermal _cam;

  // std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub_colored_image;
  std::shared_ptr<rclcpp::TimerBase> _timer_process_camera_data;
  std::shared_ptr<image_transport::CameraPublisher> _pub_colored_image;
  std::shared_ptr<sensor_msgs::msg::Image> _color_image;
};

} // end namespace francor