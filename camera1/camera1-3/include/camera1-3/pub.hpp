#ifndef CAM_PUB_HPP_
#define CAM_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CamPublisher : public rclcpp::Node
{
public:
  CamPublisher();
  ~CamPublisher();

  void publish_loop();

private:
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::WallRate loop_rate_;
  cv::VideoCapture cap_;
};

#endif  // CAM_PUB_HPP_
