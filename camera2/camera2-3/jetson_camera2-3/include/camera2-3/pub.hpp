#ifndef CAM_PUB_HPP_
#define CAM_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include "opencv2/opencv.hpp"

class CamPublisher : public rclcpp::Node
{
public:
  CamPublisher();
  void run();

private:
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  rclcpp::WallRate loop_rate_;
  std::string pipeline_;
};

#endif  // CAM_PUB_HPP_
