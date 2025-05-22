#ifndef CAM_SUB_HPP_
#define CAM_SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

class CamSubscriber : public rclcpp::Node
{
public:
  CamSubscriber();
  ~CamSubscriber();

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  cv::VideoWriter writer_;
};

#endif  // CAM_SUB_HPP_
