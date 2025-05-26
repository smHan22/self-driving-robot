#include "camera2-3/pub.hpp"
#include "cv_bridge/cv_bridge.h"

CamPublisher::CamPublisher()
: Node("campub"), loop_rate_(40.0)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos);

  pipeline_ =
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=(int)640, height=(int)360, "
    "format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, "
    "width=(int)640, height=(int)360, format=(string)BGRx ! "
    "videoconvert ! video/x-raw, format=(string)BGR ! appsink";

  cap_.open(pipeline_, cv::CAP_GSTREAMER);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video!");
    rclcpp::shutdown();
  }
}

void CamPublisher::run()
{
  cv::Mat frame;
  std_msgs::msg::Header header;

  while (rclcpp::ok())
  {
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty frame captured!");
      break;
    }

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg();
    publisher_->publish(*msg);
    loop_rate_.sleep();
  }
}
