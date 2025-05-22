#include "camera1-3/sub.hpp"

CamSubscriber::CamSubscriber()
: Node("camsub")
{
  std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                    "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                    "h264parse ! rtph264pay pt=96 ! "
                    "udpsink host=192.168.0.6 port=8001 sync=false";

  writer_.open(dst, 0, 30.0, cv::Size(640, 360), true);
  if (!writer_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Writer open failed!");
    rclcpp::shutdown();
    return;
  }

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "image/compressed", qos_profile,
    std::bind(&CamSubscriber::image_callback, this, std::placeholders::_1));
}

CamSubscriber::~CamSubscriber()
{
  writer_.release();
}

void CamSubscriber::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (!frame.empty()) {
    writer_ << frame;
    RCLCPP_INFO(this->get_logger(), "Received Image : %s, %d x %d",
                msg->format.c_str(), frame.rows, frame.cols);
  }
}
