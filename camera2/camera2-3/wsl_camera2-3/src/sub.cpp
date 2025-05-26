#include "camera2-3/sub.hpp"
#include "opencv2/opencv.hpp"

CamSubscriber::CamSubscriber()
: Node("camsub_wsl")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "image/compressed", qos,
    std::bind(&CamSubscriber::image_callback, this, std::placeholders::_1)
  );
}

void CamSubscriber::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty frame");
    return;
  }

  cv::imshow("wsl", frame);
  cv::waitKey(1);

  RCLCPP_INFO(this->get_logger(), "Received Image: %s, %d x %d",
              msg->format.c_str(), frame.rows, frame.cols);
}
