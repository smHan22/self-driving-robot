#include "camera1-3/pub.hpp"

CamPublisher::CamPublisher()
: Node("campub"), loop_rate_(40.0)
{
  std::string src = "nvarguscamerasrc sensor-id=0 ! "
                    "video/x-raw(memory:NVMM), width=(int)640, height=(int)360, "
                    "format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, "
                    "width=(int)640, height=(int)360, format=(string)BGRx ! "
                    "videoconvert ! video/x-raw, format=(string)BGR ! appsink";

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);

  cap_.open(src, cv::CAP_GSTREAMER);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video!");
    rclcpp::shutdown();
  }
}

CamPublisher::~CamPublisher()
{
  cap_.release();
}

void CamPublisher::publish_loop()
{
  cv::Mat frame;
  std_msgs::msg::Header header;

  while (rclcpp::ok()) {
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "frame empty");
      break;
    }

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg();
    publisher_->publish(*msg);

    loop_rate_.sleep();
  }
}
