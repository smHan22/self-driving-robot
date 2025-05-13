#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.163 port=8001 sync=false";

cv::VideoWriter writer;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat color_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (color_frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Failed to decode image.");
        return;
    }

    cv::Mat gray_frame;
    cv::cvtColor(color_frame, gray_frame, cv::COLOR_BGR2GRAY);

    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 100, 255, cv::THRESH_BINARY);  

    cv::Mat binary_bgr;
    cv::cvtColor(binary_frame, binary_bgr, cv::COLOR_GRAY2BGR);

    writer << binary_bgr;

    RCLCPP_INFO(node->get_logger(), "Sent binary image: %dx%d", binary_bgr.cols, binary_bgr.rows);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "GStreamer writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
