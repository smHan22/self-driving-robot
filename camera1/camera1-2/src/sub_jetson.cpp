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

std::string output_file = "output.mp4";

cv::VideoWriter gst_writer;
cv::VideoWriter file_writer;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat color_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (color_frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Failed to decode image.");
        return;
    }

    cv::Mat gray_frame, binary_frame, binary_bgr;
    cv::cvtColor(color_frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_frame, binary_frame, 100, 255, cv::THRESH_BINARY);
    cv::cvtColor(binary_frame, binary_bgr, cv::COLOR_GRAY2BGR);  

    gst_writer << binary_bgr;
    file_writer << binary_bgr;

    RCLCPP_INFO(node->get_logger(), "Processed and sent binary image: %dx%d", binary_bgr.cols, binary_bgr.rows);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    gst_writer.open(dst, 0, 30.0, cv::Size(640, 360), true);
    if (!gst_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open GStreamer writer.");
        rclcpp::shutdown();
        return -1;
    }

    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); 
    file_writer.open(output_file, fourcc, 30.0, cv::Size(640, 360), true);
    if (!file_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file writer.");
        rclcpp::shutdown();
        return -1;
    }

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
