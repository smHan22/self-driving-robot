#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;

cv::VideoWriter file_writer;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty frame received.");
        return;
    }

    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);

    cv::Mat binary_bgr;
    cv::cvtColor(binary_frame, binary_bgr, cv::COLOR_GRAY2BGR);
    file_writer << binary_bgr;

    cv::imshow("Original", frame);
    cv::imshow("Grayscale", gray_frame);
    cv::imshow("Binary", binary_frame);
    cv::waitKey(1);  

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    file_writer.open("binary_output.mp4", fourcc, 30.0, cv::Size(640, 360), true);
    if (!file_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file writer.");
        rclcpp::shutdown();
        return -1;
    }

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
