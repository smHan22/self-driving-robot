#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>

using namespace cv;
using namespace std;

class CamSubNode : public rclcpp::Node
{
public:
    CamSubNode();
    ~CamSubNode();

private:
    void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_publisher_;
};