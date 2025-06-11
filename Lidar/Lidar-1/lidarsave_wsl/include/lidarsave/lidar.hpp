#ifndef _LIDAR_HPP_
#define _LIDAR_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"
//#include <memory>
#define RAD2DEG(x) ((x)*180./M_PI)
 
using std::placeholders::_1;
 
class lidar : public rclcpp::Node 
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    static cv::Mat lidar_img;
    static cv::VideoWriter output;
 
    static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan);
public:
    lidar();
};
 
#endif //_LIDAR_HPP_


