#include "lidarsave/lidar.hpp"
 
cv::Mat lidar::lidar_img(500, 500, CV_8UC3, cv::Scalar(255,255,255));
cv::VideoWriter lidar::output("lidar.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 6.5, cv::Size(500, 500));
 
void lidar::scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;
    printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),RAD2DEG(scan->angle_max));
    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); //RAD2DEG(x) ((x)*180./M_PI)
        float distance = scan->ranges[i];
        printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, distance);
        
        // 스캔영상 그리기 (500pxl x 500pxl / 10m x 10m)
        float rad = degree * M_PI/180;
        int xpixel = 250 + (sin(rad) * distance*100);        // 원점 x: 250, distance는 거리를 픽셀로 변환
        int ypixel = 250 + (cos(rad) * distance*100);        // 원점 y: 250
        cv::rectangle(lidar_img, cv::Rect(xpixel, ypixel, 3, 3), cv::Scalar(0,0,255), -1);
    }
    // 스캔영상 화면출력 및 동영상 저장
    cv::rectangle(lidar_img, cv::Rect(lidar_img.cols/2 - 1, lidar_img.rows/2 - 1, 5, 5), cv::Scalar(0,0,0), -1);
    output << lidar_img;
    cv::imshow("lidar", lidar_img);
    cv::waitKey(1);
 
    //화면 초기화
    lidar_img = cv::Scalar(255,255,255);
}
 
lidar::lidar() : Node("sllidar_client")
{
    lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 
        rclcpp::SensorDataQoS(), scanCb); //std::bind(&lidar::scanCb, this, _1) -> scanCb
}


