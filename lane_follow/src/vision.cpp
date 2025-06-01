#include "test/vision.hpp"
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>

bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

using namespace std::placeholders;
using namespace std;
using namespace cv;

CamSubNode::CamSubNode() : Node("camsub_wsl")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, bind(&CamSubNode::mysub_callback, this, _1));

    control_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("control_values", qos_profile);

    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("Grayscale", WINDOW_AUTOSIZE);
    namedWindow("Binary with Bounding Box", WINDOW_AUTOSIZE);
}

CamSubNode::~CamSubNode()
{
    destroyAllWindows();
}

void CamSubNode::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    struct timeval start, end1;
    gettimeofday(&start, NULL);

    Mat frame = imdecode(Mat(msg->data), IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "frame empty!!!");
        return;
    }

    Mat gray, binary, resizedBinary;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    Scalar meanBrightness = mean(gray);
    gray = gray + (100 - meanBrightness[0]);

    GaussianBlur(gray, gray, Size(5, 5), 1.5);

    threshold(gray, binary, 140, 255, THRESH_BINARY);

    Rect roi(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
    resizedBinary = binary(roi);

    Mat labelImage, stats, centroids;
    int nLabels = connectedComponentsWithStats(resizedBinary, labelImage, stats, centroids, 8, CV_32S);

    Mat colorBinary;
    cvtColor(resizedBinary, colorBinary, COLOR_GRAY2BGR);

    vector<Point> lineCenters;
    for (int i = 1; i < nLabels; i++) {
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);

        Point center(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
        lineCenters.push_back(center);

        rectangle(colorBinary, Rect(x, y, width, height), Scalar(255, 0, 0), 2);
        circle(colorBinary, center, 5, Scalar(255, 0, 0), -1);
    }

    static vector<Point> previousCenters(2, Point(-1, -1));
    static bool firstFrame = true;
    const double MAX_DISTANCE = 100.0;
    double error = 0.0;
    const double k = 0.165;

    Point centerOfImage(frame.cols / 2, frame.rows / 2);

    //첫 프레임 처리
    if (firstFrame && lineCenters.size() >= 2) {        // 최소 두 개의 객체 필요
        vector<Point> leftCandidates, rightCandidates;
        for (const auto& center : lineCenters) {
            if (center.x < centerOfImage.x) {       // 중심점의 좌표 기준으로 좌측 
                leftCandidates.push_back(center);
            } else {     // 중심점의 좌표 기준으로 우측 
                rightCandidates.push_back(center);
            }
        }

        vector<Point> closestCenters(2, Point(-1, -1));     // 가장 가까운 객체 2개
        if (!leftCandidates.empty() && !rightCandidates.empty()) {
            auto leftIt = min_element(leftCandidates.begin(), leftCandidates.end(), [&](const Point& a, const Point& b) {           // leftCandidates에서 비교 기준에 따라 최소 요소의 반복자 반환
                    return norm(a - centerOfImage) < norm(b - centerOfImage);       // 두 점 간 유클리드 거리 계산, centerOfImage에 가장 가까운 좌측 중심점 선택
                });
            closestCenters[0] = *leftIt;        // 좌측 중심점 저장

            auto rightIt = min_element(rightCandidates.begin(), rightCandidates.end(), [&](const Point& a, const Point& b) {
                    return norm(a - centerOfImage) < norm(b - centerOfImage);       
                });
            closestCenters[1] = *rightIt;       // 우측 중심점 저장
        }

        if (closestCenters[0].x != -1 && closestCenters[1].x != -1) {
            rectangle(colorBinary, Rect(closestCenters[0].x - 10, closestCenters[0].y - 10, 20, 20), Scalar(0, 0, 255), 2);
            circle(colorBinary, closestCenters[0], 5, Scalar(0, 0, 255), -1);

            rectangle(colorBinary, Rect(closestCenters[1].x - 10, closestCenters[1].y - 10, 20, 20), Scalar(0, 0, 255), 2);
            circle(colorBinary, closestCenters[1], 5, Scalar(0, 0, 255), -1);

            Point laneCenter((closestCenters[0].x + closestCenters[1].x) / 2, 
                                 (closestCenters[0].y + closestCenters[1].y) / 2);
            circle(colorBinary, laneCenter, 5, Scalar(0, 255, 0), -1);

            error = centerOfImage.x - laneCenter.x;
            previousCenters = closestCenters;
            firstFrame = false;
        } else {
            error = 0.0;
        }
    }
    
    // 이후 프레임 처리
    else if (previousCenters[0].x != -1 && previousCenters[1].x != -1) {
        Point leftCenter = previousCenters[0];
        Point rightCenter = previousCenters[1];

        // 좌측 라인 추적
        auto leftIt = min_element(lineCenters.begin(), lineCenters.end(),
            [&](const Point& a, const Point& b) {
                return norm(a - previousCenters[0]) < norm(b - previousCenters[0]);
            });
        if (leftIt != lineCenters.end() && norm(*leftIt - previousCenters[0]) < MAX_DISTANCE) {
            leftCenter = *leftIt;
        }
        // 우측 라인 추적
        auto rightIt = min_element(lineCenters.begin(), lineCenters.end(),
            [&](const Point& a, const Point& b) {
                return norm(a - previousCenters[1]) < norm(b - previousCenters[1]);
            });
        if (rightIt != lineCenters.end() && norm(*rightIt - previousCenters[1]) < MAX_DISTANCE) {
            rightCenter = *rightIt;
        }

        vector<Point> closestCenters = {leftCenter, rightCenter};

        rectangle(colorBinary, Rect(closestCenters[0].x - 10, closestCenters[0].y - 10, 20, 20), Scalar(0, 0, 255), 2);
        circle(colorBinary, closestCenters[0], 5, Scalar(0, 0, 255), -1);

        rectangle(colorBinary, Rect(closestCenters[1].x - 10, closestCenters[1].y - 10, 20, 20), Scalar(0, 0, 255), 2);
        circle(colorBinary, closestCenters[1], 5, Scalar(0, 0, 255), -1);

        Point laneCenter((closestCenters[0].x + closestCenters[1].x) / 2, 
                            (closestCenters[0].y + closestCenters[1].y) / 2);

        circle(colorBinary, laneCenter, 5, Scalar(0, 255, 0), -1);

        error = centerOfImage.x - laneCenter.x;
        previousCenters = closestCenters;
    }
    else {
        error = 0.0;
        for (const auto& center : previousCenters) {
            if (center.x != -1) {
                rectangle(colorBinary, Rect(center.x - 10, center.y - 10, 20, 20), Scalar(0, 0, 255), 2);
                circle(colorBinary, center, 5, Scalar(0, 0, 255), -1);
            }
        }
        RCLCPP_WARN(this->get_logger(), "유효한 이전 중심점 없음");
    }

    int leftvel = 100 - k * error;
    int rightvel = -(100 + k * error);

    auto control_msg = std_msgs::msg::Float64MultiArray();
    control_msg.data = {error, static_cast<double>(leftvel), static_cast<double>(rightvel)};
    control_publisher_->publish(control_msg);

    imshow("Original", frame);
    imshow("Grayscale", gray);
    imshow("Binary with Bounding Box", colorBinary);
    waitKey(1);

    gettimeofday(&end1, NULL);
    double elapsedMs = (end1.tv_sec - start.tv_sec) * 1000.0 + (end1.tv_usec - start.tv_usec) / 1000.0;
    const int targetDelayMs = 30;
    int sleepMs = targetDelayMs - static_cast<int>(elapsedMs);
    if (sleepMs > 0) {
        usleep(sleepMs * 1000);
    }
    double totalTime = elapsedMs + (sleepMs > 0 ? sleepMs : 0);

    RCLCPP_INFO(this->get_logger(), "수신된 이미지: %s, %d x %d, 에러: %.2f, 좌측 속도: %d, 우측 속도: %d, 시간: %.2f ms",
                msg->format.c_str(), frame.rows, frame.cols, error, leftvel, rightvel, totalTime);

    if (ctrl_c_pressed) {
        rclcpp::shutdown();
    }
}