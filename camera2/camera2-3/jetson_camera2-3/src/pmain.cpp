#include "camera2-3/pub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisher>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
