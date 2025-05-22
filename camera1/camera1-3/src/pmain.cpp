#include "camera1-3/pub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamPublisher>();
  node->publish_loop();
  rclcpp::shutdown();
  return 0;
}
