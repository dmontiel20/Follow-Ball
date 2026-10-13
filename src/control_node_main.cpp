#include "follow_ball/control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<follow_ball::FollowerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
