#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/core_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoreNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}