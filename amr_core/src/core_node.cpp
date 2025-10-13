#include <rclcpp/rclcpp.hpp>
#include "amr_core/core_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoreNode>();
  node->initialize();

  // Use MultiThreadedExecutor for concurrent callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}