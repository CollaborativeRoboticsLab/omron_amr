#include <amr_core_cpp/core_node.hpp>
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(amr_core_cpp::CoreNode)

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node object and executor object
  auto node = std::make_shared<amr::CoreNode>();

  // Initialize the node
  node->initialize();

  // Create a MultiThreadedExecutor
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add the node to the executor
  exec->add_node(node);

  // Spin the executor
  exec->spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}