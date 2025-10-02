#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/nodes/action_server.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LDActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}