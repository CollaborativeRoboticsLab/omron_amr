#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/nodes/unified_action_client.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnifiedAmrActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}