#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/nodes/ld_states_publisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LdStatesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}