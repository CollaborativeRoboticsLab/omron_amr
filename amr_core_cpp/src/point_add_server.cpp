#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/nodes/point_add_server.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointAddServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}