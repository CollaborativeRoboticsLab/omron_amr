#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/nodes/arcl_api_service.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArclApiService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}