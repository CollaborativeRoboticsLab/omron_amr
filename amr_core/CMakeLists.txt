cmake_minimum_required(VERSION 3.8)
project(amr_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(libaria CONFIG REQUIRED)

set(AMR_CORE_ARIA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../libaria")

include_directories(
  include
)

add_executable(${PROJECT_NAME} src/core_node.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE AMR_CORE_ARIA_DIR="${AMR_CORE_ARIA_DIR}")
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_action
  tf2_ros
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  visualization_msgs
  amr_msgs
  ament_index_cpp

)
target_link_libraries(${PROJECT_NAME}
  libaria::ArNetworking
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(nav2_velocity_smoother REQUIRED)
  ament_add_gtest(test_drive_interface
    test/test_drive_command_policy.cpp
    test/test_drive_ratio_mapping.cpp
    test/test_drive_interface.cpp
    test/test_drive_tcp_sender.cpp
  )
  ament_target_dependencies(test_drive_interface
    rclcpp tf2_ros tf2_msgs std_msgs nav_msgs geometry_msgs nav2_velocity_smoother
  )
  target_link_libraries(test_drive_interface libaria::ArNetworking)

  add_executable(amr_hardware_probe test/hardware/readonly_probe.cpp)
  find_package(nlohmann_json REQUIRED)
  target_link_libraries(amr_hardware_probe libaria::ArNetworking nlohmann_json::nlohmann_json)
  ament_add_gtest(test_hybrid_config test/hardware/test_hybrid_config.cpp)
  target_link_libraries(test_hybrid_config libaria::ArNetworking nlohmann_json::nlohmann_json)
  ament_add_pytest_test(test_hardware_capture test/test_hardware_capture.py)
endif()

ament_package()
