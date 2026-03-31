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

include_directories(
  include
)

add_executable(${PROJECT_NAME} src/core_node.cpp)
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

add_executable(client src/client_node.cpp)
ament_target_dependencies(client
  rclcpp
  rclcpp_action
  std_msgs
  amr_msgs
  geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
