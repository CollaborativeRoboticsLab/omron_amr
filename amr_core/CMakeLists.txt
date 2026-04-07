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

ament_package()
