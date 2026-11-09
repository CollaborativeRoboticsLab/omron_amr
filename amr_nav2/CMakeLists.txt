cmake_minimum_required(VERSION 3.8)
project(amr_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pad_slam_map.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(amr_core REQUIRED)
  find_package(amr_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(libaria CONFIG REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(nav2_velocity_smoother REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_program(AMR_CORE_EXECUTABLE NAMES amr_core
    PATHS "${amr_core_DIR}/../../../lib/amr_core" NO_DEFAULT_PATH REQUIRED)
  ament_add_gtest(test_nav2_integration test/test_nav2_integration.cpp)
  target_include_directories(test_nav2_integration PRIVATE ../amr_core/include ../amr_core/test)
  target_compile_definitions(test_nav2_integration PRIVATE AMR_CORE_EXECUTABLE="${AMR_CORE_EXECUTABLE}")
  ament_target_dependencies(test_nav2_integration
    amr_msgs geometry_msgs nav2_msgs nav2_velocity_smoother nav_msgs rclcpp rclcpp_action
    sensor_msgs std_msgs std_srvs tf2_msgs tf2_ros)
  target_link_libraries(test_nav2_integration libaria::ArNetworking nlohmann_json::nlohmann_json)
  ament_add_pytest_test(test_navigation_contract test/test_navigation_contract.py)
  ament_add_pytest_test(test_slam_map_padding test/test_slam_map_padding.py)
endif()

ament_package()
