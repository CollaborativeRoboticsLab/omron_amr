#pragma once

#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "amr_core/socket/socket_driver.hpp"

/**
 * @brief ROS 2 ARCL driver node (without direct socket code).
 * Uses SocketDriver for ARCL communication.
 */
class Driver
{
public:
  /**
   * @brief Constructor. Initializes publishers, subscribers, and parameters.
   * @param node Shared pointer to rclcpp::Node.
   */
  Driver(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketDriver> driver)
  {
    node_ = node;
    socket_driver_ = driver;
  }

  void initialize()
  {
    // Parameters
    publish_odom_ = node_->declare_parameter<bool>("driver.publish_odom", true);
    subscribe_cmd_vel_ = node_->declare_parameter<bool>("driver.subscribe_cmd_vel", true);
    odom_topic_ = node_->declare_parameter<std::string>("driver.odom_topic", "amr/odom");
    odom_frame_ = node_->declare_parameter<std::string>("driver.odom_frame", "odom");
    base_frame_ = node_->declare_parameter<std::string>("driver.base_frame", "base_link");
    cmd_vel_topic_ = node_->declare_parameter<std::string>("driver.cmd_vel_topic", "amr/cmd_vel");
    odom_reset_topic = node_->declare_parameter<std::string>("driver.odom_reset_topic", "amr/odomReset");
    min_ang_speed_ = node_->declare_parameter<int>("driver.min_angular_speed", -30);   // deg/s
    max_ang_speed_ = node_->declare_parameter<int>("driver.max_angular_speed", 30);    // deg/s
    min_lin_speed_ = node_->declare_parameter<int>("driver.min_linear_speed", -1000);  // mm/s
    max_lin_speed_ = node_->declare_parameter<int>("driver.max_linear_speed", 1000);   // mm/s
    timeout_ms = node_->declare_parameter<int>("driver.command_timeout_ms", 15000);

    // Publishers
    if (publish_odom_)
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("amr/odom", 10);

    // Subscribers
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "amr/stop", 10, std::bind(&Driver::stopCB, this, std::placeholders::_1));
    odom_reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "amr/odomReset", 10, std::bind(&Driver::odomResetCB, this, std::placeholders::_1));

    if (subscribe_cmd_vel_)
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "amr/cmd_vel", 10, std::bind(&Driver::cmdVelCB, this, std::placeholders::_1));
  }

  /**
   * @brief Update odometry based on incoming string message.
   * @param msg String message containing odometry data.
   *
   * Example input: "Odometer: <distance> mm <heading> deg <time> sec"
   */
  void update(const std_msgs::msg::String& msg)
  {
    // Prepare odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Parse the string for distance, heading, and time
    std::istringstream iss(msg.data);
    std::string token, previous_token;
    double distance = 0.0, heading = 0.0, time = 0.0;

    while (iss >> token)
    {
      if (token == "Odometer:")
      {
        previous_token = token;
        continue;
      }
      if (token == "mm")
      {
        previous_token = token;
        continue;
      }
      if (token == "deg")
      {
        previous_token = token;
        continue;
      }
      if (token == "sec")
      {
        previous_token = token;
        continue;
      }

      // Try to parse the next tokens as numbers
      if (previous_token == "Odometer:")
      {
        distance = std::stod(token);
        continue;
      }
      if (previous_token == "mm")
      {
        heading = std::stod(token);
        continue;
      }
      if (previous_token == "deg")
      {
        time = std::stod(token);
        continue;
      }
    }

    // Fill odometry message
    // Distance in mm, heading in deg
    odom_msg.pose.pose.position.x = distance / 1000.0;  // meters
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    // Convert heading to quaternion (heading is in degrees)
    double heading_rad = heading * M_PI / 180.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(heading_rad / 2.0);
    odom_msg.pose.pose.orientation.w = cos(heading_rad / 2.0);

    // Optionally fill velocity if time > 0
    if (time > 0.0)
    {
      odom_msg.twist.twist.linear.x = (distance / 1000.0) / time;  // m/s
      odom_msg.twist.twist.angular.z = (heading_rad) / time;       // rad/s
    }
    else
    {
      odom_msg.twist.twist.linear.x = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
    }

    if (publish_odom_ && odom_pub_)
      odom_pub_->publish(odom_msg);
  }

private:
  void stopCB(const std_msgs::msg::Empty& /*msg*/)
  {
    std::string response;
    int req_id = socket_driver_->queue_command("stop", "");
    bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
    if (!got_response)
      RCLCPP_INFO(node_->get_logger(), "Stopping result: %s", response.c_str());
  }

  void odomResetCB(const std_msgs::msg::Empty& /*msg*/)
  {
    std::string response;
    int req_id = socket_driver_->queue_command("odometerReset", "");
    bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
    if (!got_response)
      RCLCPP_INFO(node_->get_logger(), "Reset result: %s", response.c_str());
  }

  void cmdVelCB(const geometry_msgs::msg::Twist& msg)
  {
    // Example: convert Twist to ARCL commands (similar logic as before).
    int lin_m = static_cast<int>(msg.linear.x);
    int ang_rad = static_cast<int>(msg.angular.z);

    // additionally convert into mm/s and deg/s for ARCL commands since twist is in m/s and rad/s
    int lin = lin_m * 1000;                                   // m/s to mm/s
    int ang = static_cast<int>(ang_rad * (180.0 / 3.14159));  // rad/s to deg/s

    // Clamp values to min/max
    lin = std::clamp(lin, min_lin_speed_, max_lin_speed_);
    ang = std::clamp(ang, min_ang_speed_, max_ang_speed_);

    // Deadband
    if (std::abs(lin) < 20)
      lin = 0;
    if (std::abs(ang) < 5)
      ang = 0;

    if (lin == 0 && ang == 0)
    {
      stopCB(std_msgs::msg::Empty());
      return;
    }
    else if (std::abs(ang) > 0)
    {
      int target = (ang > 0) ? 1440 : -1440;
      int speed = std::abs(ang);

      std::string cmd = "dotask deltaheading " + std::to_string(target) + " " + std::to_string(speed);

      std::string response;
      int req_id = socket_driver_->queue_command(cmd, "");
      bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
      return;
    }
    else if (std::abs(lin) > 0)
    {
      int target = (lin > 0) ? 10000 : -10000;
      int speed = std::abs(lin);

      std::string cmd = "dotask move " + std::to_string(target) + " " + std::to_string(speed);

      std::string response;
      int req_id = socket_driver_->queue_command(cmd, "");
      bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
      return;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SocketDriver> socket_driver_;

  // Parameters
  bool publish_odom_{ true };
  bool subscribe_cmd_vel_{ true };
  std::string odom_topic_{ "amr/odom" };
  std::string odom_frame_{ "odom" };
  std::string base_frame_{ "base_link" };
  std::string cmd_vel_topic_{ "amr/cmd_vel" };
  std::string odom_reset_topic{ "amr/odomReset" };
  int timeout_ms{ 15000 };

  int min_lin_speed_{ -1000 };  // mm/s
  int max_lin_speed_{ 1000 };   // mm/s
  int min_ang_speed_{ -30 };    // deg/s
  int max_ang_speed_{ 30 };     // deg/s

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_reset_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};
