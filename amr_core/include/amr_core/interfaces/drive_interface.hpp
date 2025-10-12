#pragma once

#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "amr_core/socket/socket_driver.hpp"

/**
 * @brief ROS 2 ARCL driver node (without direct socket code).
 * Uses SocketDriver for ARCL communication.
 */
class DriverInterface
{
public:
  /**
   * @brief Constructor. Initializes publishers, subscribers, and parameters.
   * @param node Shared pointer to rclcpp::Node.
   */
  DriverInterface(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketDriver> driver)
  {
    node_ = node;
    socket_driver_ = driver;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
  /**
   * @brief Initializes parameters, publishers, and subscribers.
   */
  void initialize()
  {
    // Parameters
    publish_odom_ = node_->declare_parameter<bool>("driver.publish_odom", true);
    subscribe_cmd_vel_ = node_->declare_parameter<bool>("driver.subscribe_cmd_vel", true);
    subscribe_goal_pose_ = node_->declare_parameter<bool>("driver.subscribe_goal_pose", true);
    subsrcibe_initial_pose_ = node_->declare_parameter<bool>("driver.subscribe_initial_pose", true);

    odom_topic_ = node_->declare_parameter<std::string>("driver.odom_topic", "amr/odom");
    cmd_vel_topic_ = node_->declare_parameter<std::string>("driver.cmd_vel_topic", "amr/cmd_vel");
    odom_reset_topic = node_->declare_parameter<std::string>("driver.odom_reset_topic", "amr/odomReset");
    goal_pose_topic_ = node_->declare_parameter<std::string>("driver.goal_pose_topic", "goal_pose");
    initial_pose_topic_ = node_->declare_parameter<std::string>("driver.initial_pose_topic", "initialpose");

    odom_frame_ = node_->declare_parameter<std::string>("driver.odom_frame", "odom");
    base_frame_ = node_->declare_parameter<std::string>("driver.base_frame", "base_link");

    expected_cmd_vel_freq_ = node_->declare_parameter<double>("driver.expected_cmd_vel_freq", 20.0);
    min_ang_speed_ = node_->declare_parameter<double>("driver.min_angular_speed", -30);   // deg/s
    max_ang_speed_ = node_->declare_parameter<double>("driver.max_angular_speed", 30);    // deg/s
    min_lin_speed_ = node_->declare_parameter<double>("driver.min_linear_speed", -1000);  // mm/s
    max_lin_speed_ = node_->declare_parameter<double>("driver.max_linear_speed", 1000);   // mm/s

    unit_move_distance_ = node_->declare_parameter<double>("driver.unit_move_distance", 100.0);  // mm
    unit_turn_angle_ = node_->declare_parameter<double>("driver.unit_turn_angle", 10.0);         // deg

    // Publishers
    if (publish_odom_)
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("amr/odom", 10);

    // Subscribers
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "amr/stop", 10, std::bind(&DriverInterface::stopCB, this, std::placeholders::_1));
    odom_reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "amr/odomReset", 10, std::bind(&DriverInterface::odomResetCB, this, std::placeholders::_1));

    if (subscribe_cmd_vel_)
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "amr/cmd_vel", 10, std::bind(&DriverInterface::cmdVelCB, this, std::placeholders::_1));

    if (subscribe_goal_pose_)
      goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_pose_topic_, 10, std::bind(&DriverInterface::goalPoseCB, this, std::placeholders::_1));

    if (subsrcibe_initial_pose_)
      initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initial_pose_topic_, 10, std::bind(&DriverInterface::initialPoseCB, this, std::placeholders::_1));
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
    odom_msg.pose.pose.position.x = distance / 1000.0;  // meters
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    double heading_rad = heading * M_PI / 180.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(heading_rad / 2.0);
    odom_msg.pose.pose.orientation.w = cos(heading_rad / 2.0);

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

    // Publish odometry
    if (publish_odom_ && odom_pub_)
      odom_pub_->publish(odom_msg);

    // Broadcast odom -> base_link TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
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
    // Convert into mm/s and deg/s for ARCL commands since twist is in m/s and rad/s
    double lin = msg.linear.x * 1000;                // m/s to mm/s
    double ang = msg.angular.z * (180.0 / 3.14159);  // rad/s to deg/s

    // Clamp values to min/max
    lin = std::clamp(lin, min_lin_speed_, max_lin_speed_);
    ang = std::clamp(ang, min_ang_speed_, max_ang_speed_);

    // Deadband
    if (std::abs(lin) < 20)
      lin = 0;
    if (std::abs(ang) < 5)
      ang = 0;

    // If angular velocity is non-zero, send turn command first
    if (std::abs(ang) > 0)
    {
      int angle = (ang > 0) ? unit_turn_angle_ : -unit_turn_angle_;
      int speed = std::abs(ang);

      std::string cmd = "doTask deltaheading " + std::to_string(angle) + " " + std::to_string(speed);

      int idx_ = socket_driver_->queue_command(cmd, "Completed doing task deltaHeading");
      bool got = socket_driver_->wait_for_response(idx_, response_, timeout_ms);
      if (!got)
        RCLCPP_ERROR(node_->get_logger(), "Robot did not respond to angular cmd_vel command: %s", response_.c_str());
    }

    // IF linear velocity is non-zero, send move command
    if (std::abs(lin) > 0)
    {
      int distance = (lin > 0) ? unit_move_distance_ : -unit_move_distance_;
      int speed = std::abs(lin);

      std::string cmd = "doTask move " + std::to_string(distance) + " " + std::to_string(speed);

      int idx_ = socket_driver_->queue_command(cmd, "Completed doing task move");
      bool got = socket_driver_->wait_for_response(idx_, response_, timeout_ms);
      if (!got)
        RCLCPP_ERROR(node_->get_logger(), "Robot did not respond to linear cmd_vel command: %s", response_.c_str());
    }
  }

  // Pose-based commands (from subscriptions)
  void goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto& p = msg->pose.position;
    const auto& q = msg->pose.orientation;

    int x_mm = static_cast<int>(std::lround(p.x * 1000.0));
    int y_mm = static_cast<int>(std::lround(p.y * 1000.0));
    int deg = static_cast<int>(std::lround(yaw_deg(q.w, q.x, q.y, q.z)));
    if (deg > 180)
      deg -= 360;

    std::string cmd =
        "doTask gotoPoint " + std::to_string(x_mm) + " " + std::to_string(y_mm) + " " + std::to_string(deg);
    std::string identifier = { "Going to point" };

    std::string response;
    int req_id = socket_driver_->queue_command(cmd, identifier);
    bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
    return;
  }

  void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;

    int x_mm = static_cast<int>(std::lround(p.x * 1000.0));
    int y_mm = static_cast<int>(std::lround(p.y * 1000.0));
    int deg = static_cast<int>(std::lround(yaw_deg(q.w, q.x, q.y, q.z)));
    if (deg > 180)
      deg -= 360;

    // xySpread=0 angleSpread=0
    std::string cmd =
        "localizetopoint " + std::to_string(x_mm) + " " + std::to_string(y_mm) + " " + std::to_string(deg) + " 0 0";
    std::string identifier = { "Localizing at point" };

    std::string response;
    int req_id = socket_driver_->queue_command(cmd, identifier);
    bool got_response = socket_driver_->wait_for_response(req_id, response, timeout_ms);
    return;
  }

  static double yaw_deg(double w, double x, double y, double z)
  {
    const double t3 = +2.0 * (w * z + x * y);
    const double t4 = +1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(t3, t4);  // radians
    return yaw * 180.0 / M_PI;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SocketDriver> socket_driver_;

  // Parameters
  bool publish_odom_{ true };
  bool subscribe_cmd_vel_{ true };
  bool subscribe_goal_pose_{ true };
  bool subsrcibe_initial_pose_{ true };

  std::string odom_topic_{ "amr/odom" };
  std::string cmd_vel_topic_{ "amr/cmd_vel" };
  std::string odom_reset_topic{ "amr/odomReset" };
  std::string goal_pose_topic_{ "goal_pose" };
  std::string initial_pose_topic_{ "initialpose" };

  std::string odom_frame_{ "odom" };
  std::string base_frame_{ "base_link" };
  int timeout_ms{ 15000 };

  int expected_cmd_vel_freq_{ 20 };  // Hz
  double min_lin_speed_{ -1000 };    // mm/s
  double max_lin_speed_{ 1000 };     // mm/s
  double min_ang_speed_{ -30 };      // deg/s
  double max_ang_speed_{ 30 };       // deg/s

  int unit_move_distance_{ 50 };  // mm
  int unit_turn_angle_{ 5 };      // deg

  std::string response_;
  std::mutex cmd_mutex_;

  int avg_lin_speed_{ 0 };
  int avg_ang_speed_{ 0 };
  int iterations_{ 0 };

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_reset_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
