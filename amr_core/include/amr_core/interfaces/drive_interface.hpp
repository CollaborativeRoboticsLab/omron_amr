#pragma once

#include <ArNetworking/ArClientRatioDrive.h>
#include <ArNetworking/ArNetworking.h>

#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "amr_core/utils/libaria_runtime.hpp"

/**
 * @brief ROS 2 drive interface backed by libaria/ArNetworking.
 */
class DriverInterface
{
public:
  /**
   * @brief Constructor. Initializes publishers, subscribers, and libaria callbacks.
   * @param node Shared pointer to rclcpp::Node.
   */
  DriverInterface(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), ratio_drive_(&client_), update_callback_(this, &DriverInterface::handleUpdateNumbers)
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  ~DriverInterface()
  {
    client_.disconnect();
  }

  /**
   * @brief Initializes parameters, publishers, and subscribers.
   */
  void initialize()
  {
    host_ = getOrDeclareParameter<std::string>("robot.ip", "127.0.0.1");
    port_ = getOrDeclareParameter<int>("robot.port", 7272);
    user_ = getOrDeclareParameter<std::string>("robot.user", "admin");
    password_ = getOrDeclareParameter<std::string>("robot.password", "");
    protocol_ = getOrDeclareParameter<std::string>("robot.protocol", "6MTX");

    odom_topic_ = getOrDeclareParameter<std::string>("driver.odom_topic", "amr/odom");
    cmd_vel_topic_ = getOrDeclareParameter<std::string>("driver.cmd_vel_topic", "amr/cmd_vel");

    odom_frame_ = getOrDeclareParameter<std::string>("driver.odom_frame", "amr/odom");
    base_frame_ = getOrDeclareParameter<std::string>("driver.base_frame", "amr/base_link");

    min_ang_speed_ = getOrDeclareParameter<double>("driver.min_angular_speed", -60.0);  // deg/s
    max_ang_speed_ = getOrDeclareParameter<double>("driver.max_angular_speed", 60.0);   // deg/s
    min_lin_speed_ = getOrDeclareParameter<double>("driver.min_linear_speed", -200.0);  // mm/s
    max_lin_speed_ = getOrDeclareParameter<double>("driver.max_linear_speed", 1200.0);  // mm/s
    drive_throttle_pct_ = getOrDeclareParameter<double>("driver.drive_throttle_pct", 100.0);
    unsafe_drive_ = getOrDeclareParameter<bool>("driver.unsafe_drive", true);
    cmd_vel_timeout_sec_ = getOrDeclareParameter<double>("driver.cmd_vel_timeout_sec", 0.2);

    // Publishers
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // Subscribers
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "amr/stop", 10, std::bind(&DriverInterface::stopCB, this, std::placeholders::_1));

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, std::bind(&DriverInterface::cmdVelCB, this, std::placeholders::_1));

    connectClient();
    configureHandlers();
    ratio_drive_.setThrottle(drive_throttle_pct_);
    configureDriveMode();
    client_.runAsync();

    odomReset();

    const int watchdog_ms = std::max(50, static_cast<int>(1000.0 / std::max(expected_cmd_vel_freq_, 1.0)));
    cmd_vel_watchdog_timer_ = node_->create_wall_timer(std::chrono::milliseconds(watchdog_ms),
                                                       std::bind(&DriverInterface::handleCmdVelWatchdog, this));
  }

private:
  /**
   * @brief Helper function to get a parameter value or declare it with a default if it doesn't exist.
   * @tparam T Parameter type.
   * @param name Parameter name.
   * @param default_value Default value to declare if parameter doesn't exist.
   * @return Parameter value.
   */
  template <typename T>
  T getOrDeclareParameter(const std::string& name, const T& default_value)
  {
    if (!node_->has_parameter(name))
    {
      return node_->declare_parameter<T>(name, default_value);
    }

    T value = default_value;
    node_->get_parameter(name, value);
    return value;
  }

  /**
   * @brief Connects to the robot server using libaria and handles connection errors.
   * @throws std::runtime_error if connection fails or is rejected by the robot.
   */
  void connectClient()
  {
    if (!protocol_.empty())
    {
      client_.enforceProtocolVersion(protocol_.c_str());
    }

    const char* password = password_.empty() ? nullptr : password_.c_str();
    if (!client_.blockingConnect(host_.c_str(), port_, true, user_.c_str(), password))
    {
      if (client_.wasRejected())
      {
        throw std::runtime_error("Robot rejected the drive interface connection.");
      }
      throw std::runtime_error("Could not connect drive interface to Omron robot server.");
    }

    client_.setRobotName(host_.c_str());
    RCLCPP_INFO(node_->get_logger(), "DriverInterface connected to %s:%d using libaria", host_.c_str(), port_);
  }

  /**
   * @brief Configures handlers for the robot server.
   */
  void configureHandlers()
  {
    if (!client_.dataExists("updateNumbers"))
    {
      RCLCPP_WARN(node_->get_logger(), "Server does not advertise updateNumbers; odometry will not update");
      return;
    }

    client_.addHandler("updateNumbers", &update_callback_);
    client_.request("updateNumbers", 50);
  }

  /**
   * @brief Configures the drive mode for the robot.
   */
  void configureDriveMode()
  {
    if (!client_.dataExists("setSafeDrive"))
    {
      RCLCPP_WARN(node_->get_logger(), "Server does not advertise setSafeDrive; leaving drive mode unchanged");
      return;
    }

    if (unsafe_drive_)
    {
      ratio_drive_.unsafeDrive();
      RCLCPP_WARN(node_->get_logger(), "Unsafe drive enabled for cmd_vel control");
    }
    else
    {
      ratio_drive_.safeDrive();
      RCLCPP_INFO(node_->get_logger(), "Safe drive enabled for cmd_vel control");
    }
  }

  /**
   * @brief Handles update numbers from the robot server.
   * @param packet The packet containing the update numbers.
   */
  void handleUpdateNumbers(ArNetPacket* packet)
  {
    packet->bufToByte2();
    const double x = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double y = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double theta_deg = static_cast<double>(packet->bufToByte2());
    const double x_vel_mps = static_cast<double>(packet->bufToByte2()) / 1000.0;
    const double theta_vel_rad_s = static_cast<double>(packet->bufToByte2()) * M_PI / 180.0;
    const double y_vel_mps = static_cast<double>(packet->bufToByte2()) / 1000.0;
    packet->bufToByte();

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose_x_ = x;
      pose_y_ = y;
      pose_theta_deg_ = theta_deg;
      have_pose_ = true;
    }

    if (!publish_odom_ || !odom_pub_)
    {
      return;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    const double theta_rad = theta_deg * M_PI / 180.0;
    odom_msg.pose.pose.orientation.z = std::sin(theta_rad / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta_rad / 2.0);
    odom_msg.twist.twist.linear.x = x_vel_mps;
    odom_msg.twist.twist.linear.y = y_vel_mps;
    odom_msg.twist.twist.angular.z = theta_vel_rad_s;
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void stopCB(const std_msgs::msg::Empty& /*msg*/)
  {
    cmd_vel_active_ = false;
    stop_sent_ = true;
    ratio_drive_.stop();
    if (client_.dataExists("stop"))
    {
      client_.requestOnce("stop");
    }
  }

  void odomReset()
  {
    static const std::vector<std::string> reset_requests = { "ResetTripOdometer", "TripReset" };
    for (const auto& request : reset_requests)
    {
      if (!client_.dataExists(request.c_str()))
      {
        continue;
      }

      client_.requestOnce(request.c_str());
      RCLCPP_INFO(node_->get_logger(), "Requested odometry reset using %s", request.c_str());
      return;
    }

    RCLCPP_WARN(node_->get_logger(), "Server does not advertise a known odometry reset request");
  }

  void cmdVelCB(const geometry_msgs::msg::Twist& msg)
  {
    last_cmd_vel_time_ = node_->now();
    stop_sent_ = false;
    cmd_vel_active_ = std::fabs(msg.linear.x) > 1e-3 || std::fabs(msg.angular.z) > 1e-3;
    if (!cmd_vel_active_)
    {
      ratio_drive_.stop();
      return;
    }

    if (!client_.dataExists("ratioDrive"))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Server does not advertise ratioDrive");
      return;
    }

    const double max_linear_mps = std::max(std::fabs(min_lin_speed_), std::fabs(max_lin_speed_)) / 1000.0;
    const double max_angular_rad_s = std::max(std::fabs(min_ang_speed_), std::fabs(max_ang_speed_)) * M_PI / 180.0;
    const double trans_ratio = toPercent(msg.linear.x, max_linear_mps);
    const double rot_ratio = toPercent(msg.angular.z, max_angular_rad_s);

    ratio_drive_.setThrottle(drive_throttle_pct_);
    ratio_drive_.setLatVelRatio(0.0);
    ratio_drive_.setTransVelRatio(trans_ratio);
    ratio_drive_.setRotVelRatio(rot_ratio);
  }

  void handleCmdVelWatchdog()
  {
    if (!cmd_vel_active_)
    {
      return;
    }

    if ((node_->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_)
    {
      cmd_vel_active_ = false;
      if (!stop_sent_)
      {
        ratio_drive_.stop();
        stop_sent_ = true;
      }
    }
  }

  static double toPercent(double value, double max_abs_value)
  {
    if (max_abs_value <= 0.0)
    {
      return 0.0;
    }

    const double percent = (value / max_abs_value) * 100.0;
    return std::clamp(percent, -100.0, 100.0);
  }

private:
  rclcpp::Node::SharedPtr node_;
  amr_core::LibAriaRuntime aria_runtime_;
  ArClientBase client_;
  ArClientRatioDrive ratio_drive_;
  ArFunctor1C<DriverInterface, ArNetPacket*> update_callback_;

  // Parameters
  std::string host_;
  int port_{ 7272 };
  std::string user_;
  std::string password_;
  std::string protocol_;
  bool publish_odom_{ true };
  bool subscribe_cmd_vel_{ true };
  bool subscribe_goal_pose_{ true };
  bool subscribe_initial_pose_{ true };
  bool subscribe_local_plan_{ false };

  std::string odom_topic_{ "amr/odom" };
  std::string cmd_vel_topic_{ "amr/cmd_vel" };
  std::string odom_reset_topic_{ "amr/odomReset" };
  std::string goal_pose_topic_{ "goal_pose" };
  std::string initial_pose_topic_{ "initialpose" };

  std::string odom_frame_{ "odom" };
  std::string base_frame_{ "base_link" };

  double expected_cmd_vel_freq_{ 20.0 };
  double min_lin_speed_{ -1000.0 };
  double max_lin_speed_{ 1000.0 };
  double min_ang_speed_{ -30.0 };
  double max_ang_speed_{ 30.0 };
  double drive_throttle_pct_{ 100.0 };
  double cmd_vel_timeout_sec_{ 0.2 };
  bool unsafe_drive_{ true };

  std::mutex pose_mutex_;
  double pose_x_{ 0.0 };
  double pose_y_{ 0.0 };
  double pose_theta_deg_{ 0.0 };
  bool have_pose_{ false };
  bool cmd_vel_active_{ false };
  bool stop_sent_{ false };
  rclcpp::Time last_cmd_vel_time_{ 0, 0, RCL_ROS_TIME };

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr odom_reset_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr cmd_vel_watchdog_timer_;
};
