#pragma once

#include <ArNetworking/ArNetworking.h>

#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <stdexcept>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "amr_core/utils/drive_command_policy.hpp"
#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/drive_ratio_mapping.hpp"
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
    : node_(std::move(node)), update_callback_(this, &DriverInterface::handleUpdateNumbers),
      drive_callback_(this, &DriverInterface::handleDriveCycle),
      mode_callback_(this, &DriverInterface::handleDriveMode)
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  ~DriverInterface()
  {
    if (network_started_)
    {
      client_.stopRunning();
      client_.join();
    }
    if (cycle_registered_)
    {
      client_.remCycleCallback(&drive_callback_);
    }
    if (client_.isConnected() && client_.dataExists("stop"))
    {
      client_.requestOnce("stop");
    }
    client_.disconnect();
  }

  /**
   * @brief Initializes parameters, publishers, and subscribers.
   *
   * This should be called after the node is fully constructed.
   * @param host Robot server IP address.
   * @param port Robot server port.
   * @param user Username for robot server authentication.
   * @param password Password for robot server authentication.
   * @param protocol Optional protocol version to enforce on the robot server connection.
   *
   * Throws std::runtime_error if connection to the robot server fails or is rejected by the robot.
   */
  void initialize(const std::string& host, int port, const std::string& user, const std::string& password,
                  const std::string& protocol)
  {
    host_ = host;
    port_ = port;
    user_ = user;
    password_ = password;
    protocol_ = protocol;

    odom_topic_ = getOrDeclareParameter<std::string>("driver.odom_topic", "amr/odom");
    cmd_vel_topic_ = getOrDeclareParameter<std::string>("driver.cmd_vel_topic", "amr/cmd_vel");
    stop_topic_ = getOrDeclareParameter<std::string>("driver.stop_topic", "amr/stop");
    publish_odom_ = getOrDeclareParameter<bool>("driver.publish_odom", true);
    publish_robot_tf_ = getOrDeclareParameter<bool>("driver.publish_robot_tf", true);

    odom_frame_ = getOrDeclareParameter<std::string>("driver.odom_frame", "amr/odom");
    base_frame_ = getOrDeclareParameter<std::string>("driver.base_frame", "amr/base_link");

    expected_cmd_vel_freq_ = getOrDeclareParameter<double>("driver.expected_cmd_vel_freq", 20.0);
    drive_send_frequency_ = getOrDeclareParameter<double>("driver.send_frequency", 20.0);
    min_ang_speed_ = getOrDeclareParameter<double>("driver.min_angular_speed", -60.0);  // deg/s
    max_ang_speed_ = getOrDeclareParameter<double>("driver.max_angular_speed", 60.0);   // deg/s
    min_lin_speed_ = getOrDeclareParameter<double>("driver.min_linear_speed", -100.0);  // mm/s
    max_lin_speed_ = getOrDeclareParameter<double>("driver.max_linear_speed", 1000.0);  // mm/s
    drive_throttle_pct_ = getOrDeclareParameter<double>("driver.drive_throttle_pct", 100.0);
    unsafe_drive_ = getOrDeclareParameter<bool>("driver.unsafe_drive", true);
    cmd_vel_timeout_sec_ = getOrDeclareParameter<double>("driver.cmd_vel_timeout_sec", 0.2);
    mode_check_period_sec_ = getOrDeclareParameter<double>("driver.mode_check_period_sec", 0.5);
    mode_confirmation_timeout_sec_ = getOrDeclareParameter<double>("driver.mode_confirmation_timeout_sec", 2.0);
    if (!std::isfinite(min_lin_speed_) || !std::isfinite(max_lin_speed_) ||
        !std::isfinite(min_ang_speed_) || !std::isfinite(max_ang_speed_) ||
        min_lin_speed_ > 0.0 || max_lin_speed_ <= 0.0 ||
        min_ang_speed_ >= 0.0 || max_ang_speed_ <= 0.0 ||
        !std::isfinite(drive_throttle_pct_) || drive_throttle_pct_ < 0.0 || drive_throttle_pct_ > 100.0)
    {
      throw std::invalid_argument("Invalid driver speed limits or throttle percentage");
    }
    if (!std::isfinite(mode_check_period_sec_) || mode_check_period_sec_ <= 0.0 ||
        !std::isfinite(mode_confirmation_timeout_sec_) || mode_confirmation_timeout_sec_ <= mode_check_period_sec_)
    {
      throw std::invalid_argument("Drive mode timeout must be finite and greater than the positive check period");
    }
    command_policy_ = std::make_unique<amr_core::DriveCommandPolicy>(drive_send_frequency_, cmd_vel_timeout_sec_);
    configureRatioMapping();

    const auto queue_packets = getOrDeclareParameter<int64_t>("driver.tcp_queue_max_packets", 32);
    const auto queue_bytes = getOrDeclareParameter<int64_t>("driver.tcp_queue_max_bytes", 65536);
    const auto queue_age = getOrDeclareParameter<double>("driver.tcp_queue_max_age_sec", 0.1);
    if (queue_packets <= 0 || queue_bytes <= 0 || !std::isfinite(queue_age) || queue_age <= 0.0 ||
        queue_age > cmd_vel_timeout_sec_ ||
        !client_.configureTransport(static_cast<std::size_t>(queue_packets), static_cast<std::size_t>(queue_bytes),
                                    queue_age))
    {
      throw std::invalid_argument("Invalid TCP queue limits: age must not exceed command timeout");
    }
    client_.setTransportFailureHandler([this]() { failDriveMode("Drive TCP transport failed"); });

    // Publishers
    if (publish_odom_)
    {
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    }

    // Subscribers
    stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        stop_topic_, 10, std::bind(&DriverInterface::stopCB, this, std::placeholders::_1));

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&DriverInterface::cmdVelCB, this, std::placeholders::_1));

    connectClient();
    if (!client_.dataExists("ratioDrive") || !client_.dataExists("stop"))
    {
      throw std::runtime_error("Robot must advertise ratioDrive and stop for velocity control");
    }
    configureHandlers();
    configureDriveMode();
    client_.addCycleCallback(&drive_callback_);
    cycle_registered_ = true;
    client_.runAsync();
    network_started_ = true;
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

  void configureRatioMapping()
  {
    const auto mode = getOrDeclareParameter<std::string>("driver.ratio_mapping", "legacy");
    if (mode == "legacy")
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Legacy ratio mapping is uncalibrated: local limits are not firmware full-throttle speeds");
      return;
    }
    if (mode != "aria_ratio_v1")
    {
      throw std::invalid_argument("driver.ratio_mapping must be legacy or aria_ratio_v1");
    }
    const auto confirmed = getOrDeclareParameter<bool>("driver.calibration.confirmed", false);
    const auto source = getOrDeclareParameter<std::string>("driver.calibration.source", "");
    if (!confirmed || source.find_first_not_of(" \t\r\n") == std::string::npos)
    {
      throw std::invalid_argument("Calibrated mapping requires a confirmed firmware contract and calibration source");
    }
    amr_core::DriveRatioMapping::Calibration calibration{
      getOrDeclareParameter<double>("driver.calibration.full_throttle_forward_mm_s", 0.0),
      getOrDeclareParameter<double>("driver.calibration.full_throttle_reverse_mm_s", 0.0),
      getOrDeclareParameter<double>("driver.calibration.rotation_stopped_deg_s", 0.0),
      getOrDeclareParameter<double>("driver.calibration.rotation_full_forward_deg_s", 0.0),
      getOrDeclareParameter<double>("driver.calibration.rotation_full_reverse_deg_s", 0.0),
      getOrDeclareParameter<double>("driver.calibration.translation_dead_zone_pct", -1.0),
      getOrDeclareParameter<double>("driver.calibration.rotation_dead_zone_pct", -1.0),
      getOrDeclareParameter<double>("driver.calibration.feedback_timeout_sec", 0.2)
    };
    ratio_mapping_ = std::make_unique<amr_core::DriveRatioMapping>(calibration);
    RCLCPP_INFO(node_->get_logger(), "Using opt-in aria_ratio_v1 calibration: %s", source.c_str());
  }

  /**
   * @brief Configures handlers for the robot server.
   */
  void configureHandlers()
  {
    if (!client_.dataExists("updateNumbers"))
    {
      if (ratio_mapping_)
      {
        throw std::runtime_error("Calibrated mapping requires updateNumbers body-speed feedback");
      }
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
    if (!client_.dataExists("setSafeDrive") || !client_.dataExists("getSafeDrive"))
    {
      throw std::runtime_error("Server must advertise setSafeDrive and getSafeDrive to verify configured control mode");
    }

    if (!client_.addHandler("getSafeDrive", &mode_callback_))
    {
      throw std::runtime_error("Could not register drive-mode confirmation handler");
    }
    ArNetPacket packet;
    packet.byteToBuf(unsafe_drive_ ? 0 : 1);
    if (!client_.requestOnce("setSafeDrive", &packet))
    {
      throw std::runtime_error("Could not queue the configured drive-mode request");
    }
    last_mode_response_ = amr_core::DriveCommandPolicy::Clock::now();
    requestDriveMode(last_mode_response_);
    const auto started = last_mode_response_;
    while (client_.isConnected() && !mode_confirmed_.load() && !mode_failed_ &&
           std::chrono::duration<double>(amr_core::DriveCommandPolicy::Clock::now() - started).count() <
               mode_confirmation_timeout_sec_)
    {
      client_.loopOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!mode_confirmed_.load())
    {
      throw std::runtime_error("Robot did not confirm the requested drive mode; movement inhibited");
    }
    RCLCPP_INFO(node_->get_logger(), "Robot confirmed %s drive; periodic mode monitoring enabled",
                unsafe_drive_ ? "unsafe" : "safe");
  }

  void failDriveMode(const char* reason)
  {
    mode_confirmed_.store(false);
    command_policy_->requestStop();
    if (!mode_failed_)
    {
      mode_failed_ = true;
      RCLCPP_ERROR(node_->get_logger(), "%s; movement inhibited until driver restart", reason);
    }
  }

  void requestDriveMode(amr_core::DriveCommandPolicy::TimePoint now)
  {
    last_mode_query_ = now;
    mode_query_pending_ = true;
    if (!client_.requestOnce("getSafeDrive", nullptr, true))
    {
      failDriveMode("Failed to queue drive-mode query");
    }
  }

  void handleDriveMode(ArNetPacket* packet)
  {
    if (mode_failed_ || !mode_query_pending_)
    {
      return;
    }
    mode_query_pending_ = false;
    const auto now = amr_core::DriveCommandPolicy::Clock::now();
    if (std::chrono::duration<double>(now - last_mode_response_).count() >= mode_confirmation_timeout_sec_)
    {
      failDriveMode("Late drive-mode confirmation");
      return;
    }
    if (packet->getDataLength() != packet->getDataReadLength() + 1)
    {
      failDriveMode("Malformed drive-mode confirmation");
      return;
    }
    const auto safe_mode = packet->bufToUByte();
    if (!packet->isValid() || safe_mode > 1 || safe_mode != (unsafe_drive_ ? 0 : 1))
    {
      failDriveMode("Robot drive mode does not match the requested mode");
      return;
    }
    last_mode_response_ = now;
    mode_confirmed_.store(true);
  }

  /**
   * @brief Handles update numbers from the robot server.
   * @param packet The packet containing the update numbers.
   */
  void handleUpdateNumbers(ArNetPacket* packet)
  {
    if (packet == nullptr || !packet->isValid() || packet->getDataLength() < packet->getDataReadLength() + 19)
    {
      body_feedback_.valid = false;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Rejected truncated or invalid updateNumbers packet; speed feedback invalidated");
      return;
    }
    // The packet structure is assumed to be:
    // byte: update type (ignored)
    // byte4: x in mm
    // byte4: y in mm
    // byte2: theta in degrees
    // byte2: x velocity in mm/s
    // byte2: theta velocity in deg/s
    // byte2: y velocity in mm/s

    packet->bufToByte2();
    const double x = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double y = static_cast<double>(packet->bufToByte4()) / 1000.0;
    const double theta_deg = static_cast<double>(packet->bufToByte2());
    const double x_vel_mps = static_cast<double>(packet->bufToByte2()) / 1000.0;
    const double theta_vel_rad_s = static_cast<double>(packet->bufToByte2()) * M_PI / 180.0;
    const double y_vel_mps = static_cast<double>(packet->bufToByte2()) / 1000.0;
    packet->bufToByte();

    if (!packet->isValid())
    {
      body_feedback_.valid = false;
      return;
    }
    body_feedback_ = { x_vel_mps, 0.0, true };
    body_feedback_received_ = amr_core::DriveCommandPolicy::Clock::now();

    // Update internal pose state within a mutex to ensure thread safety with the ROS callbacks that may access the pose.
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose_x_ = x;
      pose_y_ = y;
      pose_theta_deg_ = theta_deg;
      have_pose_ = true;
    }

    // Capture the first pose as the odom origin. Subsequent updates are transformed into that initial frame,
    // so the published odom is a proper relative pose even when the robot starts with a non-zero heading.
    if (!initial_pose_set_)
    {
      initial_pose_x_ = pose_x_;
      initial_pose_y_ = pose_y_;
      initial_pose_theta_deg_ = pose_theta_deg_;
      initial_pose_set_ = true;
      return;
    }

    const double initial_theta_rad = initial_pose_theta_deg_ * M_PI / 180.0;
    const double delta_x = pose_x_ - initial_pose_x_;
    const double delta_y = pose_y_ - initial_pose_y_;
    const double cos_initial = std::cos(initial_theta_rad);
    const double sin_initial = std::sin(initial_theta_rad);

    const double relative_x = cos_initial * delta_x + sin_initial * delta_y;
    const double relative_y = -sin_initial * delta_x + cos_initial * delta_y;
    const double relative_theta_rad = std::atan2(std::sin((pose_theta_deg_ - initial_pose_theta_deg_) * M_PI / 180.0),
                                                 std::cos((pose_theta_deg_ - initial_pose_theta_deg_) * M_PI / 180.0));
    theta_rad_ = relative_theta_rad;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = relative_x;
    odom_msg.pose.pose.position.y = relative_y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.z = std::sin(theta_rad_ / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta_rad_ / 2.0);

    odom_msg.twist.twist.linear.x = x_vel_mps;
    odom_msg.twist.twist.linear.y = y_vel_mps;
    odom_msg.twist.twist.angular.z = theta_vel_rad_s;

    if (publish_odom_ && odom_pub_)
    {
      odom_pub_->publish(odom_msg);
    }

    if (publish_robot_tf_)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = odom_msg.header;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = relative_x;
      tf_msg.transform.translation.y = relative_y;
      tf_msg.transform.translation.z = 0.0;

      tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  /**
   * @brief Callback for stop messages. Stops the robot immediately.
   * @param msg The received stop message (empty).
   */
  void stopCB(const std_msgs::msg::Empty& /*msg*/)
  {
    command_policy_->requestStop();
    RCLCPP_INFO(node_->get_logger(), "Explicit drive stop requested");
  }

  /**
   * @brief Callback for cmd_vel messages. Converts Twist messages to drive commands and sends them to the robot.
   * Also updates the last command time for the watchdog.
   * 
   * @param msg The received cmd_vel message.
   */
  void cmdVelCB(const geometry_msgs::msg::Twist& msg)
  {
    if (!mode_confirmed_.load())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Ignored cmd_vel: configured drive mode is not confirmed");
      return;
    }
    if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.y) || !std::isfinite(msg.linear.z) ||
        !std::isfinite(msg.angular.x) || !std::isfinite(msg.angular.y) || !std::isfinite(msg.angular.z) ||
        std::fabs(msg.linear.y) > 1e-6 || std::fabs(msg.linear.z) > 1e-6 ||
        std::fabs(msg.angular.x) > 1e-6 || std::fabs(msg.angular.y) > 1e-6)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Rejected invalid or unsupported cmd_vel; command freshness was not updated");
      return;
    }

    if (!command_policy_->accept({ msg.linear.x, msg.angular.z }, amr_core::DriveCommandPolicy::Clock::now()))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Ignored cmd_vel while a stop request is awaiting local submission; send fresh input afterward");
    }
  }

  void handleDriveCycle()
  {
    if (!client_.isConnected())
    {
      failDriveMode("Drive connection lost");
      return;
    }

    const auto now = amr_core::DriveCommandPolicy::Clock::now();
    if (!mode_failed_)
    {
      if (std::chrono::duration<double>(now - last_mode_response_).count() >= mode_confirmation_timeout_sec_)
      {
        failDriveMode("Drive-mode confirmation timed out");
      }
      else if (!mode_query_pending_ &&
               std::chrono::duration<double>(now - last_mode_query_).count() >= mode_check_period_sec_)
      {
        requestDriveMode(now);
      }
    }
    const auto decision = command_policy_->poll(now);
    if (decision.action == amr_core::DriveCommandPolicy::Action::Stop)
    {
      const bool queued = client_.requestOnce("stop", nullptr, true);
      command_policy_->completeStopAttempt(queued);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Drive stop submission (explicit stop or expired command): %s",
                           queued ? "queued locally; delivery not acknowledged" : "failed; retry pending");
    }
    else if (decision.action == amr_core::DriveCommandPolicy::Action::Drive && mode_confirmed_.load())
    {
      const double max_linear_mps = std::max(std::fabs(min_lin_speed_), std::fabs(max_lin_speed_)) / 1000.0;
      const double max_angular_rad_s = std::max(std::fabs(min_ang_speed_), std::fabs(max_ang_speed_)) * M_PI / 180.0;
      const double linear = std::clamp(decision.command.linear, min_lin_speed_ / 1000.0, max_lin_speed_ / 1000.0);
      const double angular = std::clamp(decision.command.angular, min_ang_speed_ * M_PI / 180.0,
                                      max_ang_speed_ * M_PI / 180.0);
      double translation_pct = drive_throttle_pct_ == 0.0 ? 0.0 : toPercent(linear, max_linear_mps);
      double rotation_pct = drive_throttle_pct_ == 0.0 ? 0.0 : toPercent(angular, max_angular_rad_s);
      if (ratio_mapping_)
      {
        auto feedback = body_feedback_;
        feedback.age_sec = std::chrono::duration<double>(now - body_feedback_received_).count();
        const auto mapped = ratio_mapping_->map(linear, angular, drive_throttle_pct_, feedback);
        if (mapped.error != amr_core::DriveRatioMapping::Error::None)
        {
          command_policy_->requestStop();
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "Velocity mapping rejected target: %s; target invalidated and stop pending",
                                amr_core::DriveRatioMapping::describe(mapped.error));
          return;
        }
        translation_pct = mapped.translation_pct;
        rotation_pct = mapped.rotation_pct;
      }
      ArNetPacket packet;
      packet.doubleToBuf(translation_pct);
      packet.doubleToBuf(rotation_pct);
      packet.doubleToBuf(drive_throttle_pct_);
      packet.doubleToBuf(0.0);
      if (!client_.requestOnce("ratioDrive", &packet, true))
      {
        command_policy_->requestStop();
        RCLCPP_ERROR(node_->get_logger(), "Failed to queue ratioDrive; movement invalidated and stop pending");
      }
    }
  }


private:
  /**
   * @brief Helper function to convert a velocity value to a percentage of the maximum, clamped to [-100, 100].
   * @param value The velocity value to convert.
   * @param max_abs_value The maximum absolute velocity corresponding to 100%.
   * @return The velocity as a percentage of the maximum, clamped to [-100, 100].
   */
  static double toPercent(double value, double max_abs_value)
  {
    if (max_abs_value <= 0.0)
    {
      return 0.0;
    }

    const double percent = (value / max_abs_value) * 100.0;
    return std::clamp(percent, -100.0, 100.0);
  }

  // ROS node and libaria client
  rclcpp::Node::SharedPtr node_;

  // Libaria runtime manager to ensure proper initialization and shutdown of libaria
  amr_core::LibAriaRuntime aria_runtime_;

  // Libaria client and drive interface
  amr_core::DriveClient client_;
  std::unique_ptr<amr_core::DriveCommandPolicy> command_policy_;
  std::unique_ptr<amr_core::DriveRatioMapping> ratio_mapping_;
  amr_core::DriveRatioMapping::Feedback body_feedback_;
  amr_core::DriveCommandPolicy::TimePoint body_feedback_received_{};
  bool network_started_{ false };
  bool cycle_registered_{ false };

  //
  ArFunctor1C<DriverInterface, ArNetPacket*> update_callback_;
  ArFunctorC<DriverInterface> drive_callback_;
  ArFunctor1C<DriverInterface, ArNetPacket*> mode_callback_;
  std::atomic<bool> mode_confirmed_{ false };
  bool mode_failed_{ false };
  bool mode_query_pending_{ false };
  amr_core::DriveCommandPolicy::TimePoint last_mode_response_{};
  amr_core::DriveCommandPolicy::TimePoint last_mode_query_{};

  // Parameters
  std::string host_;
  int port_{ 7272 };
  std::string user_;
  std::string password_;
  std::string protocol_;

  std::string odom_topic_{ "amr/odom" };
  std::string cmd_vel_topic_{ "amr/cmd_vel" };
  std::string stop_topic_{ "amr/stop" };
  bool publish_odom_{ true };
  bool publish_robot_tf_{ true };

  std::string odom_frame_{ "odom" };
  std::string base_frame_{ "base_link" };

  double expected_cmd_vel_freq_{ 20.0 };
  double drive_send_frequency_{ 20.0 };
  double min_lin_speed_{ -1000.0 };
  double max_lin_speed_{ 1000.0 };
  double min_ang_speed_{ -30.0 };
  double max_ang_speed_{ 30.0 };
  double drive_throttle_pct_{ 100.0 };
  double cmd_vel_timeout_sec_{ 0.2 };
  double mode_check_period_sec_{ 0.5 };
  double mode_confirmation_timeout_sec_{ 2.0 };
  bool unsafe_drive_{ true };

  std::mutex pose_mutex_;
  
  double pose_x_{ 0.0 };
  double pose_y_{ 0.0 };
  double pose_theta_deg_{ 0.0 };

  double initial_pose_x_{ 0.0 };
  double initial_pose_y_{ 0.0 };
  double initial_pose_theta_deg_{ 0.0 };
  double theta_rad_{ 0.0 };

  bool have_pose_{ false };
  bool initial_pose_set_{ false };

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
};
