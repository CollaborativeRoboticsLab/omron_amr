#pragma once

#include <ArNetworking/ArNetworking.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <memory>
#include <mutex>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <limits>

#include "amr_core/utils/libaria_runtime.hpp"

/**
 * @brief Node for publishing LaserScan messages from libaria laser packets while preserving amr_core frame correction.
 */
class LaserInterface
{
public:
  /**
   * @brief Constructor. Initializes node pointer.
   * @param node Shared pointer to rclcpp::Node.
   */
  LaserInterface(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
    , laser_callback_(this, &LaserInterface::handleLaser)
    , update_callback_(this, &LaserInterface::handleUpdateNumbers)
  {
  }

  ~LaserInterface()
  {
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

    frame_id_ = getOrDeclareParameter<std::string>("laser_scans.frame_id", "laser_frame");
    topic_name_ = getOrDeclareParameter<std::string>("laser_scans.topic", "scan");
    laser_request_ = getOrDeclareParameter<std::string>("laser_scans.request", "Laser_1Current");
    request_period_ms_ = getOrDeclareParameter<int>("laser_scans.request_period_ms", 200);
    angle_min_ = getOrDeclareParameter<double>("laser_scans.angle_min", -M_PI);
    angle_max_ = getOrDeclareParameter<double>("laser_scans.angle_max", M_PI);
    angle_increment_ = getOrDeclareParameter<double>("laser_scans.angle_increment", M_PI / 360.0);
    range_min_ = getOrDeclareParameter<double>("laser_scans.range_min", 0.02);
    range_max_ = getOrDeclareParameter<double>("laser_scans.range_max", 30.0);

    if (angle_increment_ <= 0.0)
    {
      RCLCPP_WARN(node_->get_logger(), "laser_scans.angle_increment must be positive; defaulting to 0.5 degree");
      angle_increment_ = M_PI / 360.0;
    }
    if (angle_max_ <= angle_min_)
    {
      RCLCPP_WARN(node_->get_logger(), "laser_scans.angle_max must exceed angle_min; defaulting to [-pi, pi]");
      angle_min_ = -M_PI;
      angle_max_ = M_PI;
    }

    laser_scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

    connectClient();
    configureHandlers();
    client_.runAsync();
  }

private:
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
        throw std::runtime_error("Robot rejected the laser interface connection.");
      }
      throw std::runtime_error("Could not connect laser interface to Omron robot server.");
    }

    client_.setRobotName(host_.c_str());
    RCLCPP_INFO(node_->get_logger(), "LaserInterface connected to %s:%d using libaria", host_.c_str(), port_);
  }

  void configureHandlers()
  {
    if (client_.dataExists("updateNumbers"))
    {
      client_.addHandler("updateNumbers", &update_callback_);
      client_.request("updateNumbers", 50);
    }

    if (!client_.dataExists(laser_request_.c_str()))
    {
      RCLCPP_WARN(node_->get_logger(), "Server does not advertise %s", laser_request_.c_str());
      return;
    }

    client_.addHandler(laser_request_.c_str(), &laser_callback_);
    client_.request(laser_request_.c_str(), request_period_ms_);
  }

  void handleUpdateNumbers(ArNetPacket* packet)
  {
    packet->bufToByte2();
    const double x = static_cast<double>(packet->bufToByte4());
    const double y = static_cast<double>(packet->bufToByte4());
    const double theta_deg = static_cast<double>(packet->bufToByte2());
    packet->bufToByte2();
    packet->bufToByte2();
    packet->bufToByte2();
    packet->bufToByte();

    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_x_mm_ = x;
    pose_y_mm_ = y;
    pose_theta_deg_ = theta_deg;
    have_pose_ = true;
  }

  void handleLaser(ArNetPacket* packet)
  {
    const int num_readings = packet->bufToByte4();
    if (num_readings <= 0)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Received empty laser packet");
      return;
    }

    double laser_x = 0.0;
    double laser_y = 0.0;
    double laser_theta = 0.0;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      laser_x = pose_x_mm_;
      laser_y = pose_y_mm_;
      laser_theta = pose_theta_deg_;
    }

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = node_->now();
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = static_cast<float>(angle_min_);
    scan_msg.angle_max = static_cast<float>(angle_max_);
    scan_msg.angle_increment = static_cast<float>(angle_increment_);
    scan_msg.range_min = static_cast<float>(range_min_);
    scan_msg.range_max = static_cast<float>(range_max_);
    scan_msg.scan_time = request_period_ms_ / 1000.0f;
    scan_msg.time_increment = 0.0f;

    const double bounded_angle_max = std::max(angle_max_, angle_min_ + angle_increment_);
    const std::size_t bin_count =
        static_cast<std::size_t>(std::floor((bounded_angle_max - angle_min_) / angle_increment_)) + 1U;
    scan_msg.ranges.assign(bin_count, std::numeric_limits<float>::infinity());

    const double theta_rad = laser_theta * M_PI / 180.0;
    const double c = std::cos(theta_rad);
    const double s = std::sin(theta_rad);

    for (int i = 0; i < num_readings; ++i)
    {
      const double x_mm = static_cast<double>(packet->bufToByte4());
      const double y_mm = static_cast<double>(packet->bufToByte4());
      const double dx = x_mm - laser_x;
      const double dy = y_mm - laser_y;
      const double lx_mm = c * dx + s * dy;
      const double ly_mm = -s * dx + c * dy;

      const double lx = lx_mm / 1000.0;
      const double ly = ly_mm / 1000.0;
      const double range = std::hypot(lx, ly);
      if (range < range_min_ || range > range_max_)
      {
        continue;
      }

      const double angle = std::atan2(ly, lx);
      if (angle < angle_min_ || angle > bounded_angle_max)
      {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(std::floor((angle - angle_min_) / angle_increment_));
      if (index >= scan_msg.ranges.size())
      {
        continue;
      }

      scan_msg.ranges[index] = std::min(scan_msg.ranges[index], static_cast<float>(range));
    }

    for (auto& range : scan_msg.ranges)
    {
      if (!std::isfinite(range))
      {
        range = std::numeric_limits<float>::quiet_NaN();
      }
    }

    laser_scan_pub_->publish(scan_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;  ///< Publisher for LaserScan
  rclcpp::Node::SharedPtr node_;
  amr_core::LibAriaRuntime aria_runtime_;
  ArClientBase client_;
  ArFunctor1C<LaserInterface, ArNetPacket*> laser_callback_;
  ArFunctor1C<LaserInterface, ArNetPacket*> update_callback_;

  std::mutex pose_mutex_;
  double pose_x_mm_{ 0.0 };
  double pose_y_mm_{ 0.0 };
  double pose_theta_deg_{ 0.0 };
  bool have_pose_{ false };

  std::string host_;
  int port_{ 7272 };
  std::string user_;
  std::string password_;
  std::string protocol_;

  std::string frame_id_{ "laser_frame" };
  std::string topic_name_{ "scan" };
  std::string laser_request_{ "Laser_1Current" };
  int request_period_ms_{ 200 };
  double angle_min_{ -M_PI };
  double angle_max_{ M_PI };
  double angle_increment_{ M_PI / 180.0 };
  double range_min_{ 0.02 };
  double range_max_{ 30.0 };
};