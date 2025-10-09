#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

/**
 * @brief Node for publishing LaserScan messages from string data.
 */
class LaserScans
{
public:
  /**
   * @brief Constructor. Initializes node pointer.
   * @param node Shared pointer to rclcpp::Node.
   */
  LaserScans(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
  }

  /**
   * @brief Initializes the LaserScan publisher, subscriptions, and parameters.
   * This should be called after the node is fully constructed.
   */
  void initialize()
  {
    // Declare and get parameters for scan angles and other LaserScan fields
    frame_id_ = node_->declare_parameter<std::string>("laser_scans.frame_id", "laser_frame");
    topic_name_ = node_->declare_parameter<std::string>("laser_scans.topic", "scan");
    angle_min_ = node_->declare_parameter<double>("laser_scans.angle_min", -1.5708);             // -90 deg default
    angle_max_ = node_->declare_parameter<double>("laser_scans.angle_max", 1.5708);              // +90 deg default
    angle_increment_ = node_->declare_parameter<double>("laser_scans.angle_increment", 0.0069);  // ~0.4 deg default
    scan_time_ = node_->declare_parameter<double>("laser_scans.scan_time", 0.1);                 // default 0.1s
    range_min_ = node_->declare_parameter<double>("laser_scans.range_min", 0.05);                // default 5cm
    range_max_ = node_->declare_parameter<double>("laser_scans.range_max", 30.0);                // default 30m
    time_increment_ = node_->declare_parameter<double>("laser_scans.time_increment", 0.0);       // default 0

    point_cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud>(topic_name_, 10);
  }

  void update(const std_msgs::msg::String& msg)
  {
    std::string raw_resp = msg.data;
    std::string rng_device = "Laser_1";
    std::string::size_type pos = raw_resp.find(rng_device);
    if (pos != std::string::npos)
    {
      std::string vals_str;
      try
      {
        // +1 to exclude the following space.
        vals_str = raw_resp.substr(pos + rng_device.length() + 1);
      }
      catch (const std::out_of_range& e)
      {
        vals_str.clear();
      }
      publish_point_cloud(vals_str);
    }
  }

private:
  /**
   * @brief Parses the laser scan string and publishes a PointCloud message.
   * @param vals_str String containing laser scan values.
   */
  void publish_point_cloud(const std::string& vals_str)
  {
    sensor_msgs::msg::PointCloud cloud_msg;
    cloud_msg.header.stamp = node_->now();
    cloud_msg.header.frame_id = frame_id_;  // Use configured frame

    // Parse x y pairs and fill points
    std::istringstream iss(vals_str);
    double x, y;
    while (iss >> x >> y)
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(x / 1000.0);  // Convert mm to meters
      pt.y = static_cast<float>(y / 1000.0);
      pt.z = 0.0;
      cloud_msg.points.push_back(pt);
    }

    point_cloud_pub->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub;  ///< Publisher for LaserScan
  rclcpp::Node::SharedPtr node_;

  std::string frame_id_{ "laser_frame" };
  std::string topic_name_{ "scan" };
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  double time_increment_;
};