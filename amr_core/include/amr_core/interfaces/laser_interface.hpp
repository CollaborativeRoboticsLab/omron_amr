#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>  // added

/**
 * @brief Node for publishing LaserScan messages from string data and broadcasting TF.
 */
class LaserInterface
{
public:
  /**
   * @brief Constructor. Initializes node pointer.
   * @param node Shared pointer to rclcpp::Node.
   */
  LaserInterface(rclcpp::Node::SharedPtr node)
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

    point_cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud>(topic_name_, 10);
  }

  void update(const std_msgs::msg::String& msg, double laser_x, double laser_y, double laser_theta)
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
      catch (const std::out_of_range&)
      {
        vals_str.clear();
      }

      // Build PointCloud in a frame with laser pose removed
      sensor_msgs::msg::PointCloud cloud_msg;
      cloud_msg.header.stamp = node_->now();
      cloud_msg.header.frame_id = frame_id_;  // Keep configured frame

      // Convert laser pose units and compute inverse transform
      // Inputs are mm (x,y) and deg (theta). We remove laser pose effect:
      // p_local = R(-theta) * ([x,y] - [laser_x, laser_y])
      const double theta_rad = laser_theta * M_PI / 180.0;
      const double c = std::cos(theta_rad);
      const double s = std::sin(theta_rad);

      std::istringstream iss(vals_str);
      double x_mm, y_mm;
      while (iss >> x_mm >> y_mm)
      {
        // Translate: world -> laser origin
        const double dx = x_mm - laser_x;
        const double dy = y_mm - laser_y;

        // Rotate by -theta (inverse of laser heading)
        const double lx_mm = c * dx + s * dy;
        const double ly_mm = -s * dx + c * dy;

        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(lx_mm / 1000.0);  // mm -> m
        pt.y = static_cast<float>(ly_mm / 1000.0);  // mm -> m
        pt.z = 0.0f;
        cloud_msg.points.push_back(pt);
      }
      
      point_cloud_pub->publish(cloud_msg);
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub;  ///< Publisher for PointCloud
  rclcpp::Node::SharedPtr node_;

  std::string frame_id_{ "laser_frame" };
  std::string topic_name_{ "scan" };
};