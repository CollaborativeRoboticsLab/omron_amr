#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "amr_core/interfaces/status_interface.hpp"
#include "amr_core/interfaces/laser_interface.hpp"
#include "amr_core/interfaces/drive_interface.hpp"

class CoreNode : public rclcpp::Node
{
public:
  CoreNode() : rclcpp::Node("core_node")
  {
  }

  /**
   * @brief Initializes and wires the libaria-backed interfaces used by the core node.
   */
  void initialize()
  {
    publish_source_ = this->declare_parameter<bool>("source_data.publish", false);
    if (publish_source_)
    {
      try
      {
        status_interface_ = std::make_shared<StatusInterface>(this->shared_from_this());
        status_interface_->initialize();
        RCLCPP_INFO(this->get_logger(), "Publishing status information enabled");
      }
      catch (const std::exception &ex)
      {
        status_interface_.reset();
        RCLCPP_ERROR(this->get_logger(), "StatusInterface initialization failed: %s", ex.what());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Publishing status information is disabled");
    }

    // Publish laser scans
    publish_laser_scans_ = this->declare_parameter<bool>("laser_scans.publish", true);
    if (publish_laser_scans_)
    {
      try
      {
        laser_scans_ = std::make_shared<LaserInterface>(this->shared_from_this());
        laser_scans_->initialize();
        RCLCPP_INFO(this->get_logger(), "Publishing laser scans enabled");
      }
      catch (const std::exception &ex)
      {
        laser_scans_.reset();
        RCLCPP_ERROR(this->get_logger(), "LaserInterface initialization failed: %s", ex.what());
      }
    }

    // DriverInterface
    try
    {
      odom_driver_ = std::make_shared<DriverInterface>(this->shared_from_this());
      odom_driver_->initialize();
    }
    catch (const std::exception &ex)
    {
      odom_driver_.reset();
      RCLCPP_ERROR(this->get_logger(), "DriverInterface initialization failed: %s", ex.what());
    }

    RCLCPP_INFO(this->get_logger(), "CoreNode initialized");
  }

private:
  bool publish_source_{ false };
  std::shared_ptr<StatusInterface> status_interface_;

  // Laser scans
  bool publish_laser_scans_{ false };
  std::shared_ptr<LaserInterface> laser_scans_;

  // DriverInterface
  std::shared_ptr<DriverInterface> odom_driver_;
};