#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>

#include <std_msgs/msg/string.hpp>
#include <amr_msgs/msg/status.hpp>
#include <amr_msgs/msg/location.hpp>

#include "amr_core/socket/socket_listener.hpp"
#include "amr_core/socket/socket_driver.hpp"
#include "amr_core/socket/socket_taskmaster.hpp"
#include "amr_core/utils/amr_exception.hpp"
#include "amr_core/interfaces/map_loader.hpp"
#include "amr_core/interfaces/laser_scans.hpp"
#include "amr_core/interfaces/driver.hpp"
#include "amr_core/interfaces/arcl_ros.hpp"

using namespace std::chrono_literals;

class CoreNode : public rclcpp::Node
{
public:
  CoreNode() : rclcpp::Node("core_node")
  {
  }

  /**
   * @brief This function initializes the core node by reading parameters, setting up publishers, services, and action
   * servers, and establishing connections to the robot.
   */
  void initialize()
  {
    // Shared connection params (service + action)
    ip_ = this->declare_parameter<std::string>("robot.ip", "127.0.0.1");
    port_ = this->declare_parameter<int>("robot.port", 0);
    passwd_ = this->declare_parameter<std::string>("robot.password", "");

    // listener endpoint (defaults to same ip/port if not set)
    host_ip_ = this->declare_parameter<std::string>("host.ip", ip_);
    host_port_ = this->declare_parameter<int>("host.port", port_);

    // Publish source data from listener
    publish_source_ = this->declare_parameter<bool>("source_data.publish", false);
    publish_frequency_ = this->declare_parameter<int>("source_data.frequency", 5);
    pub_interval_ms_ = int(1000 / publish_frequency_);

    // SocketDriver (low-level)
    RCLCPP_INFO(this->get_logger(), "Initializing SocketDriver..");
    try
    {
      driver_ = std::make_shared<SocketDriver>(this->shared_from_this());
      driver_->connect(ip_, static_cast<uint16_t>(port_));
      driver_->login(passwd_);

      RCLCPP_INFO(this->get_logger(), "SocketDriver connected to %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "SocketDriver initialization failed: %s", e.what());
    }

    // SocketTaskmaster (high-level)
    RCLCPP_INFO(this->get_logger(), "Initializing SocketTaskmaster..");
    try
    {
      task_master_ = std::make_shared<SocketTaskmaster>(this->shared_from_this());
      task_master_->connect(ip_, port_);
      task_master_->login(passwd_);

      RCLCPP_INFO(this->get_logger(), "ScoketTaskmaster is up @ %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_FATAL(this->get_logger(), "SocketTaskmaster initialization failed: %s", ex.what());
    }

    // SocketListener
    RCLCPP_INFO(this->get_logger(), "Initializing SocketListener..");
    try
    {
      listener_ = std::make_shared<SocketListener>(this->shared_from_this(), host_ip_, host_port_);
      if (!listener_->begin())
        RCLCPP_ERROR(this->get_logger(), "SocketListener begin() failed");
      else
        RCLCPP_INFO(this->get_logger(), "SocketListener started on %s:%d", host_ip_.c_str(), host_port_);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "SocketListener error: %s", ex.what());
    }

    // Publish source data from listener
    if (publish_source_)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing source data from listener enabled");

      status_pub_ = this->create_publisher<amr_msgs::msg::Status>("amr/source/status", 10);
      laser_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/laser", 10);
      goals_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/all_goals", 10);
      odom_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/odom", 10);
      app_fault_query_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/application_fault_query", 10);
      faults_get_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/faults_get", 10);
      query_faults_pub_ = this->create_publisher<std_msgs::msg::String>("amr/source/query_faults", 10);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Publishing source data from listener is disabled");
    }

    // Publisher timer
    main_timer =
        this->create_wall_timer(std::chrono::milliseconds(pub_interval_ms_), std::bind(&CoreNode::main_loop, this));

    // Publish map data from file
    publish_map_ = this->declare_parameter<bool>("map_data.publish", false);
    if (publish_map_)
    {
      data_point_marker_ = std::make_shared<MapLoader>(this->shared_from_this());
      data_point_marker_->initialize();
      RCLCPP_INFO(this->get_logger(), "Publishing map data from file enabled");
    }

    // Publish laser scans
    publish_laser_scans_ = this->declare_parameter<bool>("laser_scans.publish", true);
    if (publish_laser_scans_)
    {
      laser_scans_ = std::make_shared<LaserScans>(this->shared_from_this());
      laser_scans_->initialize();
      RCLCPP_INFO(this->get_logger(), "Publishing laser scans enabled");
    }

    // Driver
    odom_driver_ = std::make_shared<Driver>(this->shared_from_this(), driver_);
    odom_driver_->initialize();

    // ARCL service + action
    enable_arcl_access_ = this->declare_parameter<bool>("arcl.enable", false);
    timeout_ms_ = this->declare_parameter<int>("arcl.timeout_ms", 15000);
    if (enable_arcl_access_)
    {
      RCLCPP_INFO(this->get_logger(), "ARCL interface is enabled");
      arcl_ros = std::make_shared<Arcl_ROS>(this->shared_from_this(), driver_, task_master_, timeout_ms_);
      arcl_ros->initialize();
    }

    RCLCPP_INFO(this->get_logger(), "CoreNode initialized");
  }

private:
  // -------- Publisher (SocketListener -> topics) --------
  /**
   * @brief This function is called periodically by the publisher timer to poll the listener for new data and publish it
   * to topics.
   */
  void main_loop()
  {
    // Poll listener for new data
    try
    {
      while (listener_ && listener_->poll_once(0))
      {
      }
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Listener error: %s", ex.what());
    }

    pub_status();
    pub_laser();
    pub_goals();
    pub_odometer();
    pub_app_fault_query();
    pub_faults_get();
    pub_query_faults();
  }

  /**
   * @brief This function publishes the robot's status information to the status topic. This include parsing the
   * following fields from the listener:
   *
   * ExtendedStatusForHumans: Robot lost
   * Status: Teleop driving
   * StateOfCharge: 79.4
   * Location: 79031 -81597 20
   * LocalizationScore: 0.170543
   * Temperature: 36
   */
  void pub_status()
  {
    amr_msgs::msg::Status status_msg;
    amr_msgs::msg::Location loc_msg;

    try
    {
      const auto& status_status = listener_->get_response("Status");
      const auto& status_batt = listener_->get_response("StateOfCharge");
      const auto& status_loc = listener_->get_response("Location");
      const auto& status_loc_sc = listener_->get_response("LocalizationScore");
      const auto& status_temp = listener_->get_response("Temperature");
      const auto& status_ext = listener_->get_response("ExtendedStatusForHumans");

      status_msg.status = status_status.front();
      status_msg.extended_status = status_ext.front();
      status_msg.state_of_charge = std::stof(status_batt.front());
      status_msg.localization_score = std::stof(status_loc_sc.front());
      status_msg.temperature = std::stof(status_temp.front());

      std::istringstream iss(status_loc.front());
      double x = 0.0, y = 0.0, th = 0.0;
      if (iss >> x >> y >> th)
      {
        loc_msg.x = x;
        loc_msg.y = y;
        loc_msg.theta = th;
        status_msg.location = loc_msg;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Value error with location coordinates. Using zeros.");
      }
    }
    catch (const std::out_of_range&)
    {
      RCLCPP_DEBUG(this->get_logger(), "Status data not available yet");
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Status parse error: %s", ex.what());
    }

    if (publish_source_ && status_pub_)
      status_pub_->publish(status_msg);
  }

  void pub_laser()
  {
    try
    {
      const auto& scans = listener_->get_response("RangeDeviceGetCurrent");
      std_msgs::msg::String msg;
      msg.data = scans.front();

      if (publish_source_ && laser_pub_)
        laser_pub_->publish(msg);

      if (publish_laser_scans_)
        laser_scans_->update(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

  void pub_goals()
  {
    try
    {
      const auto& goals = listener_->get_response("Goal");
      std::string joined;
      for (size_t i = 0; i < goals.size(); ++i)
      {
        if (i)
          joined += ' ';
        joined += goals[i];
      }
      std_msgs::msg::String msg;
      msg.data = joined;
      if (publish_source_ && goals_pub_)
        goals_pub_->publish(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

  void pub_odometer()
  {
    try
    {
      const auto& odom = listener_->get_response("Odometer");
      std::string joined;
      for (size_t i = 0; i < odom.size(); ++i)
      {
        if (i)
          joined += ' ';
        joined += odom[i];
      }
      std_msgs::msg::String msg;
      msg.data = joined;
      if (publish_source_ && odom_pub_)
        odom_pub_->publish(msg);

      if (odom_driver_)
        odom_driver_->update(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

  void pub_app_fault_query()
  {
    try
    {
      const auto& query = listener_->get_response("ApplicationFaultQuery");
      std::string joined;
      for (size_t i = 0; i < query.size(); ++i)
      {
        if (i)
          joined += ' ';
        joined += query[i];
      }
      std_msgs::msg::String msg;
      msg.data = joined;
      if (publish_source_ && app_fault_query_pub_)
        app_fault_query_pub_->publish(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

  void pub_faults_get()
  {
    try
    {
      const auto& faults = listener_->get_response("FaultList");
      std::string joined;
      for (size_t i = 0; i < faults.size(); ++i)
      {
        if (i)
          joined += ' ';
        joined += faults[i];
      }
      std_msgs::msg::String msg;
      msg.data = joined;
      if (publish_source_ && faults_get_pub_)
        faults_get_pub_->publish(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

  void pub_query_faults()
  {
    try
    {
      const auto& faults = listener_->get_response("RobotFaultQuery");
      std::string joined;
      for (size_t i = 0; i < faults.size(); ++i)
      {
        if (i)
          joined += ' ';
        joined += faults[i];
      }
      std_msgs::msg::String msg;
      msg.data = joined;
      if (publish_source_ && query_faults_pub_)
        query_faults_pub_->publish(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

private:
  // Params
  std::string ip_;
  int port_{ 0 };
  std::string passwd_;
  int publish_frequency_{ 5 };
  int pub_interval_ms_{ 200 };

  std::string host_ip_;
  int host_port_{ 0 };

  bool publish_source_{ false };

  bool enable_arcl_access_{ false };

  // Service (driver)
  std::shared_ptr<SocketDriver> driver_;

  // Action (taskmaster)
  std::shared_ptr<SocketTaskmaster> task_master_;

  // Publisher (listener)
  std::shared_ptr<SocketListener> listener_;
  rclcpp::TimerBase::SharedPtr main_timer;

  // Source Pubs
  rclcpp::Publisher<amr_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goals_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr app_fault_query_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr faults_get_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_faults_pub_;

  // Map pubs
  bool publish_map_{ false };
  std::shared_ptr<MapLoader> data_point_marker_;  ///< Data point marker instance

  // Laser scans
  bool publish_laser_scans_{ false };
  std::shared_ptr<LaserScans> laser_scans_;

  // Driver
  std::shared_ptr<Driver> odom_driver_;

  // ARCL ROS interface
  std::shared_ptr<Arcl_ROS> arcl_ros;
  int timeout_ms_{ 15000 };
};