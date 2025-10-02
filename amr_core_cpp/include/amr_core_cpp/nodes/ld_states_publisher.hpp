#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <sstream>
#include <vector>
#include <memory>

#include <std_msgs/msg/string.hpp>
#include <amr_msgs/msg/status.hpp>
#include <amr_msgs/msg/location.hpp>

#include "amr_core_cpp/socket_listener.hpp"
#include "amr_core_cpp/amr_exception.hpp"

class LdStatesPublisher : public rclcpp::Node
{
public:
  LdStatesPublisher() : rclcpp::Node("ld_states_publisher_node")
  {
    using namespace std::chrono_literals;

    // Parameters for SocketListener endpoint
    ip_ = this->declare_parameter<std::string>("local_ip", "127.0.0.1");
    port_ = this->declare_parameter<int>("local_port", 0);

    // Publishers
    status_pub_ = this->create_publisher<amr_msgs::msg::Status>("ldarcl_status", 10);
    laser_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_laser", 10);
    goals_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_all_goals", 10);
    odom_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_odom", 10);
    app_fault_query_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_application_fault_query", 10);
    faults_get_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_faults_get", 10);
    query_faults_pub_ = this->create_publisher<std_msgs::msg::String>("ldarcl_query_faults", 10);

    // Defer listener creation until node is fully constructed
    init_timer_ = this->create_wall_timer(0ms, [this]() {
      init_timer_->cancel();
      init_listener_and_timers();
    });
  }

private:
  void init_listener_and_timers()
  {
    using namespace std::chrono_literals;

    try
    {
      // Create and start listener
      auto self = this->shared_from_this();
      listener_ = std::make_shared<SocketListener>(std::static_pointer_cast<rclcpp::Node>(self), ip_, port_);
      if (!listener_->begin())
      {
        RCLCPP_ERROR(this->get_logger(), "Listener begin() failed");
        throw amr_exception("SocketListener begin() failed");
      }
      RCLCPP_INFO(this->get_logger(), "LD_States_Publisher is up on %s:%d", ip_.c_str(), port_);

      // Start periodic processing timer
      timer_ = this->create_wall_timer(200ms, [this]() { on_timer(); });
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Listener error: %s", ex.what());
    }
  }

  void on_timer()
  {
    try
    {
      // Drain available input quickly; parsing happens inside listener
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

  void pub_status()
  {
    amr_msgs::msg::Status status_msg;
    amr_msgs::msg::Location loc_msg;

    try
    {
      const auto& status_status = listener_->get_response("Status");
      const auto& status_batt = listener_->get_response("StateOfCharge");
      const auto& status_loc = listener_->get_response("Location");
      const auto& status_loc_score = listener_->get_response("LocalizationScore");
      const auto& status_temp = listener_->get_response("Temperature");
      const auto& status_ext = listener_->get_response("ExtendedStatusForHumans");

      status_msg.status = status_status.front();
      status_msg.extended_status = status_ext.front();
      status_msg.state_of_charge = std::stof(status_batt.front());
      status_msg.localization_score = std::stof(status_loc_score.front());
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
        RCLCPP_WARN(this->get_logger(), "Value error with location coordinates. Setting them to zeroes.");
      }
    }
    catch (const std::out_of_range&)
    {
      // Missing keys; publish partial/default message
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Status parse error: %s", ex.what());
    }

    status_pub_->publish(status_msg);
  }

  void pub_laser()
  {
    try
    {
      const auto& scans = listener_->get_response("RangeDeviceGetCurrent");
      std_msgs::msg::String msg;
      msg.data = scans.front();
      laser_pub_->publish(msg);
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
      odom_pub_->publish(msg);
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
      query_faults_pub_->publish(msg);
    }
    catch (const std::out_of_range&)
    {
    }
  }

private:
  // Endpoint params
  std::string ip_;
  int port_{ 0 };

  // Listener and timers
  std::shared_ptr<SocketListener> listener_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp::Publisher<amr_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goals_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr app_fault_query_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr faults_get_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_faults_pub_;
};