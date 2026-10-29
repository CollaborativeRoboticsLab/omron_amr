#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <stdexcept>
#include <nav2_msgs/action/undock_robot.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "amr_core/interfaces/status_interface.hpp"
#include "amr_core/interfaces/laser_interface.hpp"
#include "amr_core/interfaces/drive_interface.hpp"
#include "amr_core/interfaces/map_interface.hpp"
#include "amr_core/utils/robot_session.hpp"

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
    host_ = getOrDeclareParameter<std::string>("robot.ip", "127.0.0.1");
    port_ = getOrDeclareParameter<int>("robot.port", 7272);
    user_ = getOrDeclareParameter<std::string>("robot.user", "admin");
    password_ = getOrDeclareParameter<std::string>("robot.password", "");
    protocol_ = getOrDeclareParameter<std::string>("robot.protocol", "6MTX");
    publish_status_ = getOrDeclareParameter<bool>("status.publish", true);
    publish_main_laser_ = getOrDeclareParameter<bool>("laser.main_laser.enabled", true);
    publish_low_laser_ = getOrDeclareParameter<bool>("laser.low_laser.enabled", false);

    if (getOrDeclareParameter<bool>("map.ros_service.enabled", false))
    {
      try
      {
        map_session_ = std::make_unique<amr_core::RobotSession>();
        if (!map_session_->connect({ host_, port_, user_, password_, protocol_ }) ||
            !map_session_->client().dataExists("getMapName"))
          throw std::runtime_error("Connected getMapName service required");
        map_interface_ = std::make_unique<amr_core::MapInterface>(map_session_->client());
        const auto service_name = getOrDeclareParameter<std::string>("map.ros_service.name", "map/get_active_map");
        map_server_ = create_service<std_srvs::srv::Trigger>(service_name,
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
              std::lock_guard<std::mutex> lock(map_mutex_);
              try
              {
                response->message = map_interface_->getName();
                response->success = true;
              }
              catch (const std::exception&)
              {
                response->success = false;
                response->message = "Active map query failed";
              }
            });
      }
      catch (const std::exception& ex)
      {
        map_interface_.reset();
        map_session_.reset();
        RCLCPP_ERROR(this->get_logger(), "Map interface initialization failed: %s", ex.what());
      }
    }

    if (publish_status_)
    {
      try
      {
        status_interface_ = std::make_shared<StatusInterface>(this->shared_from_this());
        status_interface_->initialize(host_, port_, user_, password_, protocol_);
        RCLCPP_INFO(this->get_logger(), "Publishing status information enabled");
      }
      catch (const std::exception& ex)
      {
        status_interface_.reset();
        RCLCPP_ERROR(this->get_logger(), "StatusInterface initialization failed: %s", ex.what());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Publishing status information disabled");
    }

    if (publish_main_laser_ || publish_low_laser_)
    {
      try
      {
        laser_scans_ = std::make_shared<LaserInterface>(this->shared_from_this());
        laser_scans_->initialize(host_, port_, user_, password_, protocol_);
        RCLCPP_INFO(this->get_logger(), "Publishing laser scans enabled");
      }
      catch (const std::exception& ex)
      {
        laser_scans_.reset();
        RCLCPP_ERROR(this->get_logger(), "LaserInterface initialization failed: %s", ex.what());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Publishing laser scans disabled");
    }

    // DriverInterface
    try
    {
      driver_ = std::make_shared<DriverInterface>(this->shared_from_this());
      driver_->initialize(host_, port_, user_, password_, protocol_);
      if (!driver_->forwardDepartureBypassEnabled()) initializeUndockAction();
    }
    catch (const std::exception& ex)
    {
      driver_.reset();
      RCLCPP_ERROR(this->get_logger(), "DriverInterface initialization failed: %s", ex.what());
    }

    RCLCPP_INFO(this->get_logger(), "CoreNode initialized");
  }

private:
  using Undock = nav2_msgs::action::UndockRobot;
  using UndockGoal = rclcpp_action::ServerGoalHandle<Undock>;

  void initializeUndockAction()
  {
    docking_ = driver_->dockingInterface();
    dock_type_ = getOrDeclareParameter<std::string>("docking.dock_type", "omron");
    if (dock_type_.empty()) throw std::invalid_argument("docking.dock_type must not be empty");
    getOrDeclareParameter<bool>("docking.exclusive_control_confirmed", false);
    undock_server_ = rclcpp_action::create_server<Undock>(shared_from_this(), "undock_robot",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Undock::Goal> goal) {
        std::lock_guard<std::mutex> lock(undock_mutex_);
        if (undock_goal_ || (!goal->dock_type.empty() && goal->dock_type != dock_type_) ||
          !std::isfinite(goal->max_undocking_time) || goal->max_undocking_time <= 0.0f ||
          !docking_->reserveUndock(get_parameter("docking.contact_dock_confirmed").as_bool(),
                      get_parameter("docking.exclusive_control_confirmed").as_bool(),
                      goal->max_undocking_time))
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<UndockGoal> goal) {
        std::lock_guard<std::mutex> lock(undock_mutex_);
        if (goal != undock_goal_ || !docking_->cancelUndock()) return rclcpp_action::CancelResponse::REJECT;
        undock_cancel_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<UndockGoal> goal) {
        std::lock_guard<std::mutex> lock(undock_mutex_);
        undock_goal_ = std::move(goal);
        undock_cancel_requested_ = false;
        docking_->beginUndock();
      });
    undock_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publishUndock(); });
  }

  void publishUndock()
  {
    std::lock_guard<std::mutex> lock(undock_mutex_);
    if (!undock_goal_) return;
    const auto status = docking_->operationStatus();
    if (!status.terminal)
    {
      undock_goal_->publish_feedback(std::make_shared<Undock::Feedback>());
      return;
    }
    if (undock_cancel_requested_ && !undock_goal_->is_canceling()) return;
    auto result = std::make_shared<Undock::Result>();
    result->success = status.success;
    result->error_code = status.success ? Undock::Result::NONE : Undock::Result::FAILED_TO_CONTROL;
    result->error_msg = status.success ? "" : status.message;
    if (undock_goal_->is_canceling()) undock_goal_->canceled(result);
    else if (status.success) undock_goal_->succeed(result);
    else undock_goal_->abort(result);
    undock_goal_.reset();
    docking_->finishUndock();
  }

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
    if (!this->has_parameter(name))
    {
      return this->declare_parameter<T>(name, default_value);
    }

    T value = default_value;
    this->get_parameter(name, value);
    return value;
  }

private:
  std::unique_ptr<amr_core::RobotSession> map_session_;
  std::unique_ptr<amr_core::MapInterface> map_interface_;
  std::mutex map_mutex_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_server_;

  // Status information
  std::shared_ptr<StatusInterface> status_interface_;

  // Laser scans
  std::shared_ptr<LaserInterface> laser_scans_;

  // DriverInterface
  std::shared_ptr<DriverInterface> driver_;
  std::shared_ptr<amr_core::DockingInterface> docking_;
  rclcpp_action::Server<Undock>::SharedPtr undock_server_;
  rclcpp::TimerBase::SharedPtr undock_timer_;
  std::shared_ptr<UndockGoal> undock_goal_;
  std::mutex undock_mutex_;
  bool undock_cancel_requested_{ false };
  std::string dock_type_;

  // Parameters
  std::string host_;
  int port_{ 7272 };
  std::string user_;
  std::string password_;
  std::string protocol_;
  bool publish_status_{ true };
  bool publish_main_laser_{ true };
  bool publish_low_laser_{ false };
};