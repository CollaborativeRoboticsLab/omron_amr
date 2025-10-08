#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <amr_msgs/action/action.hpp>

class ClientNode : public rclcpp::Node {
public:
  using ActionT = amr_msgs::action::Action;
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  /**
   * @brief constructor
   */
  ClientNode()
  : rclcpp::Node("amr_client_node")
  {}

  /**
   * 
   */
  void initialize()
  {
    // Parameters
    this->declare_parameter<std::string>("command_type", "none");  // none|goto_goal|execute_macro|dock|undock
    this->declare_parameter<std::string>("goal_name", "Goal1");
    this->declare_parameter<std::string>("macro_name", "Macro1");
    this->declare_parameter<bool>("exit_on_result", false);

    // Action client
    ac_ = rclcpp_action::create_client<ActionT>(shared_from_this(), "action_server");

    // Defer optional one-shot command to allow executor to start
    using namespace std::chrono_literals;
    start_timer_ = this->create_wall_timer(0ms, [this]() {
      start_timer_->cancel();
      auto cmd = get_param<std::string>("command_type");
      if (cmd == "goto_goal") {
        goto_goal(get_param<std::string>("goal_name"));
      } else if (cmd == "execute_macro") {
        execute_macro(get_param<std::string>("macro_name"));
      } else if (cmd == "dock") {
        dock();
      } else if (cmd == "undock") {
        undock();
      }
    });
  }

private:
  // ========== High-level commands ==========
  void goto_goal(const std::string& name) {
    ActionT::Goal goal;
    goal.command = "goto " + name;
    goal.identifier = { "Arrived at " + name };
    send_goal(goal);
  }

  void execute_macro(const std::string& name) {
    ActionT::Goal goal;
    goal.command = "executeMacro " + name;
    goal.identifier = { "Completed macro " + name };
    send_goal(goal);
  }

  void dock() {
    ActionT::Goal goal;
    goal.command = "dock";
    goal.identifier = { "Arrived at dock" };
    send_goal(goal);
  }

  void undock() {
    ActionT::Goal goal;
    goal.command = "undock";
    goal.identifier = { "Stopped" };
    send_goal(goal);
  }


  // ========== Action helpers ==========
  void send_goal(const ActionT::Goal& goal) {
    // Wait briefly for server
    using namespace std::chrono_literals;
    if (!ac_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto options = typename ClientT::SendGoalOptions();

    options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback) {
          RCLCPP_INFO(this->get_logger(), "%s", feedback->feed_msg.c_str());
        };
        
    options.result_callback =
        [this](const GoalHandle::WrappedResult& result) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "%s", result.result->res_msg.c_str());
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_WARN(this->get_logger(), "Aborted: %s", result.result ? result.result->res_msg.c_str() : "");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(this->get_logger(), "Canceled");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code");
              break;
          }
          if (get_param<bool>("exit_on_result")) {
            rclcpp::shutdown();
          }
        };

    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
          }
        };

    ac_->async_send_goal(goal, options);
  }

  // ========== Math ==========
  template<typename T>
  T get_param(const std::string& name) const {
    return this->get_parameter(name).get_parameter_value().template get<T>();
  }

private:
  rclcpp_action::Client<ActionT>::SharedPtr ac_;

  rclcpp::TimerBase::SharedPtr start_timer_;
};