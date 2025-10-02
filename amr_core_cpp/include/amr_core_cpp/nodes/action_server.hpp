#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "amr_msgs/action/action.hpp"
#include "amr_core_cpp/utils/socket_taskmaster.hpp"
#include "amr_core_cpp/utils/parser.hpp"
#include "amr_core_cpp/amr_exception.hpp"

/**
 * @brief Header-only LDActionServer node (ROS 2 action server).
 *
 * Parameters:
 *  - ip_address (string)
 *  - port (int)
 *  - def_arcl_passwd (string)
 *
 * On construction:
 *  - Creates SocketTaskmaster, connects, and logs in.
 *  - Starts action server "action_server".
 */
class LDActionServer : public rclcpp::Node
{
public:
  using ActionT = amr_msgs::action::Action;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  LDActionServer() : rclcpp::Node("action_server")
  {
    using namespace std::chrono_literals;

    ip_ = this->declare_parameter<std::string>("ip_address", "127.0.0.1");
    port_ = this->declare_parameter<int>("port", 0);
    passwd_ = this->declare_parameter<std::string>("def_arcl_passwd", "");

    // Defer initialization that requires shared_from_this()
    init_timer_ = this->create_wall_timer(0ms, [this]() {
      init_timer_->cancel();
      init();
    });
  }

private:
  void init()
  {
    try
    {
      tm_ = std::make_shared<SocketTaskmaster>(this->shared_from_this());
      (void)tm_->connect(ip_, port_);
      tm_->login(passwd_);
      RCLCPP_INFO(this->get_logger(), "Action server connected and logged in to %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "SocketTaskmaster error: %s", ex.what());
      throw;
    }

    server_ = rclcpp_action::create_server<ActionT>(
        this->shared_from_this(), "action_server",
        std::bind(&LDActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LDActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&LDActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server is up!");
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ActionT::Goal> goal)
  {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread(&LDActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    Parser parser;

    // Prepare end markers from identifier[0] if present
    std::vector<std::string> end_lines;
    if (!goal->identifier.empty())
    {
      end_lines.emplace_back(goal->identifier.front());
    }

    try
    {
      tm_->push_command(goal->command, true, end_lines);
    }
    catch (const amr_exception& ex)
    {
      auto result = std::make_shared<ActionT::Result>();
      result->res_msg = std::string("push_command failed: ") + ex.what();
      goal_handle->abort(result);
      return;
    }

    auto feedback = std::make_shared<ActionT::Feedback>();
    auto result = std::make_shared<ActionT::Result>();
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        result->res_msg = "Canceled";
        goal_handle->canceled(result);
        return;
      }

      SocketTaskmaster::WaitResult wr{ false, "", "" };
      try
      {
        wr = tm_->wait_command(100);
      }
      catch (const amr_exception& ex)
      {
        result->res_msg = std::string("wait_command error: ") + ex.what();
        goal_handle->abort(result);
        return;
      }

      // Check immediate "No goal" condition in feedback
      if (!wr.feedback.empty() && wr.feedback.find("No goal") != std::string::npos)
      {
        result->res_msg = "No such goal";
        goal_handle->abort(result);
        return;
      }

      if (wr.complete)
      {
        feedback->feed_msg = wr.feedback;
        goal_handle->publish_feedback(feedback);
        result->res_msg = wr.result;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "action_server: %s", result->res_msg.c_str());
        return;
      }
      else if (!wr.feedback.empty())
      {
        auto pr = parser.process_arcl_server(wr.feedback);
        feedback->feed_msg = pr.message;
        goal_handle->publish_feedback(feedback);

        if (pr.code == Parser::Code::PASS)
        {
          result->res_msg = pr.message;
          goal_handle->succeed(result);
          return;
        }
        else if (pr.code == Parser::Code::FAIL)
        {
          result->res_msg = pr.message;
          goal_handle->abort(result);
          return;
        }
      }

      // Optional timeout guard (matches Python sleep cadence)
      if (std::chrono::steady_clock::now() - start > std::chrono::minutes(10))
      {
        result->res_msg = "Timeout";
        goal_handle->abort(result);
        return;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Node shutting down
    if (goal_handle->is_active())
    {
      result->res_msg = "Node shutdown";
      goal_handle->abort(result);
    }
  }

private:
  // Parameters
  std::string ip_;
  int port_{ 0 };
  std::string passwd_;

  // Socket manager
  std::shared_ptr<SocketTaskmaster> tm_;

  // Action server
  rclcpp_action::Server<ActionT>::SharedPtr server_;

  // Defer init that needs shared_from_this()
  rclcpp::TimerBase::SharedPtr init_timer_;
};