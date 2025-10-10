#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <amr_msgs/srv/arcl_api.hpp>
#include <amr_msgs/action/action.hpp>

#include "amr_core/socket/socket_driver.hpp"
#include "amr_core/socket/socket_taskmaster.hpp"
#include "amr_core/utils/parser.hpp"
#include "amr_core/utils/amr_exception.hpp"

/**
 * @brief ROS 2 ARCL driver node (without direct socket code).
 * Uses SocketDriver for ARCL communication.
 */
class ARCL_Interface
{
public:
  using ServiceARCL = amr_msgs::srv::ArclApi;
  using ActionARCL = amr_msgs::action::Action;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionARCL>;

  /**
   * @brief Constructor. Initializes publishers, subscribers, and parameters.
   * @param node Shared pointer to rclcpp::Node.
   */
  ARCL_Interface(rclcpp::Node::SharedPtr node, std::shared_ptr<SocketDriver> driver,
           std::shared_ptr<SocketTaskmaster> taskmaster, int service_timeout_ms = 15000)
  {
    node_ = node;
    socket_driver_ = driver;
    socket_taskmaster_ = taskmaster;
    service_timeout_ms_ = service_timeout_ms;
  }

  void initialize()
  {
    // Service server
    if (!socket_driver_)
      service_ = node_->create_service<ServiceARCL>(
          "arcl_api_service", std::bind(&ARCL_Interface::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    else
      RCLCPP_ERROR(node_->get_logger(), "Service server not created: driver unavailable");

    // Action server
    if (socket_taskmaster_)
      action_server_ = rclcpp_action::create_server<ActionARCL>(
          node_, "arcl_api_action",
          std::bind(&ARCL_Interface::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&ARCL_Interface::handle_cancel, this, std::placeholders::_1),
          std::bind(&ARCL_Interface::handle_accepted, this, std::placeholders::_1));
    else
      RCLCPP_ERROR(node_->get_logger(), "Action server not created: taskmaster unavailable");
  }

private:
  /**
   * @brief This function handles incoming service requests to send ARCL commands to the robot.
   */
  void handle_service(const std::shared_ptr<ServiceARCL::Request> req, std::shared_ptr<ServiceARCL::Response> resp)
  {
    std::string response;
    bool got_response;

    // Check driver availability
    if (!socket_driver_)
    {
      response = "ERROR: driver not available";
      got_response = true;
    }
    // Send command to robot
    else
    {
      int req_id = socket_driver_->queue_command(req->command, req->line_identifier);
      got_response = socket_driver_->wait_for_response(req_id, response, service_timeout_ms_);
    }

    // Delegate queueing and response handling to SocketDriver
    if (got_response)
      resp->response = response;
    else
      resp->response = "TIMEOUT";
  }

  // -------- Action server (SocketTaskmaster) --------
  /**
   * @brief This function is called when a new goal is received by the action server.
   *
   * @param uuid The unique identifier for the goal.
   * @param goal The goal message containing the command and identifiers.
   * @return rclcpp_action::GoalResponse Indicates whether the goal is accepted or rejected
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ActionARCL::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief This function is called when a goal is requested to be canceled by the action client.
   *
   * @param goal_handle The handle of the goal to be canceled.
   * @return rclcpp_action::CancelResponse Indicates whether the cancel request is accepted or rejected.
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief This function is called when a new goal is accepted by the action server.
   *
   * @param goal_handle The handle of the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    action_thread_ = std::thread(&ARCL_Interface::execute, this, goal_handle);
    action_thread_.detach();
  }

  /**
   * @brief This function executes the action goal by sending commands to the robot and processing feedback.
   *
   * @param goal_handle The handle of the accepted goal.
   */
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Check taskmaster availability
    if (!socket_taskmaster_)
    {
      auto result = std::make_shared<ActionARCL::Result>();
      result->res_msg = "Taskmaster unavailable";
      goal_handle->abort(result);
      return;
    }

    Parser parser;

    // Get goal details
    auto goal = goal_handle->get_goal();

    std::vector<std::string> end_lines;

    // Use first identifier as end line if available
    if (!goal->identifier.empty())
      end_lines.emplace_back(goal->identifier.front());

    try
    {
      // Send command to robot
      socket_taskmaster_->push_command(goal->command, true, end_lines);
    }
    catch (const amr_exception& ex)
    {
      // Failed to send command
      RCLCPP_ERROR(node_->get_logger(), "push_command error: %s", ex.what());

      auto result = std::make_shared<ActionARCL::Result>();
      result->res_msg = std::string("push_command failed: ") + ex.what();
      goal_handle->abort(result);
      return;
    }

    auto feedback = std::make_shared<ActionARCL::Feedback>();
    auto result = std::make_shared<ActionARCL::Result>();
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
      // Check for cancelation
      if (goal_handle->is_canceling())
      {
        result->res_msg = "Canceled";
        goal_handle->canceled(result);
        return;
      }

      // Wait for feedback or completion
      SocketTaskmaster::WaitResult wr{ false, "", "" };
      try
      {
        wr = socket_taskmaster_->wait_command(100);
      }
      catch (const amr_exception& ex)
      {
        // Failed to get feedback
        RCLCPP_ERROR(node_->get_logger(), "wait_command error: %s", ex.what());

        result->res_msg = std::string("wait_command error: ") + ex.what();
        goal_handle->abort(result);
        return;
      }

      // Handle lack of feedback or no goal
      if (!wr.feedback.empty() && wr.feedback.find("No goal") != std::string::npos)
      {
        result->res_msg = "No such goal";
        goal_handle->abort(result);
        return;
      }

      // Process feedback or completion
      if (wr.complete)
      {
        feedback->feed_msg = wr.feedback;
        goal_handle->publish_feedback(feedback);
        result->res_msg = wr.result;
        goal_handle->succeed(result);
        RCLCPP_INFO(node_->get_logger(), "action_server: %s", result->res_msg.c_str());
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
      // Check for timeout
      if (std::chrono::steady_clock::now() - start > std::chrono::minutes(10))
      {
        result->res_msg = "Timeout";
        goal_handle->abort(result);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (goal_handle->is_active())
    {
      result->res_msg = "Node shutdown";
      goal_handle->abort(result);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SocketDriver> socket_driver_;
  std::shared_ptr<SocketTaskmaster> socket_taskmaster_;

  rclcpp::Service<ServiceARCL>::SharedPtr service_;
  rclcpp_action::Server<ActionARCL>::SharedPtr action_server_;

  int service_timeout_ms_{ 15000 };

  std::thread action_thread_;
};