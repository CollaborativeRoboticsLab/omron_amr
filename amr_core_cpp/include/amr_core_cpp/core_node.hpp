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
#include <amr_msgs/srv/arcl_api.hpp>
#include <amr_msgs/action/action.hpp>

#include "amr_core_cpp/socket/socket_listener.hpp"
#include "amr_core_cpp/socket/socket_driver.hpp"
#include "amr_core_cpp/socket/socket_taskmaster.hpp"
#include "amr_core_cpp/utils/parser.hpp"
#include "amr_core_cpp/utils/amr_exception.hpp"

using namespace std::chrono_literals;

class CoreNode : public rclcpp::Node
{
public:
  using ServiceARCL = amr_msgs::srv::ArclApi;
  using ActionARCL = amr_msgs::action::Action;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionARCL>;

  CoreNode() : rclcpp::Node("core_node")
  {
  }

  void initialize()
  {
    // Shared connection params (service + action)
    ip_ = this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
    port_ = this->declare_parameter<int>("robot_port", 0);
    passwd_ = this->declare_parameter<std::string>("robot_password", "");
    svc_timeout_ms_ = this->declare_parameter<int>("service_timeout_ms", 15000);

    // Publisher (listener) endpoint (defaults to same ip/port if not set)
    host_ip_ = this->declare_parameter<std::string>("host_ip", ip_);
    host_port_ = this->declare_parameter<int>("host_port", port_);
    publisher_frequency_ = this->declare_parameter<int>("publisher_frequency", 5);

    pub_interval_ms_ = int(1000 / publisher_frequency_);

    status_pub_ = this->create_publisher<amr_msgs::msg::Status>("amr/status", 10);
    laser_pub_ = this->create_publisher<std_msgs::msg::String>("amr/laser", 10);
    goals_pub_ = this->create_publisher<std_msgs::msg::String>("amr/all_goals", 10);
    odom_pub_ = this->create_publisher<std_msgs::msg::String>("amr/odom", 10);
    app_fault_query_pub_ = this->create_publisher<std_msgs::msg::String>("amr/application_fault_query", 10);
    faults_get_pub_ = this->create_publisher<std_msgs::msg::String>("amr/faults_get", 10);
    query_faults_pub_ = this->create_publisher<std_msgs::msg::String>("amr/query_faults", 10);

    // Srvice: SocketDriver (low-level)
    try
    {
      driver_ = std::make_shared<SocketDriver>(this->shared_from_this());
      driver_->connect(ip_, static_cast<uint16_t>(port_));
      driver_->login(passwd_);

      RCLCPP_INFO(this->get_logger(), "SocketDriver(APIServer) connected to %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "SocketDriver(APIServer) initialization failed: %s", e.what());
    }

    // Action server: SocketTaskmaster (high-level)
    try
    {
      task_master_ = std::make_shared<SocketTaskmaster>(this->shared_from_this());
      task_master_->connect(ip_, port_);
      task_master_->login(passwd_);

      RCLCPP_INFO(this->get_logger(), "ScoketTaskmaster(ActionServer) is up @ %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_FATAL(this->get_logger(), "SocketTaskmaster(ActionServer) init failed: %s", ex.what());
    }

    // Publisher: SocketListener
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

    srv_ = this->create_service<ServiceARCL>(
        "arcl_api_service", std::bind(&CoreNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    action_ = rclcpp_action::create_server<ActionARCL>(
        this->shared_from_this(), "action_server",
        std::bind(&CoreNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CoreNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&CoreNode::handle_accepted, this, std::placeholders::_1));

    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(pub_interval_ms_), [this]() { on_pub_timer(); });

    RCLCPP_INFO(this->get_logger(), "CoreNode initialized");
  }

private:
  void handle_service(const std::shared_ptr<ServiceARCL::Request> req, std::shared_ptr<ServiceARCL::Response> resp)
  {
    std::string response;
    bool got_response;

    if (!driver_)
    {
      response = "ERROR: driver not available";
      got_response = true;
    }
    else
    {
      int req_id = driver_->queue_command(req->command, req->line_identifier);
      got_response = driver_->wait_for_response(req_id, response, svc_timeout_ms_);
    }

    // Delegate queueing and response handling to SocketDriver
    if (got_response)
      resp->response = response;
    else
      resp->response = "TIMEOUT";
  }

  // -------- Action server (SocketTaskmaster) --------
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ActionARCL::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    action_thread_ = std::thread(&CoreNode::execute, this, goal_handle);
    action_thread_.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (!task_master_)
    {
      auto result = std::make_shared<ActionARCL::Result>();
      result->res_msg = "Taskmaster unavailable";
      goal_handle->abort(result);
      return;
    }

    Parser parser;
    auto goal = goal_handle->get_goal();

    std::vector<std::string> end_lines;
    if (!goal->identifier.empty())
      end_lines.emplace_back(goal->identifier.front());

    try
    {
      task_master_->push_command(goal->command, true, end_lines);
    }
    catch (const amr_exception& ex)
    {
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
      if (goal_handle->is_canceling())
      {
        result->res_msg = "Canceled";
        goal_handle->canceled(result);
        return;
      }

      SocketTaskmaster::WaitResult wr{ false, "", "" };
      try
      {
        wr = task_master_->wait_command(100);
      }
      catch (const amr_exception& ex)
      {
        result->res_msg = std::string("wait_command error: ") + ex.what();
        goal_handle->abort(result);
        return;
      }

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

  // -------- Publisher (SocketListener -> topics) --------
  void on_pub_timer()
  {
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
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Status parse error: %s", ex.what());
    }

    if (status_pub_)
      status_pub_->publish(status_msg);
  }

  void pub_laser()
  {
    try
    {
      const auto& scans = listener_->get_response("RangeDeviceGetCurrent");
      std_msgs::msg::String msg;
      msg.data = scans.front();
      if (laser_pub_)
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
      if (goals_pub_)
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
      if (odom_pub_)
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
      if (app_fault_query_pub_)
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
      if (faults_get_pub_)
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
      if (query_faults_pub_)
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
  int svc_timeout_ms_{ 15000 };
  int publisher_frequency_{ 5 };
  int pub_interval_ms_{ 200 };

  std::string host_ip_;
  int host_port_{ 0 };

  bool enable_pub_{ true };
  bool enable_service_{ true };
  bool enable_action_{ true };

  // Service (driver)
  std::shared_ptr<SocketDriver> driver_;
  rclcpp::Service<ServiceARCL>::SharedPtr srv_;

  // Action (taskmaster)
  std::shared_ptr<SocketTaskmaster> task_master_;
  rclcpp_action::Server<ActionARCL>::SharedPtr action_;
  std::thread action_thread_;

  // Publisher (listener)
  std::shared_ptr<SocketListener> listener_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Pubs
  rclcpp::Publisher<amr_msgs::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr laser_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goals_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr app_fault_query_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr faults_get_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr query_faults_pub_;
};