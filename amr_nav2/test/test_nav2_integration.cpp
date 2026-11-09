#include <nav2_velocity_smoother/velocity_smoother.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include "robot_interface_test_fixture.hpp"

TEST_F(DriverInterfaceLoopbackTest, CoreNodeUndockActionUsesNav2ContractAndEnforcesDeadline)
{
  using Action = nav2_msgs::action::UndockRobot;
  driver_.reset();
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 0;
  }
  const auto port_argument = std::string("robot.port:=") + std::to_string(server_.getTcpPort());
  core_process_ = fork();
  ASSERT_GE(core_process_, 0);
  if (core_process_ == 0)
  {
    execl(AMR_CORE_EXECUTABLE, AMR_CORE_EXECUTABLE, "--ros-args", "-r", "__ns:=/dock_action_test", "-p",
          "robot.ip:=127.0.0.1", "-p", port_argument.c_str(), "-p", "robot.user:=loopback", "-p", "robot.protocol:=''",
          "-p", "status.publish:=false", "-p", "laser.main_laser.enabled:=false", "-p", "driver.tf.enabled:=false",
          "-p", "driver.cmd_vel.topic:=/drive_test/cmd_vel", static_cast<char*>(nullptr));
    _exit(127);
  }
  std::vector<std::string> warnings;
  auto logs = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 100, [&warnings](const rcl_interfaces::msg::Log& message) {
        if (message.name == "dock_action_test.core_node") warnings.push_back(message.msg);
      });
  auto client = rclcpp_action::create_client<Action>(node_, "/dock_action_test/undock_robot");
  ASSERT_TRUE(waitWithExecutor([&client]() { return client->action_server_is_ready(); }));
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 700ms; }));
  Action::Goal goal;
  auto rejected = client->async_send_goal(goal);
  ASSERT_TRUE(waitWithExecutor([&rejected]() { return rejected.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_FALSE(rejected.get());
  ASSERT_TRUE(waitWithExecutor([&warnings]() {
    return std::any_of(warnings.begin(), warnings.end(), [](const std::string& message) {
      return message.find("Undock goal rejected: driver.docking.contact_dock_confirmed and "
                          "driver.drive.exclusive_control_confirmed are false") != std::string::npos;
    });
  }));
  auto parameters = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/dock_action_test/core_node");
  ASSERT_TRUE(waitWithExecutor([&parameters]() { return parameters->service_is_ready(); }));
  auto confirmed = parameters->set_parameters({ rclcpp::Parameter("driver.docking.contact_dock_confirmed", true),
                                                rclcpp::Parameter("driver.drive.exclusive_control_confirmed", true) });
  ASSERT_TRUE(waitWithExecutor([&confirmed]() { return confirmed.wait_for(0ms) == std::future_status::ready; }));
  for (const auto& result : confirmed.get())
    ASSERT_TRUE(result.successful);
  goal.dock_type = "unsupported";
  auto invalid_type = client->async_send_goal(goal);
  ASSERT_TRUE(waitWithExecutor([&invalid_type]() { return invalid_type.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_FALSE(invalid_type.get());
  ASSERT_TRUE(waitWithExecutor([&warnings]() {
    return std::any_of(warnings.begin(), warnings.end(), [](const std::string& message) {
      return message.find("Undock goal rejected: dock_type does not match") != std::string::npos;
    });
  }));
  goal.dock_type = "omron";
  for (const auto timeout :
       { 0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() })
  {
    goal.max_undocking_time = timeout;
    auto invalid_time = client->async_send_goal(goal);
    ASSERT_TRUE(
        waitWithExecutor([&invalid_time]() { return invalid_time.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_FALSE(invalid_time.get());
  }
  goal.max_undocking_time = 10.0f;
  int feedback_count = 0;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&feedback_count](auto, auto) { ++feedback_count; };
  auto accepted = client->async_send_goal(goal, options);
  ASSERT_TRUE(waitWithExecutor([&accepted]() { return accepted.wait_for(0ms) == std::future_status::ready; }));
  auto handle = accepted.get();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(result.get().result->error_code, Action::Result::NONE);
  EXPECT_TRUE(result.get().result->error_msg.empty());
  EXPECT_GT(feedback_count, 0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(undock_requests_, 1);
    EXPECT_TRUE(records_.empty());
    fake_docking_state_ = 2;
  }
  const auto redocked = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([redocked]() { return std::chrono::steady_clock::now() - redocked > 250ms; }));
  goal.dock_type.clear();
  goal.max_undocking_time = 0.1f;
  auto short_goal = client->async_send_goal(goal);
  ASSERT_TRUE(waitWithExecutor([&short_goal]() { return short_goal.wait_for(0ms) == std::future_status::ready; }));
  auto short_handle = short_goal.get();
  ASSERT_TRUE(short_handle);
  auto timed_out = client->async_get_result(short_handle);
  ASSERT_TRUE(waitWithExecutor([&timed_out]() { return timed_out.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_EQ(timed_out.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(timed_out.get().result->success);
  EXPECT_EQ(timed_out.get().result->error_code, Action::Result::FAILED_TO_CONTROL);
  EXPECT_NE(timed_out.get().result->error_msg.find("Undocking deadline exceeded"), std::string::npos);
  EXPECT_NE(timed_out.get().result->error_msg.find("last dockInfo state="), std::string::npos);
  EXPECT_NE(timed_out.get().result->error_msg.find("control remains inhibited"), std::string::npos);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(undock_requests_, 2);
    ASSERT_FALSE(records_.empty());
    EXPECT_TRUE(std::all_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; }));
  }
}

class Nav2SmootherChainTest : public DriverInterfaceLoopbackTest
{
protected:
  void initializeSmootherChain()
  {
    node_->declare_parameter<double>("driver.cmd_vel.timeout_sec", 0.2);
    node_->set_parameter(rclcpp::Parameter("driver.cmd_vel.timeout_sec", 0.2));
    initializeDriver();
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__ns:=/drive_chain_test", "-r", "cmd_vel:=/drive_chain_test/cmd_vel_nav",
                        "-r", "cmd_vel_smoothed:=/drive_test/cmd_vel" });
    options.parameter_overrides({ { "use_sim_time", false },
                                  { "enable_stamped_cmd_vel", false },
                                  { "feedback", "OPEN_LOOP" },
                                  { "smoothing_frequency", 20.0 },
                                  { "velocity_timeout", 1.0 },
                                  { "max_velocity", std::vector<double>{ 1.0, 0.0, 1.0 } },
                                  { "min_velocity", std::vector<double>{ -0.1, 0.0, -1.0 } },
                                  { "max_accel", std::vector<double>{ 2.5, 0.0, 3.2 } },
                                  { "max_decel", std::vector<double>{ -2.5, 0.0, -3.2 } } });
    smoother_ = std::make_shared<nav2_velocity_smoother::VelocitySmoother>(options);
    executor_.add_node(smoother_->get_node_base_interface());
    ASSERT_EQ(smoother_->configure().label(), "inactive");
    ASSERT_EQ(smoother_->activate().label(), "active");
    upstream_ = node_->create_publisher<geometry_msgs::msg::Twist>("/drive_chain_test/cmd_vel_nav", 1);
    ASSERT_TRUE(waitWithExecutor([this]() {
      geometry_msgs::msg::Twist command;
      command.linear.x = 0.6;
      upstream_->publish(command);
      return receivedDrive(60.0, 0.0);
    }));
  }

  void TearDown() override
  {
    if (smoother_)
    {
      if (smoother_->get_current_state().label() == "active")
        smoother_->deactivate();
      if (smoother_->get_current_state().label() == "inactive")
        smoother_->cleanup();
      executor_.remove_node(smoother_->get_node_base_interface());
      smoother_.reset();
    }
    DriverInterfaceLoopbackTest::TearDown();
  }

  std::shared_ptr<nav2_velocity_smoother::VelocitySmoother> smoother_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr upstream_;
};

TEST_F(Nav2SmootherChainTest, UpstreamLossWaitsForSmootherTimeoutThenDeceleratesAndExpires)
{
  initializeSmootherChain();
  const auto upstream_lost = std::chrono::steady_clock::now();
  ASSERT_TRUE(
      waitWithExecutor([upstream_lost]() { return std::chrono::steady_clock::now() - upstream_lost >= 350ms; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(std::any_of(records_.begin(), records_.end(), [upstream_lost](const Record& record) {
      return record.received_at > upstream_lost + 250ms && !record.stop && record.translation > 0.0;
    }));
    EXPECT_FALSE(std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; }));
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(0.0, 0.0); }));
  std::chrono::steady_clock::time_point zero_received;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto zero = std::find_if(records_.begin(), records_.end(), [](const Record& record) {
      return !record.stop && record.translation == 0.0 && record.rotation == 0.0;
    });
    ASSERT_NE(zero, records_.end());
    zero_received = zero->received_at;
  }
  EXPECT_GE(zero_received - upstream_lost, 900ms);
  EXPECT_LT(zero_received - upstream_lost, 2s);
  RecordProperty("upstream_loss_to_zero_sec", std::chrono::duration<double>(zero_received - upstream_lost).count());
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !records_.empty() && records_.back().stop;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GE(records_.back().received_at - zero_received, 150ms);
    EXPECT_LT(records_.back().received_at - zero_received, 700ms);
    RecordProperty("upstream_loss_to_stop_sec",
                   std::chrono::duration<double>(records_.back().received_at - upstream_lost).count());
  }
  const auto stopped = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([stopped]() { return std::chrono::steady_clock::now() - stopped >= 300ms; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(std::any_of(records_.begin(), records_.end(), [stopped](const Record& record) {
      return record.received_at > stopped && !record.stop && record.translation != 0.0;
    }));
  }
  const auto resumed = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([this, resumed]() {
    geometry_msgs::msg::Twist command;
    command.linear.x = 0.3;
    upstream_->publish(command);
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [resumed](const Record& record) {
      return record.received_at > resumed && !record.stop && std::fabs(record.translation - 30.0) < 1e-3;
    });
  }));
}

TEST_F(Nav2SmootherChainTest, SmootherFailureStopsViaDriverWatchdogWithoutExecutorProgress)
{
  initializeSmootherChain();
  ASSERT_EQ(smoother_->deactivate().label(), "inactive");
  const auto smoother_stopped = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  ASSERT_TRUE(changed_.wait_for(lock, 1s, [this]() { return !records_.empty() && records_.back().stop; }));
  EXPECT_LT(records_.back().received_at - smoother_stopped, 700ms);
  EXPECT_GE(records_.back().received_at - smoother_stopped, 100ms);
  RecordProperty("smoother_shutdown_to_stop_sec",
                 std::chrono::duration<double>(records_.back().received_at - smoother_stopped).count());
}