#include <gtest/gtest.h>
#include "robot_interface_test_fixture.hpp"
#include "amr_core/interfaces/docking_interface.hpp"
#include "amr_core/utils/docking_protocol.hpp"

using Docking = amr_core::DockingInterface;
using namespace std::chrono_literals;

TEST(DockingProtocol, DecodesAdvertisedStatesAndRejectsInvalidOrTruncatedReplies)
{
  for (int state = 0; state < 4; ++state)
  {
    ArNetPacket packet;
    packet.uByteToBuf(state);
    packet.uByteToBuf(1);
    packet.uByte2ToBuf(512);
    packet.finalizePacket();
    packet.resetRead();
    amr_core::DockInfo info;
    ASSERT_TRUE(amr_core::readDockInfo(&packet, info));
    EXPECT_EQ(static_cast<int>(info.state), state);
    EXPECT_TRUE(info.forced);
    EXPECT_EQ(info.shutdown_countdown, 512);
  }
  for (int length = 0; length < 6; ++length)
  {
    if (length == 4)
      continue;
    ArNetPacket packet;
    for (int index = 0; index < length; ++index)
      packet.uByteToBuf(0);
    packet.finalizePacket();
    packet.resetRead();
    amr_core::DockInfo info{ amr_core::DockInfo::State::Docked, true, 123 };
    EXPECT_FALSE(amr_core::readDockInfo(&packet, info));
    EXPECT_EQ(info.state, amr_core::DockInfo::State::Docked);
    EXPECT_EQ(info.shutdown_countdown, 123);
  }
  for (const auto invalid : { 4, 255 })
  {
    ArNetPacket packet;
    packet.uByteToBuf(invalid);
    packet.uByteToBuf(0);
    packet.uByte2ToBuf(0);
    packet.finalizePacket();
    packet.resetRead();
    amr_core::DockInfo info;
    EXPECT_FALSE(amr_core::readDockInfo(&packet, info));
  }
}

TEST(DockingInterface, StartsUnknownAndRequiresFreshFeedback)
{
  Docking docking;
  const auto now = Docking::Clock::now();
  EXPECT_EQ(docking.readiness(now).state, Docking::State::Unknown);
  EXPECT_FALSE(docking.requestUndock(now));
  docking.update(Docking::Feedback::Undocked, now);
  EXPECT_EQ(docking.readiness(now).state, Docking::State::Ready);
  EXPECT_EQ(docking.readiness(now + 1s).state, Docking::State::Unknown);
}

TEST(DockingInterface, ExplicitUndockSubmitsOnceAndWaitsForFeedback)
{
  int requests = 0;
  Docking docking([&]() {
    ++requests;
    return true;
  });
  const auto now = Docking::Clock::now();
  docking.update(Docking::Feedback::Docked, now);
  EXPECT_EQ(requests, 0);
  ASSERT_TRUE(docking.requestUndock(now));
  EXPECT_FALSE(docking.requestUndock(now));
  EXPECT_EQ(requests, 1);
  EXPECT_EQ(docking.readiness(now).state, Docking::State::Undocking);
  docking.update(Docking::Feedback::Undocked, now + 100ms);
  EXPECT_EQ(docking.readiness(now + 100ms).state, Docking::State::Ready);
}

TEST(DockingInterface, CableUnknownAndMissingBackendPreventDeparture)
{
  int requests = 0;
  Docking docking([&]() {
    ++requests;
    return true;
  });
  docking.update(Docking::Feedback::CableConnected);
  EXPECT_FALSE(docking.requestUndock());
  docking.update(Docking::Feedback::Unknown);
  EXPECT_FALSE(docking.requestUndock());
  EXPECT_EQ(requests, 0);
  Docking missing_backend;
  missing_backend.update(Docking::Feedback::Docked);
  EXPECT_FALSE(missing_backend.requestUndock());
}

TEST(DockingInterface, FailureAndFeedbackLossLatchInhibition)
{
  const auto now = Docking::Clock::now();
  Docking rejected([]() { return false; });
  rejected.update(Docking::Feedback::Docked, now);
  EXPECT_FALSE(rejected.requestUndock(now));
  rejected.update(Docking::Feedback::Undocked, now + 10ms);
  EXPECT_EQ(rejected.readiness(now + 10ms).state, Docking::State::Failed);
  Docking silent([]() { return true; });
  silent.update(Docking::Feedback::Docked, now);
  ASSERT_TRUE(silent.requestUndock(now));
  EXPECT_EQ(silent.readiness(now + 1s).state, Docking::State::Failed);
}

TEST(DockingInterface, DeadlineIsNotExtendedByDockedFeedback)
{
  const auto now = Docking::Clock::now();
  Docking docking([]() { return true; }, 1.0, 0.5);
  docking.update(Docking::Feedback::Docked, now);
  ASSERT_TRUE(docking.requestUndock(now));
  docking.update(Docking::Feedback::Docked, now + 400ms);
  EXPECT_EQ(docking.readiness(now + 500ms).state, Docking::State::Failed);
}

TEST(DockingInterface, ExceptionsAndInvalidTimeoutsAreRejected)
{
  Docking docking([]() -> bool { throw std::runtime_error("failure"); });
  docking.update(Docking::Feedback::Docked);
  EXPECT_FALSE(docking.requestUndock());
  EXPECT_EQ(docking.readiness().state, Docking::State::Failed);
  EXPECT_THROW(Docking({}, 0.0), std::invalid_argument);
}

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
          "-p", "status.publish:=false", "-p", "laser.main_laser.enabled:=false", "-p",
          "driver.publish_robot_tf:=false", "-p", "driver.cmd_vel_topic:=/drive_test/cmd_vel",
          static_cast<char*>(nullptr));
    _exit(127);
  }
  auto client = rclcpp_action::create_client<Action>(node_, "/dock_action_test/undock_robot");
  ASSERT_TRUE(waitWithExecutor([&client]() { return client->action_server_is_ready(); }));
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 700ms; }));
  Action::Goal goal;
  auto rejected = client->async_send_goal(goal);
  ASSERT_TRUE(waitWithExecutor([&rejected]() { return rejected.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_FALSE(rejected.get());
  auto parameters = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/dock_action_test/core_node");
  ASSERT_TRUE(waitWithExecutor([&parameters]() { return parameters->service_is_ready(); }));
  auto confirmed = parameters->set_parameters({ rclcpp::Parameter("docking.contact_dock_confirmed", true),
                                                rclcpp::Parameter("docking.exclusive_control_confirmed", true) });
  ASSERT_TRUE(waitWithExecutor([&confirmed]() { return confirmed.wait_for(0ms) == std::future_status::ready; }));
  for (const auto& result : confirmed.get())
    ASSERT_TRUE(result.successful);
  goal.dock_type = "unsupported";
  auto invalid_type = client->async_send_goal(goal);
  ASSERT_TRUE(waitWithExecutor([&invalid_type]() { return invalid_type.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_FALSE(invalid_type.get());
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

TEST_F(DriverInterfaceLoopbackTest, UndockedReadinessRemainsFreshDuringNormalMotion)
{
  using Docking = amr_core::DockingInterface;
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fake_docking_state_ = 0;
    telemetry_.forward_mm_s = 0;
  }
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  docking_ = driver_->dockingInterface();
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->readiness().state == Docking::State::Ready; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 100;
  }
  const auto moving = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([moving]() { return std::chrono::steady_clock::now() - moving >= 400ms; }));
  EXPECT_EQ(docking_->readiness().state, Docking::State::Ready);
  publish(0.5, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(50.0, 0.0); }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fake_docking_state_ = 1;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->readiness().state != Docking::State::Ready; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fake_docking_state_ = 0;
  }
  const auto still_moving = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([still_moving]() { return std::chrono::steady_clock::now() - still_moving >= 400ms; }));
  EXPECT_NE(docking_->readiness().state, Docking::State::Ready);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 0;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->readiness().state == Docking::State::Ready; }));
}

TEST_F(DriverInterfaceLoopbackTest, DockingBackendRequiresExplicitConfirmedReservationAndSettledDeparture)
{
  initializeDockingBackend();
  EXPECT_FALSE(docking_->reserveUndock(false, true));
  EXPECT_FALSE(docking_->reserveUndock(true, false));
  ASSERT_TRUE(docking_->reserveUndock(true, true));
  EXPECT_FALSE(docking_->reserveUndock(true, true));
  docking_->beginUndock();
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->operationStatus().terminal; }));
  EXPECT_TRUE(docking_->operationStatus().success);
  EXPECT_TRUE(docking_->operationStatus().stationary_observed);
  EXPECT_EQ(docking_->readiness().state, amr_core::DockingInterface::State::Ready);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(undock_requests_, 1);
    EXPECT_TRUE(records_.empty());
  }
  docking_->finishUndock();
}

TEST_F(DriverInterfaceLoopbackTest, DockingFeedbackGapLatchesFailureAfterFreshUndockedReply)
{
  initializeDockingBackend();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_docking_after_undock_ = true;
  }
  ASSERT_TRUE(docking_->reserveUndock(true, true));
  docking_->beginUndock();
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
  }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(undock_requests_, 1);
    suppress_docking_replies_ = false;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->operationStatus().terminal; }));
  const auto status = docking_->operationStatus();
  EXPECT_FALSE(status.success);
  EXPECT_TRUE(status.stationary_observed);
  EXPECT_EQ(status.docking_state, 0);
  EXPECT_NE(status.message.find("Docking feedback stale during undocking"), std::string::npos);
  EXPECT_NE(status.message.find("elapsed_ms="), std::string::npos);
  const std::string age_field = "last_info_age_ms=";
  const auto age_position = status.message.find(age_field);
  ASSERT_NE(age_position, std::string::npos);
  EXPECT_GE(std::stoll(status.message.substr(age_position + age_field.size())), 1000);
  EXPECT_NE(status.message.find("last_reply_after_latest_query_ms="), std::string::npos);
  EXPECT_NE(status.message.find("query_pending="), std::string::npos);
  EXPECT_NE(status.message.find("query_age_ms="), std::string::npos);
  EXPECT_NE(status.message.find("query_timeouts="), std::string::npos);
  EXPECT_NE(status.message.find("last dockInfo state=0 (fresh)"), std::string::npos);
  EXPECT_EQ(docking_->readiness().state, Docking::State::Failed);
  docking_->finishUndock();
}

TEST_F(DriverInterfaceLoopbackTest, DockingBackendCancellationBeforeBeginNeverSubmitsDeparture)
{
  initializeDockingBackend();
  ASSERT_TRUE(docking_->reserveUndock(true, true));
  ASSERT_TRUE(docking_->cancelUndock());
  docking_->beginUndock();
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->operationStatus().terminal; }));
  EXPECT_FALSE(docking_->operationStatus().success);
  EXPECT_NE(docking_->operationStatus().message.find("Undocking canceled by caller"), std::string::npos);
  EXPECT_EQ(docking_->readiness().state, amr_core::DockingInterface::State::Failed);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(undock_requests_, 0);
  EXPECT_TRUE(std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; }));
}

TEST_F(DriverInterfaceLoopbackTest, ReadinessStatusTracksDockingInhibition)
{
  using Docking = amr_core::DockingInterface;
  docking_->update(Docking::Feedback::Docked);
  bool received = false;
  bool ready = true;
  auto subscription = node_->create_subscription<std_msgs::msg::Bool>(
      "amr/drive_ready", 10, [&received, &ready](const std_msgs::msg::Bool& message) {
        received = true;
        ready = message.data;
      });
  initializeDriver();
  ASSERT_TRUE(waitWithExecutor([&received, &ready]() { return received && !ready; }));
  docking_->update(Docking::Feedback::Undocked);
  ASSERT_TRUE(waitWithExecutor([&ready]() { return ready; }));
  docking_->update(Docking::Feedback::Unknown);
  ASSERT_TRUE(waitWithExecutor([&ready]() { return !ready; }));
}

TEST_F(DriverInterfaceLoopbackTest, MissingDockingBackendDefaultsToVelocityInhibited)
{
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  publish(0.4, 0.0);
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 100ms; }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, DockedVelocityRemainsInhibitedWithContactAndExclusiveConfirmations)
{
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  node_->declare_parameter<bool>("docking.exclusive_control_confirmed", true);
  initializeDockingBackend();
  publish(0.2, 0.0);
  const auto blocked = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([blocked]() { return std::chrono::steady_clock::now() - blocked > 150ms; }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, DockingBlocksVelocityUntilExplicitDepartureAndFreshInput)
{
  using Docking = amr_core::DockingInterface;
  docking_->update(Docking::Feedback::Docked);
  initializeDriver();
  publish(0.4, 0.0);
  const auto blocked = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([blocked]() { return std::chrono::steady_clock::now() - blocked > 100ms; }));
  EXPECT_EQ(undock_requests_, 0);
  EXPECT_FALSE(receivedDrive(40.0, 0.0));
  ASSERT_TRUE(docking_->requestUndock());
  publish(0.5, 0.0);
  const auto departing = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([departing]() { return std::chrono::steady_clock::now() - departing > 100ms; }));
  EXPECT_FALSE(receivedDrive(50.0, 0.0));
  docking_->update(Docking::Feedback::Undocked);
  const auto ready = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([ready]() { return std::chrono::steady_clock::now() - ready > 100ms; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(records_.empty());
  }
  publish(0.3, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.0); }));
  EXPECT_EQ(undock_requests_, 1);
}

TEST_F(DriverInterfaceLoopbackTest, BriefReadinessLossInvalidatesAcceptedVelocity)
{
  using Docking = amr_core::DockingInterface;
  initializeDriver();
  publish(0.4, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(40.0, 0.0); }));
  docking_->update(Docking::Feedback::Unknown);
  docking_->update(Docking::Feedback::Undocked);
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !records_.empty() && records_.back().stop;
  }));
  const auto stopped = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([stopped]() { return std::chrono::steady_clock::now() - stopped > 100ms; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(records_.back().stop);
  }
  publish(0.3, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.0); }));
}
