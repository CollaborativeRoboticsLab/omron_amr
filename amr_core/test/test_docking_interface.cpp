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

TEST(DockingInterface, AwayFromDockAssertionIsRevokedByDockedFeedback)
{
  Docking docking({}, 1.0, 30.0, false);
  const auto now = Docking::Clock::now();
  EXPECT_TRUE(docking.readiness(now).unverified_start_allowed);
  docking.update(Docking::Feedback::Docked, now);
  EXPECT_EQ(docking.readiness(now).state, Docking::State::Docked);
  EXPECT_FALSE(docking.readiness(now).unverified_start_allowed);
  EXPECT_EQ(docking.readiness(now + 1s).state, Docking::State::Unknown);
  EXPECT_FALSE(docking.readiness(now + 1s).unverified_start_allowed);
}

TEST_F(DriverInterfaceLoopbackTest, UndockedStartupCanDriveWithoutDockingBackend)
{
  node_->declare_parameter<bool>("driver.undocking.start_from_dock", false);
  bool ready = false;
  auto subscription = node_->create_subscription<std_msgs::msg::Bool>(
      "amr/drive_ready", 10, [&ready](const std_msgs::msg::Bool& message) { ready = message.data; });
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  docking_ = driver_->dockingInterface();
  EXPECT_EQ(docking_->readiness().state, Docking::State::Unknown);
  ASSERT_TRUE(waitWithExecutor([&ready]() { return ready; }));
  publish(0.5, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(50.0, 0.0); }));
}

TEST_F(DriverInterfaceLoopbackTest, ForcedDockingFeedbackRevokesUnverifiedStartup)
{
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  node_->declare_parameter<bool>("driver.undocking.start_from_dock", false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fake_docking_state_ = 0;
    fake_docking_forced_ = true;
  }
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  docking_ = driver_->dockingInterface();
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().unverified_start_allowed; }));
  EXPECT_EQ(docking_->readiness().state, Docking::State::Unknown);
  EXPECT_FALSE(docking_->reserveUndock(true, true));
  EXPECT_FALSE(docking_->reserveShutdown());
  EXPECT_FALSE(docking_->faultAcknowledgementEligible());
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownCountdownRevokesUnverifiedStartup)
{
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  node_->declare_parameter<bool>("driver.undocking.start_from_dock", false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fake_docking_state_ = 0;
    fake_docking_shutdown_countdown_ = 1;
  }
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  docking_ = driver_->dockingInterface();
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().unverified_start_allowed; }));
  EXPECT_EQ(docking_->readiness().state, Docking::State::Unknown);
  EXPECT_FALSE(docking_->reserveUndock(true, true));
  EXPECT_FALSE(docking_->reserveShutdown());
  EXPECT_FALSE(docking_->faultAcknowledgementEligible());
}

TEST_F(DriverInterfaceLoopbackTest, DockedFeedbackStopsUndockedStartupAndStaleFeedbackDoesNotReenableIt)
{
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  node_->declare_parameter<bool>("driver.undocking.start_from_dock", false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_docking_replies_ = true;
  }
  driver_ = std::make_unique<DriverInterface>(node_);
  initializeDriver();
  docking_ = driver_->dockingInterface();
  publish(0.5, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(50.0, 0.0); }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_docking_replies_ = false;
    records_.clear();
  }
  ASSERT_TRUE(waitWithExecutor([this]() {
    publish(0.5, 0.0);
    return docking_->readiness().state == Docking::State::Docked;
  }));
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
  }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_docking_replies_ = true;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->readiness().state == Docking::State::Unknown; }));
  EXPECT_FALSE(docking_->readiness().unverified_start_allowed);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
  }
  publish(0.5, 0.0);
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 300ms; }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(std::none_of(records_.begin(), records_.end(), [](const Record& record) { return !record.stop; }));
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

TEST_F(DriverInterfaceLoopbackTest, UndockedReadinessRemainsFreshDuringNormalMotion)
{
  using Docking = amr_core::DockingInterface;
  ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
  ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
  node_->declare_parameter<bool>("driver.undocking.contact_dock_confirmed", true);
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
  node_->declare_parameter<bool>("driver.undocking.start_from_dock", false);
  initializeDockingBackend();
  const char* reason = nullptr;
  EXPECT_FALSE(docking_->reserveUndock(false, false, 30.0, &reason));
  EXPECT_STREQ(reason, "driver.undocking.contact_dock_confirmed and driver.undocking.exclusive_control_confirmed are false");
  EXPECT_FALSE(docking_->reserveUndock(false, true, 30.0, &reason));
  EXPECT_STREQ(reason, "driver.undocking.contact_dock_confirmed is false");
  EXPECT_FALSE(docking_->reserveUndock(true, false, 30.0, &reason));
  EXPECT_STREQ(reason, "driver.undocking.exclusive_control_confirmed is false");
  ASSERT_TRUE(docking_->reserveUndock(true, true, 30.0, &reason));
  EXPECT_EQ(reason, nullptr);
  EXPECT_FALSE(docking_->reserveUndock(true, true, 30.0, &reason));
  EXPECT_STREQ(reason, "another undocking operation is reserved");
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
    EXPECT_GE(suppressed_docking_queries_, 1);
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
  node_->declare_parameter<bool>("driver.undocking.contact_dock_confirmed", true);
  node_->declare_parameter<bool>("driver.undocking.exclusive_control_confirmed", true);
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
