#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <thread>

#include "amr_core/utils/drive_command_policy.hpp"

using Policy = amr_core::DriveCommandPolicy;
using Action = Policy::Action;
using namespace std::chrono_literals;

TEST(DriveCommandPolicy, StartsIdle)
{
  Policy policy(20.0, 0.2);
  EXPECT_EQ(policy.poll(Policy::TimePoint{}).action, Action::None);
}

TEST(DriveCommandPolicy, KeepsOnlyTheLatestCompleteCommand)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.1, 0.2 }, now));
  ASSERT_TRUE(policy.accept({ 0.3, -0.4 }, now + 1ms));
  const auto decision = policy.poll(now + 2ms);
  EXPECT_EQ(decision.action, Action::Drive);
  EXPECT_DOUBLE_EQ(decision.command.linear, 0.3);
  EXPECT_DOUBLE_EQ(decision.command.angular, -0.4);
}

TEST(DriveCommandPolicy, ZeroIsAnOrdinaryDriveCommand)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.2, 0.1 }, now));
  EXPECT_EQ(policy.poll(now).action, Action::Drive);
  ASSERT_TRUE(policy.accept({ 0.0, 0.0 }, now + 50ms));
  const auto zero = policy.poll(now + 50ms);
  EXPECT_EQ(zero.action, Action::Drive);
  EXPECT_DOUBLE_EQ(zero.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(zero.command.angular, 0.0);
  ASSERT_TRUE(policy.accept({ 0.3, 0.0 }, now + 100ms));
  EXPECT_EQ(policy.poll(now + 100ms).action, Action::Drive);
}

TEST(DriveCommandPolicy, RateLimitsAndDoesNotCatchUpAfterADelay)
{
  Policy policy(20.0, 1.0);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.2, 0.1 }, now));
  EXPECT_EQ(policy.poll(now).action, Action::Drive);
  EXPECT_EQ(policy.poll(now + 49ms).action, Action::None);
  EXPECT_EQ(policy.poll(now + 300ms).action, Action::Drive);
  EXPECT_EQ(policy.poll(now + 300ms).action, Action::None);
  EXPECT_EQ(policy.poll(now + 349ms).action, Action::None);
  EXPECT_EQ(policy.poll(now + 350ms).action, Action::Drive);
}

TEST(DriveCommandPolicy, ExpiresAtTheSteadyClockBoundary)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.2, 0.1 }, now));
  EXPECT_EQ(policy.poll(now).action, Action::Drive);
  EXPECT_EQ(policy.poll(now + 199ms).action, Action::Drive);
  EXPECT_EQ(policy.poll(now + 200ms).action, Action::Stop);
  policy.completeStopAttempt(true);
  EXPECT_EQ(policy.poll(now + 1s).action, Action::None);
}

TEST(DriveCommandPolicy, ZeroInputAlsoExpires)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({}, now));
  EXPECT_EQ(policy.poll(now).action, Action::Drive);
  EXPECT_EQ(policy.poll(now + 200ms).action, Action::Stop);
}

TEST(DriveCommandPolicy, StopPreemptsRateLimitAndRequiresNewInputAfterSubmission)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.2, 0.1 }, now));
  EXPECT_EQ(policy.poll(now).action, Action::Drive);
  policy.requestStop();
  EXPECT_FALSE(policy.accept({ 0.3, 0.0 }, now + 1ms));
  EXPECT_EQ(policy.poll(now + 1ms).action, Action::Stop);
  policy.completeStopAttempt(true);
  EXPECT_EQ(policy.poll(now + 100ms).action, Action::None);
  ASSERT_TRUE(policy.accept({ 0.1, 0.0 }, now + 101ms));
  EXPECT_EQ(policy.poll(now + 101ms).action, Action::Drive);
}

TEST(DriveCommandPolicy, FailedStopRetriesAreRateLimitedAndBlockMovement)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  policy.requestStop();
  EXPECT_EQ(policy.poll(now).action, Action::Stop);
  policy.completeStopAttempt(false);
  EXPECT_FALSE(policy.accept({ 0.3, 0.0 }, now + 1ms));
  policy.requestStop();
  EXPECT_EQ(policy.poll(now + 1ms).action, Action::None);
  EXPECT_EQ(policy.poll(now + 50ms).action, Action::Stop);
}

TEST(DriveCommandPolicy, InvalidCommandsDoNotRefreshFreshness)
{
  Policy policy(20.0, 0.2);
  const Policy::TimePoint now{};
  ASSERT_TRUE(policy.accept({ 0.2, 0.1 }, now));
  EXPECT_FALSE(policy.accept({ std::numeric_limits<double>::quiet_NaN(), 0.0 }, now + 100ms));
  EXPECT_FALSE(policy.accept({ 0.0, std::numeric_limits<double>::infinity() }, now + 150ms));
  EXPECT_EQ(policy.poll(now + 200ms).action, Action::Stop);
}

TEST(DriveCommandPolicy, RejectsInvalidTimingConfiguration)
{
  EXPECT_THROW(Policy(0.0, 0.2), std::invalid_argument);
  EXPECT_THROW(Policy(20.0, -1.0), std::invalid_argument);
  EXPECT_THROW(Policy(std::numeric_limits<double>::infinity(), 0.2), std::invalid_argument);
  EXPECT_THROW(Policy(20.0, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(Policy(std::numeric_limits<double>::denorm_min(), 0.2), std::invalid_argument);
  EXPECT_THROW(Policy(20.0, std::numeric_limits<double>::max()), std::invalid_argument);
  EXPECT_THROW(Policy(std::numeric_limits<double>::max(), 0.2), std::invalid_argument);
}

TEST(DriveCommandPolicy, ConcurrentUpdatesPreserveCommandPairs)
{
  Policy policy(20.0, 1.0);
  const Policy::TimePoint now{};
  std::thread producer([&policy, now]() {
    for (int sample = 1; sample <= 10000; ++sample)
    {
      policy.accept({ static_cast<double>(sample), 2.0 * sample }, now);
    }
  });
  for (int sample = 0; sample < 10000; ++sample)
  {
    const auto decision = policy.poll(now);
    if (decision.action == Action::Drive)
    {
      EXPECT_DOUBLE_EQ(decision.command.angular, 2.0 * decision.command.linear);
    }
  }
  producer.join();
  const auto decision = policy.poll(now + 50ms);
  ASSERT_EQ(decision.action, Action::Drive);
  EXPECT_DOUBLE_EQ(decision.command.linear, 10000.0);
  EXPECT_DOUBLE_EQ(decision.command.angular, 20000.0);
}