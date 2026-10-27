#include <gtest/gtest.h>
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