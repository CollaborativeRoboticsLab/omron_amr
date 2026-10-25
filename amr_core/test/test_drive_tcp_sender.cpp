#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <poll.h>
#include <netinet/in.h>

#include "amr_core/utils/drive_tcp_sender.hpp"
#include "amr_core/utils/drive_client.hpp"

TEST(DriveClient, RejectsMissingOrRepeatedConfiguration)
{
  amr_core::DriveClient client;
  EXPECT_FALSE(client.blockingConnect("127.0.0.1", 1));
  EXPECT_FALSE(client.configureTransport(0, 4096, 1.0));
  EXPECT_TRUE(client.configureTransport(4, 4096, 1.0));
  EXPECT_FALSE(client.configureTransport(4, 4096, 1.0));
}

class DriveClientTcpTest : public ::testing::Test
{
protected:
  class Client : public amr_core::DriveClient
  {
  public:
    explicit Client(amr_core::DriveTcpSender::Now now) : DriveClient(std::move(now))
    {
    }

    void attachSocket(int descriptor)
    {
      myTcpSocket.copy(descriptor, true);
      myTcpOnlyFrom = true;
      myTcpOnlyTo = true;
      internalSwitchState(STATE_CONNECTED);
    }
  };

  DriveClientTcpTest() : client_([this]() { return now_; })
  {
  }

  void SetUp() override
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    ASSERT_GE(listener_, 0);
    const int buffer_size = 4096;
    ASSERT_EQ(::setsockopt(listener_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)), 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
    ASSERT_EQ(::listen(listener_, 1), 0);
    socklen_t length = sizeof(address);
    ASSERT_EQ(::getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &length), 0);
    const auto descriptor = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    ASSERT_GE(descriptor, 0);
    client_.attachSocket(descriptor);
    ASSERT_EQ(::setsockopt(descriptor, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size)), 0);
    ASSERT_EQ(::connect(descriptor, reinterpret_cast<sockaddr*>(&address), length), 0);
    peer_ = ::accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    ASSERT_GE(peer_, 0);
    ASSERT_TRUE(client_.getTcpSocket()->setNonBlock());
    ASSERT_TRUE(client_.configureTransport(32, 65536, 0.1));
    client_.setTransportFailureHandler([this]() { ++failures_; });
  }

  void TearDown() override
  {
    client_.setTransportFailureHandler({});
    if (peer_ >= 0)
    {
      ::close(peer_);
    }
    if (listener_ >= 0)
    {
      ::close(listener_);
    }
  }

  void enqueueLargePacket()
  {
    ArNetPacket packet;
    const std::string payload(8192, 'x');
    packet.dataToBuf(payload.data(), payload.size());
    ASSERT_TRUE(client_.requestOnceByCommand(256, &packet));
  }

  void stallCycle()
  {
    now_ += std::chrono::milliseconds(100);
  }

  void runStalledCycle()
  {
    ArFunctorC<DriveClientTcpTest> callback(this, &DriveClientTcpTest::stallCycle);
    client_.addCycleCallback(&callback);
    client_.loopOnce();
    client_.remCycleCallback(&callback);
  }

  std::string readBytes(std::size_t count)
  {
    std::string result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (result.size() < count && std::chrono::steady_clock::now() < deadline)
    {
      pollfd descriptor{ peer_, POLLIN, 0 };
      if (::poll(&descriptor, 1, 50) <= 0)
      {
        continue;
      }
      char buffer[4096];
      const auto received = ::recv(peer_, buffer, std::min(sizeof(buffer), count - result.size()), MSG_DONTWAIT);
      if (received <= 0)
      {
        break;
      }
      result.append(buffer, static_cast<std::size_t>(received));
    }
    return result;
  }

  amr_core::DriveTcpSender::Clock::time_point now_{};
  Client client_;
  int listener_{ -1 };
  int peer_{ -1 };
  int failures_{ 0 };
};

TEST_F(DriveClientTcpTest, PreservesSdkControlAndMotionPacketOrder)
{
  ASSERT_TRUE(client_.requestByCommand(256, 50));
  ArNetPacket motion;
  motion.doubleToBuf(50.0);
  ASSERT_TRUE(client_.requestOnceByCommand(256, &motion));
  ArNetPacket control;
  control.setCommand(ArClientCommands::REQUEST);
  control.uByte2ToBuf(256);
  control.byte4ToBuf(50);
  control.finalizePacket();
  const std::string expected =
      std::string(control.getBuf(), control.getLength()) + std::string(motion.getBuf(), motion.getLength());
  client_.loopOnce();
  EXPECT_FALSE(client_.hasTransportFailure());
  EXPECT_EQ(readBytes(expected.size()), expected);
}

TEST_F(DriveClientTcpTest, BoundsSdkStagingBeforeAnyNetworkWrite)
{
  for (int packet = 0; packet < 12; ++packet)
  {
    enqueueLargePacket();
  }
  client_.loopOnce();
  EXPECT_TRUE(client_.hasTransportFailure());
  EXPECT_EQ(client_.getPendingPacketCount(), 0U);
  EXPECT_EQ(failures_, 1);
  EXPECT_TRUE(readBytes(1).empty());
}

TEST_F(DriveClientTcpTest, NonReadingPeerTriggersBoundedFaultWithoutRearming)
{
  for (int packet = 0; packet < 1000 && !client_.hasTransportFailure(); ++packet)
  {
    enqueueLargePacket();
    client_.loopOnce();
  }
  EXPECT_TRUE(client_.hasTransportFailure());
  EXPECT_FALSE(client_.isConnected());
  EXPECT_EQ(client_.getPendingPacketCount(), 0U);
  EXPECT_EQ(failures_, 1);
  EXPECT_FALSE(client_.requestOnceByCommand(256));
  client_.loopOnce();
  EXPECT_EQ(failures_, 1);
  EXPECT_FALSE(client_.configureTransport(32, 65536, 0.1));
}

TEST_F(DriveClientTcpTest, ExpiresBacklogBeforePeerResumes)
{
  for (int packet = 0; packet < 1000 && client_.getPendingPacketCount() == 0; ++packet)
  {
    enqueueLargePacket();
    client_.loopOnce();
  }
  ASSERT_FALSE(client_.hasTransportFailure());
  ASSERT_GT(client_.getPendingPacketCount(), 0U);
  now_ += std::chrono::milliseconds(100);
  client_.loopOnce();
  EXPECT_TRUE(client_.hasTransportFailure());
  EXPECT_FALSE(client_.isConnected());
  EXPECT_EQ(client_.getPendingPacketCount(), 0U);
  EXPECT_EQ(failures_, 1);
  EXPECT_FALSE(client_.requestOnceByCommand(256));
}

TEST_F(DriveClientTcpTest, RejectsStalledStagingBeforeAnyNetworkWrite)
{
  enqueueLargePacket();
  runStalledCycle();
  EXPECT_TRUE(client_.hasTransportFailure());
  EXPECT_EQ(failures_, 1);
  EXPECT_TRUE(readBytes(1).empty());
}

class TestQueueSender : public amr_core::DriveTcpSender
{
public:
  TestQueueSender()
    : DriveTcpSender([this](const char*, std::size_t) { return partial_write_ ? exchangePartialWrite() : 0; },
                     [this]() { ++abort_count; }, [this]() { return now_; })
  {
  }

  void expireQueuedPacket()
  {
    now_ += std::chrono::seconds(2);
  }

  void simulateExpiredPartialWrite()
  {
    partial_write_ = true;
    EXPECT_TRUE(sendData());
    expireQueuedPacket();
  }

  int abort_count{ 0 };

private:
  std::ptrdiff_t exchangePartialWrite()
  {
    partial_write_ = false;
    return 1;
  }

  Clock::time_point now_{};
  bool partial_write_{ false };
};

TEST(DriveTcpSender, BoundsPacketCountAndLatchesFault)
{
  TestQueueSender sender;
  ASSERT_TRUE(sender.setQueueLimits(2, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  EXPECT_TRUE(sender.trySendPacket(&packet));
  EXPECT_TRUE(sender.trySendPacket(&packet));
  EXPECT_EQ(sender.getPendingPacketCount(), 2U);
  EXPECT_FALSE(sender.trySendPacket(&packet));
  EXPECT_TRUE(sender.hasQueueFailure());
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
  EXPECT_FALSE(sender.trySendPacket(&packet));
  EXPECT_FALSE(sender.setQueueLimits(10, 4096, 1.0));
  EXPECT_EQ(sender.abort_count, 1);
}

TEST(DriveTcpSender, BoundsBytesWithoutOverflow)
{
  TestQueueSender sender;
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.setQueueLimits(10, packet.getLength(), 1.0));
  EXPECT_TRUE(sender.trySendPacket(&packet));
  EXPECT_FALSE(sender.trySendPacket(&packet));
  EXPECT_TRUE(sender.hasQueueFailure());
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
}

TEST(DriveTcpSender, DropsExpiredPacketsBeforeAnyWrite)
{
  TestQueueSender sender;
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet));
  sender.expireQueuedPacket();
  EXPECT_FALSE(sender.sendData());
  EXPECT_TRUE(sender.hasQueueFailure());
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
}

TEST(DriveTcpSender, DropsExpiredPartialPacketAndFollowingPackets)
{
  TestQueueSender sender;
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet));
  ASSERT_TRUE(sender.trySendPacket(&packet));
  sender.simulateExpiredPartialWrite();
  EXPECT_EQ(sender.getPendingPacketCount(), 2U);
  EXPECT_FALSE(sender.sendData());
  EXPECT_TRUE(sender.hasQueueFailure());
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
}

TEST(DriveTcpSender, RejectsExpiredQueueBeforeAcceptingNewData)
{
  TestQueueSender sender;
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet));
  sender.expireQueuedPacket();
  EXPECT_FALSE(sender.trySendPacket(&packet));
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
}

TEST(DriveTcpSender, RejectsInvalidLimits)
{
  TestQueueSender sender;
  EXPECT_FALSE(sender.setQueueLimits(0, 4096, 1.0));
  EXPECT_FALSE(sender.setQueueLimits(4, 0, 1.0));
  EXPECT_FALSE(sender.setQueueLimits(4, 4096, 0.0));
  EXPECT_FALSE(sender.setQueueLimits(4, 4096, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(sender.setQueueLimits(4, 4096, std::numeric_limits<double>::quiet_NaN()));
}

TEST(DriveTcpSender, PreservesFramingAcrossPartialWrites)
{
  std::string written;
  bool blocked = false;
  amr_core::DriveTcpSender sender([&](const char* data, std::size_t remaining) -> std::ptrdiff_t {
    blocked = !blocked;
    if (!blocked)
    {
      return 0;
    }
    const auto count = std::min<std::size_t>(remaining, 3);
    written.append(data, count);
    return static_cast<std::ptrdiff_t>(count);
  });
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket first;
  first.setCommand(256);
  first.byteToBuf(42);
  first.finalizePacket();
  ArNetPacket second;
  second.setCommand(257);
  second.byteToBuf(17);
  second.finalizePacket();
  const std::string expected =
      std::string(first.getBuf(), first.getLength()) + std::string(second.getBuf(), second.getLength());
  ASSERT_TRUE(sender.trySendPacket(&first));
  ASSERT_TRUE(sender.trySendPacket(&second));
  for (int attempt = 0; attempt < 20 && sender.getPendingPacketCount() != 0; ++attempt)
  {
    ASSERT_TRUE(sender.sendData());
  }
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
  EXPECT_EQ(written, expected);
}

TEST(DriveTcpSender, WriteFailureDiscardsDataAndAbortsOnce)
{
  int abort_count = 0;
  amr_core::DriveTcpSender sender([](const char*, std::size_t) { return -1; }, [&]() { ++abort_count; });
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet));
  EXPECT_FALSE(sender.sendData());
  EXPECT_FALSE(sender.sendData());
  EXPECT_FALSE(sender.trySendPacket(&packet));
  EXPECT_TRUE(sender.hasQueueFailure());
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
  EXPECT_EQ(abort_count, 1);
}

TEST(DriveTcpSender, ExpiresBeforeWriteEvenIfPreviousAttemptMadeProgress)
{
  amr_core::DriveTcpSender::Clock::time_point now{};
  int writes = 0;
  bool blocked = false;
  amr_core::DriveTcpSender sender(
      [&](const char*, std::size_t) {
        ++writes;
        blocked = !blocked;
        return blocked ? 1 : 0;
      },
      {}, [&]() { return now; });
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 1.0));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet));
  ASSERT_TRUE(sender.sendData());
  ASSERT_EQ(writes, 2);
  now += std::chrono::seconds(1);
  EXPECT_FALSE(sender.sendData());
  EXPECT_EQ(writes, 2);
  EXPECT_EQ(sender.getPendingPacketCount(), 0U);
}

TEST(DriveTcpSender, PreservesOriginalTimestampAcrossStaging)
{
  const amr_core::DriveTcpSender::Clock::time_point queued_at{};
  auto now = queued_at + std::chrono::milliseconds(50);
  int writes = 0;
  amr_core::DriveTcpSender sender(
      [&](const char*, std::size_t) {
        ++writes;
        return 0;
      },
      {}, [&]() { return now; });
  ASSERT_TRUE(sender.setQueueLimits(4, 4096, 0.1));
  ArNetPacket packet;
  packet.finalizePacket();
  ASSERT_TRUE(sender.trySendPacket(&packet, queued_at));
  now += std::chrono::milliseconds(50);
  EXPECT_FALSE(sender.sendData());
  EXPECT_EQ(writes, 0);
}