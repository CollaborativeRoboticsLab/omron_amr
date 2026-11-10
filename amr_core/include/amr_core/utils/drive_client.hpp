#pragma once

#include <ArNetworking/ArNetworking.h>

#include <cerrno>
#include <functional>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "amr_core/utils/drive_tcp_sender.hpp"

namespace amr_core
{

class DriveClient : public ArClientBase
{
public:
  explicit DriveClient(DriveTcpSender::Now now = DriveTcpSender::Clock::now)
    : packet_callback_(this, &DriveClient::queuePacket),
      sender_([this](const char* data, std::size_t size) -> std::ptrdiff_t {
        const auto written = ::send(myTcpSocket.getFD(), data, size, MSG_DONTWAIT | MSG_NOSIGNAL);
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        {
          return 0;
        }
        return written == 0 ? -1 : written;
      }, [this]() { abortConnection(); }, now), now_(std::move(now))
  {
  }

  ~DriveClient() override
  {
    stopRunning();
    if (thread_started_)
    {
      join();
    }
    finishNonBlockingDisconnect();
    myTcpSender.setSocket(&myTcpSocket);
  }

  bool configureTransport(std::size_t max_packets, std::size_t max_bytes, double max_age_seconds)
  {
    if (configured_ || !sender_.setQueueLimits(max_packets, max_bytes, max_age_seconds))
    {
      return false;
    }
    int sockets[2];
    if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sockets) != 0)
    {
      return false;
    }
    staging_writer_.copy(sockets[0], true);
    staging_reader_.copy(sockets[1], true);
    staging_receiver_.setSocket(&staging_reader_);
    staging_receiver_.setProcessPacketCB(&packet_callback_);
    myTcpSender.setSocket(&staging_writer_);
    max_staged_bytes_ = max_bytes;
    max_age_seconds_ = max_age_seconds;
    configured_ = true;
    return true;
  }

  void setTransportFailureHandler(std::function<void()> handler)
  {
    failure_handler_ = std::move(handler);
  }

  bool hasTransportFailure() const
  {
    return sender_.hasQueueFailure();
  }

  std::size_t getPendingPacketCount() const
  {
    return sender_.getPendingPacketCount();
  }

  bool blockingConnect(const char* host, int port, bool log = true, const char* user = nullptr,
                       const char* password = nullptr)
  {
    if (!configured_ || connection_attempted_ || hasTransportFailure())
    {
      return false;
    }
    connection_attempted_ = true;
    auto result = internalNonBlockingConnectStart(host, port, log, user, password, nullptr);
    while (result == NON_BLOCKING_CONTINUE)
    {
      const auto started = now_();
      result = internalNonBlockingConnectContinue();
      if (!pumpTransport(started))
      {
        return false;
      }
      if (result == NON_BLOCKING_CONTINUE)
      {
        ArUtil::sleep(1);
      }
    }
    return result == NON_BLOCKING_CONNECTED;
  }

  void loopOnce()
  {
    if (!configured_ || hasTransportFailure())
    {
      return;
    }
    if (!sender_.sendData())
    {
      return;
    }
    const auto started = now_();
    ArClientBase::loopOnce();
    if (myState == STATE_CONNECTED)
    {
      pumpTransport(started);
    }
    else
    {
      sender_.fail();
    }
  }

  bool disconnect()
  {
    if (isConnected() && !hasTransportFailure())
    {
      const auto started = now_();
      startNonBlockingDisconnect();
      pumpTransport(started);
    }
    finishNonBlockingDisconnect();
    return !hasTransportFailure();
  }

  void runAsync()
  {
    thread_started_ = true;
    ArClientBase::runAsync();
  }

  int join(void** result = nullptr) override
  {
    if (!thread_started_)
    {
      return 0;
    }
    const auto status = ArClientBase::join(result);
    if (status == 0)
    {
      thread_started_ = false;
    }
    return status;
  }

  void* runThread(void*) override
  {
    threadStarted();
    while (myRunning)
    {
      loopOnce();
      ArUtil::sleep(1);
    }
    threadFinished();
    return nullptr;
  }

private:
  class StagingReceiver : public ArNetPacketReceiverTcp
  {
  public:
    bool atPacketBoundary() const
    {
      return myState == STATE_SYNC1;
    }
  };

  bool pumpTransport(DriveTcpSender::Clock::time_point started)
  {
    if (hasTransportFailure())
    {
      return false;
    }
    std::size_t staged_bytes = 0;
    batch_started_ = started;
    for (;;)
    {
      if (!myTcpSender.sendData())
      {
        return sender_.fail();
      }
      int available = 0;
      if (::ioctl(staging_reader_.getFD(), FIONREAD, &available) != 0 || available < 0 ||
          static_cast<std::size_t>(available) > max_staged_bytes_ - staged_bytes)
      {
        return sender_.fail();
      }
      if (available == 0)
      {
        if (!staging_receiver_.atPacketBoundary())
        {
          return sender_.fail();
        }
        break;
      }
      staged_bytes += static_cast<std::size_t>(available);
      if (std::chrono::duration<double>(now_() - started).count() >= max_age_seconds_ ||
          !staging_receiver_.readData() || hasTransportFailure())
      {
        return sender_.fail();
      }
    }
    if (staged_bytes != 0 && std::chrono::duration<double>(now_() - started).count() >= max_age_seconds_)
    {
      return sender_.fail();
    }
    return sender_.sendData();
  }

  void queuePacket(ArNetPacket* packet)
  {
    sender_.trySendPacket(packet, batch_started_);
  }

  void abortConnection()
  {
    myTcpSocket.setLinger(0);
    myTcpSocket.close();
    myUdpSocket.close();
    staging_writer_.close();
    staging_reader_.close();
    internalSwitchState(STATE_LOST_CONNECTION);
    stopRunning();
    if (failure_handler_)
    {
      failure_handler_();
    }
  }

  ArSocket staging_writer_;
  ArSocket staging_reader_;
  StagingReceiver staging_receiver_;
  ArFunctor1C<DriveClient, ArNetPacket*> packet_callback_;
  DriveTcpSender sender_;
  DriveTcpSender::Now now_;
  std::function<void()> failure_handler_;
  std::size_t max_staged_bytes_{ 0 };
  double max_age_seconds_{ 0.0 };
  DriveTcpSender::Clock::time_point batch_started_{};
  bool configured_{ false };
  bool connection_attempted_{ false };
  bool thread_started_{ false };
};

}