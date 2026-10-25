#pragma once

#include <ArNetworking/ArNetPacket.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

namespace amr_core
{

class DriveTcpSender
{
public:
  using Clock = std::chrono::steady_clock;
  using Writer = std::function<std::ptrdiff_t(const char*, std::size_t)>;
  using Now = std::function<Clock::time_point()>;

  explicit DriveTcpSender(Writer writer = {}, std::function<void()> abort = {}, Now now = Clock::now)
    : writer_(std::move(writer)), abort_(std::move(abort)), now_(std::move(now))
  {
  }

  bool setQueueLimits(std::size_t max_packets, std::size_t max_bytes, double max_age_seconds)
  {
    if (failed_ || !packets_.empty() || max_packets == 0 || max_bytes == 0 || !std::isfinite(max_age_seconds) ||
        max_age_seconds <= 0.0)
    {
      return false;
    }
    max_packets_ = max_packets;
    max_bytes_ = max_bytes;
    max_age_seconds_ = max_age_seconds;
    return true;
  }

  bool trySendPacket(const ArNetPacket* packet)
  {
    return trySendPacket(packet, now_());
  }

  bool trySendPacket(const ArNetPacket* packet, Clock::time_point queued_at)
  {
    if (failed_)
    {
      return false;
    }
    if (expired() || std::chrono::duration<double>(now_() - queued_at).count() >= max_age_seconds_ ||
        packet == nullptr || max_packets_ == 0 || packets_.size() >= max_packets_ ||
        packet->getLength() > max_bytes_ - pending_bytes_)
    {
      return fail();
    }
    packets_.push_back({ std::vector<char>(packet->getBuf(), packet->getBuf() + packet->getLength()), queued_at, 0 });
    pending_bytes_ += packet->getLength();
    return true;
  }

  bool sendData()
  {
    if (failed_)
    {
      return false;
    }
    while (!packets_.empty())
    {
      if (expired() || !writer_)
      {
        return fail();
      }
      auto& pending = packets_.front();
      const auto remaining = pending.bytes.size() - pending.offset;
      const auto written = writer_(pending.bytes.data() + pending.offset, remaining);
      if (written < 0 || static_cast<std::size_t>(written) > remaining)
      {
        return fail();
      }
      if (written == 0)
      {
        return true;
      }
      pending.offset += static_cast<std::size_t>(written);
      if (pending.offset == pending.bytes.size())
      {
        pending_bytes_ -= pending.bytes.size();
        packets_.pop_front();
      }
    }
    return true;
  }

  bool hasQueueFailure() const
  {
    return failed_;
  }
  std::size_t getPendingPacketCount() const
  {
    return packets_.size();
  }

  bool fail()
  {
    if (!failed_)
    {
      failed_ = true;
      packets_.clear();
      pending_bytes_ = 0;
      if (abort_)
      {
        abort_();
      }
    }
    return false;
  }

private:
  struct PendingPacket
  {
    std::vector<char> bytes;
    Clock::time_point queued_at;
    std::size_t offset;
  };

  bool expired() const
  {
    return !packets_.empty() &&
           std::chrono::duration<double>(now_() - packets_.front().queued_at).count() >= max_age_seconds_;
  }

  Writer writer_;
  std::function<void()> abort_;
  Now now_;
  std::deque<PendingPacket> packets_;
  std::size_t pending_bytes_{ 0 };
  std::size_t max_packets_{ 0 };
  std::size_t max_bytes_{ 0 };
  double max_age_seconds_{ 0.0 };
  bool failed_{ false };
};

}  // namespace amr_core