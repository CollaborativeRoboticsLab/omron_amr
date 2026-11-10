#pragma once

#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace amr_core
{

class DriveCommandPolicy
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Command
  {
    double linear{ 0.0 };
    double angular{ 0.0 };
  };

  enum class Action { None, Drive, Stop };

  struct Decision
  {
    Action action{ Action::None };
    Command command;
  };

  DriveCommandPolicy(double send_frequency, double timeout_seconds)
  {
    if (!std::isfinite(send_frequency) || send_frequency <= 0.0 ||
        !std::isfinite(timeout_seconds) || timeout_seconds <= 0.0)
    {
      throw std::invalid_argument("Drive frequency and command timeout must be finite and positive");
    }
    const double interval_seconds = 1.0 / send_frequency;
    const double max_seconds = std::chrono::duration<double>(Clock::duration::max()).count();
    if (!std::isfinite(interval_seconds) || interval_seconds >= max_seconds || timeout_seconds >= max_seconds)
    {
      throw std::invalid_argument("Drive timing exceeds steady clock range");
    }
    send_interval_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(interval_seconds));
    timeout_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_seconds));
    if (send_interval_ <= Clock::duration::zero() || timeout_ <= Clock::duration::zero())
    {
      throw std::invalid_argument("Drive frequency and timeout exceed steady clock resolution");
    }
  }

  bool accept(Command command, TimePoint now)
  {
    if (!std::isfinite(command.linear) || !std::isfinite(command.angular))
    {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_pending_)
    {
      return false;
    }
    command_ = command;
    received_at_ = now;
    have_command_ = true;
    return true;
  }

  void requestStop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requestStopLocked();
  }

  Decision poll(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_command_ && now - received_at_ >= timeout_)
    {
      requestStopLocked();
    }
    if (!have_command_ && !stop_pending_)
    {
      return {};
    }
    if (!urgent_stop_ && scheduled_ && now < next_send_)
    {
      return {};
    }
    scheduled_ = true;
    urgent_stop_ = false;
    next_send_ = now + send_interval_;
    return { stop_pending_ ? Action::Stop : Action::Drive, command_ };
  }

  void completeStopAttempt(bool queued)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued)
    {
      stop_pending_ = false;
    }
  }

private:
  void requestStopLocked()
  {
    have_command_ = false;
    command_ = {};
    if (!stop_pending_)
    {
      stop_pending_ = true;
      urgent_stop_ = true;
    }
  }

  std::mutex mutex_;
  Clock::duration send_interval_;
  Clock::duration timeout_;
  Command command_;
  TimePoint received_at_{};
  TimePoint next_send_{};
  bool have_command_{ false };
  bool stop_pending_{ false };
  bool urgent_stop_{ false };
  bool scheduled_{ false };
};

}