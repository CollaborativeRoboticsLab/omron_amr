#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <string>
#include <algorithm>
#include "amr_core/utils/docking_protocol.hpp"
#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/drive_telemetry.hpp"

namespace amr_core
{
class DockingInterface
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  enum class State { Unknown, Docked, Undocking, Ready, Failed };
  enum class Feedback { Unknown, Docked, Undocked, CableConnected };
  using UndockRequest = std::function<bool()>;

  explicit DockingInterface(UndockRequest request = {}, double feedback_timeout_sec = 1.0,
                            double undock_timeout_sec = 30.0)
    : request_(std::move(request)), feedback_timeout_sec_(feedback_timeout_sec),
      undock_timeout_sec_(undock_timeout_sec), info_callback_(this, &DockingInterface::handleInfo)
  {
    if (!std::isfinite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0 ||
        !std::isfinite(undock_timeout_sec_) || undock_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("Docking timeouts must be positive and finite");
    }
  }

  ~DockingInterface()
  {
    detach();
  }

  void detach()
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (client_ && supported_) client_->remHandler("dockInfo", &info_callback_);
    client_ = nullptr;
    supported_ = false;
    request_ = {};
    fail();
  }

  void attach(DriveClient& client, bool contact_confirmed)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (client_) throw std::logic_error("Docking transport is already attached");
    client_ = &client;
    contact_confirmed_ = contact_confirmed;
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      departure_contact_confirmed_ = contact_confirmed;
    }
    request_ = [this]() { return client_->requestOnce("undock", nullptr, true); };
    supported_ = client.dataExists("dockInfo") && client.dataExists("undock") && client.dataExists("stop") &&
                 client.dataExists("updateNumbers") && client.addHandler("dockInfo", &info_callback_);
  }

  bool reserveUndock(bool confirm_contact, bool confirm_exclusive, double max_undocking_time = 30.0)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    const auto now = Clock::now();
    if (!std::isfinite(max_undocking_time) || max_undocking_time <= 0.0 ||
      !supported_ || reserved_ || !mode_ready_ || !confirm_contact || !confirm_exclusive ||
        !infoFresh(now) || info_.forced || info_.shutdown_countdown != 0 ||
        info_.state != DockInfo::State::Docked || !stationary(now) || readiness().state == State::Failed)
    {
      return false;
    }
    submitted_ = stop_requested_ = stop_submitted_ = stop_queued_ = terminal_ = success_ = stopped_ = false;
    message_.clear();
    failure_reason_.clear();
    operation_timeout_sec_ = std::min(max_undocking_time, undock_timeout_sec_);
    reserved_ = true;
    return true;
  }

  void beginUndock()
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (!reserved_ || terminal_) return;
    contact_confirmed_ = true;
    request_pending_ = true;
  }

  bool cancelUndock()
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (!reserved_ || terminal_) return false;
    stop_requested_ = true;
    if (failure_reason_.empty()) failure_reason_ = "Undocking canceled by caller";
    fail();
    return true;
  }

  struct OperationStatus
  {
    bool terminal;
    bool success;
    bool stationary_observed;
    std::uint8_t docking_state;
    std::string phase;
    std::string message;
  };

  OperationStatus operationStatus()
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    return { terminal_, success_, stopped_, have_info_ ? static_cast<std::uint8_t>(info_.state) : std::uint8_t{255},
             stop_requested_ ? "stopping" : (submitted_ ? "undocking" : "preflight"), message_ };
  }

  void finishUndock()
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (terminal_) reserved_ = false;
  }

  void observe(const DriveTelemetry& telemetry)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    const auto now = Clock::now();
    const bool zero = telemetry.forward_mm_s == 0 && telemetry.angular_deg_s == 0 && telemetry.lateral_mm_s == 0;
    if (!zero || !zero_velocity_ || now - telemetry_received_ >= std::chrono::milliseconds(200))
    {
      stationary_since_ = now;
    }
    zero_velocity_ = zero;
    telemetry_received_ = now;
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      departure_telemetry_received_ = now;
      departure_have_telemetry_ = true;
      departure_stationary_ = zero && now - stationary_since_ >= std::chrono::milliseconds(500);
    }
  }

  void invalidateObservation()
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    departure_have_telemetry_ = false;
    departure_stationary_ = false;
  }

  void cycle(bool mode_ready)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    const auto now = Clock::now();
    mode_ready_ = mode_ready;
    if (!supported_ || !client_->isConnected())
    {
      if (reserved_)
      {
        fail();
        terminal_ = true;
        if (failure_reason_.empty()) failure_reason_ = "Docking connection unavailable";
        message_ = "Docking connection unavailable; stop unconfirmed; operator intervention required; " +
             failureContext(now);
      }
      return;
    }
    if (query_pending_ && now - queried_ >= std::chrono::seconds(1))
    {
      query_pending_ = false;
      update(Feedback::Unknown);
    }
    if (!query_pending_ && now - queried_ >= std::chrono::milliseconds(100))
    {
      queried_ = now;
      query_pending_ = client_->requestOnce("dockInfo", nullptr, true);
      if (!query_pending_) fail();
    }
    if (request_pending_)
    {
      request_pending_ = false;
      if (stop_requested_)
      {
        fail();
      }
      else if (!mode_ready || !client_->isConnected() || !infoFresh(now) || info_.forced ||
               info_.shutdown_countdown != 0 || info_.state != DockInfo::State::Docked || !stationary(now))
      {
        fail();
        terminal_ = true;
        message_ = "Undocking preconditions changed; no departure requested";
      }
      else
      {
        update(Feedback::Docked);
        operation_started_ = now;
        stationary_since_ = now;
        if (!requestUndock())
        {
          stop_requested_ = true;
          failure_reason_ = "Undock submission failed; outcome uncertain";
        }
        else submitted_ = true;
      }
    }
    if (reserved_ && submitted_ && !terminal_ && !stop_requested_)
    {
      if (std::chrono::duration<double>(now - operation_started_).count() >= operation_timeout_sec_)
        failure_reason_ = "Undocking deadline exceeded";
      else if (!mode_ready)
        failure_reason_ = "Confirmed drive mode lost during undocking";
      else if (!infoFresh(now))
        failure_reason_ = "Docking feedback stale during undocking";
      else if (now - telemetry_received_ >= std::chrono::milliseconds(200))
        failure_reason_ = "Velocity telemetry stale during undocking";
      else if (readiness().state == State::Failed)
        failure_reason_ = "Docking readiness failed during undocking";
      if (!failure_reason_.empty())
      {
        stop_requested_ = true;
        fail();
      }
    }
    if (stop_requested_ && !stop_submitted_)
    {
      fail();
      stop_started_ = now;
      stationary_since_ = now;
      stop_submitted_ = true;
      stop_queued_ = client_->requestOnce("stop", nullptr, true);
    }
    if (stop_submitted_ && !terminal_)
    {
      stopped_ = stop_queued_ && infoFresh(now) && info_received_ > stop_started_ &&
                 info_.state != DockInfo::State::Undocking && stationary(now) && stationary_since_ >= stop_started_;
      if (stopped_ || now - stop_started_ >= std::chrono::seconds(3))
      {
        terminal_ = true;
        message_ = stopped_ ? "Departure interrupted; stationary feedback observed; control remains inhibited" :
                             "Stop requested but not confirmed; use operator protective controls; control remains inhibited";
        message_ += "; " + failureContext(now);
      }
    }
    else if (reserved_ && submitted_ && !terminal_ && readiness().state == State::Ready)
    {
      terminal_ = true;
      success_ = true;
      stopped_ = true;
      message_ = "Undocked with fresh status and settled zero-velocity feedback";
    }
  }

  void update(Feedback feedback, TimePoint now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refresh(now);
    received_ = now;
    have_feedback_ = true;
    if (state_ == State::Failed)
    {
      return;
    }
    if (feedback == Feedback::CableConnected || feedback == Feedback::Unknown)
    {
      transition(state_ == State::Undocking ? State::Failed : State::Unknown);
    }
    else if (feedback == Feedback::Undocked)
    {
      transition(State::Ready);
    }
    else if (state_ != State::Undocking)
    {
      transition(State::Docked);
    }
  }

  bool requestUndock(TimePoint now = Clock::now())
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      refresh(now);
      if (state_ != State::Docked || !request_)
      {
        return false;
      }
      transition(State::Undocking);
      started_ = now;
    }
    bool submitted = false;
    try
    {
      submitted = request_();
    }
    catch (...)
    {
      fail();
      return false;
    }
    if (!submitted)
    {
      fail();
    }
    return submitted;
  }

  void fail()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    transition(State::Failed);
  }

  struct Readiness
  {
    State state;
    unsigned long long generation;
    bool departure_telemetry_fresh;
    bool departure_stationary;
  };

  Readiness readiness(TimePoint now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refresh(now);
    return { state_, generation_, departureTelemetryFresh(now), departure_stationary_ };
  }

  template <typename Operation>
  void withReadiness(Operation operation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    refresh(Clock::now());
    operation(Readiness{ state_, generation_, departureTelemetryFresh(Clock::now()), departure_stationary_ });
  }

private:
  bool departureTelemetryFresh(TimePoint now) const
  {
    return departure_contact_confirmed_ && departure_have_telemetry_ && now >= departure_telemetry_received_ &&
           now - departure_telemetry_received_ < std::chrono::milliseconds(200);
  }

  std::string failureContext(TimePoint now) const
  {
    return failure_reason_ + "; last dockInfo state=" +
           (have_info_ ? std::to_string(static_cast<unsigned int>(info_.state)) : "unknown") +
           " (" + (infoFresh(now) ? "fresh" : "stale or unavailable") + ")";
  }

  bool infoFresh(TimePoint now) const
  {
    return have_info_ && now - info_received_ < std::chrono::seconds(1);
  }

  bool stationary(TimePoint now) const
  {
    return zero_velocity_ && now - telemetry_received_ < std::chrono::milliseconds(200) &&
           now - stationary_since_ >= std::chrono::milliseconds(500);
  }

  void handleInfo(ArNetPacket* packet)
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    const auto now = Clock::now();
    if (!query_pending_) return;
    query_pending_ = false;
    if (now - queried_ >= std::chrono::seconds(1) || !readDockInfo(packet, info_))
    {
      fail();
      return;
    }
    have_info_ = true;
    info_received_ = now;
    if (info_.forced || info_.shutdown_countdown != 0)
    {
      update(Feedback::Unknown);
    }
    else if (info_.state == DockInfo::State::Undocked && contact_confirmed_ &&
         (readiness(now).state == State::Ready ||
          (stationary(now) && (!submitted_ || (now > operation_started_ && stationary_since_ >= operation_started_)))))
    {
      update(Feedback::Undocked);
    }
    else if (info_.state == DockInfo::State::Docked || (submitted_ && info_.state == DockInfo::State::Undocking) ||
             (submitted_ && info_.state == DockInfo::State::Undocked))
    {
      update(Feedback::Docked);
    }
    else update(Feedback::Unknown);
  }

  void transition(State next)
  {
    if (state_ != next)
    {
      state_ = next;
      ++generation_;
    }
  }

  void refresh(TimePoint now)
  {
    if (state_ == State::Failed)
    {
      return;
    }
    if (state_ == State::Undocking &&
        std::chrono::duration<double>(now - started_).count() >= undock_timeout_sec_)
    {
      transition(State::Failed);
    }
    else if (!have_feedback_ || now < received_ ||
             std::chrono::duration<double>(now - received_).count() >= feedback_timeout_sec_)
    {
      transition(state_ == State::Undocking ? State::Failed : State::Unknown);
    }
  }

  UndockRequest request_;
  double feedback_timeout_sec_;
  double undock_timeout_sec_;
  double operation_timeout_sec_{ 30.0 };
  std::mutex mutex_;
  State state_{ State::Unknown };
  TimePoint received_{};
  TimePoint started_{};
  bool have_feedback_{ false };
  unsigned long long generation_{ 0 };
  TimePoint departure_telemetry_received_{};
  bool departure_contact_confirmed_{ false }, departure_have_telemetry_{ false }, departure_stationary_{ false };
  DriveClient* client_{ nullptr };
  ArFunctor1C<DockingInterface, ArNetPacket*> info_callback_;
  std::mutex operation_mutex_;
  DockInfo info_;
  TimePoint queried_{}, info_received_{}, telemetry_received_{}, stationary_since_{}, operation_started_{}, stop_started_{};
  bool supported_{ false }, mode_ready_{ false }, contact_confirmed_{ false }, query_pending_{ false };
  bool have_info_{ false }, zero_velocity_{ false }, reserved_{ false }, request_pending_{ false };
  bool submitted_{ false }, stop_requested_{ false }, stop_submitted_{ false }, stop_queued_{ false };
  bool terminal_{ false }, success_{ false }, stopped_{ false };
  std::string message_;
  std::string failure_reason_;
};
}