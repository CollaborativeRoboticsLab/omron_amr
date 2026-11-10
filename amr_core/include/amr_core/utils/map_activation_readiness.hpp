#pragma once

#include "amr_core/utils/docking_protocol.hpp"
#include "amr_core/utils/drive_telemetry.hpp"

#include <cstdlib>

namespace amr_core
{
class MapActivationReadiness
{
public:
  void docking(ArNetPacket* packet)
  {
    DockInfo info;
    if (!readDockInfo(packet, info)) { failed = true; return; }
    ++dock_samples;
    if (info.state != DockInfo::State::Docked || info.shutdown_countdown != 0) failed = true;
  }
  void numbers(ArNetPacket* packet)
  {
    DriveTelemetry telemetry;
    if (!readDriveTelemetry(packet, telemetry)) { failed = true; return; }
    ++motion_samples;
    if (std::abs(telemetry.forward_mm_s) > 5 || std::abs(telemetry.angular_deg_s) > 1 ||
        std::abs(telemetry.lateral_mm_s) > 5) failed = true;
  }
  bool ready() const { return !failed && dock_samples >= 1 && motion_samples >= 3; }
  int dock_samples{ 0 };
  int motion_samples{ 0 };
  bool failed{ false };
};
}  // namespace amr_core