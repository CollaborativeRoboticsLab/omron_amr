#pragma once

#include <ArNetworking/ArNetPacket.h>
#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>

namespace amr_core
{
struct DriveTelemetry
{
  int state_of_charge_tenths = 0;
  int x_mm = 0;
  int y_mm = 0;
  int heading_deg = 0;
  int forward_mm_s = 0;
  int angular_deg_s = 0;
  int lateral_mm_s = 0;

  std::array<int, 3> position() const { return { x_mm, y_mm, heading_deg }; }

  nlohmann::json toJson() const
  {
    return { { "x_mm", x_mm }, { "y_mm", y_mm }, { "heading_deg", heading_deg },
             { "forward_mm_s", forward_mm_s }, { "angular_deg_s", angular_deg_s }, { "lateral_mm_s", lateral_mm_s } };
  }
};

inline bool readDriveTelemetry(ArNetPacket* packet, DriveTelemetry& telemetry)
{
  if (packet == nullptr || !packet->isValid() || packet->getDataLength() < packet->getDataReadLength() + 19)
  {
    return false;
  }
  DriveTelemetry decoded;
  decoded.state_of_charge_tenths = packet->bufToByte2();
  decoded.x_mm = packet->bufToByte4();
  decoded.y_mm = packet->bufToByte4();
  decoded.heading_deg = packet->bufToByte2();
  decoded.forward_mm_s = packet->bufToByte2();
  decoded.angular_deg_s = packet->bufToByte2();
  decoded.lateral_mm_s = packet->bufToByte2();
  packet->bufToByte();
  if (!packet->isValid())
  {
    return false;
  }
  telemetry = decoded;
  return true;
}

inline bool readSafeDrive(ArNetPacket* packet, std::uint8_t& mode)
{
  if (packet == nullptr || !packet->isValid() || packet->getDataLength() - packet->getDataReadLength() != 1)
  {
    return false;
  }
  const auto decoded = packet->bufToUByte();
  if (!packet->isValid() || decoded > 1)
  {
    return false;
  }
  mode = decoded;
  return true;
}
}