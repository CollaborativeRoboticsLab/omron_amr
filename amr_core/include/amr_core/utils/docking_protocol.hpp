#pragma once

#include <ArNetworking/ArNetworking.h>
#include <cstdint>

namespace amr_core
{
struct DockInfo
{
  enum class State : std::uint8_t { Undocked = 0, Docking = 1, Docked = 2, Undocking = 3 };
  State state{ State::Undocked };
  bool forced{ false };
  std::uint16_t shutdown_countdown{ 0 };
};

inline bool readDockInfo(ArNetPacket* packet, DockInfo& output)
{
  if (!packet || !packet->isValid() || packet->getDataLength() - packet->getDataReadLength() != 4)
  {
    return false;
  }
  const auto state = packet->bufToUByte();
  const auto forced = packet->bufToUByte();
  const auto shutdown = packet->bufToUByte2();
  if (!packet->isValid() || state > 3 || forced > 1)
  {
    return false;
  }
  output = { static_cast<DockInfo::State>(state), forced != 0, shutdown };
  return true;
}
}