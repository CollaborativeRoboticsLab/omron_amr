#pragma once

#include <ArNetworking/ArNetworking.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <string>

namespace amr_core
{
class DebugInterface
{
public:
  static bool readMotorState(ArNetPacket* packet, std::uint8_t& state)
  {
    if (!packet || !packet->isValid() || packet->getDataLength() - packet->getDataReadLength() != 1)
      return false;
    const auto value = packet->bufToUByte();
    if (!packet->isValid() || value > 3) return false;
    state = value;
    return true;
  }

  static bool readFaultCount(ArNetPacket* packet, std::uint16_t& count)
  {
    if (!packet || !packet->isValid() || packet->getDataLength() - packet->getDataReadLength() < 2)
      return false;
    const auto value = packet->bufToUByte2();
    if (!packet->isValid() || (value == 0 && packet->getDataLength() != packet->getDataReadLength()))
      return false;
    count = value;
    return true;
  }

  static nlohmann::json discover(const std::map<unsigned int, ArClientData*>& data_map)
  {
    auto services = nlohmann::json::array();
    for (const auto& entry : data_map)
    {
      auto& data = *entry.second;
      std::string name = data.getName();
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char character) { return std::tolower(character); });
      if (name.find("debug") == std::string::npos && name.find("fault") == std::string::npos &&
          name.find("status") == std::string::npos && name.find("motor") == std::string::npos &&
          name.find("inhibit") == std::string::npos && name.find("safety") == std::string::npos &&
          name.find("estop") == std::string::npos && name.find("mode") == std::string::npos &&
          name != "updatestrings")
      {
        continue;
      }
      services.push_back({ { "name", data.getName() }, { "description", data.getDescription() },
                           { "arguments", data.getArgumentDescription() }, { "returns", data.getReturnDescription() },
                           { "read_only_verified", false } });
    }
    return services;
  }
};
}  // namespace amr_core