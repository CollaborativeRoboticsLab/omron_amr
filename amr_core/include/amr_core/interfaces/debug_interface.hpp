#pragma once

#include <ArNetworking/ArNetworking.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace amr_core
{
class DebugInterface
{
public:
  struct Fault
  {
    std::string type;
    std::string name;
    std::string short_description;
    std::string long_description;
  };

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

  static bool readFaults(ArNetPacket* packet, std::vector<Fault>& faults)
  {
    std::uint16_t count = 0;
    if (!readFaultCount(packet, count) || count > 64) return false;
    const auto* cursor = packet->getBuf() + packet->getReadLength();
    const auto* end = cursor + packet->getDataLength() - packet->getDataReadLength();
    std::vector<Fault> parsed;
    std::set<std::string> names;
    for (std::uint16_t index = 0; index < count; ++index)
    {
      Fault fault;
      for (auto* field : { &fault.type, &fault.name, &fault.short_description, &fault.long_description })
      {
        const auto* terminator = std::find(cursor, end, '\0');
        if (terminator == end) return false;
        field->assign(cursor, terminator);
        cursor = terminator + 1;
      }
      if (fault.name.empty() || fault.name.size() > 128 ||
          std::any_of(fault.name.begin(), fault.name.end(), [](unsigned char ch) { return std::iscntrl(ch); }) ||
          std::none_of(fault.name.begin(), fault.name.end(), [](unsigned char ch) { return !std::isspace(ch); }) ||
          !names.insert(fault.name).second)
        return false;
      parsed.push_back(std::move(fault));
    }
    if (cursor != end) return false;
    faults = std::move(parsed);
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