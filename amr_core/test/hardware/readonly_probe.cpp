#include <ArNetworking/ArNetworking.h>
#include "amr_core/utils/drive_client.hpp"
#include "hybrid_config_client.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <string>

namespace
{
const std::set<std::string> drive_parameters = {
  "FullThrottleForwards", "FullThrottleBackwards", "RotAtStopped",     "RotAtFullForwards", "RotAtFullBackwards",
  "TransDeadZone",        "RotDeadZone",           "TransVelMax",      "RotVelMax",         "TeleopMaxTransVel",
  "TeleopMaxRotVel",      "TeleopTransAccel",      "TeleopTransDecel", "TeleopRotAccel",    "TeleopRotDecel",
  "TeleopTimeout",        "TeleopCommandTimeout",
};

bool relevantName(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char value) { return std::tolower(value); });
  for (const auto* term : { "teleop", "throttle", "deadzone", "timeout", "speed", "version", "firmware", "accel",
                            "decel", "transvel", "rotvel" })
  {
    if (name.find(term) != std::string::npos)
    {
      return true;
    }
  }
  return false;
}

int captureConfiguration(amr_core::DriveClient& client)
{
  nlohmann::json snapshot = { { "schema_version", 1 },
                              { "read_only", true },
                              { "parameters", nlohmann::json::array() },
                              { "candidate_names", nlohmann::json::array() },
                              { "discovery_services", nlohmann::json::array() } };
  bool available = false;
  for (const auto* service : { "getConfigBySectionsV4", "getConfigBySectionsV3", "getConfigBySectionsV2",
                               "getConfigBySections", "getConfig" })
  {
    if (client.dataExists(service))
    {
      available = true;
      snapshot["config_service"] = service;
      break;
    }
  }
  for (const auto& entry : *client.getDataMap())
  {
    const std::string name = entry.second->getName();
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char value) { return std::tolower(value); });
    if (lower.find("config") != std::string::npos || lower.find("version") != std::string::npos ||
        lower.find("info") != std::string::npos)
    {
      snapshot["discovery_services"].push_back({ { "name", name },
        { "arguments", entry.second->getArgumentDescription() }, { "returns", entry.second->getReturnDescription() } });
    }
  }
  if (!available && client.dataExists("getConfigValues"))
  {
    amr_hardware::captureHybridConfiguration(client, snapshot);
    std::cout << "CAPTURE " << snapshot.dump() << std::endl;
    return snapshot["complete"].get<bool>() ? 0 : 1;
  }
  ArClientHandlerConfig handler(&client);
  handler.setQuiet(true);
  if (available)
  {
    handler.requestConfigFromServer();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (client.isConnected() && !handler.haveGottenConfig() && std::chrono::steady_clock::now() < deadline)
    {
      client.loopOnce();
    }
  }
  snapshot["complete"] =
      available && handler.haveGottenConfig() && client.isConnected() && !client.hasTransportFailure();
  if (!available)
  {
    snapshot["failure_reason"] = "No supported SDK getConfig service is advertised; vendor-supported export required.";
  }
  else if (!snapshot["complete"].get<bool>())
  {
    snapshot["failure_reason"] = "Configuration transfer did not complete within the bounded connection.";
  }
  if (snapshot["complete"].get<bool>())
  {
    for (auto* section : *handler.getConfig()->getSections())
    {
      for (const auto& parameter : *section->getParams())
      {
        const std::string name = parameter.getName();
        if (relevantName(name))
        {
          snapshot["candidate_names"].push_back({ { "section", section->getName() }, { "name", name } });
        }
        if (drive_parameters.count(name) == 0)
        {
          continue;
        }
        bool valid = false;
        const auto integer = parameter.getInt(&valid);
        nlohmann::json value = nullptr;
        if (valid)
        {
          value = integer;
        }
        else
        {
          const auto number = parameter.getDouble(&valid);
          if (valid && std::isfinite(number))
          {
            value = number;
          }
        }
        snapshot["parameters"].push_back({ { "section", section->getName() }, { "name", name }, { "value", value } });
      }
    }
  }
  std::cout << "CAPTURE " << snapshot.dump() << std::endl;
  return snapshot["complete"].get<bool>() ? 0 : 1;
}

class Observation
{
public:
  int samples = 0;
  int modes = 0;
  bool moving = false;
  bool malformed = false;
  bool calibration_observation = false;

  void mode(ArNetPacket* packet)
  {
    if (packet->getDataLength() != 1)
    {
      malformed = true;
      return;
    }
    const auto value = packet->bufToByte();
    if (value != 0 && value != 1)
    {
      malformed = true;
      return;
    }
    ++modes;
    std::cout << "SAFE_DRIVE " << static_cast<int>(value) << std::endl;
  }

  void numbers(ArNetPacket* packet)
  {
    if (packet->getDataLength() < 19)
    {
      malformed = true;
      return;
    }
    packet->bufToByte2();
    const auto position_x = packet->bufToByte4();
    const auto position_y = packet->bufToByte4();
    const auto heading = packet->bufToByte2();
    const auto forward = packet->bufToByte2();
    const auto angular = packet->bufToByte2();
    const auto lateral = packet->bufToByte2();
    moving = moving || std::abs(forward) > 5 || std::abs(angular) > 1 || std::abs(lateral) > 5;
    ++samples;
    if (calibration_observation)
    {
      const nlohmann::json sample = {
        { "x_mm", position_x }, { "y_mm", position_y }, { "heading_deg", heading },
        { "forward_mm_s", forward }, { "angular_deg_s", angular }, { "lateral_mm_s", lateral }
      };
      std::cout << "ODOMETRY " << sample.dump() << std::endl;
    }
    if (samples <= 3 || samples % 10 == 0)
    {
      std::cout << "TELEMETRY x_mm=" << position_x << " y_mm=" << position_y << " heading_deg=" << heading
                << " forward_mm_s=" << forward << " angular_deg_s=" << angular << " lateral_mm_s=" << lateral
                << std::endl;
    }
  }
};
}  // namespace
int main(int argc, char** argv)
{
  if (argc != 2 || (std::string(argv[1]) != "connectivity" && std::string(argv[1]) != "config" &&
                    std::string(argv[1]) != "config-capture" && std::string(argv[1]) != "observe"))
  {
    std::cerr << "Usage: amr_hardware_probe connectivity|config|config-capture|observe (connection details on stdin)\n";
    return 2;
  }
  std::string host, port, user, password, protocol;
  if (!std::getline(std::cin, host) || !std::getline(std::cin, port) || !std::getline(std::cin, user) ||
      !std::getline(std::cin, password) || !std::getline(std::cin, protocol))
  {
    std::cerr << "Missing connection details\n";
    return 2;
  }
  int port_number;
  try
  {
    std::size_t consumed = 0;
    port_number = std::stoi(port, &consumed);
    if (consumed != port.size() || port_number < 1 || port_number > 65535)
    {
      return 2;
    }
  }
  catch (const std::exception&)
  {
    return 2;
  }
  Aria::init();
  ArLog::init(ArLog::None, ArLog::Terse, "", false, false, false);
  int result = 1;
    {
      Observation observation;
      observation.calibration_observation = std::string(argv[1]) == "observe";
      ArFunctor1C<Observation, ArNetPacket*> mode(&observation, &Observation::mode);
      ArFunctor1C<Observation, ArNetPacket*> numbers(&observation, &Observation::numbers);
      amr_core::DriveClient client;
      if (!protocol.empty())
      {
        client.enforceProtocolVersion(protocol.c_str());
      }
      if (!client.configureTransport(32, 65536, 0.1) ||
          !client.blockingConnect(host.c_str(), port_number, false, user.c_str(),
                                  password.empty() ? nullptr : password.c_str()))
      {
        std::cout << "AUTHENTICATION_FAILED rejected=" << client.wasRejected()
                  << " rejection_code=" << client.getRejected() << " transport_fault=" << client.hasTransportFailure()
                  << std::endl;
      }
      else
      {
        std::cout << "AUTHENTICATION_OK" << std::endl;
        for (const auto& entry : *client.getDataMap())
        {
          const std::string name = entry.second->getName();
          if (name == "ratioDrive" || name == "stop" || name == "setSafeDrive" || name == "getSafeDrive" ||
              name == "updateNumbers")
          {
            std::cout << "SERVICE " << name << std::endl;
          }
        }
        if (std::string(argv[1]) == "config-capture")
        {
          result = captureConfiguration(client);
        }
        else if (std::string(argv[1]) == "connectivity")
        {
          result = 0;
        }
        else
        {
          const bool mode_ready = client.dataExists("getSafeDrive") && client.addHandler("getSafeDrive", &mode) &&
                                  client.requestOnce("getSafeDrive");
          const bool numbers_ready = client.dataExists("updateNumbers") &&
                                     client.addHandler("updateNumbers", &numbers) &&
                                     client.request("updateNumbers", 100);
          const auto duration = observation.calibration_observation ? 10 : 4;
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
          while (mode_ready && numbers_ready && client.isConnected() && std::chrono::steady_clock::now() < deadline)
          {
            client.loopOnce();
          }
          std::cout << "SUMMARY samples=" << observation.samples << " mode_replies=" << observation.modes
                    << " motion_observed=" << observation.moving << " malformed=" << observation.malformed
                    << " transport_fault=" << client.hasTransportFailure() << std::endl;
          result = observation.samples >= 2 && observation.modes > 0 &&
                           (observation.calibration_observation || !observation.moving) && !observation.malformed &&
                           client.isConnected() && !client.hasTransportFailure() ?
                       0 :
                       1;
        }
        client.disconnect();
      }
    }
  Aria::shutdown();
  return result;
}