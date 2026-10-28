#include <ArNetworking/ArNetworking.h>
#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/drive_telemetry.hpp"
#include "amr_core/utils/docking_protocol.hpp"
#include "amr_core/utils/robot_session.hpp"
#include "amr_core/interfaces/configuration_interface.hpp"
#include "amr_core/interfaces/debug_interface.hpp"
#include <nlohmann/json.hpp>

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
int captureConfiguration(amr_core::DriveClient& client)
{
  nlohmann::json snapshot = { { "schema_version", 1 },
                              { "read_only", true },
                              { "complete", false },
                              { "config_service", "getConfigValues" },
                              { "parameters", nlohmann::json::array() } };
  try
  {
    amr_core::ConfigurationInterface configuration(client);
    snapshot.update(configuration.getValues());
  }
  catch (const std::exception&)
  {
    snapshot["failure_reason"] = "Connected getConfigValues service required; legacy configuration services are unsupported.";
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
  int docking_samples = 0;
  int motor_samples = 0;
  int fault_samples = 0;

  void motors(ArNetPacket* packet)
  {
    std::uint8_t state = 0;
    if (!amr_core::DebugInterface::readMotorState(packet, state)) { malformed = true; return; }
    ++motor_samples;
    std::cout << "MOTOR_STATE " << static_cast<int>(state) << std::endl;
  }

  void faults(ArNetPacket* packet)
  {
    std::uint16_t count = 0;
    if (!amr_core::DebugInterface::readFaultCount(packet, count)) { malformed = true; return; }
    ++fault_samples;
    std::cout << "FAULT_COUNT " << count << std::endl;
  }

  void docking(ArNetPacket* packet)
  {
    amr_core::DockInfo info;
    if (!amr_core::readDockInfo(packet, info))
    {
      malformed = true;
      return;
    }
    ++docking_samples;
    std::cout << "DOCK_INFO " << nlohmann::json({ { "state", static_cast<int>(info.state) },
      { "forced", info.forced }, { "shutdown_countdown", info.shutdown_countdown } }).dump() << std::endl;
  }

  void mode(ArNetPacket* packet)
  {
    std::uint8_t value = 0;
    if (!amr_core::readSafeDrive(packet, value))
    {
      malformed = true;
      return;
    }
    ++modes;
    std::cout << "SAFE_DRIVE " << static_cast<int>(value) << std::endl;
  }

  void numbers(ArNetPacket* packet)
  {
    amr_core::DriveTelemetry telemetry;
    if (!amr_core::readDriveTelemetry(packet, telemetry))
    {
      malformed = true;
      return;
    }
    moving = moving || std::abs(telemetry.forward_mm_s) > 5 || std::abs(telemetry.angular_deg_s) > 1 ||
         std::abs(telemetry.lateral_mm_s) > 5;
    ++samples;
    if (calibration_observation)
    {
      std::cout << "ODOMETRY " << telemetry.toJson().dump() << std::endl;
    }
    if (samples <= 3 || samples % 10 == 0)
    {
      std::cout << "TELEMETRY x_mm=" << telemetry.x_mm << " y_mm=" << telemetry.y_mm
            << " heading_deg=" << telemetry.heading_deg << " forward_mm_s=" << telemetry.forward_mm_s
            << " angular_deg_s=" << telemetry.angular_deg_s << " lateral_mm_s=" << telemetry.lateral_mm_s
                << std::endl;
    }
  }
};
}  // namespace
int main(int argc, char** argv)
{
  if (argc != 2 || (std::string(argv[1]) != "connectivity" && std::string(argv[1]) != "config" &&
                    std::string(argv[1]) != "config-capture" && std::string(argv[1]) != "observe" &&
                    std::string(argv[1]) != "docking-discovery" && std::string(argv[1]) != "docking-status" &&
                    std::string(argv[1]) != "debug-discovery" && std::string(argv[1]) != "debug-status"))
  {
    std::cerr << "Usage: amr_hardware_probe connectivity|config|config-capture|observe|docking-discovery|docking-status|debug-discovery|debug-status (connection details on stdin)\n";
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
  int result = 1;
  try
    {
      Observation observation;
      observation.calibration_observation = std::string(argv[1]) == "observe";
      ArFunctor1C<Observation, ArNetPacket*> mode(&observation, &Observation::mode);
      ArFunctor1C<Observation, ArNetPacket*> numbers(&observation, &Observation::numbers);
      ArFunctor1C<Observation, ArNetPacket*> docking(&observation, &Observation::docking);
      ArFunctor1C<Observation, ArNetPacket*> motors(&observation, &Observation::motors);
      ArFunctor1C<Observation, ArNetPacket*> faults(&observation, &Observation::faults);
      const amr_core::RobotConnection connection{ host, port_number, user, password, protocol };
      connection.validate();
      amr_core::RobotSession session;
      auto& client = session.client();
      if (!session.connect(connection))
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
        if (std::string(argv[1]) == "debug-status")
        {
          const bool ready = client.dataExists("queryMotors") && client.dataExists("getFaults") &&
                             client.addHandler("queryMotors", &motors) && client.addHandler("getFaults", &faults) &&
                             client.requestOnce("queryMotors", nullptr, true) &&
                             client.requestOnce("getFaults", nullptr, true);
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
          while (ready && client.isConnected() && !observation.malformed &&
                 (observation.motor_samples == 0 || observation.fault_samples == 0) &&
                 std::chrono::steady_clock::now() < deadline)
            client.loopOnce();
          result = ready && observation.motor_samples > 0 && observation.fault_samples > 0 &&
                   !observation.malformed && client.isConnected() && !client.hasTransportFailure() ? 0 : 1;
        }
        else if (std::string(argv[1]) == "debug-discovery")
        {
          std::cout << "DEBUG_SERVICES " << amr_core::DebugInterface::discover(*client.getDataMap()).dump() << std::endl;
          result = 0;
        }
        else if (std::string(argv[1]) == "docking-discovery")
        {
          auto services = nlohmann::json::array();
          for (const auto& entry : *client.getDataMap())
          {
            auto& data = *entry.second;
            std::string name = data.getName();
            std::transform(name.begin(), name.end(), name.begin(),
                           [](unsigned char character) { return std::tolower(character); });
            if (name == "stop" || name.find("cancel") != std::string::npos ||
              name.find("dock") != std::string::npos || name.find("charg") != std::string::npos)
            {
              services.push_back({ { "name", data.getName() }, { "description", data.getDescription() },
                                   { "arguments", data.getArgumentDescription() },
                                   { "returns", data.getReturnDescription() } });
            }
          }
          std::cout << "DOCKING_SERVICES " << services.dump() << std::endl;
          result = 0;
        }
        else if (std::string(argv[1]) == "docking-status")
        {
          const bool ready = client.dataExists("dockInfo") && client.addHandler("dockInfo", &docking) &&
                             client.requestOnce("dockInfo", nullptr, true);
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
          while (ready && client.isConnected() && observation.docking_samples == 0 && !observation.malformed &&
                 std::chrono::steady_clock::now() < deadline)
          {
            client.loopOnce();
          }
          result = observation.docking_samples > 0 && !observation.malformed ? 0 : 1;
        }
        else if (std::string(argv[1]) == "config-capture")
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
      }
    }
  catch (const std::exception&)
  {
    std::cerr << "Connection or read-only operation failed (values withheld).\n";
  }
  return result;
}