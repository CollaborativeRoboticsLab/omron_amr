#include <ArNetworking/ArNetworking.h>
#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/drive_telemetry.hpp"
#include "amr_core/utils/docking_protocol.hpp"
#include "amr_core/utils/robot_session.hpp"
#include "amr_core/utils/map_activation_readiness.hpp"
#include "amr_core/interfaces/configuration_interface.hpp"
#include "amr_core/interfaces/debug_interface.hpp"
#include "amr_core/interfaces/map_interface.hpp"
#include <nlohmann/json.hpp>

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <cerrno>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

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
    snapshot["failure_reason"] = "Connected getConfigValues service required; legacy configuration services are "
                                 "unsupported.";
  }
  std::cout << "CAPTURE " << snapshot.dump() << std::endl;
  return snapshot["complete"].get<bool>() ? 0 : 1;
}

int saveMap(const amr_core::MapSnapshot& snapshot, const std::string& path, bool require_parse)
{
  const int file = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (file < 0)
  {
    std::cerr << "Map snapshot output could not be created without overwriting an existing file\n";
    return 1;
  }
  std::size_t written = 0;
  while (written < snapshot.content.size())
  {
    const auto count = ::write(file, snapshot.content.data() + written, snapshot.content.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) break;
    written += static_cast<std::size_t>(count);
  }
  const bool saved = written == snapshot.content.size() && ::fsync(file) == 0;
  const bool closed = ::close(file) == 0;
  ArMap map;
  const bool parsed = saved && closed && map.readFile(path.c_str());
  std::cout << "MAP_SNAPSHOT " << nlohmann::json({ { "name", snapshot.name }, { "bytes", written },
                                                   { "saved", saved && closed }, { "parsed", parsed } }).dump()
            << std::endl;
  return saved && closed && (!require_parse || parsed) ? 0 : 1;
}

int captureMap(amr_core::DriveClient& client, const std::string& path)
{
  amr_core::MapInterface interface(client);
  return saveMap(interface.snapshot(), path, true);
}

int downloadMap(amr_core::DriveClient& client, const std::string& path, const std::string& name)
{
  amr_core::MapInterface interface(client);
  return saveMap({ name, interface.download(name) }, path, false);
}

int uploadMap(amr_core::DriveClient& client, const std::string& path, const std::string& name = "")
{
  ArMap parsed_map;
  if (!parsed_map.readFile(path.c_str()))
    throw std::runtime_error("Map file could not be parsed");
  const int file = ::open(path.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
  if (file < 0) throw std::runtime_error("Map file could not be opened");
  struct stat info;
  if (::fstat(file, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 || info.st_size > 64 * 1024 * 1024)
  {
    ::close(file);
    throw std::runtime_error("Map file is not a regular file of an acceptable size");
  }
  std::string content(static_cast<std::size_t>(info.st_size), '\0');
  std::size_t offset = 0;
  while (offset < content.size())
  {
    const auto count = ::read(file, &content[offset], content.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) break;
    offset += static_cast<std::size_t>(count);
  }
  struct stat after;
  const bool stable = ::fstat(file, &after) == 0 && info.st_size == after.st_size &&
                      info.st_mtim.tv_sec == after.st_mtim.tv_sec && info.st_mtim.tv_nsec == after.st_mtim.tv_nsec;
  const bool closed = ::close(file) == 0;
  if (offset != content.size() || !stable || !closed) throw std::runtime_error("Map file changed or could not be read");
  amr_core::MapInterface interface(client);
  const auto uploaded_name = interface.upload(content, name);
  std::cout << "MAP_UPLOAD " << nlohmann::json({ { "name", uploaded_name }, { "bytes", content.size() },
                                                 { "readback_matches", true }, { "activated", false } }).dump()
            << std::endl;
  return 0;
}

int inspectMapConfiguration(amr_core::DriveClient& client)
{
  if (!client.dataExists("getConfigValues")) return 1;
  amr_core::HybridObservation observation("Files");
  ArFunctor1C<amr_core::HybridObservation, ArNetPacket*> callback(&observation, &amr_core::HybridObservation::receive);
  if (!client.addHandler("getConfigValues", &callback)) return 1;
  const auto encoded = amr_core::sectionRequest("Files");
  ArNetPacket request;
  request.dataToBuf(encoded.data(), static_cast<int>(encoded.size()));
  const bool submitted = client.requestOnce("getConfigValues", &request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (submitted && client.isConnected() && !client.hasTransportFailure() && !observation.failed &&
         !observation.transfer.complete() && std::chrono::steady_clock::now() < deadline)
    client.loopOnce();
  client.remHandler("getConfigValues", &callback);
  if (!submitted || observation.failed || !observation.transfer.complete() || !client.isConnected() ||
      client.hasTransportFailure())
  {
    std::cout << "MAP_CONFIG {\"complete\":false}" << std::endl;
    return 1;
  }
  try
  {
    const auto values = observation.transfer.parameters();
    const bool present = values.size() == 1 && values[0].at("name") == "Map";
    nlohmann::json result{ { "complete", true }, { "map_parameter_present", present } };
    if (present) result["map"] = values[0].at("value");
    std::cout << "MAP_CONFIG " << result.dump() << std::endl;
    return present ? 0 : 1;
  }
  catch (const std::exception&)
  {
    std::cout << "MAP_CONFIG {\"complete\":false}" << std::endl;
    return 1;
  }
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
  int map_name_samples = 0;

  void mapName(ArNetPacket* packet)
  {
    if (!packet || !packet->isValid() || packet->getDataLength() <= packet->getDataReadLength())
    {
      malformed = true;
      return;
    }
    const auto name = packet->bufToString();
    if (!packet->isValid() || packet->getDataLength() != packet->getDataReadLength())
    {
      malformed = true;
      return;
    }
    ++map_name_samples;
    std::cout << "ACTIVE_MAP " << nlohmann::json(name).dump() << std::endl;
  }

  void motors(ArNetPacket* packet)
  {
    std::uint8_t state = 0;
    if (!amr_core::DebugInterface::readMotorState(packet, state))
    {
      malformed = true;
      return;
    }
    ++motor_samples;
    std::cout << "MOTOR_STATE " << static_cast<int>(state) << std::endl;
  }

  void faults(ArNetPacket* packet)
  {
    std::uint16_t count = 0;
    if (!amr_core::DebugInterface::readFaultCount(packet, count))
    {
      malformed = true;
      return;
    }
    if (count > 64)
    {
      malformed = true;
      return;
    }
    const auto* cursor = packet->getBuf() + packet->getReadLength();
    const auto* end = cursor + packet->getDataLength() - packet->getDataReadLength();
    std::cout << "FAULT_PAYLOAD count=" << count << " bytes=" << (end - cursor)
              << " terminators=" << std::count(cursor, end, '\0') << std::endl;
    auto details = nlohmann::json::array();
    for (std::uint16_t index = 0; index < count; ++index)
    {
      std::string fields[4];
      for (auto& field : fields)
      {
        const auto* terminator = std::find(cursor, end, '\0');
        if (terminator == end)
        {
          malformed = true;
          return;
        }
        field.assign(cursor, terminator);
        cursor = terminator + 1;
      }
      details.push_back({ { "type", fields[0] },
                          { "name", fields[1] },
                          { "short_description", fields[2] },
                          { "long_description", fields[3] } });
    }
    if (cursor != end)
    {
      std::cout << "FAULT_DETAILS_UNVERIFIED " << details.dump() << " trailing_bytes=" << (end - cursor) << std::endl;
      auto trailing = nlohmann::json::array();
      for (const auto* byte = cursor; byte != end; ++byte)
        trailing.push_back(static_cast<unsigned int>(static_cast<unsigned char>(*byte)));
      std::cout << "FAULT_TRAILING_BYTES " << trailing.dump() << std::endl;
      malformed = true;
      return;
    }
    ++fault_samples;
    std::cout << "FAULT_COUNT " << count << std::endl;
    std::cout << "FAULT_DETAILS " << details.dump() << std::endl;
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
    std::cout << "DOCK_INFO "
              << nlohmann::json({ { "state", static_cast<int>(info.state) },
                                  { "forced", info.forced },
                                  { "shutdown_countdown", info.shutdown_countdown } })
                     .dump()
              << std::endl;
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

int activateMap(amr_core::DriveClient& client, const std::string& path, const std::string& target,
                const std::string& expected)
{
  amr_core::mapSelectionRequest(expected, target);
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input || input.tellg() <= 0 || input.tellg() > 64 * 1024 * 1024)
    throw std::runtime_error("Expected map file cannot be read");
  const auto size = static_cast<std::size_t>(input.tellg());
  std::string contents(size, '\0');
  input.seekg(0);
  if (!input.read(&contents[0], static_cast<std::streamsize>(size)))
    throw std::runtime_error("Expected map file cannot be read");
  amr_core::MapInterface map(client);
  amr_core::ConfigurationInterface configuration(client);
  if (map.download(target) != contents) throw std::runtime_error("Remote map differs from expected bytes");
  if (map.getName() != expected || configuration.getMapSelection() != expected)
    throw std::runtime_error("Active and configured map do not match expected value");
  if (!client.dataExists("dockInfo") || !client.dataExists("updateNumbers"))
    throw std::runtime_error("Docking or velocity feedback unavailable");
  amr_core::MapActivationReadiness readiness;
  ArFunctor1C<amr_core::MapActivationReadiness, ArNetPacket*> dock_callback(&readiness, &amr_core::MapActivationReadiness::docking);
  ArFunctor1C<amr_core::MapActivationReadiness, ArNetPacket*> number_callback(&readiness, &amr_core::MapActivationReadiness::numbers);
  if (!client.addHandler("dockInfo", &dock_callback)) throw std::runtime_error("Cannot register docking feedback");
  if (!client.addHandler("updateNumbers", &number_callback))
  {
    client.remHandler("dockInfo", &dock_callback);
    throw std::runtime_error("Cannot register velocity feedback");
  }
  const bool submitted = client.requestOnce("dockInfo", nullptr, true) && client.request("updateNumbers", 100);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (submitted && !readiness.failed && client.isConnected() && !client.hasTransportFailure() &&
         std::chrono::steady_clock::now() < deadline)
    client.loopOnce();
  client.requestStop("updateNumbers");
  client.remHandler("dockInfo", &dock_callback);
  client.remHandler("updateNumbers", &number_callback);
  if (!submitted || !readiness.ready() ||
      !client.isConnected() || client.hasTransportFailure())
    throw std::runtime_error("Robot is not confirmed docked and stationary");
  if (map.getName() != expected || configuration.getMapSelection() != expected)
    throw std::runtime_error("Map changed during activation preflight");
  std::cout << "MAP_ACTIVATION_WRITE_ATTEMPTED" << std::endl;
  const bool receipt = configuration.setMapSelection(expected, target);
  bool active = false;
  bool configured = false;
  const auto verify_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (client.isConnected() && !client.hasTransportFailure() && std::chrono::steady_clock::now() < verify_deadline)
  {
    active = map.getName() == target;
    configured = configuration.getMapSelection() == target;
    if (active && configured) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  std::cout << "MAP_ACTIVATION " << nlohmann::json({ { "receipt", receipt }, { "active_name_matches", active },
                                                      { "configuration_matches", configured } }).dump() << std::endl;
  return receipt && active && configured ? 0 : 1;
}
}  // namespace
int main(int argc, char** argv)
{
  if (argc != 2 || (std::string(argv[1]) != "connectivity" && std::string(argv[1]) != "config" &&
                    std::string(argv[1]) != "config-capture" && std::string(argv[1]) != "observe" &&
                    std::string(argv[1]) != "map-discovery" && std::string(argv[1]) != "map-status" &&
                    std::string(argv[1]) != "power-discovery" && std::string(argv[1]) != "docking-discovery" &&
                    std::string(argv[1]) != "docking-status" && std::string(argv[1]) != "map-snapshot" &&
                    std::string(argv[1]) != "map-upload" && std::string(argv[1]) != "map-upload-named" &&
                    std::string(argv[1]) != "map-download" && std::string(argv[1]) != "map-activate" &&
                    std::string(argv[1]) != "map-config" &&
                    std::string(argv[1]) != "debug-discovery" &&
                    std::string(argv[1]) != "debug-status"))
  {
    std::cerr << "Usage: amr_hardware_probe "
                 "connectivity|config|config-capture|observe|map-discovery|map-status|power-discovery|docking-"
                 "discovery|docking-status|map-snapshot|map-download|map-upload|map-upload-named|map-activate|map-config|debug-discovery|debug-status (connection details on stdin)\n";
    return 2;
  }
  std::string host, port, user, password, protocol;
  if (!std::getline(std::cin, host) || !std::getline(std::cin, port) || !std::getline(std::cin, user) ||
      !std::getline(std::cin, password) || !std::getline(std::cin, protocol))
  {
    std::cerr << "Missing connection details\n";
    return 2;
  }
  std::string snapshot_path;
  if ((std::string(argv[1]) == "map-snapshot" || std::string(argv[1]) == "map-upload" ||
      std::string(argv[1]) == "map-upload-named" || std::string(argv[1]) == "map-download" ||
      std::string(argv[1]) == "map-activate") &&
      (!std::getline(std::cin, snapshot_path) || snapshot_path.empty()))
    return 2;
  std::string upload_name;
  if ((std::string(argv[1]) == "map-upload-named" || std::string(argv[1]) == "map-download" ||
       std::string(argv[1]) == "map-activate") &&
      (!std::getline(std::cin, upload_name) || upload_name.empty()))
    return 2;
  std::string expected_map;
  if (std::string(argv[1]) == "map-activate" && (!std::getline(std::cin, expected_map) || expected_map.empty()))
    return 2;
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
    ArFunctor1C<Observation, ArNetPacket*> map_name(&observation, &Observation::mapName);
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
        std::cout << "DEBUG_SUMMARY ready=" << ready << " motor_replies=" << observation.motor_samples
                  << " fault_replies=" << observation.fault_samples << " malformed=" << observation.malformed
                  << " connected=" << client.isConnected() << std::endl;
        result = ready && observation.motor_samples > 0 && observation.fault_samples > 0 && !observation.malformed &&
                         client.isConnected() && !client.hasTransportFailure() ?
                     0 :
                     1;
      }
      else if (std::string(argv[1]) == "debug-discovery")
      {
        std::cout << "DEBUG_SERVICES " << amr_core::DebugInterface::discover(*client.getDataMap()).dump() << std::endl;
        result = 0;
      }
      else if (std::string(argv[1]) == "map-snapshot")
      {
        try
        {
          result = captureMap(client, snapshot_path);
        }
        catch (const std::runtime_error& error)
        {
          std::cerr << "Map snapshot failed: " << error.what() << std::endl;
        }
      }
      else if (std::string(argv[1]) == "map-upload" || std::string(argv[1]) == "map-upload-named")
      {
        try
        {
          result = uploadMap(client, snapshot_path, upload_name);
        }
        catch (const std::runtime_error& error)
        {
          std::cerr << "Map upload failed: " << error.what() << std::endl;
        }
      }
      else if (std::string(argv[1]) == "map-download")
      {
        try
        {
          result = downloadMap(client, snapshot_path, upload_name);
        }
        catch (const std::runtime_error& error)
        {
          std::cerr << "Map download failed: " << error.what() << std::endl;
        }
      }
      else if (std::string(argv[1]) == "map-activate")
      {
        try
        {
          result = activateMap(client, snapshot_path, upload_name, expected_map);
        }
        catch (const std::exception& error)
        {
          std::cerr << "Map activation failed or is unverified: " << error.what() << std::endl;
        }
      }
      else if (std::string(argv[1]) == "map-status")
      {
        const bool ready = client.dataExists("getMapName") && client.addHandler("getMapName", &map_name) &&
                           client.requestOnce("getMapName", nullptr, true);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
        while (ready && client.isConnected() && observation.map_name_samples == 0 && !observation.malformed &&
               std::chrono::steady_clock::now() < deadline)
          client.loopOnce();
        result = ready && observation.map_name_samples > 0 && !observation.malformed && client.isConnected() &&
                         !client.hasTransportFailure() ?
                     0 :
                     1;
      }
      else if (std::string(argv[1]) == "map-config")
      {
        result = inspectMapConfiguration(client);
      }
      else if (std::string(argv[1]) == "power-discovery")
      {
        auto services = nlohmann::json::array();
        for (const auto& entry : *client.getDataMap())
        {
          auto& data = *entry.second;
          std::string name = data.getName();
          std::transform(name.begin(), name.end(), name.begin(),
                         [](unsigned char character) { return std::tolower(character); });
          if (name.find("shutdown") != std::string::npos || name.find("power") != std::string::npos ||
              name.find("reboot") != std::string::npos || name.find("restart") != std::string::npos)
          {
            services.push_back({ { "name", data.getName() },
                                 { "description", data.getDescription() },
                                 { "arguments", data.getArgumentDescription() },
                                 { "returns", data.getReturnDescription() } });
          }
        }
        std::cout << "POWER_SERVICES " << services.dump() << std::endl;
        result = 0;
      }
      else if (std::string(argv[1]) == "map-discovery")
      {
        auto services = nlohmann::json::array();
        for (const auto& entry : *client.getDataMap())
        {
          auto& data = *entry.second;
          std::string name = data.getName();
          std::transform(name.begin(), name.end(), name.begin(),
                         [](unsigned char character) { return std::tolower(character); });
          if (name.find("map") != std::string::npos || name.find("file") != std::string::npos ||
              name.find("directory") != std::string::npos || name.find("config") != std::string::npos)
          {
            services.push_back({ { "name", data.getName() },
                                 { "description", data.getDescription() },
                                 { "arguments", data.getArgumentDescription() },
                                 { "returns", data.getReturnDescription() } });
          }
        }
        std::cout << "MAP_SERVICES " << services.dump() << std::endl;
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
          if (name == "stop" || name.find("cancel") != std::string::npos || name.find("dock") != std::string::npos ||
              name.find("charg") != std::string::npos)
          {
            services.push_back({ { "name", data.getName() },
                                 { "description", data.getDescription() },
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
        const bool numbers_ready = client.dataExists("updateNumbers") && client.addHandler("updateNumbers", &numbers) &&
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