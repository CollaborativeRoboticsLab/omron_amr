#pragma once

#include "amr_core/utils/deadzone_calibration.hpp"
#include "amr_core/utils/robot_session.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <charconv>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <poll.h>
#include <set>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>

namespace amr_calibration
{
namespace client_detail
{
using Json = nlohmann::json;
inline volatile std::sig_atomic_t interrupted = 0;

inline void interrupt(int)
{
  interrupted = 1;
}

inline std::string utcNow()
{
  const auto now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  std::ostringstream text;
  text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return text.str();
}

class AuditFile
{
public:
  explicit AuditFile(const std::filesystem::path& path)
  {
    if (!path.parent_path().empty())
    {
      std::filesystem::create_directories(path.parent_path());
    }
    descriptor_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (descriptor_ < 0)
    {
      throw std::runtime_error("Unable to create exclusive audit file");
    }
  }

  ~AuditFile() { ::close(descriptor_); }
  AuditFile(const AuditFile&) = delete;
  AuditFile& operator=(const AuditFile&) = delete;

  void save(const Json& report)
  {
    const auto text = report.dump(2) + "\n";
    std::size_t offset = 0;
    while (offset < text.size())
    {
      const auto written = ::pwrite(descriptor_, text.data() + offset, text.size() - offset, offset);
      if (written < 0 && errno == EINTR)
      {
        continue;
      }
      if (written <= 0)
      {
        throw std::runtime_error("Unable to write audit file");
      }
      offset += static_cast<std::size_t>(written);
    }
    if (::ftruncate(descriptor_, text.size()) != 0 || ::fsync(descriptor_) != 0)
    {
      throw std::runtime_error("Unable to persist audit file");
    }
  }

private:
  int descriptor_ = -1;
};

inline Json boundedOperation(const std::function<Json()>& operation, int seconds)
{
  if (interrupted)
  {
    interrupted = 0;
    throw std::runtime_error("Operation interrupted");
  }
  int descriptors[2];
  if (::pipe2(descriptors, O_CLOEXEC) != 0)
  {
    throw std::runtime_error("Unable to create worker pipe");
  }
  const auto worker = ::fork();
  if (worker < 0)
  {
    ::close(descriptors[0]);
    ::close(descriptors[1]);
    throw std::runtime_error("Unable to create worker");
  }
  if (worker == 0)
  {
    ::signal(SIGINT, SIG_DFL);
    ::signal(SIGTERM, SIG_DFL);
    ::close(descriptors[0]);
    Json response;
    try
    {
      response = { { "success", true }, { "result", operation() } };
    }
    catch (const std::exception&)
    {
      response = { { "success", false } };
    }
    const auto text = response.dump();
    std::size_t offset = 0;
    while (offset < text.size())
    {
      const auto written = ::write(descriptors[1], text.data() + offset, text.size() - offset);
      if (written < 0 && errno == EINTR)
      {
        continue;
      }
      if (written <= 0)
      {
        ::_exit(1);
      }
      offset += static_cast<std::size_t>(written);
    }
    ::close(descriptors[1]);
    ::_exit(0);
  }
  ::close(descriptors[1]);
  bool reaped = false;
  try
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    std::string text;
    while (true)
    {
      if (interrupted || std::chrono::steady_clock::now() >= deadline)
      {
        interrupted = 0;
        throw std::runtime_error("Worker timed out or was interrupted");
      }
      pollfd event{ descriptors[0], POLLIN, 0 };
      const auto ready = ::poll(&event, 1, 100);
      if (ready < 0 && errno != EINTR)
      {
        throw std::runtime_error("Worker pipe failed");
      }
      if (ready <= 0)
      {
        continue;
      }
      char buffer[8192];
      const auto count = ::read(descriptors[0], buffer, sizeof(buffer));
      if (count < 0 && errno == EINTR)
      {
        continue;
      }
      if (count < 0 || text.size() > 4194304)
      {
        throw std::runtime_error("Worker result rejected");
      }
      if (count == 0)
      {
        break;
      }
      text.append(buffer, static_cast<std::size_t>(count));
    }
    int status = 0;
    while (::waitpid(worker, &status, 0) < 0)
    {
      if (errno != EINTR)
      {
        throw std::runtime_error("Unable to reap worker");
      }
    }
    reaped = true;
    const auto response = Json::parse(text);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 || !response.at("success").get<bool>())
    {
      throw std::runtime_error("Worker operation failed");
    }
    ::close(descriptors[0]);
    return response.at("result");
  }
  catch (...)
  {
    if (!reaped)
    {
      ::kill(worker, SIGKILL);
      while (::waitpid(worker, nullptr, 0) < 0 && errno == EINTR) {}
    }
    ::close(descriptors[0]);
    throw;
  }
}

struct Options
{
  std::filesystem::path parameters;
  std::filesystem::path output;
  double translation = -1;
  double rotation = -1;
  std::optional<int> target;
  bool robot = false;
  bool confirmed = false;
};

inline double number(const std::string& text)
{
  double value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !std::isfinite(value))
  {
    throw std::invalid_argument("Invalid numeric option");
  }
  return value;
}

inline Options parseOptions(int argc, char** argv)
{
  Options options;
  std::set<std::string> seen;
  for (int index = 1; index < argc; ++index)
  {
    const std::string option(argv[index]);
    if (!seen.insert(option).second)
    {
      throw std::invalid_argument("Duplicate option");
    }
    if (option == "--robot") options.robot = true;
    else if (option == "--confirm-write") options.confirmed = true;
    else
    {
      if (++index >= argc)
      {
        throw std::invalid_argument("Missing option value");
      }
      const std::string value(argv[index]);
      if (option == "--params") options.parameters = value;
      else if (option == "--output") options.output = value;
      else if (option == "--expected-translation") options.translation = number(value);
      else if (option == "--expected-rotation") options.rotation = number(value);
      else if (option == "--apply-translation")
      {
        const auto target = number(value);
        if (target < 1 || target > 99 || std::floor(target) != target)
        {
          throw std::invalid_argument("Invalid target percentage");
        }
        options.target = static_cast<int>(target);
      }
      else throw std::invalid_argument("Unknown option");
    }
  }
  if (!options.robot || options.output.empty() || options.translation < 0 || options.translation >= 100 ||
      options.rotation < 0 || options.rotation >= 100 || options.confirmed != options.target.has_value() ||
      std::filesystem::exists(options.output))
  {
    throw std::invalid_argument("Missing confirmation, invalid percentages, or existing output");
  }
  if (options.target && (options.translation < 1 || options.rotation < 1 ||
      std::floor(options.translation) != options.translation || std::floor(options.rotation) != options.rotation ||
      *options.target == options.translation))
  {
    throw std::invalid_argument("Write requires distinct integer percentages in [1, 99]");
  }
  if (options.parameters.empty())
  {
    options.parameters = std::filesystem::path(ament_index_cpp::get_package_share_directory("amr_ros")) /
                         "config/ld250_parameters.yaml";
  }
  return options;
}

using Connection = amr_core::RobotConnection;

inline Connection loadConnection(const std::filesystem::path& path)
{
  const auto robot = YAML::LoadFile(path.string())["amr_core"]["ros__parameters"]["robot"];
  Connection connection{ robot["ip"].as<std::string>(), robot["port"].as<int>(), robot["user"].as<std::string>(),
                         robot["password"].as<std::string>(), robot["protocol"].as<std::string>("6MTX") };
  connection.validate();
  return connection;
}

class Observation
{
public:
  void mode(ArNetPacket* packet)
  {
    std::uint8_t mode = 0;
    if (!amr_core::readSafeDrive(packet, mode))
    {
      valid = false;
      return;
    }
    ++modes;
  }

  void numbers(ArNetPacket* packet)
  {
    amr_core::DriveTelemetry telemetry;
    if (!amr_core::readDriveTelemetry(packet, telemetry))
    {
      valid = false;
      return;
    }
    samples.push_back(telemetry.toJson());
    if (samples.size() > 10000) valid = false;
  }

  Json samples = Json::array();
  bool valid = true;
  int modes = 0;
};

inline Json observe(amr_core::DriveClient& client)
{
  Observation observation;
  ArFunctor1C<Observation, ArNetPacket*> mode(&observation, &Observation::mode);
  ArFunctor1C<Observation, ArNetPacket*> numbers(&observation, &Observation::numbers);
  const bool mode_registered = client.dataExists("getSafeDrive") && client.addHandler("getSafeDrive", &mode);
  const bool numbers_registered = client.dataExists("updateNumbers") && client.addHandler("updateNumbers", &numbers);
  const bool requested = mode_registered && numbers_registered && client.requestOnce("getSafeDrive") &&
                         client.request("updateNumbers", 100);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (requested && observation.valid && client.isConnected() && !client.hasTransportFailure() &&
         std::chrono::steady_clock::now() < deadline)
  {
    client.loopOnce();
  }
  if (numbers_registered)
  {
    client.requestStop("updateNumbers");
    client.remHandler("updateNumbers", &numbers);
  }
  if (mode_registered) client.remHandler("getSafeDrive", &mode);
  if (!requested || !observation.valid || observation.modes == 0 || observation.samples.size() < 2 ||
      !client.isConnected() || client.hasTransportFailure())
  {
    throw std::runtime_error("Observation did not complete");
  }
  return observation.samples;
}
}

inline int runCalibrationClient(int argc, char** argv)
{
  using namespace client_detail;
  if (argc == 2 && std::string(argv[1]) == "--help")
  {
    std::cout << "Usage: amr_deadzone_calibration --robot [--params FILE]\n"
                 "  --expected-translation PCT --expected-rotation PCT --output NEW_JSON\n"
                 "  [--apply-translation PCT --confirm-write]\n"
                 "Default: ten-second read-only observation. Never drives.\n"
                 "Writes require both options, one attempt, and independent read-back.\n";
    return 0;
  }
  Options options;
  std::optional<Connection> connection;
  try
  {
    options = parseOptions(argc, argv);
    connection = loadConnection(options.parameters);
  }
  catch (const std::exception&)
  {
    std::cerr << "Invalid options, confirmation, output, or connection file (values withheld); no connection opened.\n";
    return 2;
  }
  try
  {
    AuditFile audit(options.output);
    Json report{ { "started_at_utc", utcNow() }, { "complete", false }, { "ready_for_motion", false } };
    ::signal(SIGINT, interrupt);
    ::signal(SIGTERM, interrupt);
    const auto read = [&]() {
      return boundedOperation([&]() {
        return amr_core::withRobotSession(*connection, [](amr_core::DriveClient& client) {
          return amr_core::ConfigurationInterface(client).getValues();
        });
      }, 15);
    };
    if (options.target)
    {
      const auto apply = [&]() {
        return boundedOperation([&]() {
          return amr_core::withRobotSession(*connection, [&](amr_core::DriveClient& client) {
            Json evidence{ { "complete", false }, { "write_attempted", false }, { "ready_for_motion", false } };
            try
            {
              amr_calibration::changeTranslationDeadzone(client, static_cast<int>(options.translation),
                  *options.target, static_cast<int>(options.rotation), evidence);
            }
            catch (const std::exception&)
            {
              evidence["failure_reason"] = "Preflight, receipt, or read-back failed. No retry.";
            }
            return evidence;
          });
        }, 45);
      };
      amr_calibration::applyTranslation(static_cast<int>(options.translation), *options.target,
          static_cast<int>(options.rotation), apply, read, [&](const Json& value) { audit.save(value); }, report);
    }
    else
    {
      report["read_only"] = true;
      audit.save(report);
      try
      {
        amr_calibration::observeDeadzones(options.translation, options.rotation, read, [&]() {
          return boundedOperation([&]() { return amr_core::withRobotSession(*connection, observe); }, 15);
        }, report);
      }
      catch (const std::exception&)
      {
        report["failure_reason"] = "Observation failed or read-back mismatched; trial is not accepted.";
      }
    }
    report["finished_at_utc"] = utcNow();
    audit.save(report);
    auto summary = report;
    summary.erase("samples");
    summary.erase("evidence");
    std::cout << summary.dump(2) << std::endl;
    return report.at("complete").get<bool>() ? 0 : 1;
  }
  catch (const std::exception&)
  {
    std::cerr << "Audit creation or persistence failed; no retry. Inspect the audit before further robot access.\n";
    return 1;
  }
}
}