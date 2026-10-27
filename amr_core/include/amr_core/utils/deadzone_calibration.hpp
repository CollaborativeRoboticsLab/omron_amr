#pragma once

#include "amr_core/interfaces/configuration_interface.hpp"
#include "amr_core/utils/drive_telemetry.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace amr_calibration
{
inline nlohmann::json deadzoneValues(const nlohmann::json& snapshot)
{
  if (!snapshot.value("complete", false) || !snapshot.value("read_only", false) ||
      !snapshot.at("parameters").is_array())
  {
    throw std::invalid_argument("A complete read-only snapshot is required");
  }
  nlohmann::json result = nlohmann::json::object();
  for (const auto& parameter : snapshot.at("parameters"))
  {
    const auto name = parameter.at("name").get<std::string>();
    if (parameter.at("section") != "Teleop Settings" || (name != "TransDeadZone" && name != "RotDeadZone"))
    {
      continue;
    }
    const auto& value = parameter.at("value");
    if (result.contains(name) || !value.is_number() || !std::isfinite(value.get<double>()) ||
        value.get<double>() < 0 || value.get<double>() >= 100)
    {
      throw std::invalid_argument("Invalid or duplicate dead-zone value");
    }
    result[name] = value;
  }
  if (result.size() != 2)
  {
    throw std::invalid_argument("Both dead-zone values are required");
  }
  return result;
}

inline nlohmann::json summarizeObservation(const nlohmann::json& samples)
{
  if (!samples.is_array() || samples.size() < 2)
  {
    throw std::invalid_argument("Insufficient odometry samples");
  }
  const std::array<const char*, 6> fields{
    "x_mm", "y_mm", "heading_deg", "forward_mm_s", "angular_deg_s", "lateral_mm_s"
  };
  double position_change = 0;
  double heading_change = 0;
  std::array<double, 3> peaks{};
  for (const auto& sample : samples)
  {
    for (const auto* field : fields)
    {
      if (!sample.contains(field) || !sample.at(field).is_number() ||
          !std::isfinite(sample.at(field).get<double>()))
      {
        throw std::invalid_argument("Malformed odometry sample");
      }
    }
    const auto& first = samples.front();
    position_change = std::max(position_change,
        std::hypot(sample.at("x_mm").get<double>() - first.at("x_mm").get<double>(),
                   sample.at("y_mm").get<double>() - first.at("y_mm").get<double>()));
    heading_change = std::max(heading_change, std::fabs(std::remainder(
        sample.at("heading_deg").get<double>() - first.at("heading_deg").get<double>(), 360.0)));
    for (std::size_t index = 0; index < peaks.size(); ++index)
    {
      peaks[index] = std::max(peaks[index], std::fabs(sample.at(fields[index + 3]).get<double>()));
    }
  }
  return { { "samples", samples.size() }, { "max_position_change_mm", position_change },
           { "max_heading_change_deg", heading_change },
           { "peak_reported_speeds", { { fields[3], peaks[0] }, { fields[4], peaks[1] }, { fields[5], peaks[2] } } },
           { "motion_evidence", position_change > 0 || heading_change > 0 ||
                                peaks[0] > 0 || peaks[1] > 0 || peaks[2] > 0 },
           { "no_creep_proven", false } };
}

template <typename Read, typename Observe>
void observeDeadzones(double translation, double rotation, Read read, Observe observe, nlohmann::json& report)
{
  report["read_only"] = true;
  report["complete"] = false;
  report["ready_for_motion"] = false;
  report["expected_deadzones"] = { { "TransDeadZone", translation }, { "RotDeadZone", rotation } };
  report["samples"] = nlohmann::json::array();
  report["before"] = deadzoneValues(read());
  if (report.at("before") != report.at("expected_deadzones"))
  {
    throw std::runtime_error("Read-back differs from expected settings; trial not started");
  }
  report["samples"] = observe();
  report["odometry"] = summarizeObservation(report.at("samples"));
  report["after"] = deadzoneValues(read());
  if (report.at("after") != report.at("expected_deadzones"))
  {
    throw std::runtime_error("Settings changed during observation; trial invalid");
  }
  report["complete"] = true;
}

template <typename Apply, typename Readback, typename Save>
void applyTranslation(int expected, int target, int rotation, Apply apply, Readback readback, Save save,
                      nlohmann::json& report)
{
  report.update({ { "operation", "translation_deadzone_write" }, { "expected_translation", expected },
                  { "target_translation", target }, { "expected_rotation", rotation },
                  { "complete", false }, { "write_attempted", nullptr }, { "ready_for_motion", false },
                  { "persistence_verified", false }, { "state", "pending" } });
  save(report);
  try
  {
    report["evidence"] = apply();
    const auto& evidence = report.at("evidence");
    report["write_attempted"] = evidence.value("write_attempted", nlohmann::json(nullptr));
    report["complete"] = evidence.at("complete").get<bool>();
    report["state"] = report.at("complete").get<bool>() ? "verified" : "failed_or_uncertain";
  }
  catch (const std::exception&)
  {
    report["complete"] = false;
    report["state"] = "failed_or_uncertain";
    report["failure_reason"] = "Calibration invocation failed; no retry. Independent read-back required.";
  }
  save(report);
  try
  {
    report["independent_readback"] = deadzoneValues(readback());
    if (report.at("independent_readback") !=
        nlohmann::json{ { "TransDeadZone", target }, { "RotDeadZone", rotation } })
    {
      report["complete"] = false;
      report["state"] = "failed_or_uncertain";
    }
  }
  catch (const std::exception&)
  {
    report["complete"] = false;
    report["state"] = "failed_or_uncertain";
    report["readback_failure"] = "Independent read-back unavailable; current robot state is unverified.";
  }
  save(report);
}

using Configuration = std::map<std::pair<std::string, std::string>, nlohmann::json>;

inline Configuration checkedConfiguration(const nlohmann::json& snapshot)
{
  if (!snapshot.value("complete", false) || !snapshot.value("read_only", false) ||
      !snapshot.at("parameters").is_array())
  {
    throw std::runtime_error("A complete read-only configuration is required");
  }
  Configuration configuration;
  for (const auto& parameter : snapshot.at("parameters"))
  {
    const auto& value = parameter.at("value");
    const auto identity = std::make_pair(parameter.at("section").get<std::string>(),
                                         parameter.at("name").get<std::string>());
    if (!value.is_number() || !std::isfinite(value.get<double>()) ||
        !configuration.emplace(identity, parameter).second)
    {
      throw std::runtime_error("Invalid or duplicate configuration value");
    }
  }
  if (configuration.size() != 22)
  {
    throw std::runtime_error("Expected the complete 22-parameter drive snapshot");
  }
  return configuration;
}

template <typename Read, typename Stationary, typename Write>
void changeTranslationDeadzone(int expected, int target, int rotation, Read read, Stationary stationary,
                               Write write, nlohmann::json& report)
{
  const auto encoded = amr_core::translationDeadzoneRequest(expected, target);
  if (rotation < 1 || rotation > 99)
  {
    throw std::invalid_argument("Expected rotation must be in [1, 99]");
  }
  const auto translation_key = std::make_pair(std::string("Teleop Settings"), std::string("TransDeadZone"));
  const auto rotation_key = std::make_pair(std::string("Teleop Settings"), std::string("RotDeadZone"));
  report["before"] = read();
  auto before = checkedConfiguration(report.at("before"));
  if (before.at(translation_key).at("value") != expected || before.at(rotation_key).at("value") != rotation)
  {
    throw std::runtime_error("Expected dead zones do not match read-back");
  }
  if (!stationary())
  {
    throw std::runtime_error("Stationary telemetry preflight failed");
  }
  report["pre_write"] = read();
  if (checkedConfiguration(report.at("pre_write")) != before)
  {
    throw std::runtime_error("Configuration changed during preflight");
  }
  report["write_attempted"] = true;
  report["receipt_matches"] = write(encoded);
  report["after"] = read();
  before.at(translation_key)["value"] = target;
  if (checkedConfiguration(report.at("after")) != before)
  {
    throw std::runtime_error("Post-write configuration differs from the single authorized value change");
  }
  if (!report.at("receipt_matches").get<bool>())
  {
    throw std::runtime_error("Write receipt did not match; independent read-back is required");
  }
  report["complete"] = true;
}

namespace detail
{
class StationaryObservation
{
public:
  void receive(ArNetPacket* packet)
  {
    amr_core::DriveTelemetry telemetry;
    if (!amr_core::readDriveTelemetry(packet, telemetry))
    {
      valid = false;
      return;
    }
    const auto position = telemetry.position();
    if (samples == 0)
    {
      origin = position;
    }
        valid = valid && position == origin && telemetry.forward_mm_s == 0 &&
          telemetry.angular_deg_s == 0 && telemetry.lateral_mm_s == 0;
    ++samples;
    latest = std::chrono::steady_clock::now();
  }

  std::array<int, 3> origin{};
  int samples = 0;
  bool valid = true;
  std::chrono::steady_clock::time_point latest{};
};

inline bool stationary(amr_core::DriveClient& client, nlohmann::json& report)
{
  StationaryObservation observation;
  ArFunctor1C<StationaryObservation, ArNetPacket*> callback(&observation, &StationaryObservation::receive);
  if (!client.dataExists("updateNumbers") || !client.addHandler("updateNumbers", &callback))
  {
    return false;
  }
  const bool requested = client.request("updateNumbers", 100);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (requested && client.isConnected() && !client.hasTransportFailure() &&
         std::chrono::steady_clock::now() < deadline)
  {
    client.loopOnce();
  }
  client.requestStop("updateNumbers");
  client.remHandler("updateNumbers", &callback);
  const bool fresh = std::chrono::steady_clock::now() - observation.latest <= std::chrono::milliseconds(300);
  report["stationary_preflight"] = { { "samples", observation.samples }, { "unchanged", observation.valid },
                                      { "fresh", fresh } };
  return requested && observation.valid && fresh && observation.samples >= 10 &&
         client.isConnected() && !client.hasTransportFailure();
}

}

inline void changeTranslationDeadzone(amr_core::DriveClient& client, int expected, int target, int rotation,
                                      nlohmann::json& report)
{
  report = { { "complete", false }, { "write_attempted", false }, { "ready_for_motion", false },
              { "expected_translation", expected }, { "target_translation", target },
              { "expected_rotation", rotation }, { "properties_written", false },
              { "persistence_verified", false } };
  if (!client.isConnected() || client.hasTransportFailure() || !client.dataExists("getConfigValues") ||
      !client.dataExists("setConfigValues"))
  {
    throw std::runtime_error("A connected client with named configuration services is required");
  }
  amr_core::ConfigurationInterface configuration(client);
  const auto read = [&]() { return configuration.getValues(); };
  changeTranslationDeadzone(expected, target, rotation, read,
      [&]() { return detail::stationary(client, report); },
      [&](const std::vector<std::uint8_t>&) { return configuration.setTranslationDeadzone(expected, target); }, report);
}
}