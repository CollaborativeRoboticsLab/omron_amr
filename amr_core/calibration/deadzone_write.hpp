#pragma once

#include "../test/hardware/hybrid_config_client.hpp"

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
inline std::vector<std::uint8_t> translationDeadzoneRequest(int expected, int target)
{
  if (expected < 1 || expected > 99 || target < 1 || target > 99 || expected == target)
  {
    throw std::invalid_argument("Dead-zone change requires distinct integer percentages in [1, 99]");
  }
  std::vector<std::uint8_t> body{ 1, 0, 0, 0, 1, 4, 1 };
  for (const auto& field : { std::string("Robot Operation"), std::string("Teleop Settings"), std::string(),
                             std::string("TransDeadZone") })
  {
    body.insert(body.end(), field.begin(), field.end());
    body.push_back(0);
  }
  body.push_back('d');
  for (const auto& field : { std::to_string(expected), std::to_string(target) })
  {
    body.insert(body.end(), field.begin(), field.end());
    body.push_back(0);
  }
  std::vector<std::uint8_t> request{ 1, 0, 1, 0, 0, 0 };
  for (int length_field = 0; length_field < 2; ++length_field)
  {
    const auto length = static_cast<std::uint64_t>(body.size());
    for (unsigned shift = 0; shift < 64; shift += 8)
    {
      request.push_back(static_cast<std::uint8_t>(length >> shift));
    }
  }
  request.insert(request.end(), body.begin(), body.end());
  return request;
}

inline bool matchesSingleChangeReceipt(const std::vector<std::uint8_t>& payload)
{
  return payload == std::vector<std::uint8_t>{ 1, 0, 1, 0, 0, 0 };
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
  const auto encoded = translationDeadzoneRequest(expected, target);
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
    if (packet->getDataLength() < 19)
    {
      valid = false;
      return;
    }
    packet->bufToByte2();
    const std::array<int, 3> position{ packet->bufToByte4(), packet->bufToByte4(), packet->bufToByte2() };
    const auto forward = packet->bufToByte2();
    const auto angular = packet->bufToByte2();
    const auto lateral = packet->bufToByte2();
    if (samples == 0)
    {
      origin = position;
    }
    valid = valid && position == origin && forward == 0 && angular == 0 && lateral == 0;
    ++samples;
    latest = std::chrono::steady_clock::now();
  }

  std::array<int, 3> origin{};
  int samples = 0;
  bool valid = true;
  std::chrono::steady_clock::time_point latest{};
};

class Receipt
{
public:
  void receive(ArNetPacket* packet)
  {
    ++count;
    const auto* begin = reinterpret_cast<const std::uint8_t*>(packet->getBuf()) + packet->getHeaderLength();
    matches = matchesSingleChangeReceipt({ begin, begin + packet->getDataLength() });
  }

  int count = 0;
  bool matches = false;
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

inline bool writeOnce(amr_core::DriveClient& client, const std::vector<std::uint8_t>& encoded)
{
  Receipt receipt;
  ArFunctor1C<Receipt, ArNetPacket*> callback(&receipt, &Receipt::receive);
  if (!client.addHandler("setConfigValues", &callback))
  {
    return false;
  }
  ArNetPacket packet;
  packet.dataToBuf(encoded.data(), static_cast<int>(encoded.size()));
  const bool submitted = client.requestOnce("setConfigValues", &packet);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (submitted && receipt.count == 0 && client.isConnected() && !client.hasTransportFailure() &&
         std::chrono::steady_clock::now() < deadline)
  {
    client.loopOnce();
  }
  client.remHandler("setConfigValues", &callback);
  return submitted && receipt.count == 1 && receipt.matches && client.isConnected() && !client.hasTransportFailure();
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
  const auto read = [&]() {
    nlohmann::json snapshot{ { "read_only", true } };
    amr_hardware::captureHybridConfiguration(client, snapshot);
    return snapshot;
  };
  changeTranslationDeadzone(expected, target, rotation, read,
      [&]() { return detail::stationary(client, report); },
      [&](const std::vector<std::uint8_t>& encoded) { return detail::writeOnce(client, encoded); }, report);
}
}