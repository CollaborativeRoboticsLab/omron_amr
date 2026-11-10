#pragma once

#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/configuration_protocol.hpp"

#include <chrono>

namespace amr_core
{
class HybridObservation
{
public:
  explicit HybridObservation(const std::string& section) : transfer(section) {}

  void receive(ArNetPacket* packet)
  {
    if (failed)
    {
      return;
    }
    try
    {
      const auto* begin = reinterpret_cast<const std::uint8_t*>(packet->getBuf()) + packet->getHeaderLength();
      transfer.accept(std::vector<std::uint8_t>(begin, begin + packet->getDataLength()));
    }
    catch (const std::exception&)
    {
      failed = true;
    }
  }

  SectionTransfer transfer;
  bool failed = false;
};

inline void captureHybridConfiguration(amr_core::DriveClient& client, nlohmann::json& snapshot)
{
  snapshot["config_service"] = "getConfigValues";
  snapshot["protocol_profile"] = "aram_hybrid_v8";
  snapshot["scope"] = "allowlisted_drive_parameters";
  snapshot["parameters"] = nlohmann::json::array();
  snapshot["complete"] = false;
  try
  {
    for (const auto* section : { "Robot config", "Teleop Settings" })
    {
      HybridObservation observation(section);
      ArFunctor1C<HybridObservation, ArNetPacket*> callback(&observation, &HybridObservation::receive);
      if (!client.addHandler("getConfigValues", &callback))
      {
        throw std::runtime_error("Unable to register configuration reader");
      }
      const auto encoded = sectionRequest(section);
      ArNetPacket request;
      request.dataToBuf(encoded.data(), static_cast<int>(encoded.size()));
      const bool requested = client.requestOnce("getConfigValues", &request);
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (requested && client.isConnected() && !client.hasTransportFailure() && !observation.failed &&
             !observation.transfer.complete() && std::chrono::steady_clock::now() < deadline)
      {
        client.loopOnce();
      }
      client.remHandler("getConfigValues", &callback);
      if (!requested || observation.failed || !client.isConnected() || client.hasTransportFailure())
      {
        throw std::runtime_error("Configuration reply rejected or connection failed");
      }
      for (const auto& parameter : observation.transfer.parameters())
      {
        snapshot["parameters"].push_back(parameter);
      }
    }
    std::set<std::string> received;
    for (const auto& parameter : snapshot["parameters"])
    {
      received.insert(parameter.at("name").get<std::string>());
    }
    if (received != driveParameters)
    {
      throw std::runtime_error("Required drive configuration parameters are missing");
    }
    snapshot["complete"] = true;
  }
  catch (const std::exception& error)
  {
    snapshot["failure_reason"] = error.what();
  }
}

class ConfigurationInterface
{
public:
  explicit ConfigurationInterface(DriveClient& client) : client_(client) {}

  nlohmann::json getValues()
  {
    requireService("getConfigValues");
    nlohmann::json snapshot{ { "read_only", true } };
    captureHybridConfiguration(client_, snapshot);
    return snapshot;
  }

  std::string getMapSelection()
  {
    requireService("getConfigValues");
    HybridObservation observation("Files");
    ArFunctor1C<HybridObservation, ArNetPacket*> callback(&observation, &HybridObservation::receive);
    if (!client_.addHandler("getConfigValues", &callback)) throw std::runtime_error("Cannot register configuration reader");
    const auto encoded = sectionRequest("Files");
    ArNetPacket request;
    request.dataToBuf(encoded.data(), static_cast<int>(encoded.size()));
    const bool submitted = client_.requestOnce("getConfigValues", &request);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (submitted && client_.isConnected() && !client_.hasTransportFailure() && !observation.failed &&
           !observation.transfer.complete() && std::chrono::steady_clock::now() < deadline)
      client_.loopOnce();
    client_.remHandler("getConfigValues", &callback);
    if (!submitted || observation.failed || !observation.transfer.complete() || !client_.isConnected() ||
        client_.hasTransportFailure())
      throw std::runtime_error("Map configuration read failed");
    const auto values = observation.transfer.parameters();
    if (values.size() != 1 || values[0].at("name") != "Map") throw std::runtime_error("Map setting unavailable");
    return values[0].at("value").get<std::string>();
  }

  bool setMapSelection(const std::string& expected, const std::string& target)
  {
    return writeSingleValue(mapSelectionRequest(expected, target), 2);
  }

  bool setTranslationDeadzone(int expected, int target)
  {
    return writeSingleValue(translationDeadzoneRequest(expected, target));
  }

  bool setRotationDeadzone(int expected, int target)
  {
    return writeSingleValue(rotationDeadzoneRequest(expected, target));
  }

  bool setTeleopTimeout(double expected, double target)
  {
    return writeSingleValue(teleopTimeoutRequest(expected, target));
  }

private:
  bool writeSingleValue(const std::vector<std::uint8_t>& encoded, std::uint8_t expected_count = 1)
  {
    requireService("setConfigValues");
    Receipt receipt(expected_count);
    ArFunctor1C<Receipt, ArNetPacket*> callback(&receipt, &Receipt::receive);
    if (!client_.addHandler("setConfigValues", &callback))
    {
      return false;
    }
    ArNetPacket packet;
    packet.dataToBuf(encoded.data(), static_cast<int>(encoded.size()));
    const bool submitted = client_.requestOnce("setConfigValues", &packet);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (submitted && receipt.count == 0 && client_.isConnected() && !client_.hasTransportFailure() &&
           std::chrono::steady_clock::now() < deadline)
    {
      client_.loopOnce();
    }
    client_.remHandler("setConfigValues", &callback);
    return submitted && receipt.count == 1 && receipt.matches && client_.isConnected() &&
           !client_.hasTransportFailure();
  }

  class Receipt
  {
  public:
    explicit Receipt(std::uint8_t expected_count) : expected_count_(expected_count) {}
    void receive(ArNetPacket* packet)
    {
      ++count;
      const auto* begin = reinterpret_cast<const std::uint8_t*>(packet->getBuf()) + packet->getHeaderLength();
      matches = std::vector<std::uint8_t>(begin, begin + packet->getDataLength()) ==
            std::vector<std::uint8_t>{ 1, 0, expected_count_, 0, 0, 0 };
    }

    int count = 0;
    bool matches = false;
  private:
    std::uint8_t expected_count_;
  };

  void requireService(const char* service) const
  {
    if (!client_.isConnected() || client_.hasTransportFailure() || !client_.dataExists(service))
    {
      throw std::runtime_error("A connected client with the named configuration service is required");
    }
  }

  DriveClient& client_;
};
}