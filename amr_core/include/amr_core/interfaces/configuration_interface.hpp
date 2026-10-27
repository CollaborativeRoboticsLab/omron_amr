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

  bool setTranslationDeadzone(int expected, int target)
  {
    const auto encoded = translationDeadzoneRequest(expected, target);
    requireService("setConfigValues");
    Receipt receipt;
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

private:
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