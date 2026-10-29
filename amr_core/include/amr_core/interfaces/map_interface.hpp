#pragma once

#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/map_stream.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

namespace amr_core
{
struct MapSnapshot
{
  std::string name;
  std::string content;
};

class MapInterface
{
public:
  explicit MapInterface(DriveClient& client) : client_(client) {}

  std::string getName()
  {
    requireService("getMapName");
    NameReply reply;
    ArFunctor1C<NameReply, ArNetPacket*> callback(&reply, &NameReply::receive);
    if (!client_.addHandler("getMapName", &callback)) throw std::runtime_error("Cannot register map-name handler");
    const bool submitted = client_.requestOnce("getMapName", nullptr, true);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (submitted && !reply.received && client_.isConnected() && !client_.hasTransportFailure() &&
           std::chrono::steady_clock::now() < deadline)
      client_.loopOnce();
    client_.remHandler("getMapName", &callback);
    if (!submitted || !reply.received || !reply.valid || reply.name.empty() || !client_.isConnected() ||
        client_.hasTransportFailure())
      throw std::runtime_error("Map-name query failed or returned an invalid name");
    return reply.name;
  }

  MapSnapshot snapshot()
  {
    const auto initial_name = getName();
    requireService("getMap");
    StreamReply reply;
    ArFunctor1C<StreamReply, ArNetPacket*> callback(&reply, &StreamReply::receive);
    if (!client_.addHandler("getMap", &callback)) throw std::runtime_error("Cannot register map-stream handler");
    const bool submitted = client_.requestOnce("getMap", nullptr, true);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (submitted && !reply.stream.complete() && !reply.stream.failed() && client_.isConnected() &&
           !client_.hasTransportFailure() && std::chrono::steady_clock::now() < deadline)
      client_.loopOnce();
    client_.remHandler("getMap", &callback);
    if (!submitted || !reply.stream.complete() || !client_.isConnected() || client_.hasTransportFailure())
      throw std::runtime_error("Map stream incomplete or invalid: packets=" + std::to_string(reply.stream.packets()) +
                               " lines=" + std::to_string(reply.stream.lines()) +
                               " last_bytes=" + std::to_string(reply.stream.lastBytes()) +
                               " last_terminators=" + std::to_string(reply.stream.lastTerminators()) +
                               " rejected=" + std::to_string(reply.stream.failed()));
    if (getName() != initial_name) throw std::runtime_error("Active map changed during snapshot");
    return { initial_name, reply.stream.content() };
  }

private:
  struct NameReply
  {
    void receive(ArNetPacket* packet)
    {
      received = true;
      if (!packet || !packet->isValid() || packet->getDataLength() <= packet->getDataReadLength()) return;
      name = packet->bufToString();
      valid = packet->isValid() && packet->getDataReadLength() == packet->getDataLength() && name.size() <= 4096;
    }
    std::string name;
    bool received{ false };
    bool valid{ false };
  };

  struct StreamReply
  {
    void receive(ArNetPacket* packet) { stream.accept(packet); }
    MapStream stream;
  };

  void requireService(const char* service) const
  {
    if (!client_.isConnected() || client_.hasTransportFailure() || !client_.dataExists(service))
      throw std::runtime_error("Connected map service required");
  }

  DriveClient& client_;
};
}  // namespace amr_core