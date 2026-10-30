#pragma once

#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/map_stream.hpp"

#include <chrono>
#include <array>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <strings.h>
#include <sys/random.h>
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

  std::string download(const std::string& name)
  {
    if (name.empty() || name.size() > 128 || name.find_first_of("/\\\r\n") != std::string::npos ||
        name.find('\0') != std::string::npos || name.find("..") != std::string::npos)
      throw std::invalid_argument("Invalid remote map name");
    requireService("getFile");
    FileReply reply(name);
    ArFunctor1C<FileReply, ArNetPacket*> callback(&reply, &FileReply::receive);
    if (!client_.addHandler("getFile", &callback)) throw std::runtime_error("Cannot register file reader");
    ArNetPacket request;
    request.strToBuf(name.c_str());
    request.uByte2ToBuf(0);
    const bool submitted = client_.requestOnce("getFile", &request, true);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (submitted && !reply.complete && !reply.failed && client_.isConnected() &&
           !client_.hasTransportFailure() && std::chrono::steady_clock::now() < deadline)
      client_.loopOnce();
    client_.remHandler("getFile", &callback);
    if (!submitted || !reply.complete || reply.failed || !client_.isConnected() || client_.hasTransportFailure())
      throw std::runtime_error("Map file download failed or was incomplete");
    return reply.content;
  }

  std::string upload(const std::string& content, const std::string& requested_name = "")
  {
    if (content.empty() || content.size() > 64 * 1024 * 1024)
      throw std::invalid_argument("Invalid map size");
    requireService("putFileInterleaved");
    requireService("getFile");
    std::string name = requested_name;
    if (!name.empty())
    {
      if (name.size() > 128 || name.size() <= 4 || name.substr(name.size() - 4) != ".map" ||
          !std::all_of(name.begin(), name.end() - 4, [](unsigned char character) {
            return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
                   (character >= '0' && character <= '9') || character == '_' || character == '-';
          }))
        throw std::invalid_argument("Invalid map staging name");
      ensureNameAvailable(name);
    }
    else
    {
      std::array<unsigned char, 16> random{};
      std::size_t filled = 0;
      while (filled < random.size())
      {
        const auto count = ::getrandom(random.data() + filled, random.size() - filled, 0);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) throw std::runtime_error("Cannot generate map staging name");
        filled += static_cast<std::size_t>(count);
      }
      static constexpr char hex[] = "0123456789abcdef";
      name = "staged_";
      for (const auto byte : random)
      {
        name.push_back(hex[byte >> 4]);
        name.push_back(hex[byte & 15]);
      }
      name += ".map";
    }
    if (::strcasecmp(getName().c_str(), name.c_str()) == 0)
      throw std::runtime_error("Staging name matches active map");

    UploadReply reply;
    ArFunctor1C<UploadReply, ArNetPacket*> callback(&reply, &UploadReply::receive);
    if (!client_.addHandler("putFileInterleaved", &callback))
      throw std::runtime_error("Cannot register map-upload handler");
    struct HandlerGuard
    {
      DriveClient& client;
      ArFunctor1<ArNetPacket*>* callback;
      ~HandlerGuard() { client.remHandler("putFileInterleaved", callback); }
    } guard{ client_, &callback };

    const auto send = [&](unsigned short command, const char* data, std::size_t size, unsigned short expected) {
      ArNetPacket packet;
      packet.uByte2ToBuf(command);
      packet.strToBuf(name.c_str());
      if (command == 1)
      {
        packet.uByte4ToBuf(static_cast<ArTypes::UByte4>(size));
        packet.dataToBuf(data, size);
      }
      reply.received = false;
      const bool submitted = client_.requestOnce("putFileInterleaved", &packet, true);
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (submitted && !reply.received && client_.isConnected() && !client_.hasTransportFailure() &&
             std::chrono::steady_clock::now() < deadline)
        client_.loopOnce();
      if (!submitted || !reply.received || !reply.valid || reply.name != name || reply.code != expected ||
          !client_.isConnected() || client_.hasTransportFailure())
        throw std::runtime_error("Map upload failed or completion is unknown; staging name: " + name +
                                 (reply.received && reply.valid ? "; server code: " + std::to_string(reply.code) : ""));
    };
    send(0, nullptr, 0, 10);
    for (std::size_t offset = 0; offset < content.size();)
    {
      const auto size = std::min<std::size_t>(16384, content.size() - offset);
      send(1, content.data() + offset, size, 10);
      offset += size;
    }
    send(2, nullptr, 0, 0);
    verifyUpload(name, content);
    return name;
  }

private:
  struct DirectoryReply
  {
    explicit DirectoryReply(const std::string& requested_name) : name(requested_name) {}
    void receive(ArNetPacket* packet)
    {
      received = true;
      if (!packet || !packet->isValid() || packet->getDataLength() < 5) return;
      const auto code = packet->bufToUByte2();
      const auto directory = packet->bufToString();
      if (!packet->isValid() || code != 0 || !directory.empty()) return;
      for (int group = 0; group < 2; ++group)
      {
        if (packet->getDataLength() - packet->getDataReadLength() < 2) return;
        const auto count = packet->bufToUByte2();
        for (unsigned int index = 0; index < count; ++index)
        {
          const auto entry = packet->bufToString();
          if (!packet->isValid() || packet->getDataLength() - packet->getDataReadLength() < 12) return;
          if (::strcasecmp(entry.c_str(), name.c_str()) == 0) exists = true;
          packet->bufToUByte4();
          packet->bufToUByte4();
          packet->bufToUByte4();
        }
      }
      valid = packet->isValid() && packet->getDataReadLength() == packet->getDataLength();
    }
    const std::string& name;
    bool received{ false };
    bool valid{ false };
    bool exists{ false };
  };

  void ensureNameAvailable(const std::string& name)
  {
    requireService("getDirListing");
    DirectoryReply reply(name);
    ArFunctor1C<DirectoryReply, ArNetPacket*> callback(&reply, &DirectoryReply::receive);
    if (!client_.addHandler("getDirListing", &callback)) throw std::runtime_error("Cannot register directory handler");
    ArNetPacket request;
    request.strToBuf("");
    const bool submitted = client_.requestOnce("getDirListing", &request, true);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (submitted && !reply.received && client_.isConnected() && !client_.hasTransportFailure() &&
           std::chrono::steady_clock::now() < deadline)
      client_.loopOnce();
    client_.remHandler("getDirListing", &callback);
    if (!submitted || !reply.received || !reply.valid || !client_.isConnected() || client_.hasTransportFailure())
      throw std::runtime_error("Cannot verify remote map name is available");
    if (reply.exists) throw std::runtime_error("Remote map name already exists");
  }

  struct FileReply
  {
    explicit FileReply(const std::string& expected_name) : name(expected_name) {}
    void receive(ArNetPacket* packet)
    {
      if (!packet || !packet->isValid()) { failed = true; return; }
      if (packet->getDataLength() == packet->getDataReadLength()) { if (!complete) failed = true; return; }
      if (complete || packet->getDataLength() - packet->getDataReadLength() < 3) { failed = true; return; }
      const auto code = packet->bufToUByte2();
      const auto received_name = packet->bufToString();
      if (!packet->isValid() || code != 0 || received_name != name ||
          packet->getDataLength() - packet->getDataReadLength() < 4) { failed = true; return; }
      const auto size = packet->bufToUByte4();
      const auto remaining = static_cast<std::size_t>(packet->getDataLength() - packet->getDataReadLength());
      if (!packet->isValid() || size != remaining || size > 64 * 1024 * 1024 - content.size()) { failed = true; return; }
      if (size == 0) { complete = true; return; }
      content.append(packet->getBuf() + packet->getReadLength(), size);
    }
    const std::string& name;
    std::string content;
    bool complete{ false };
    bool failed{ false };
  };

  void verifyUpload(const std::string& name, const std::string& content)
  {
    try
    {
      if (download(name) == content) return;
    }
    catch (const std::exception&)
    {
    }
    throw std::runtime_error("Map uploaded but readback failed or differed; staging name: " + name);
  }

  struct UploadReply
  {
    void receive(ArNetPacket* packet)
    {
      received = true;
      valid = false;
      if (!packet || !packet->isValid() || packet->getDataLength() - packet->getDataReadLength() < 3) return;
      code = packet->bufToUByte2();
      name = packet->bufToString();
      valid = packet->isValid() && packet->getDataReadLength() == packet->getDataLength();
    }
    std::string name;
    unsigned short code{ 0 };
    bool received{ false };
    bool valid{ false };
  };

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