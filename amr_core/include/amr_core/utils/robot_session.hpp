#pragma once

#include "amr_core/utils/drive_client.hpp"
#include "amr_core/utils/libaria_runtime.hpp"

#include <stdexcept>
#include <string>

namespace amr_core
{
struct RobotConnection
{
  std::string host;
  int port = 7272;
  std::string user;
  std::string password;
  std::string protocol = "6MTX";

  void validate() const
  {
    if (port < 1 || port > 65535 || host.empty() || protocol.empty())
    {
      throw std::invalid_argument("Invalid connection settings (values withheld)");
    }
    for (const auto* field : { &host, &user, &password, &protocol })
    {
      if (field->size() > 4096 || field->find_first_of("\r\n") != std::string::npos ||
          field->find('\0') != std::string::npos)
      {
        throw std::invalid_argument("Invalid connection field (values withheld)");
      }
    }
  }
};

class RobotSession
{
public:
  RobotSession() : runtime_(LibAriaRuntime::Logging::Silent) {}
  ~RobotSession() { client_.disconnect(); }

  bool connect(const RobotConnection& connection)
  {
    connection.validate();
    client_.enforceProtocolVersion(connection.protocol.c_str());
    client_.setConnectTimeoutTime(10);
    return client_.configureTransport(32, 65536, 0.1) &&
           client_.blockingConnect(connection.host.c_str(), connection.port, false, connection.user.c_str(),
                                    connection.password.empty() ? nullptr : connection.password.c_str());
  }

  DriveClient& client() { return client_; }

private:
  LibAriaRuntime runtime_;
  DriveClient client_;
};

template <typename Operation>
auto withRobotSession(const RobotConnection& connection, Operation operation)
{
  connection.validate();
  RobotSession session;
  if (!session.connect(connection))
  {
    throw std::runtime_error("Connection or authentication failed (values withheld)");
  }
  return operation(session.client());
}
}