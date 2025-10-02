#pragma once

#include <string>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "amr_core_cpp/amr_exception.hpp"
#include "amr_core_cpp/utils/socket_taskmaster.hpp"

/**
 * @brief Thin wrapper that connects to an ARCL-like server and performs login.
 *
 * Reads parameters from the provided node (declaring them if needed):
 *  - ip_address (string)
 *  - port (int)
 *  - def_arcl_passwd (string)
 *
 * Behavior mirrors the original Python:
 *  - Retries connect up to MAX_CONN_RETRY with CONN_RETRY_SLP delay
 *  - Sends password with CRLF and waits for "End of commands"
 */
class ConnectSocket
{
public:
  static constexpr int DEFAULT_SOCKET_TIMEOUT = 10;  // kept for parity (not used; non-blocking)
  static constexpr size_t RECV_BUFFER = 2048;        // kept for parity
  static constexpr int MAX_CONN_RETRY = 5;
  static constexpr int CONN_RETRY_SLP_SEC = 2;

  static constexpr const char* PASSWD_PROMPT_STRING = "Enter password:";
  static constexpr const char* WELCOME_STRING = "Welcome to the server.";
  static constexpr const char* END_OF_CMDS_STRING = "End of commands";
  static constexpr const char* MAX_TRIES_ERROR = "Maximum number of connection retries reached. Are you sure you are "
                                                 "connected? Is the password correct?";

  /**
   * @brief Construct and prepare connection credentials from node parameters.
   * @param node rclcpp node for logging and parameter access
   *
   * Declares parameters if missing:
   *  ip_address (default "127.0.0.1"), port (default 0), def_arcl_passwd (default "")
   */
  explicit ConnectSocket(const rclcpp::Node::SharedPtr& node) : node_(node), tm_(node_)
  {
    ip_addr_ = node_->declare_parameter<std::string>("ip_address", "127.0.0.1");
    port_ = static_cast<uint16_t>(node_->declare_parameter<int>("port", 0));
    passwd_ = node_->declare_parameter<std::string>("def_arcl_passwd", "");
    if (passwd_.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "def_arcl_passwd parameter is empty");
    }
  }

  /**
   * @brief Construct with explicit endpoint/password (bypasses parameter fetch).
   */
  ConnectSocket(const rclcpp::Node::SharedPtr& node, std::string ip_addr, uint16_t port, std::string passwd)
    : node_(node), tm_(node_), ip_addr_(std::move(ip_addr)), port_(port), passwd_(std::move(passwd))
  {
  }

  /**
   * @brief Attempt to connect with retry and perform login handshake.
   * @throws amr_exception on final failure.
   */
  void connect_and_login()
  {
    int tries = 0;
    for (;;)
    {
      try
      {
        (void)tm_.connect(ip_addr_, static_cast<int>(port_));
        break;
      }
      catch (const amr_exception& ex)
      {
        ++tries;
        RCLCPP_INFO(node_->get_logger(), "Socket make connection failed: %s", ex.what());
        if (tries >= MAX_CONN_RETRY)
        {
          throw amr_exception(MAX_TRIES_ERROR);
        }
        std::this_thread::sleep_for(std::chrono::seconds(CONN_RETRY_SLP_SEC));
      }
    }

    // Send password and wait for "End of commands"
    tm_.login(passwd_);
    RCLCPP_INFO(node_->get_logger(), "Login to %s using the socket wrapper", ip_addr_.c_str());
  }

  /**
   * @brief Access the underlying taskmaster to send commands/responses after login.
   */
  SocketTaskmaster& taskmaster()
  {
    return tm_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  SocketTaskmaster tm_;
  std::string ip_addr_;
  uint16_t port_{ 0 };
  std::string passwd_;
};