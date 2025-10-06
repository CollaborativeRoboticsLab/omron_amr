#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cerrno>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

#include "amr_core_cpp/utils/amr_exception.hpp"
#include "amr_core_cpp/socket/socket_common.hpp"

/**
 * @brief Non-blocking TCP client for ARCL-like command/response workflows.
 *
 * SocketTaskmaster manages a single TCP connection using poll-based I/O.
 * It supports sending a command (optionally CRLF-terminated), collecting
 * feedback lines (CRLF-terminated), and detecting completion by matching
 * one or more configured end-line substrings.
 *
 * Uses rclcpp for logging and throws amr_exception on errors.
 */
class SocketTaskmaster
{
public:
  static constexpr size_t RECV_BUFFER = 4096;

  /**
   * @brief Construct a new SocketTaskmaster
   * @param node rclcpp node for logging
   */
  explicit SocketTaskmaster(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }

  /**
   * @brief Destructor (closes the socket if open)
   */
  ~SocketTaskmaster()
  {
    close_socket();
  }

  /**
   * @brief Connect to addr:port using a non-blocking socket (IPv4).
   * @param addr Dotted-quad IP (e.g., "127.0.0.1")
   * @param port TCP port
   * @return true if connect started (connected or EINPROGRESS)
   * @throws amr_exception on socket setup failures
   *
   * If connect() returns EINPROGRESS, completion is finalized on POLLOUT
   * inside process_events() via SO_ERROR.
   */
  bool connect(const std::string& addr, int port)
  {
    close_socket();
    connecting_ = false;

    RCLCPP_INFO(node_->get_logger(), "Starting connection to %s:%d", addr.c_str(), port);

    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0)
    {
      throw amr_exception(std::string("socket() failed: ") + std::strerror(errno));
    }

    amr::net::set_nonblocking(sock_fd_);

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, addr.c_str(), &sa.sin_addr) != 1)
    {
      int err = errno;
      close_socket();
      throw amr_exception(std::string("inet_pton failed: ") + std::strerror(err));
    }

    int rc = ::connect(sock_fd_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    if (rc == 0)
    {
      connecting_ = false;
      RCLCPP_INFO(node_->get_logger(), "Connected immediately");
      return true;
    }
    if (rc < 0 && errno == EINPROGRESS)
    {
      connecting_ = true;
      RCLCPP_DEBUG(node_->get_logger(), "Connect in progress (EINPROGRESS)");
      return true;
    }

    int err = errno;
    close_socket();
    throw amr_exception(std::string("connect() failed: ") + std::strerror(err));
  }

  /**
   * @brief Process ready I/O events indicated by poll().
   * @param revents Bitmask of returned poll events.
   * @throws amr_exception on socket errors or I/O failures.
   */
  void process_events(short revents)
  {
    if (revents & (POLLERR | POLLHUP | POLLNVAL))
    {
      int soerr = amr::net::get_so_error(sock_fd_);
      std::string msg = "socket error: " + std::string(std::strerror(soerr));
      close_socket();
      throw amr_exception(msg);
    }

    if (connecting_ && (revents & POLLOUT))
    {
      int soerr = amr::net::get_so_error(sock_fd_);
      if (soerr != 0)
      {
        std::string msg = "connect completion error: " + std::string(std::strerror(soerr));
        close_socket();
        throw amr_exception(msg);
      }
      connecting_ = false;
      RCLCPP_INFO(node_->get_logger(), "Connect completed");
    }

    if (revents & POLLIN)
      read();
    if (revents & POLLOUT)
      write();
  }

  /**
   * @brief Queue a command to send and configure completion criteria.
   * @param command Command content to send.
   * @param newline If true, appends "\r\n" to the command.
   * @param end_lines List of substrings that indicate command completion if seen in any parsed line.
   *
   * A leading "\r\n" is prepended to mimic the original Python behavior.
   * Any currently available incoming bytes are drained first.
   */
  void push_command(const std::string& command, bool newline = false, const std::vector<std::string>& end_lines = {})
  {
    std::string cmd = command;
    if (newline)
      cmd += "\r\n";

    _recv_buffer.clear();
    _send_buffer = "\r\n" + cmd;  // send a blank line first
    _check_end = end_lines;
    _result.clear();
    _feedback.clear();

    clear_recv_buf();  // drain stale incoming data
  }

  /**
   * @brief Result struct returned by wait_command().
   */
  struct WaitResult
  {
    bool complete;         ///< True if a completion line was matched
    std::string result;    ///< The matched line (without CRLF)
    std::string feedback;  ///< Accumulated feedback lines joined with CRLF
  };

  /**
   * @brief Poll once for I/O and process data.
   * @param poll_timeout_ms Timeout in milliseconds (-1 blocks indefinitely).
   * @return WaitResult with complete=true when an end_line is matched.
   * @throws amr_exception on poll or I/O failure.
   *
   * When complete, the result and feedback are returned and cleared internally.
   */
  WaitResult wait_command(int poll_timeout_ms = -1)
  {
    if (!is_open())
    {
      return { false, "", _feedback };
    }

    struct pollfd pfd{};
    pfd.fd = sock_fd_;
    pfd.events = POLLIN;
    if (connecting_ || !_send_buffer.empty())
      pfd.events |= POLLOUT;

    int rc = ::poll(&pfd, 1, poll_timeout_ms);
    if (rc < 0)
    {
      throw amr_exception(std::string("poll() failed: ") + std::strerror(errno));
    }
    if (rc > 0)
    {
      process_events(pfd.revents);
    }

    if (_result.empty())
    {
      return { false, "", _feedback };
    }
    else
    {
      auto res = _result;
      auto feed = _feedback;
      _result.clear();
      _feedback.clear();
      return { true, res, feed };
    }
  }

  /**
   * @brief Send password and wait for "End of commands".
   * @param passwd Password to send (appends CRLF).
   * @throws amr_exception on I/O failure.
   */
  void login(const std::string& passwd)
  {
    push_command(passwd, /*newline=*/true, /*end_lines=*/{ "End of commands" });
    wait_until_login();
  }

  /**
   * @brief Block until one of the end markers is detected.
   *
   * Polls indefinitely. Logs errors via rclcpp; on fatal I/O errors it
   * closes the socket and returns.
   */
  void wait_until_login()
  {
    while (true)
    {
      if (!is_open())
        break;

      struct pollfd pfd{};
      pfd.fd = sock_fd_;
      pfd.events = POLLIN | POLLOUT;

      int rc = ::poll(&pfd, 1, -1);
      if (rc < 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "poll() during login failed: %s", std::strerror(errno));
        continue;
      }
      if (rc > 0)
      {
        try
        {
          process_events(pfd.revents);
        }
        catch (const amr_exception& ex)
        {
          RCLCPP_ERROR(node_->get_logger(), "Login processing error: %s", ex.what());
          break;
        }
      }

      if (has_match_in_result())
      {
        _result.clear();
        _feedback.clear();
        _check_end.clear();
        break;
      }
    }
  }

  /**
   * @brief Check if the socket is open.
   */
  bool is_open() const
  {
    return sock_fd_ >= 0;
  }

private:
  /**
   * @brief Read from socket into receive buffer (non-blocking).
   * @throws amr_exception on recv error (other than EAGAIN/EWOULDBLOCK).
   */
  void read()
  {
    if (!is_open())
      return;

    std::string tmp;
    bool peer_closed = amr::net::recv_append_nb(sock_fd_, tmp);
    if (!tmp.empty())
    {
      _recv_buffer.append(tmp);
      if (!_check_end.empty())
      {
        extract_resp();
      }
    }
    if (peer_closed)
    {
      close_socket();
    }
  }

  /**
   * @brief Write pending bytes from send buffer (non-blocking).
   * @throws amr_exception on send error (other than EAGAIN/EWOULDBLOCK).
   */
  void write()
  {
    if (!is_open() || _send_buffer.empty() || connecting_)
      return;
    // send_erase_nb returns false if would-block; un-sent tail remains in _send_buffer.
    (void)amr::net::send_erase_nb(sock_fd_, _send_buffer);
  }

  /**
   * @brief Drain any currently available incoming bytes (used before sending new command).
   * @throws amr_exception on recv error (other than EAGAIN/EWOULDBLOCK).
   */
  void clear_recv_buf()
  {
    if (!is_open())
      return;
    std::string tmp;
    bool peer_closed = amr::net::recv_append_nb(sock_fd_, tmp);
    // discard tmp (stale data)
    if (peer_closed)
    {
      close_socket();
    }
  }

  /**
   * @brief Parse _recv_buffer by CRLF lines and update _feedback/_result.
   *
   * Moves complete lines from _recv_buffer into _feedback until a line containing
   * any of the configured end-line substrings is encountered. That line is stored
   * in _result (without CRLF).
   */
  void extract_resp()
  {
    std::vector<std::string> lines;
    if (amr::net::extract_crlf_lines(_recv_buffer, lines) == 0)
      return;

    for (const auto& line : lines)
    {
      if (any_end_in(line))
      {
        _result = line;
        return;
      }
      else
      {
        if (!_feedback.empty())
          _feedback += "\r\n";
        _feedback += line;
      }
    }
  }

  /**
   * @brief Test whether any end-line substring appears in the provided line.
   */
  bool any_end_in(const std::string& line) const
  {
    for (const auto& chk : _check_end)
    {
      if (!chk.empty() && line.find(chk) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Return true if the stored result matches any end-line substring.
   */
  bool has_match_in_result() const
  {
    return any_end_in(_result);
  }

  /**
   * @brief Close the socket if open.
   */
  void close_socket()
  {
    amr::net::close_fd(sock_fd_);
    connecting_ = false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  int sock_fd_{ -1 };
  bool connecting_{ false };

  // Buffers and state
  std::string _recv_buffer;
  std::string _send_buffer;
  std::vector<std::string> _check_end;
  std::string _result;
  std::string _feedback;
};