#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <cerrno>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>
#include "amr_core_cpp/utils/amr_exception.hpp"
#include "amr_core_cpp/utils/socket_common.hpp"

/**
 * @brief Non-blocking TCP socket driver for simple client/server use.
 *
 * - Client mode: connect() to host:port (non-blocking, EINPROGRESS-safe).
 * - Server mode: begin_listen() binds, listens, and accepts a single client.
 * - Poll-based I/O with internal send/receive buffers.
 * - CRLF line helpers: send_line(), read_lines().
 * - rclcpp logging; errors throw amr_exception.
 */
class SocketDriver
{
public:
  /// Receive buffer chunk size
  static constexpr size_t RECV_CHUNK = 4096;

  /// Operating mode of the driver
  enum class Mode
  {
    None,
    Client,
    Server
  };

  /**
   * @brief Construct a new SocketDriver.
   * @param node ROS 2 node for logging.
   */
  explicit SocketDriver(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }

  /**
   * @brief Destructor closes any open sockets.
   */
  ~SocketDriver()
  {
    close();
  }

  /**
   * @brief Initiate a non-blocking client connection to addr:port (IPv4).
   * @param addr Dotted-quad IP (e.g., "127.0.0.1").
   * @param port TCP port.
   * @return true if connect initiated (connected or in progress).
   * @throws amr_exception on socket creation or configuration failure.
   *
   * If connect returns EINPROGRESS, completion is detected on POLLOUT in poll_once().
   */
  bool connect(const std::string& addr, uint16_t port)
  {
    close();
    mode_ = Mode::Client;

    RCLCPP_INFO(node_->get_logger(), "Connecting to %s:%u", addr.c_str(), port);

    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0)
    {
      throw amr_exception(std::string("socket() failed: ") + std::strerror(errno));
    }

    // Non-blocking
    amr::net::set_nonblocking(sock_fd_);

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    if (::inet_pton(AF_INET, addr.c_str(), &sa.sin_addr) != 1)
    {
      close();
      throw amr_exception("Invalid IPv4 address: " + addr);
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
    close();
    throw amr_exception(std::string("connect() failed: ") + std::strerror(err));
  }

  /**
   * @brief Bind, listen, and accept a single client (server mode).
   * @param addr Bind address (e.g., "127.0.0.1" or "0.0.0.0").
   * @param port TCP port to bind.
   * @param backlog Listen backlog (default 1).
   * @return true on success.
   * @throws amr_exception on any socket error.
   *
   * This method blocks in accept() until a client connects, then sets the
   * accepted socket to non-blocking mode and closes the listening socket.
   */
  bool begin_listen(const std::string& addr, uint16_t port, int backlog = 1)
  {
    close();
    mode_ = Mode::Server;

    RCLCPP_INFO(node_->get_logger(), "Listening on %s:%u", addr.c_str(), port);

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0)
    {
      throw amr_exception(std::string("socket() failed: ") + std::strerror(errno));
    }

    int yes = 1;
    if (::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
    {
      int err = errno;
      close();
      throw amr_exception(std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(err));
    }

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    if (::inet_pton(AF_INET, addr.c_str(), &sa.sin_addr) != 1)
    {
      close();
      throw amr_exception("Invalid bind address: " + addr);
    }

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0)
    {
      int err = errno;
      close();
      throw amr_exception(std::string("bind() failed: ") + std::strerror(err));
    }

    if (::listen(listen_fd_, backlog) < 0)
    {
      int err = errno;
      close();
      throw amr_exception(std::string("listen() failed: ") + std::strerror(err));
    }

    sockaddr_in peer{};
    socklen_t plen = sizeof(peer);
    sock_fd_ = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &plen);
    if (sock_fd_ < 0)
    {
      int err = errno;
      close();
      throw amr_exception(std::string("accept() failed: ") + std::strerror(err));
    }

    // We only serve one client; close the listener
    amr::net::close_fd(listen_fd_);

    // Non-blocking on accepted socket
    amr::net::set_nonblocking(sock_fd_);

    connecting_ = false;

    char addrbuf[64]{};
    ::inet_ntop(AF_INET, &peer.sin_addr, addrbuf, sizeof(addrbuf));
    RCLCPP_INFO(node_->get_logger(), "Accepted client %s:%u", addrbuf, ntohs(peer.sin_port));
    return true;
  }

  /**
   * @brief Poll for readiness and process I/O once.
   * @param timeout_ms Timeout in milliseconds (-1 to block).
   * @param extra_events Additional events to poll for (default POLLIN|POLLOUT).
   * @return true if any events processed; false on timeout.
   * @throws amr_exception on poll or I/O error.
   */
  bool poll_once(int timeout_ms, short extra_events = (POLLIN | POLLOUT))
  {
    if (sock_fd_ < 0)
      return false;

    struct pollfd pfd{};
    pfd.fd = sock_fd_;
    pfd.events = POLLIN;
    if (!send_buf_.empty() || connecting_)
      pfd.events |= POLLOUT;
    pfd.events |= (extra_events & ~(POLLIN | POLLOUT));  // allow caller extras

    int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc < 0)
    {
      throw amr_exception(std::string("poll() failed: ") + std::strerror(errno));
    }
    if (rc == 0)
      return false;

    process_events(pfd.revents);
    return true;
  }

  /**
   * @brief Handle readiness flags from poll().
   * @param revents Returned event mask.
   * @throws amr_exception on socket error conditions.
   */
  void process_events(short revents)
  {
    if (revents & (POLLERR | POLLHUP | POLLNVAL))
    {
      int soerr = amr::net::get_so_error(sock_fd_);
      std::string msg = "socket error: " + std::string(std::strerror(soerr));
      close();
      throw amr_exception(msg);
    }

    if (connecting_ && (revents & POLLOUT))
    {
      int soerr = amr::net::get_so_error(sock_fd_);
      if (soerr != 0)
      {
        std::string msg = "connect completion error: " + std::string(std::strerror(soerr));
        close();
        throw amr_exception(msg);
      }
      connecting_ = false;
      RCLCPP_INFO(node_->get_logger(), "Connect completed");
    }

    if (revents & POLLIN)
      read_ready();
    if (revents & POLLOUT)
      write_ready();
  }

  /**
   * @brief Queue raw bytes for sending.
   * @param data Pointer to bytes.
   * @param len Number of bytes.
   */
  void send_bytes(const void* data, size_t len)
  {
    const char* p = static_cast<const char*>(data);
    send_buf_.append(p, p + len);
  }

  /**
   * @brief Queue a CRLF-terminated line for sending.
   * @param s Line content (CRLF appended).
   */
  void send_line(const std::string& s)
  {
    send_buf_.append(s);
    send_buf_.append("\r\n");
  }

  /**
   * @brief Extract complete CRLF-terminated lines from the receive buffer.
   * @param out_lines Appended with parsed lines (without CRLF).
   * @return number of lines extracted.
   */
  size_t read_lines(std::vector<std::string>& out_lines)
  {
    return amr::net::extract_crlf_lines(recv_buf_, out_lines);
  }

  /**
   * @brief Close the active (and listening) sockets and reset state.
   */
  void close()
  {
    amr::net::close_fd(sock_fd_);
    amr::net::close_fd(listen_fd_);
    connecting_ = false;
    send_buf_.clear();
    // keep any partial recv_buf_ for inspection if desired
    mode_ = Mode::None;
  }

  /**
   * @brief Check if the data socket is open.
   */
  bool is_open() const
  {
    return sock_fd_ >= 0;
  }

  /**
   * @brief Check if the client connection is still in progress.
   */
  bool is_connecting() const
  {
    return connecting_;
  }

  /**
   * @brief Current mode.
   */
  Mode mode() const
  {
    return mode_;
  }

private:
  void read_ready()
  {
    if (sock_fd_ < 0)
      return;
    bool peer_closed = amr::net::recv_append_nb(sock_fd_, recv_buf_);
    if (peer_closed)
    {
      RCLCPP_INFO(node_->get_logger(), "Peer closed connection");
      close();
    }
  }

  void write_ready()
  {
    if (sock_fd_ < 0 || send_buf_.empty() || connecting_)
      return;
    // send_erase_nb returns false when EAGAIN/EWOULDBLOCK; just try later
    (void)amr::net::send_erase_nb(sock_fd_, send_buf_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  Mode mode_{ Mode::None };
  int listen_fd_{ -1 };
  int sock_fd_{ -1 };
  bool connecting_{ false };

  std::string recv_buf_;
  std::string send_buf_;
};