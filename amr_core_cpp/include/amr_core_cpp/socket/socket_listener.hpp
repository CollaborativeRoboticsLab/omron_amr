#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <cstring>
#include <thread>
#include <chrono>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

#include "amr_core_cpp/amr_exception.hpp"
#include "amr_core_cpp/utils/socket_common.hpp"

class SocketListener
{
public:
  static constexpr size_t RECV_BUFFER = 4096;

  /**
   * @brief Construct a new Socket Listener object
   * @param node Shared pointer to the ROS2 node for logging
   * @param addr Address to bind the socket to (e.g., "0.0.0.0")
   * @param port Port number to listen on
   */
  explicit SocketListener(const rclcpp::Node::SharedPtr& node, const std::string& addr, int port)
    : node_(node), addr_(addr), port_(port)
  {
  }

  // Destructor closes any open sockets
  ~SocketListener()
  {
    close();
  }

  /**
   * @brief Begin listening for incoming socket connections
   * @return true if successfully listening and accepted a connection
   * @throws amr_exception on socket errors
   *
   * This method creates a socket, sets the SO_REUSEADDR option, and binds to the specified address and port. If
   * the bind fails, it retries every 3 seconds until successful. Once bound, it listens for incoming connections
   * and accepts a single client connection. The accepted socket is set to non-blocking mode. If any socket operation
   * fails, an amr_exception is thrown with an appropriate error message.
   */
  bool begin()
  {
    RCLCPP_INFO(node_->get_logger(), "Attempt to listen for incoming on %s at port %d", addr_.c_str(), port_);

    // Create, set SO_REUSEADDR, and bind (retry bind on failure)
    for (;;)
    {
      listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
      if (listen_fd_ < 0)
      {
        throw amr_exception(std::string("socket() failed: ") + std::strerror(errno));
      }

      int yes = 1;
      if (::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
      {
        amr::net::close_fd(listen_fd_);
        throw amr_exception(std::string("setsockopt(SO_REUSEADDR) failed: ") + std::strerror(errno));
      }

      sockaddr_in sa{};
      sa.sin_family = AF_INET;
      sa.sin_port = htons(static_cast<uint16_t>(port_));
      if (::inet_pton(AF_INET, addr_.c_str(), &sa.sin_addr) != 1)
      {
        amr::net::close_fd(listen_fd_);
        throw amr_exception("Invalid listen address: " + addr_);
      }

      if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0)
      {
        RCLCPP_INFO(node_->get_logger(), "Socket listener connection failed: %s", std::strerror(errno));
        RCLCPP_INFO(node_->get_logger(), "Retrying");
        amr::net::close_fd(listen_fd_);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        continue;
      }
      break;
    }

    if (::listen(listen_fd_, 1) < 0)
    {
      amr::net::close_fd(listen_fd_);
      throw amr_exception(std::string("listen() failed: ") + std::strerror(errno));
    }

    sockaddr_in peer{};
    socklen_t plen = sizeof(peer);
    conn_fd_ = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &plen);
    if (conn_fd_ < 0)
    {
      amr::net::close_fd(listen_fd_);
      throw amr_exception(std::string("accept() failed: ") + std::strerror(errno));
    }

    // Close the listening socket (single-connection behavior to match Python)
    amr::net::close_fd(listen_fd_);

    char addrbuf[64]{};
    ::inet_ntop(AF_INET, &peer.sin_addr, addrbuf, sizeof(addrbuf));
    RCLCPP_INFO(node_->get_logger(), "Listening to new socket on %s at port %d", addrbuf,
                static_cast<int>(ntohs(peer.sin_port)));

    // Make the accepted socket non-blocking
    amr::net::set_nonblocking(conn_fd_);

    return true;
  }

  /**
   * @brief Poll the socket for incoming data once
   * @param timeout_ms Timeout in milliseconds for the poll operation
   * @return true if data was received and processed, false if timeout occurred
   * @throws amr_exception on poll errors
   *
   * This method uses the poll() system call to check for incoming data on the connected socket. If data is
   * available, it reads the data and processes it. If the poll operation times out without any events, it
   * returns false. If an error occurs during the poll operation, an amr_exception is thrown with an appropriate
   * error message.
   */
  bool poll_once(int timeout_ms)
  {
    if (conn_fd_ < 0)
      return false;

    struct pollfd pfd{};
    pfd.fd = conn_fd_;
    pfd.events = POLLIN;

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
   * @brief Process events indicated by poll()
   * @param revents The event flags returned by poll()
   */
  void process_events(short revents)
  {
    if ((revents & POLLIN) != 0)
    {
      _read();
    }
    sort_data();
  }

  /**
   * @brief Close any open sockets and clean up resources
   *
   * This method shuts down and closes both the connection and listening sockets if they are open. It also
   * resets their file descriptors to -1.
   */
  void close()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    amr::net::close_fd(conn_fd_);
    amr::net::close_fd(listen_fd_);
  }

  /**
   * @brief Retrieve stored responses for a given key
   * @param key The key for which to retrieve responses
   * @return A reference to a vector of response strings associated with the key
   * @throws std::out_of_range if the key does not exist or has no associated responses
   *
   * This method looks up the provided key in the internal response storage and returns a reference to the
   * vector of response strings associated with that key. If the key is not found or if there are no responses
   * stored for that key, it throws a std::out_of_range exception.
   */
  const std::vector<std::string>& get_response(const std::string& key) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = responses_.find(key);
    if (it == responses_.end() || it->second.empty())
    {
      throw std::out_of_range("No response for key: " + key);
    }
    return it->second;
  }

  /**
   * @brief Public method to read data from the socket
   */
  void read()
  {
    _read();
  }

private:
  /**
   * @brief Read incoming data from the socket and append to the receive buffer
   * @throws amr_exception on recv errors
   *
   * Uses amr::net::recv_append_nb to fetch bytes into a temporary buffer, then
   * appends to the guarded recv_buffer_. Closes on peer shutdown.
   */
  void _read()
  {
    if (conn_fd_ < 0)
      return;

    std::string tmp;
    bool peer_closed = amr::net::recv_append_nb(conn_fd_, tmp);
    if (!tmp.empty())
    {
      std::lock_guard<std::mutex> lk(mtx_);
      recv_buffer_.append(tmp);
    }
    if (peer_closed)
    {
      close();
    }
  }

  /**
   * @brief Trim leading and trailing whitespace from a string
   * @param s The input string to trim
   * @return A new string with leading and trailing whitespace removed
   */
  static std::string trim_copy(const std::string& s)
  {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
      return {};
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
  }

  /**
   * @brief Parse complete lines from the receive buffer and store key-value pairs
   *
   * Extracts complete CRLF lines using amr::net::extract_crlf_lines while
   * holding the buffer lock, then parses and stores them.
   */
  void sort_data()
  {
    for (;;)
    {
      std::vector<std::string> lines;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (amr::net::extract_crlf_lines(recv_buffer_, lines) == 0)
        {
          return;
        }
      }
      for (const auto& line : lines)
      {
        // Parse "Key: value" or bare key
        std::string key, value;
        auto colon = line.find(':');
        if (colon == std::string::npos)
        {
          key = line;
          store(key, std::string{});
        }
        else
        {
          key = line.substr(0, colon);
          value = trim_copy(line.substr(colon + 1));
          store(key, value);
        }
      }
    }
  }

  /**
   * @brief Store a key-value pair in the response storage with special handling for certain keys
   * @param key The key to store
   * @param value The value associated with the key
   */
  void store(const std::string& key, const std::string& value)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    if (key == "Goal")
    {
      if (goal_f_)
      {
        responses_[key].push_back(value);
      }
      else
      {
        responses_[key] = { value };
        goal_f_ = true;
      }
    }
    else if (key == "End of goals")
    {
      goal_f_ = false;
    }
    else if (key == "ApplicationFaultQuery")
    {
      if (app_fault_f_)
      {
        responses_[key].push_back(value);
      }
      else
      {
        responses_[key] = { value };
        app_fault_f_ = true;
      }
    }
    else if (key == "End of ApplicationFaultQuery")
    {
      app_fault_f_ = false;
    }
    else if (key == "FaultList")
    {
      if (faults_get_f_)
      {
        responses_[key].push_back(value);
      }
      else
      {
        responses_[key] = { value };
        faults_get_f_ = true;
      }
    }
    else if (key == "End of FaultList")
    {
      faults_get_f_ = false;
    }
    else if (key == "RobotFaultQuery")
    {
      if (query_faults_f_)
      {
        responses_[key].push_back(value);
      }
      else
      {
        responses_[key] = { value };
        query_faults_f_ = true;
      }
    }
    else if (key == "EndQueryFaults")
    {
      query_faults_f_ = false;
    }
    else
    {
      responses_[key] = { value };
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  int listen_fd_{ -1 };
  int conn_fd_{ -1 };
  std::string addr_;
  int port_{ 0 };

  mutable std::mutex mtx_;
  std::string recv_buffer_;
  std::unordered_map<std::string, std::vector<std::string>> responses_;

  bool goal_f_{ false };
  bool app_fault_f_{ false };
  bool faults_get_f_{ false };
  bool query_faults_f_{ false };
};