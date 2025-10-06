#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "amr_core_cpp/socket/socket_taskmaster.hpp"
#include "amr_core_cpp/utils/amr_exception.hpp"

using namespace std::chrono_literals;

static void help_server(uint16_t port)
{
  int sfd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sfd < 0)
    return;

  int yes = 1;
  ::setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(port);
  ::inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);

  if (::bind(sfd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0)
  {
    ::close(sfd);
    return;
  }
  if (::listen(sfd, 1) < 0)
  {
    ::close(sfd);
    return;
  }

  sockaddr_in peer{};
  socklen_t plen = sizeof(peer);
  int cfd = ::accept(sfd, reinterpret_cast<sockaddr*>(&peer), &plen);
  ::shutdown(sfd, SHUT_RDWR);
  ::close(sfd);
  if (cfd < 0)
    return;

  // Read until we observe "help"
  std::string req;
  char buf[1024];
  for (;;)
  {
    ssize_t n = ::recv(cfd, buf, sizeof(buf), 0);
    if (n > 0)
    {
      req.append(buf, buf + n);
      if (req.find("help") != std::string::npos)
        break;
    }
    else if (n == 0)
    {
      break;
    }
    else
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        continue;
      break;
    }
  }

  auto send_line = [&](const std::string& s) {
    std::string line = s + "\r\n";
    ::send(cfd, line.data(), line.size(), 0);
  };

  // Feedback + end marker
  send_line("Help header");
  send_line("Usage: help");
  send_line("End of help");

  ::shutdown(cfd, SHUT_RDWR);
  ::close(cfd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_socket_taskmaster");

  const std::string addr = "127.0.0.1";
  const uint16_t port = 12345;

  // Start local server
  std::thread server_thr([&] { help_server(port); });
  std::this_thread::sleep_for(100ms);

  try
  {
    SocketTaskmaster stm(node);
    if (!stm.connect(addr, static_cast<int>(port)))
    {
      RCLCPP_ERROR(node->get_logger(), "connect returned false");
      if (server_thr.joinable())
        server_thr.join();
      rclcpp::shutdown();
      return 1;
    }

    stm.push_command("help", /*newline=*/true, { "End of help" });

    SocketTaskmaster::WaitResult wr{ false, "", "" };
    // Poll until complete or timeout window
    auto start = std::chrono::steady_clock::now();
    while (!(wr = stm.wait_command(100)).complete)
    {
      if (std::chrono::steady_clock::now() - start > 5s)
      {
        RCLCPP_ERROR(node->get_logger(), "Timeout waiting for response");
        break;
      }
    }

    if (wr.complete)
    {
      RCLCPP_INFO(node->get_logger(), "Result: %s", wr.result.c_str());
      RCLCPP_INFO(node->get_logger(), "Feedback:\n%s", wr.feedback.c_str());
    }
  }
  catch (const amr_exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "amr_exception: %s", ex.what());
    if (server_thr.joinable())
      server_thr.join();
    rclcpp::shutdown();
    return 2;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "std::exception: %s", ex.what());
    if (server_thr.joinable())
      server_thr.join();
    rclcpp::shutdown();
    return 3;
  }

  if (server_thr.joinable())
    server_thr.join();
  rclcpp::shutdown();
  return 0;
}
