#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "amr_core_cpp/utils/socket_driver.hpp"
#include "amr_core_cpp/amr_exception.hpp"

using namespace std::chrono_literals;

// Simple blocking server that replies with 2 lines + end marker once it sees "help"
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

  // read until "help" observed
  std::string req;
  char buf[1024];
  for (;;)
  {
    ssize_t n = ::recv(cfd, buf, sizeof(buf), 0);
    if (n > 0)
    {
      req.append(buf, buf + n);
      if (req.find("help\r\n") != std::string::npos)
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

  send_line("Help header");
  send_line("Usage: help");
  send_line("End of help");

  ::shutdown(cfd, SHUT_RDWR);
  ::close(cfd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_socket_driver");

  const uint16_t port = 12456;

  // launch server
  std::thread srv([&] {
    std::this_thread::sleep_for(100ms);
    help_server(port);
  });

  try
  {
    SocketDriver drv(node);
    drv.connect("127.0.0.1", port);

    // Send a CRLF-terminated command
    drv.send_line("help");

    std::vector<std::string> lines;
    bool done = false;
    auto start = std::chrono::steady_clock::now();

    while (!done)
    {
      drv.poll_once(100);
      lines.clear();
      drv.read_lines(lines);
      for (const auto& ln : lines)
      {
        RCLCPP_INFO(node->get_logger(), "Line: %s", ln.c_str());
        if (ln.find("End of help") != std::string::npos)
        {
          done = true;
        }
      }
      if (std::chrono::steady_clock::now() - start > 5s)
      {
        RCLCPP_ERROR(node->get_logger(), "Timeout waiting for response");
        break;
      }
    }
  }
  catch (const amr_exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "amr_exception: %s", ex.what());
    if (srv.joinable())
      srv.join();
    rclcpp::shutdown();
    return 2;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "std::exception: %s", ex.what());
    if (srv.joinable())
      srv.join();
    rclcpp::shutdown();
    return 3;
  }

  if (srv.joinable())
    srv.join();
  rclcpp::shutdown();
  return 0;
}