#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <string>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "amr_core_cpp/utils/connect_socket.hpp"
#include "amr_core_cpp/amr_exception.hpp"

using namespace std::chrono_literals;

// Minimal ARCL-like server:
// 1) Send "Enter password:" prompt
// 2) Receive a line (password)
// 3) Send welcome + "End of commands"
static void password_server(uint16_t port)
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

  auto send_line = [&](const std::string& s) {
    std::string line = s + "\r\n";
    ::send(cfd, line.data(), line.size(), 0);
  };

  // Prompt
  send_line("Enter password:");

  // Read one line (password)
  std::string req;
  char buf[1024];
  for (;;)
  {
    ssize_t n = ::recv(cfd, buf, sizeof(buf), 0);
    if (n > 0)
    {
      req.append(buf, buf + n);
      if (req.find("\r\n") != std::string::npos)
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

  // Welcome + end
  send_line("Welcome to the server.");
  send_line("End of commands");

  ::shutdown(cfd, SHUT_RDWR);
  ::close(cfd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_connect_socket");

  // Set parameters for the wrapper
  node->declare_parameter<std::string>("ip_address", "127.0.0.1");
  node->declare_parameter<int>("port", 12555);
  node->declare_parameter<std::string>("def_arcl_passwd", "secret");

  const uint16_t port = static_cast<uint16_t>(node->get_parameter("port").as_int());

  // Launch local server
  std::thread srv([&] {
    std::this_thread::sleep_for(100ms);
    password_server(port);
  });

  try
  {
    ConnectSocket cs(node);
    cs.connect_and_login();  // will wait for "End of commands"

    // Optionally, send a follow-up command using the underlying taskmaster:
    // cs.taskmaster().push_command("help", true, {"End of help"});
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