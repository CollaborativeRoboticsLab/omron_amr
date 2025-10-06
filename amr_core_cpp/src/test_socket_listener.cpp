#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <chrono>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "amr_core_cpp/socket/socket_listener.hpp"
#include "amr_core_cpp/utils/amr_exception.hpp"

using namespace std::chrono_literals;

static void client_sender(uint16_t port)
{
  std::this_thread::sleep_for(200ms);  // let server bind/listen

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    std::perror("client socket");
    return;
  }

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(port);
  ::inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);

  if (::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0)
  {
    std::perror("client connect");
    ::close(fd);
    return;
  }

  auto send_line = [&](const std::string& s) {
    std::string line = s + "\r\n";
    ::send(fd, line.data(), line.size(), 0);
  };

  // Send sample data blocks
  send_line("Goal: A");
  send_line("Goal: B");
  send_line("End of goals");

  send_line("ApplicationFaultQuery: AppFault1");
  send_line("End of ApplicationFaultQuery");

  send_line("FaultList: Fault1");
  send_line("End of FaultList");

  send_line("RobotFaultQuery: RF1");
  send_line("EndQueryFaults");

  ::shutdown(fd, SHUT_RDWR);
  ::close(fd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_socket_listener");

  const std::string addr = "127.0.0.1";
  const uint16_t port = 12345;

  try
  {
    SocketListener listener(node, addr, static_cast<int>(port));

    // Start client in background
    std::thread client([&] { client_sender(port); });

    // Begin server (bind, listen, accept one client)
    if (!listener.begin())
    {
      std::cerr << "Begin failed" << std::endl;
      if (client.joinable())
        client.join();
      rclcpp::shutdown();
      return 1;
    }

    // Poll until client closes (poll_once returns false when conn_fd_ is closed)
    while (listener.poll_once(100))
    {
      // parsing happens inside
    }

    if (client.joinable())
      client.join();

    // Verify parsed results
    auto print_vec = [](const std::string& key, const std::vector<std::string>& v) {
      std::cout << key << " (" << v.size() << "):" << std::endl;
      for (auto& s : v)
        std::cout << "  - " << s << std::endl;
    };

    try
    {
      print_vec("Goal", listener.get_response("Goal"));
    }
    catch (...)
    {
      std::cout << "No Goal entries" << std::endl;
    }

    try
    {
      print_vec("ApplicationFaultQuery", listener.get_response("ApplicationFaultQuery"));
    }
    catch (...)
    {
      std::cout << "No ApplicationFaultQuery entries" << std::endl;
    }

    try
    {
      print_vec("FaultList", listener.get_response("FaultList"));
    }
    catch (...)
    {
      std::cout << "No FaultList entries" << std::endl;
    }

    try
    {
      print_vec("RobotFaultQuery", listener.get_response("RobotFaultQuery"));
    }
    catch (...)
    {
      std::cout << "No RobotFaultQuery entries" << std::endl;
    }
  }
  catch (const amr_exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "amr_exception: %s", ex.what());
    rclcpp::shutdown();
    return 2;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "std::exception: %s", ex.what());
    rclcpp::shutdown();
    return 3;
  }

  rclcpp::shutdown();
  return 0;
}