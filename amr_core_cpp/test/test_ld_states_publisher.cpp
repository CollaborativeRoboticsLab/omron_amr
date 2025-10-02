#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std::chrono_literals;

// Minimal client that connects and sends ARCL-like status lines
static void sender_client(uint16_t port)
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    return;

  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons(port);
  ::inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);

  if (::connect(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0)
  {
    ::close(fd);
    return;
  }

  auto send_line = [&](const std::string& s) {
    std::string line = s + "\r\n";
    ::send(fd, line.data(), line.size(), 0);
  };

  send_line("Status: Operational");
  send_line("ExtendedStatusForHumans: All good");
  send_line("StateOfCharge: 94.5");
  send_line("LocalizationScore: 0.99");
  send_line("Temperature: 32.1");
  send_line("Location: 1.0 2.0 3.14");

  send_line("RangeDeviceGetCurrent: LASER_DATA");
  send_line("Goal: A");
  send_line("Goal: B");
  send_line("End of goals");
  send_line("Odometer: 12.3 4.5 6.7");
  send_line("ApplicationFaultQuery: None");
  send_line("End of ApplicationFaultQuery");
  send_line("FaultList: F1");
  send_line("End of FaultList");
  send_line("RobotFaultQuery: RF1");
  send_line("EndQueryFaults");

  ::shutdown(fd, SHUT_RDWR);
  ::close(fd);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Configure parameters expected by ld_states_publisher node
  auto param_node = rclcpp::Node::make_shared("ld_param_node");
  param_node->declare_parameter<std::string>("local_ip", "127.0.0.1");
  param_node->declare_parameter<int>("local_port", 15123);
  const uint16_t port = static_cast<uint16_t>(param_node->get_parameter("local_port").as_int());

  // Launch the publisher node executable (built separately) in another terminal in practice.
  // For quick manual test: start your ld_states_publisher executable, then run this test to feed data.

  // Start client after a short delay to allow listener to bind+accept
  std::thread cli([&] {
    std::this_thread::sleep_for(300ms);
    sender_client(port);
  });

  // Keep the test running briefly so the publisher can receive and publish messages
  std::this_thread::sleep_for(2s);

  if (cli.joinable())
    cli.join();
  rclcpp::shutdown();
  return 0;
}