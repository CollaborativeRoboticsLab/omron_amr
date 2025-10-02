#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "amr_msgs/srv/arcl_api.hpp"
#include "amr_core_cpp/utils/socket_taskmaster.hpp"
#include "amr_core_cpp/amr_exception.hpp"

/**
 * @brief Header-only ARCL API Service node.
 *
 * Parameters:
 *  - ip_address (string, default "127.0.0.1")
 *  - port (int, default 0)
 *  - def_arcl_passwd (string, default "")
 *  - service_timeout_ms (int, default 15000)
 *
 * Service:
 *  - name: "arcl_api_service" (amr_msgs/srv/ArclApi)
 *    Request: command (string), line_identifier (string[])
 *    Response: response (string)
 *
 * On construction:
 *  - Creates SocketTaskmaster, connects to ip:port, performs login with def_arcl_passwd.
 *  - Advertises the service and handles requests synchronously by issuing the command,
 *    then polling until any of the provided line_identifier substrings is matched
 *    in a received line (or until timeout).
 */
class ArclApiService : public rclcpp::Node
{
public:
  using ServiceT = amr_msgs::srv::ArclApi;

  ArclApiService() : rclcpp::Node("arcl_api_server")
  {
    // Parameters
    ip_ = this->declare_parameter<std::string>("ip_address", "127.0.0.1");
    port_ = this->declare_parameter<int>("port", 0);
    passwd_ = this->declare_parameter<std::string>("def_arcl_passwd", "");
    timeout_ms_ = this->declare_parameter<int>("service_timeout_ms", 15000);

    // Socket session
    tm_ = std::make_shared<SocketTaskmaster>(this->shared_from_this());
    try
    {
      (void)tm_->connect(ip_, port_);
      tm_->login(passwd_);
      RCLCPP_INFO(this->get_logger(), "ARCL API Service connected and logged in to %s:%d", ip_.c_str(), port_);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_FATAL(this->get_logger(), "SocketTaskmaster init failed: %s", ex.what());
      throw;
    }

    // Service
    srv_ = this->create_service<ServiceT>("arcl_api_service",
                                          std::bind(&ArclApiService::handle_request, this, std::placeholders::_1,
                                                    std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "ARCL API Service initialised!");
  }

private:
  void handle_request(const std::shared_ptr<rmw_request_id_t>&, const std::shared_ptr<ServiceT::Request> req,
                      std::shared_ptr<ServiceT::Response> resp)
  {
    // Convert identifiers
    std::vector<std::string> end_lines(req->line_identifier.begin(), req->line_identifier.end());

    // Queue command (append CRLF if needed)
    try
    {
      tm_->push_command(req->command, /*newline=*/true, end_lines);
    }
    catch (const amr_exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "push_command failed: %s", ex.what());
      resp->response = std::string("ERROR: ") + ex.what();
      return;
    }

    // Wait for completion or timeout
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
      try
      {
        auto wr = tm_->wait_command(100);
        if (wr.complete)
        {
          resp->response = wr.result;
          return;
        }
      }
      catch (const amr_exception& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "wait_command error: %s", ex.what());
        resp->response = std::string("ERROR: ") + ex.what();
        return;
      }

      if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(timeout_ms_))
      {
        RCLCPP_WARN(this->get_logger(), "Service request timeout after %d ms", timeout_ms_);
        resp->response = "TIMEOUT";
        return;
      }
    }

    resp->response = "SHUTDOWN";
  }

private:
  // Parameters
  std::string ip_;
  int port_{ 0 };
  std::string passwd_;
  int timeout_ms_{ 15000 };

  // Socket manager
  std::shared_ptr<SocketTaskmaster> tm_;

  // Service
  rclcpp::Service<ServiceT>::SharedPtr srv_;
};