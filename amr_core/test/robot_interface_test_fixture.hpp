#pragma once

#include <gtest/gtest.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav2_velocity_smoother/velocity_smoother.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#include <nav2_msgs/action/undock_robot.hpp>
#include <rclcpp/parameter_client.hpp>
#include <limits>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amr_core/interfaces/drive_interface.hpp"
#include "amr_core/interfaces/configuration_interface.hpp"
#include "amr_core/interfaces/map_interface.hpp"
#include "amr_core/utils/robot_session.hpp"

using namespace std::chrono_literals;

class DriverInterfaceLoopbackTest : public ::testing::Test
{
protected:
  struct Record
  {
    bool stop;
    double translation;
    double rotation;
    double throttle;
    double lateral;
    std::chrono::steady_clock::time_point received_at{ std::chrono::steady_clock::now() };
  };

  struct Telemetry
  {
    std::int32_t x_mm{ 0 };
    std::int32_t y_mm{ 0 };
    std::int16_t heading_deg{ 90 };
    std::int16_t forward_mm_s{ 1000 };
    std::int16_t rotation_deg_s{ 0 };
    std::int16_t lateral_mm_s{ 0 };
  };

  DriverInterfaceLoopbackTest()
    : server_(false, "drive-loopback", false, "unused", "unused", false, false, false, false, false)
    , drive_callback_(this, &DriverInterfaceLoopbackTest::recordDrive)
    , stop_callback_(this, &DriverInterfaceLoopbackTest::recordStop)
    , mode_callback_(this, &DriverInterfaceLoopbackTest::recordMode)
    , query_mode_callback_(this, &DriverInterfaceLoopbackTest::sendMode)
    , odom_callback_(this, &DriverInterfaceLoopbackTest::sendOdometry)
    , docking_callback_(this, &DriverInterfaceLoopbackTest::sendDocking)
    , undock_callback_(this, &DriverInterfaceLoopbackTest::recordUndock)
    , shutdown_callback_(this, &DriverInterfaceLoopbackTest::recordShutdown)
    , fault_ack_callback_(this, &DriverInterfaceLoopbackTest::recordFaultAcknowledgement)
    , faults_callback_(this, &DriverInterfaceLoopbackTest::sendFaults)
    , map_name_callback_(this, &DriverInterfaceLoopbackTest::sendMapName)
    , map_callback_(this, &DriverInterfaceLoopbackTest::sendMap)
    , map_upload_callback_(this, &DriverInterfaceLoopbackTest::receiveMapUpload)
    , map_download_callback_(this, &DriverInterfaceLoopbackTest::sendUploadedMap)
  {
  }

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    runtime_ = std::make_unique<amr_core::LibAriaRuntime>();
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
    runtime_.reset();
  }

  void SetUp() override
  {
    ASSERT_TRUE(server_.addData("ratioDrive", "record ratios", &drive_callback_, "four doubles", "none"));
    ASSERT_TRUE(server_.addData("stop", "record stop", &stop_callback_, "none", "none"));
    ASSERT_TRUE(server_.addData("setSafeDrive", "record mode", &mode_callback_, "byte", "none"));
    ASSERT_TRUE(server_.addData("getSafeDrive", "query mode", &query_mode_callback_, "none", "byte"));
    ASSERT_TRUE(server_.addData("updateNumbers", "fake body velocities", &odom_callback_, "none", "numbers"));
    bool opened = false;
    for (unsigned int port = 24000; port < 24100 && !opened; ++port)
    {
      opened = server_.open(port, "127.0.0.1");
    }
    ASSERT_TRUE(opened);
    server_.runAsync();
    server_started_ = true;

    rclcpp::NodeOptions options;
    options.parameter_overrides({ { "driver.cmd_vel.topic", "/drive_test/cmd_vel" },
                                  { "driver.odom.topic", "/drive_test/odom" },
                                  { "driver.stop.topic", "/drive_test/stop" },
                                  { "driver.tf.enabled", false },
                                  { "driver.drive.send_frequency", 20.0 },
                                  { "driver.cmd_vel.timeout_sec", 0.5 },
                                  { "driver.drive.mode_check_period_sec", 0.05 },
                                  { "driver.drive.mode_confirmation_timeout_sec", 0.3 },
                                  { "driver.drive.unsafe_drive", true } });
    node_ = std::make_shared<rclcpp::Node>("drive_interface_test", options);
    docking_ = std::make_shared<amr_core::DockingInterface>(
        [this]() {
          ++undock_requests_;
          return true;
        },
        60.0);
    docking_->update(amr_core::DockingInterface::Feedback::Undocked);
    driver_ = std::make_unique<DriverInterface>(node_, docking_);
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/drive_test/cmd_vel", 1);
    executor_.add_node(node_);
    executor_added_ = true;
  }

  void initializeDriver()
  {
    driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "");
    ASSERT_TRUE(waitWithExecutor([this]() { return publisher_->get_subscription_count() > 0; }));
  }

  void initializeDockingBackend()
  {
    ASSERT_TRUE(server_.addData("dockInfo", "docking state", &docking_callback_, "none", "four bytes"));
    ASSERT_TRUE(server_.addData("undock", "departure", &undock_callback_, "none", "none"));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      telemetry_.forward_mm_s = 0;
    }
    driver_ = std::make_unique<DriverInterface>(node_);
    initializeDriver();
    docking_ = driver_->dockingInterface();
    ASSERT_TRUE(waitWithExecutor(
        [this]() { return docking_->readiness().state == amr_core::DockingInterface::State::Docked; }));
    const auto started = std::chrono::steady_clock::now();
    ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 600ms; }));
  }

  void sendDocking(ArServerClient* client, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (suppress_docking_replies_)
      return;
    ArNetPacket packet;
    packet.uByteToBuf(fake_docking_state_);
    packet.uByteToBuf(fake_docking_forced_);
    packet.uByte2ToBuf(fake_docking_shutdown_countdown_);
    client->sendPacketTcp(&packet);
  }

  void recordUndock(ArServerClient*, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++undock_requests_;
    fake_docking_state_ = 0;
    suppress_docking_replies_ = suppress_docking_after_undock_;
    changed_.notify_all();
  }

  void recordShutdown(ArServerClient*, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++shutdown_requests_;
    changed_.notify_all();
  }

  void enableShutdownBackend()
  {
    ASSERT_TRUE(server_.addData("shutdownRobot", "record shutdown", &shutdown_callback_, "none", "none"));
  }

  void recordFaultAcknowledgement(ArServerClient*, ArNetPacket* packet)
  {
    const auto acknowledge_all = packet->bufToUByte();
    const auto name = packet->bufToString();
    std::lock_guard<std::mutex> lock(mutex_);
    fault_ack_requests_.emplace_back(acknowledge_all, name);
    changed_.notify_all();
  }

  void enableFaultAcknowledgementBackend()
  {
    ASSERT_TRUE(server_.addData("acknowledgeFault", "record fault acknowledgement", &fault_ack_callback_, "", "none"));
  }

  void sendFaults(ArServerClient* client, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++fault_queries_;
    ArNetPacket packet;
    packet.uByte2ToBuf(static_cast<unsigned short>(fake_faults_.size()));
    for (const auto& name : fake_faults_)
    {
      packet.strToBuf("Fault_ClearedOnAcknowledgement");
      packet.strToBuf(name.c_str());
      packet.strToBuf("description");
      packet.strToBuf("details");
    }
    if (fault_trailing_bytes_)
      packet.uByteToBuf(1);
    client->sendPacketTcp(&packet);
    changed_.notify_all();
  }

  void enableAutomaticFaultAcknowledgement()
  {
    enableFaultAcknowledgementBackend();
    ASSERT_TRUE(server_.addData("getFaults", "query faults", &faults_callback_, "none", "faults"));
    node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
    node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
    node_->declare_parameter<bool>("fault.monitor.auto_acknowledge_enabled", true);
    node_->declare_parameter<std::vector<std::string>>("fault.auto_acknowledge_fault_list", { "IrregularShutdown" });
  }

  void sendMapName(ArServerClient* client, ArNetPacket*)
  {
    ArNetPacket packet;
    packet.strToBuf("fixture.map");
    client->sendPacketTcp(&packet);
  }

  void sendMap(ArServerClient* client, ArNetPacket*)
  {
    for (const auto* line : { "2D-Map", "DATA", "" })
    {
      ArNetPacket packet;
      packet.strToBuf(line);
      client->sendPacketTcp(&packet);
    }
  }

  void enableMapBackend()
  {
    server_.enforceProtocolVersion("6MTX");
    ASSERT_TRUE(server_.addData("getMapName", "active map", &map_name_callback_, "none", "string"));
    ASSERT_TRUE(server_.addData("getMap", "map stream", &map_callback_, "none", "lines"));
  }

  void receiveMapUpload(ArServerClient* client, ArNetPacket* packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto command = packet->bufToUByte2();
    const auto name = packet->bufToString();
    if (command == 0) uploaded_map_name_ = name;
    if (command == 1)
    {
      const auto size = packet->bufToUByte4();
      std::vector<char> data(size);
      packet->bufToData(data.data(), size);
      uploaded_map_.append(data.data(), size);
    }
    ArNetPacket response;
    response.uByte2ToBuf(command == 2 ? 0 : 10);
    response.strToBuf(name.c_str());
    client->sendPacketTcp(&response);
  }

  void sendUploadedMap(ArServerClient* client, ArNetPacket* packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto name = packet->bufToString();
    for (std::size_t offset = 0; offset < uploaded_map_.size();)
    {
      const auto size = std::min<std::size_t>(16000, uploaded_map_.size() - offset);
      ArNetPacket response;
      response.uByte2ToBuf(0);
      response.strToBuf(name.c_str());
      response.uByte4ToBuf(size);
      if (corrupt_map_readback_ && offset == 0)
      {
        auto corrupted = uploaded_map_.substr(offset, size);
        corrupted[0] ^= 1;
        response.dataToBuf(corrupted.data(), size);
      }
      else
        response.dataToBuf(uploaded_map_.data() + offset, size);
      client->sendPacketTcp(&response);
      offset += size;
    }
    ArNetPacket response;
    response.uByte2ToBuf(0);
    response.strToBuf(name.c_str());
    response.uByte4ToBuf(0);
    client->sendPacketTcp(&response);
  }

  void configureCalibration(bool confirmed = true)
  {
    node_->declare_parameter<std::string>("driver.drive.ratio_mapping", "aria_ratio_v1");
    node_->declare_parameter<bool>("driver.calibration.confirmed", confirmed);
    node_->declare_parameter<std::string>("driver.calibration.source", "loopback:test-only");
    node_->declare_parameter<double>("driver.calibration.full_throttle_forward_mm_s", 2000.0);
    node_->declare_parameter<double>("driver.calibration.full_throttle_reverse_mm_s", 500.0);
    node_->declare_parameter<double>("driver.calibration.rotation_stopped_deg_s", 60.0);
    node_->declare_parameter<double>("driver.calibration.rotation_full_forward_deg_s", 30.0);
    node_->declare_parameter<double>("driver.calibration.rotation_full_reverse_deg_s", 20.0);
    node_->declare_parameter<double>("driver.calibration.translation_dead_zone_pct", 0.0);
    node_->declare_parameter<double>("driver.calibration.rotation_dead_zone_pct", 0.0);
    node_->declare_parameter<double>("driver.calibration.feedback_timeout_sec", 0.1);
  }

  void TearDown() override
  {
    if (core_process_ > 0)
    {
      kill(core_process_, SIGINT);
      if (!waitWithExecutor([this]() { return waitpid(core_process_, nullptr, WNOHANG) == core_process_; }))
      {
        kill(core_process_, SIGKILL);
        waitpid(core_process_, nullptr, 0);
      }
      core_process_ = -1;
    }
    if (smoother_)
    {
      if (smoother_->get_current_state().label() == "active")
      {
        smoother_->deactivate();
      }
      if (smoother_->get_current_state().label() == "inactive")
      {
        smoother_->cleanup();
      }
      executor_.remove_node(smoother_->get_node_base_interface());
      smoother_.reset();
    }
    driver_.reset();
    if (executor_added_)
    {
      executor_.remove_node(node_);
    }
    publisher_.reset();
    node_.reset();
    if (server_started_)
    {
      server_.stopRunning();
      server_.join();
      server_.close();
    }
  }

  bool waitWithExecutor(const std::function<bool()>& ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      if (ready())
      {
        return true;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait_for(lock, 5ms);
    }
    return ready();
  }

  void publish(double linear, double angular)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    publisher_->publish(command);
  }

  bool receivedDrive(double translation, double rotation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& record : records_)
    {
      if (!record.stop && std::fabs(record.translation - translation) < 1e-3 &&
          std::fabs(record.rotation - rotation) < 1e-3)
      {
        return true;
      }
    }
    return false;
  }

  void checkVelocityStream(double linear, double angular)
  {
    node_->declare_parameter<double>("driver.cmd_vel.timeout_sec", 0.2);
    node_->set_parameter(rclcpp::Parameter("driver.cmd_vel.timeout_sec", 0.2));
    node_->declare_parameter<double>("driver.cmd_vel.min_linear", -200.0);
    auto stop_publisher = node_->create_publisher<std_msgs::msg::Empty>("/drive_test/stop", 10);
    initializeDriver();
    ASSERT_TRUE(waitWithExecutor([&stop_publisher]() { return stop_publisher->get_subscription_count() > 0; }));
    const auto started = std::chrono::steady_clock::now();
    auto next_publish = started;
    ASSERT_TRUE(waitWithExecutor([this, linear, angular, started, &next_publish]() {
      const auto now = std::chrono::steady_clock::now();
      if (now - started >= 500ms)
        return true;
      if (now >= next_publish)
      {
        publish(linear, angular);
        next_publish = now + 50ms;
      }
      return false;
    }));
    stop_publisher->publish(std_msgs::msg::Empty{});
    ASSERT_TRUE(waitWithExecutor([this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      return std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
    }));
    const auto expected_translation = linear * 100.0;
    const auto expected_rotation = angular * 180.0 / M_PI / 60.0 * 100.0;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stop = std::find_if(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
    ASSERT_NE(stop, records_.end());
    EXPECT_GE(std::count_if(records_.begin(), stop,
                            [expected_translation, expected_rotation](const Record& record) {
                              return !record.stop && std::fabs(record.translation - expected_translation) < 1e-3 &&
                                     std::fabs(record.rotation - expected_rotation) < 1e-3;
                            }),
              3);
  }

  void initializeSmootherChain()
  {
    node_->declare_parameter<double>("driver.cmd_vel.timeout_sec", 0.2);
    node_->set_parameter(rclcpp::Parameter("driver.cmd_vel.timeout_sec", 0.2));
    initializeDriver();
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__ns:=/drive_chain_test", "-r", "cmd_vel:=/drive_chain_test/cmd_vel_nav",
                        "-r", "cmd_vel_smoothed:=/drive_test/cmd_vel" });
    options.parameter_overrides({ { "use_sim_time", false },
                                  { "enable_stamped_cmd_vel", false },
                                  { "feedback", "OPEN_LOOP" },
                                  { "smoothing_frequency", 20.0 },
                                  { "velocity_timeout", 1.0 },
                                  { "max_velocity", std::vector<double>{ 1.0, 0.0, 1.0 } },
                                  { "min_velocity", std::vector<double>{ -0.1, 0.0, -1.0 } },
                                  { "max_accel", std::vector<double>{ 2.5, 0.0, 3.2 } },
                                  { "max_decel", std::vector<double>{ -2.5, 0.0, -3.2 } } });
    smoother_ = std::make_shared<nav2_velocity_smoother::VelocitySmoother>(options);
    executor_.add_node(smoother_->get_node_base_interface());
    ASSERT_EQ(smoother_->configure().label(), "inactive");
    ASSERT_EQ(smoother_->activate().label(), "active");
    upstream_ = node_->create_publisher<geometry_msgs::msg::Twist>("/drive_chain_test/cmd_vel_nav", 1);
    ASSERT_TRUE(waitWithExecutor([this]() {
      geometry_msgs::msg::Twist command;
      command.linear.x = 0.6;
      upstream_->publish(command);
      return receivedDrive(60.0, 0.0);
    }));
  }

  void recordDrive(ArServerClient*, ArNetPacket* packet)
  {
    const Record record{ false, packet->bufToDouble(), packet->bufToDouble(), packet->bufToDouble(),
                         packet->bufToDouble() };
    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back(record);
    changed_.notify_all();
  }

  void recordStop(ArServerClient*, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back({ true, 0.0, 0.0, 0.0, 0.0 });
    changed_.notify_all();
  }

  void recordMode(ArServerClient*, ArNetPacket* packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requested_safe_mode_ = packet->bufToByte();
    ++mode_set_count_;
    changed_.notify_all();
  }

  void sendOdometry(ArServerClient* client, ArNetPacket*)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (suppress_odometry_)
    {
      return;
    }
    ArNetPacket packet;
    if (truncate_odometry_)
    {
      packet.byteToBuf(0);
      client->sendPacketTcp(&packet);
      return;
    }
    packet.byte2ToBuf(100);
    packet.byte4ToBuf(telemetry_.x_mm);
    packet.byte4ToBuf(telemetry_.y_mm);
    packet.byte2ToBuf(telemetry_.heading_deg);
    packet.byte2ToBuf(telemetry_.forward_mm_s);
    packet.byte2ToBuf(telemetry_.rotation_deg_s);
    packet.byte2ToBuf(telemetry_.lateral_mm_s);
    packet.byteToBuf(20);
    client->sendPacketTcp(&packet);
  }

  void checkRelativeOdometry(std::int16_t initial_heading, std::int16_t final_heading, double relative_heading,
                             bool check_tf = false)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      telemetry_ = { 1200, -2300, initial_heading, 1000, 15, 0 };
    }
    node_->declare_parameter<std::string>("driver.odom.frame_id", "drive_test/odom");
    node_->declare_parameter<std::string>("driver.tf.base_frame", "drive_test/base_link");
    nav_msgs::msg::Odometry::SharedPtr received;
    auto subscription = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/drive_test/odom", 10,
        [&received](nav_msgs::msg::Odometry::SharedPtr message) { received = std::move(message); });
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription;
    if (check_tf)
    {
      node_->declare_parameter<bool>("driver.tf.enabled", true);
      node_->set_parameter(rclcpp::Parameter("driver.tf.enabled", true));
      tf_subscription = node_->create_subscription<tf2_msgs::msg::TFMessage>(
          "/tf", 100, [&transforms](const tf2_msgs::msg::TFMessage& message) {
            for (const auto& transform : message.transforms)
            {
              if (transform.header.frame_id == "drive_test/odom" && transform.child_frame_id == "drive_test/base_link")
              {
                transforms.push_back(transform);
              }
            }
          });
    }
    initializeDriver();
    ASSERT_TRUE(waitWithExecutor([&received]() { return received != nullptr; }));
    EXPECT_NEAR(received->pose.pose.position.x, 0.0, 1e-12);
    EXPECT_NEAR(received->pose.pose.position.y, 0.0, 1e-12);
    EXPECT_NEAR(received->pose.pose.orientation.z, 0.0, 1e-12);
    EXPECT_NEAR(received->pose.pose.orientation.w, 1.0, 1e-12);
    const auto initial_stamp = rclcpp::Time(received->header.stamp);
    {
      const auto heading_rad = initial_heading * M_PI / 180.0;
      std::lock_guard<std::mutex> lock(mutex_);
      telemetry_.x_mm += static_cast<std::int32_t>(std::lround(1000.0 * std::cos(heading_rad)));
      telemetry_.y_mm += static_cast<std::int32_t>(std::lround(1000.0 * std::sin(heading_rad)));
      telemetry_.heading_deg = final_heading;
    }
    ASSERT_TRUE(
        waitWithExecutor([&received]() { return received != nullptr && received->pose.pose.position.x > 0.9; }));
    EXPECT_NEAR(received->pose.pose.position.x, 1.0, 1e-3);
    EXPECT_NEAR(received->pose.pose.position.y, 0.0, 1e-3);
    EXPECT_DOUBLE_EQ(received->twist.twist.linear.x, 1.0);
    EXPECT_DOUBLE_EQ(received->twist.twist.linear.y, 0.0);
    EXPECT_NEAR(received->twist.twist.angular.z, 15.0 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(received->pose.pose.orientation.z, std::sin(relative_heading * M_PI / 360.0), 1e-12);
    EXPECT_NEAR(received->pose.pose.orientation.w, std::cos(relative_heading * M_PI / 360.0), 1e-12);
    EXPECT_EQ(received->header.frame_id, "drive_test/odom");
    EXPECT_EQ(received->child_frame_id, "drive_test/base_link");
    EXPECT_GT(rclcpp::Time(received->header.stamp), initial_stamp);
    if (check_tf)
    {
      const auto odometry = *received;
      const auto matches_stamp = [&odometry](const geometry_msgs::msg::TransformStamped& transform) {
        return transform.header.stamp == odometry.header.stamp;
      };
      ASSERT_TRUE(waitWithExecutor([&transforms, &matches_stamp]() {
        return std::find_if(transforms.begin(), transforms.end(), matches_stamp) != transforms.end();
      }));
      const auto& transform = *std::find_if(transforms.begin(), transforms.end(), matches_stamp);
      EXPECT_EQ(transform.header.frame_id, odometry.header.frame_id);
      EXPECT_EQ(transform.child_frame_id, odometry.child_frame_id);
      EXPECT_DOUBLE_EQ(transform.transform.translation.x, odometry.pose.pose.position.x);
      EXPECT_DOUBLE_EQ(transform.transform.translation.y, odometry.pose.pose.position.y);
      EXPECT_DOUBLE_EQ(transform.transform.translation.z, odometry.pose.pose.position.z);
      EXPECT_EQ(transform.transform.rotation, odometry.pose.pose.orientation);
    }
  }

  void sendMode(ArServerClient* client, ArNetPacket*)
  {
    ArNetPacket packet;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++mode_query_count_;
      changed_.notify_all();
      if (suppress_mode_replies_)
      {
        return;
      }
      if (!empty_mode_reply_)
      {
        packet.uByteToBuf(mode_reply_override_ >= 0 ? mode_reply_override_ : requested_safe_mode_);
      }
    }
    client->sendPacketTcp(&packet);
  }

  inline static std::unique_ptr<amr_core::LibAriaRuntime> runtime_;
  ArServerBase server_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> drive_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> stop_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> mode_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> query_mode_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> odom_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> docking_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> undock_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> shutdown_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> fault_ack_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> faults_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> map_name_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> map_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> map_upload_callback_;
  ArFunctor2C<DriverInterfaceLoopbackTest, ArServerClient*, ArNetPacket*> map_download_callback_;
  std::string uploaded_map_name_;
  std::string uploaded_map_;
  bool corrupt_map_readback_{ false };
  int fake_docking_state_{ 2 };
  bool fake_docking_forced_{ false };
  unsigned short fake_docking_shutdown_countdown_{ 0 };
  bool suppress_docking_replies_{ false };
  bool suppress_docking_after_undock_{ false };
  bool server_started_{ false };
  pid_t core_process_{ -1 };
  bool executor_added_{ false };
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<amr_core::DockingInterface> docking_;
  int undock_requests_{ 0 };
  int shutdown_requests_{ 0 };
  std::vector<std::pair<std::uint8_t, std::string>> fault_ack_requests_;
  std::vector<std::string> fake_faults_;
  int fault_queries_{ 0 };
  bool fault_trailing_bytes_{ false };
  std::unique_ptr<DriverInterface> driver_;
  std::shared_ptr<nav2_velocity_smoother::VelocitySmoother> smoother_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr upstream_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<Record> records_;
  int requested_safe_mode_{ -1 };
  int mode_reply_override_{ -1 };
  int mode_set_count_{ 0 };
  int mode_query_count_{ 0 };
  bool suppress_mode_replies_{ false };
  bool empty_mode_reply_{ false };
  bool suppress_odometry_{ false };
  bool truncate_odometry_{ false };
  Telemetry telemetry_;
};

