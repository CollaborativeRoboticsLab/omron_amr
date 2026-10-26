#include <gtest/gtest.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav2_velocity_smoother/velocity_smoother.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "amr_core/interfaces/drive_interface.hpp"

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
    options.parameter_overrides({ { "driver.cmd_vel_topic", "/drive_test/cmd_vel" },
                                  { "driver.odom_topic", "/drive_test/odom" },
                                  { "driver.stop_topic", "/drive_test/stop" },
                                  { "driver.publish_robot_tf", false },
                                  { "driver.send_frequency", 20.0 },
                                  { "driver.cmd_vel_timeout_sec", 0.5 },
                                  { "driver.mode_check_period_sec", 0.05 },
                                  { "driver.mode_confirmation_timeout_sec", 0.3 },
                                  { "driver.unsafe_drive", true } });
    node_ = std::make_shared<rclcpp::Node>("drive_interface_test", options);
    driver_ = std::make_unique<DriverInterface>(node_);
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/drive_test/cmd_vel", 1);
    executor_.add_node(node_);
    executor_added_ = true;
  }

  void initializeDriver()
  {
    driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "");
    ASSERT_TRUE(waitWithExecutor([this]() { return publisher_->get_subscription_count() > 0; }));
  }

  void configureCalibration(bool confirmed = true)
  {
    node_->declare_parameter<std::string>("driver.ratio_mapping", "aria_ratio_v1");
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

  void initializeSmootherChain()
  {
    node_->declare_parameter<double>("driver.cmd_vel_timeout_sec", 0.2);
    node_->set_parameter(rclcpp::Parameter("driver.cmd_vel_timeout_sec", 0.2));
    initializeDriver();
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__ns:=/drive_chain_test", "-r",
                        "cmd_vel:=/drive_chain_test/cmd_vel_nav", "-r",
                        "cmd_vel_smoothed:=/drive_test/cmd_vel" });
    options.parameter_overrides({ { "use_sim_time", false }, { "enable_stamped_cmd_vel", false },
                                  { "feedback", "OPEN_LOOP" }, { "smoothing_frequency", 20.0 },
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
    node_->declare_parameter<std::string>("driver.odom_frame", "drive_test/odom");
    node_->declare_parameter<std::string>("driver.base_frame", "drive_test/base_link");
    nav_msgs::msg::Odometry::SharedPtr received;
    auto subscription = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/drive_test/odom", 10,
        [&received](nav_msgs::msg::Odometry::SharedPtr message) { received = std::move(message); });
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription;
    if (check_tf)
    {
      node_->declare_parameter<bool>("driver.publish_robot_tf", true);
      node_->set_parameter(rclcpp::Parameter("driver.publish_robot_tf", true));
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
    ASSERT_TRUE(waitWithExecutor([&received]() {
      return received != nullptr && received->pose.pose.position.x > 0.9;
    }));
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
  bool server_started_{ false };
  bool executor_added_{ false };
  rclcpp::Node::SharedPtr node_;
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

TEST_F(DriverInterfaceLoopbackTest, UpstreamLossWaitsForSmootherTimeoutThenDeceleratesAndExpires)
{
  initializeSmootherChain();
  const auto upstream_lost = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([upstream_lost]() {
    return std::chrono::steady_clock::now() - upstream_lost >= 350ms;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(std::any_of(records_.begin(), records_.end(), [upstream_lost](const Record& record) {
      return record.received_at > upstream_lost + 250ms && !record.stop && record.translation > 0.0;
    }));
    EXPECT_FALSE(std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; }));
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(0.0, 0.0); }));
  std::chrono::steady_clock::time_point zero_received;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto zero = std::find_if(records_.begin(), records_.end(), [](const Record& record) {
      return !record.stop && record.translation == 0.0 && record.rotation == 0.0;
    });
    ASSERT_NE(zero, records_.end());
    zero_received = zero->received_at;
  }
  EXPECT_GE(zero_received - upstream_lost, 900ms);
  EXPECT_LT(zero_received - upstream_lost, 2s);
  RecordProperty("upstream_loss_to_zero_sec", std::chrono::duration<double>(zero_received - upstream_lost).count());
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !records_.empty() && records_.back().stop;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GE(records_.back().received_at - zero_received, 150ms);
    EXPECT_LT(records_.back().received_at - zero_received, 700ms);
    RecordProperty("upstream_loss_to_stop_sec",
             std::chrono::duration<double>(records_.back().received_at - upstream_lost).count());
  }
  const auto stopped = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([stopped]() { return std::chrono::steady_clock::now() - stopped >= 300ms; }));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(std::any_of(records_.begin(), records_.end(), [stopped](const Record& record) {
      return record.received_at > stopped && !record.stop && record.translation != 0.0;
    }));
  }
  const auto resumed = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([this, resumed]() {
    geometry_msgs::msg::Twist command;
    command.linear.x = 0.3;
    upstream_->publish(command);
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [resumed](const Record& record) {
      return record.received_at > resumed && !record.stop && std::fabs(record.translation - 30.0) < 1e-3;
    });
  }));
}

TEST_F(DriverInterfaceLoopbackTest, SmootherFailureStopsViaDriverWatchdogWithoutExecutorProgress)
{
  initializeSmootherChain();
  ASSERT_EQ(smoother_->deactivate().label(), "inactive");
  const auto smoother_stopped = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  ASSERT_TRUE(changed_.wait_for(lock, 1s, [this]() { return !records_.empty() && records_.back().stop; }));
  EXPECT_LT(records_.back().received_at - smoother_stopped, 700ms);
  EXPECT_GE(records_.back().received_at - smoother_stopped, 100ms);
  RecordProperty("smoother_shutdown_to_stop_sec",
                 std::chrono::duration<double>(records_.back().received_at - smoother_stopped).count());
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryAtZeroDegreeOrigin)
{
  checkRelativeOdometry(0, 0, 0.0);
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryAtPositiveNinetyDegreeOrigin)
{
  checkRelativeOdometry(90, 90, 0.0);
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryAtNegativeNinetyDegreeOrigin)
{
  checkRelativeOdometry(-90, -90, 0.0);
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryWrapsPositiveAcrossPi)
{
  checkRelativeOdometry(179, -179, 2.0);
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryWrapsNegativeAcrossPi)
{
  checkRelativeOdometry(-179, 179, -2.0);
}

TEST_F(DriverInterfaceLoopbackTest, RelativeOdometryMatchesTfDirectionPoseAndTimestamp)
{
  checkRelativeOdometry(90, 105, 15.0, true);
}

TEST_F(DriverInterfaceLoopbackTest, CalibratesReverseIndependentlyOfPermittedSpeedLimit)
{
  configureCalibration();
  initializeDriver();
  publish(-1.0, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(-20.0, 0.0); }));
}

TEST_F(DriverInterfaceLoopbackTest, CalibratesThrottleAndTurningUsingBodySpeedFeedback)
{
  configureCalibration();
  node_->declare_parameter<double>("driver.drive_throttle_pct", 50.0);
  initializeDriver();
  publish(0.6, M_PI / 12.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 100.0 * 15.0 / 22.5); }));
}

TEST_F(DriverInterfaceLoopbackTest, RejectsUnconfirmedCalibrationBeforeConnecting)
{
  configureCalibration(false);
  EXPECT_THROW(initializeDriver(), std::invalid_argument);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
  EXPECT_EQ(mode_set_count_, 0);
}

TEST_F(DriverInterfaceLoopbackTest, UsesCalibrationFeedbackWhenOdometryPublicationIsDisabled)
{
  configureCalibration();
  node_->declare_parameter<bool>("driver.publish_odom", false);
  initializeDriver();
  publish(0.6, 0.2);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.2 * 180.0 / M_PI / 45.0 * 100.0); }));
}

TEST_F(DriverInterfaceLoopbackTest, RejectsUnresolvedCalibrationBeforeConnecting)
{
  configureCalibration();
  node_->set_parameter(rclcpp::Parameter("driver.calibration.full_throttle_forward_mm_s", 0.0));
  EXPECT_THROW(initializeDriver(), std::invalid_argument);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, StopsOnStaleTurningFeedbackBeforeCommandExpiry)
{
  configureCalibration();
  initializeDriver();
  publish(0.6, 0.2);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.2 * 180.0 / M_PI / 45.0 * 100.0); }));
  std::unique_lock<std::mutex> lock(mutex_);
  suppress_odometry_ = true;
  ASSERT_TRUE(changed_.wait_for(lock, 350ms, [this]() { return !records_.empty() && records_.back().stop; }));
}

TEST_F(DriverInterfaceLoopbackTest, InvalidatesTurningFeedbackOnTruncatedTelemetry)
{
  configureCalibration();
  initializeDriver();
  publish(0.6, 0.2);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.2 * 180.0 / M_PI / 45.0 * 100.0); }));
  std::unique_lock<std::mutex> lock(mutex_);
  truncate_odometry_ = true;
  ASSERT_TRUE(changed_.wait_for(lock, 350ms, [this]() { return !records_.empty() && records_.back().stop; }));
}

TEST_F(DriverInterfaceLoopbackTest, StopsInsteadOfBoostingAnUnrepresentableSmallVelocity)
{
  configureCalibration();
  node_->set_parameter(rclcpp::Parameter("driver.calibration.translation_dead_zone_pct", 10.0));
  initializeDriver();
  publish(0.01, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !records_.empty() && records_.back().stop;
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& record : records_)
  {
    EXPECT_TRUE(record.stop);
  }
}

TEST_F(DriverInterfaceLoopbackTest, ZeroLegacyThrottleStreamsZeroRatios)
{
  node_->declare_parameter<double>("driver.drive_throttle_pct", 0.0);
  initializeDriver();
  publish(0.6, 0.2);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(0.0, 0.0); }));
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& record : records_)
  {
    EXPECT_FALSE(record.stop);
    EXPECT_DOUBLE_EQ(record.throttle, 0.0);
  }
}

TEST_F(DriverInterfaceLoopbackTest, RejectsNonzeroCalibratedTargetAtZeroThrottle)
{
  configureCalibration();
  node_->declare_parameter<double>("driver.drive_throttle_pct", 0.0);
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !records_.empty() && records_.back().stop;
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& record : records_)
  {
    EXPECT_TRUE(record.stop);
  }
}

TEST_F(DriverInterfaceLoopbackTest, StreamsZeroWithoutStopAndPreservesUnsafeMode)
{
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 0.0); }));
  publish(0.0, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(0.0, 0.0); }));
  publish(0.3, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(30.0, 0.0); }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(requested_safe_mode_, 0);
  for (const auto& record : records_)
  {
    EXPECT_FALSE(record.stop);
    EXPECT_DOUBLE_EQ(record.throttle, 100.0);
    EXPECT_DOUBLE_EQ(record.lateral, 0.0);
  }
}

TEST_F(DriverInterfaceLoopbackTest, ExpiresEvenWhenRosExecutorIsNotSpinning)
{
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 0.0); }));
  std::unique_lock<std::mutex> lock(mutex_);
  ASSERT_TRUE(changed_.wait_for(lock, 2s, [this]() { return !records_.empty() && records_.back().stop; }));
  const auto stopped_count = records_.size();
  EXPECT_FALSE(changed_.wait_for(lock, 150ms, [this, stopped_count]() { return records_.size() != stopped_count; }));
}

TEST_F(DriverInterfaceLoopbackTest, EnforcesConfiguredReverseLimitBeforeRatioConversion)
{
  initializeDriver();
  publish(-1.0, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(-10.0, 0.0); }));
}

TEST_F(DriverInterfaceLoopbackTest, KeepsOdometryTwistInBodyFrameAtNinetyDegreeOrigin)
{
  initializeDriver();
  nav_msgs::msg::Odometry::SharedPtr received;
  auto subscription = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/drive_test/odom", 1,
      [&received](nav_msgs::msg::Odometry::SharedPtr message) { received = std::move(message); });
  ASSERT_TRUE(waitWithExecutor([&received]() { return received != nullptr; }));
  EXPECT_DOUBLE_EQ(received->twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(received->twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(received->pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(received->pose.pose.orientation.w, 1.0);
}

TEST_F(DriverInterfaceLoopbackTest, RejectsStartupWhenRequestedModeIsNotApplied)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_reply_override_ = 1;
  }
  EXPECT_THROW(initializeDriver(), std::runtime_error);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
  EXPECT_EQ(mode_set_count_, 1);
}

TEST_F(DriverInterfaceLoopbackTest, RejectsStartupWhenModeConfirmationIsSilent)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_mode_replies_ = true;
  }
  const auto started = std::chrono::steady_clock::now();
  EXPECT_THROW(initializeDriver(), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 3s);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
  EXPECT_EQ(mode_query_count_, 1);
}

TEST_F(DriverInterfaceLoopbackTest, RejectsStartupWhenModeReplyIsEmpty)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    empty_mode_reply_ = true;
  }
  EXPECT_THROW(initializeDriver(), std::runtime_error);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, RejectsStartupWhenModeReplyIsInvalid)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_reply_override_ = 2;
  }
  EXPECT_THROW(initializeDriver(), std::runtime_error);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, StopsAndLatchesOffWhenModeChangesWithoutForcingItBack)
{
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 0.0); }));
  {
    std::unique_lock<std::mutex> lock(mutex_);
    mode_reply_override_ = 1;
    ASSERT_TRUE(changed_.wait_for(lock, 1s, [this]() { return !records_.empty() && records_.back().stop; }));
    mode_reply_override_ = 0;
  }
  publish(0.3, 0.0);
  const auto deadline = std::chrono::steady_clock::now() + 150ms;
  EXPECT_TRUE(waitWithExecutor([deadline]() { return std::chrono::steady_clock::now() >= deadline; }));
  EXPECT_FALSE(receivedDrive(30.0, 0.0));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(records_.back().stop);
  EXPECT_EQ(mode_set_count_, 1);
}

TEST_F(DriverInterfaceLoopbackTest, StopsWhenModeRepliesCeaseWithoutRosExecutorProgress)
{
  node_->declare_parameter<double>("driver.cmd_vel_timeout_sec", 3.0);
  node_->set_parameter(rclcpp::Parameter("driver.cmd_vel_timeout_sec", 3.0));
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 0.0); }));
  std::unique_lock<std::mutex> lock(mutex_);
  suppress_mode_replies_ = true;
  ASSERT_TRUE(changed_.wait_for(lock, 1s, [this]() { return !records_.empty() && records_.back().stop; }));
  EXPECT_EQ(mode_set_count_, 1);
}