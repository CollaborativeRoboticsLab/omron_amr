#include "robot_interface_test_fixture.hpp"

TEST_F(DriverInterfaceLoopbackTest, ShippedYamlConfiguresDriverTopicsLimitsAndTf)
{
  driver_.reset();
  executor_.remove_node(node_);
  publisher_.reset();
  const auto yaml = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                    "amr_ros/config/ld250_parameters.yaml";
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "--params-file", yaml.string(), "-p", "driver.tf.enabled:=false" });
  node_ = std::make_shared<rclcpp::Node>("amr_core", options);
  executor_.add_node(node_);
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  auto stop_publisher = node_->create_publisher<std_msgs::msg::Empty>("stop", 1);
  bool received_odom = false;
  bool received_tf = false;
  auto odom_subscription = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [&received_odom](const nav_msgs::msg::Odometry&) { received_odom = true; });
  auto tf_subscription = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&received_tf](const tf2_msgs::msg::TFMessage&) { received_tf = true; });
  driver_ = std::make_unique<DriverInterface>(node_, docking_);
  initializeDriver();
  ASSERT_TRUE(waitWithExecutor([&stop_publisher, &received_odom]() {
    return stop_publisher->get_subscription_count() > 0 && received_odom;
  }));
  publish(-0.5, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(-20.0, 0.0); }));
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(waitWithExecutor([started]() { return std::chrono::steady_clock::now() - started > 300ms; }));
  EXPECT_FALSE(received_tf);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsForwardThenStops)
{
  checkVelocityStream(0.5, 0.0);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsReverseThenStops)
{
  checkVelocityStream(-0.2, 0.0);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsLeftTurnThenStops)
{
  checkVelocityStream(0.0, 0.5);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsRightTurnThenStops)
{
  checkVelocityStream(0.0, -0.5);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsForwardLeftThenStops)
{
  checkVelocityStream(0.5, 0.5);
}

TEST_F(DriverInterfaceLoopbackTest, StreamsForwardRightThenStops)
{
  checkVelocityStream(0.5, -0.5);
}

TEST_F(DriverInterfaceLoopbackTest, ForwardHalfMetreCommandStopsWithoutResuming)
{
  node_->declare_parameter<double>("driver.cmd_vel.timeout_sec", 0.2);
  node_->set_parameter(rclcpp::Parameter("driver.cmd_vel.timeout_sec", 0.2));
  auto stop_publisher = node_->create_publisher<std_msgs::msg::Empty>("/drive_test/stop", 10);
  initializeDriver();
  ASSERT_TRUE(waitWithExecutor([&stop_publisher]() { return stop_publisher->get_subscription_count() > 0; }));

  publish(0.5, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(50.0, 0.0); }));
  stop_publisher->publish(std_msgs::msg::Empty{});
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
  }));
  const auto stop_received = std::chrono::steady_clock::now();
  ASSERT_TRUE(
      waitWithExecutor([stop_received]() { return std::chrono::steady_clock::now() - stop_received >= 250ms; }));
  std::lock_guard<std::mutex> lock(mutex_);
  const auto stop = std::find_if(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
  ASSERT_NE(stop, records_.end());
  EXPECT_TRUE(std::none_of(stop + 1, records_.end(), [](const Record& record) { return !record.stop; }));
}

TEST_F(DriverInterfaceLoopbackTest, UpstreamLossWaitsForSmootherTimeoutThenDeceleratesAndExpires)
{
  initializeSmootherChain();
  const auto upstream_lost = std::chrono::steady_clock::now();
  ASSERT_TRUE(
      waitWithExecutor([upstream_lost]() { return std::chrono::steady_clock::now() - upstream_lost >= 350ms; }));
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
  node_->declare_parameter<double>("driver.drive.throttle_pct", 50.0);
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
  node_->declare_parameter<bool>("driver.odom.publish", false);
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
  node_->declare_parameter<double>("driver.drive.throttle_pct", 0.0);
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
  node_->declare_parameter<double>("driver.drive.throttle_pct", 0.0);
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
  node_->declare_parameter<double>("driver.cmd_vel.timeout_sec", 3.0);
  node_->set_parameter(rclcpp::Parameter("driver.cmd_vel.timeout_sec", 3.0));
  initializeDriver();
  publish(0.6, 0.0);
  ASSERT_TRUE(waitWithExecutor([this]() { return receivedDrive(60.0, 0.0); }));
  std::unique_lock<std::mutex> lock(mutex_);
  suppress_mode_replies_ = true;
  ASSERT_TRUE(changed_.wait_for(lock, 1s, [this]() { return !records_.empty() && records_.back().stop; }));
  EXPECT_EQ(mode_set_count_, 1);
}