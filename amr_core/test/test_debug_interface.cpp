#include "robot_interface_test_fixture.hpp"

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementIsDisabledByDefault)
{
  initializeDriver();
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("acknowledge_fault");
  EXPECT_FALSE(service->wait_for_service(200ms));
}

TEST_F(DriverInterfaceLoopbackTest, FaultMonitorAcknowledgesOnlyAllowlistedFaultOnce)
{
  enableAutomaticFaultAcknowledgement();
  fake_faults_ = { "IrregularShutdown", "OtherFault" };
  initializeDockingBackend();
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("acknowledge_fault");
  EXPECT_FALSE(service->wait_for_service(200ms));
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_ack_requests_.size() == 1 && fault_queries_ >= 4;
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(fault_ack_requests_.size(), 1u);
  EXPECT_EQ(fault_ack_requests_.front().first, 0);
  EXPECT_EQ(fault_ack_requests_.front().second, "IrregularShutdown");
}

TEST_F(DriverInterfaceLoopbackTest, FaultMonitorRejectsUndocumentedTrailingBytes)
{
  enableAutomaticFaultAcknowledgement();
  fake_faults_ = { "IrregularShutdown" };
  fault_trailing_bytes_ = true;
  initializeDockingBackend();
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_queries_ >= 3;
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(fault_ack_requests_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, FaultMonitorAutomaticAcknowledgementRequiresOptIn)
{
  node_->declare_parameter<bool>("fault.monitor.auto_acknowledge_enabled", true);
  EXPECT_THROW(driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", ""),
               std::invalid_argument);
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementRequiresConfirmation)
{
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  EXPECT_THROW(driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", ""),
               std::invalid_argument);
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementRequiresAdvertisedCommand)
{
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  driver_ = std::make_unique<DriverInterface>(node_);
  EXPECT_THROW(driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", ""),
               std::runtime_error);
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementSendsOnlyNamedFault)
{
  enableFaultAcknowledgementBackend();
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  node_->declare_parameter<std::string>("fault.ros_service.name", "custom_acknowledge_fault");
  node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("custom_acknowledge_fault");
  ASSERT_TRUE(service->wait_for_service(1s));
  for (const std::string& name : std::vector<std::string>{ "", "   ", "bad\nname", std::string(129, 'a') })
  {
    auto request = std::make_shared<amr_msgs::srv::AcknowledgeFault::Request>();
    request->fault_name = name;
    auto result = service->async_send_request(request);
    ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
    EXPECT_FALSE(result.get()->success);
  }
  auto request = std::make_shared<amr_msgs::srv::AcknowledgeFault::Request>();
  request->fault_name = "IrregularShutdown";
  auto result = service->async_send_request(request);
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(result.get()->success);
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return !fault_ack_requests_.empty();
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(fault_ack_requests_.size(), 1u);
  EXPECT_EQ(fault_ack_requests_[0].first, 0);
  EXPECT_EQ(fault_ack_requests_[0].second, "IrregularShutdown");
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementRejectsStaleTelemetry)
{
  enableFaultAcknowledgementBackend();
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_odometry_ = true;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().departure_telemetry_fresh; }));
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("acknowledge_fault");
  ASSERT_TRUE(service->wait_for_service(1s));
  auto request = std::make_shared<amr_msgs::srv::AcknowledgeFault::Request>();
  request->fault_name = "IrregularShutdown";
  auto result = service->async_send_request(request);
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(result.get()->success);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(fault_ack_requests_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementRejectsMovingOrReservedUndock)
{
  enableFaultAcknowledgementBackend();
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("acknowledge_fault");
  ASSERT_TRUE(service->wait_for_service(1s));
  auto request = std::make_shared<amr_msgs::srv::AcknowledgeFault::Request>();
  request->fault_name = "ExampleFault";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 100;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().departure_stationary; }));
  auto moving = service->async_send_request(request);
  ASSERT_TRUE(waitWithExecutor([&moving]() { return moving.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(moving.get()->success);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 0;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return docking_->readiness().departure_stationary; }));
  ASSERT_TRUE(docking_->reserveUndock(true, true));
  auto reserved = service->async_send_request(request);
  ASSERT_TRUE(waitWithExecutor([&reserved]() { return reserved.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(reserved.get()->success);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(fault_ack_requests_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, FaultAcknowledgementRejectsReservedShutdown)
{
  enableFaultAcknowledgementBackend();
  enableShutdownBackend();
  node_->declare_parameter<bool>("fault.ros_service.enabled", true);
  node_->declare_parameter<bool>("fault.ack.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("shutdown.enabled", true);
  node_->declare_parameter<bool>("shutdown.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  auto shutdown = node_->create_client<std_srvs::srv::Trigger>("shutdown_robot");
  auto service = node_->create_client<amr_msgs::srv::AcknowledgeFault>("acknowledge_fault");
  ASSERT_TRUE(shutdown->wait_for_service(1s));
  ASSERT_TRUE(service->wait_for_service(1s));
  auto shutdown_result = shutdown->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(
      waitWithExecutor([&shutdown_result]() { return shutdown_result.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(shutdown_result.get()->success);
  auto request = std::make_shared<amr_msgs::srv::AcknowledgeFault::Request>();
  request->fault_name = "ExampleFault";
  auto result = service->async_send_request(request);
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(result.get()->success);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(fault_ack_requests_.empty());
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownRequiresConfirmationAtStartup)
{
  node_->declare_parameter<bool>("shutdown.enabled", true);
  EXPECT_THROW(driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", ""),
               std::invalid_argument);
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownIsDisabledByDefault)
{
  initializeDriver();
  auto shutdown = node_->create_client<std_srvs::srv::Trigger>("shutdown_robot");
  EXPECT_FALSE(shutdown->wait_for_service(200ms));
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownRequiresAdvertisedServerCommand)
{
  node_->declare_parameter<bool>("shutdown.enabled", true);
  node_->declare_parameter<bool>("shutdown.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  driver_ = std::make_unique<DriverInterface>(node_);
  EXPECT_THROW(driver_->initialize("127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", ""),
               std::runtime_error);
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownIsOneShotAndBlocksUndocking)
{
  enableShutdownBackend();
  node_->declare_parameter<bool>("shutdown.enabled", true);
  node_->declare_parameter<bool>("shutdown.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  auto shutdown = node_->create_client<std_srvs::srv::Trigger>("shutdown_robot");
  ASSERT_TRUE(shutdown->wait_for_service(1s));
  auto first = shutdown->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(waitWithExecutor([&first]() { return first.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(first.get()->success);
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_requests_ == 1;
  }));
  EXPECT_FALSE(docking_->reserveUndock(true, true));
  auto second = shutdown->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(waitWithExecutor([&second]() { return second.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(second.get()->success);
  auto stop = node_->create_publisher<std_msgs::msg::Empty>("/drive_test/stop", 10);
  ASSERT_TRUE(waitWithExecutor([&stop]() { return stop->get_subscription_count() > 0; }));
  stop->publish(std_msgs::msg::Empty{});
  ASSERT_TRUE(waitWithExecutor([this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(records_.begin(), records_.end(), [](const Record& record) { return record.stop; });
  }));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(shutdown_requests_, 1);
  EXPECT_EQ(undock_requests_, 0);
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownRejectsMovingTelemetry)
{
  enableShutdownBackend();
  node_->declare_parameter<bool>("shutdown.enabled", true);
  node_->declare_parameter<bool>("shutdown.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_.forward_mm_s = 100;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().departure_stationary; }));
  auto shutdown = node_->create_client<std_srvs::srv::Trigger>("shutdown_robot");
  ASSERT_TRUE(shutdown->wait_for_service(1s));
  auto result = shutdown->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(result.get()->success);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(shutdown_requests_, 0);
}

TEST_F(DriverInterfaceLoopbackTest, ShutdownRejectsStaleTelemetry)
{
  enableShutdownBackend();
  node_->declare_parameter<bool>("shutdown.enabled", true);
  node_->declare_parameter<bool>("shutdown.exclusive_control_confirmed", true);
  node_->declare_parameter<bool>("docking.contact_dock_confirmed", true);
  initializeDockingBackend();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    suppress_odometry_ = true;
  }
  ASSERT_TRUE(waitWithExecutor([this]() { return !docking_->readiness().departure_telemetry_fresh; }));
  auto shutdown = node_->create_client<std_srvs::srv::Trigger>("shutdown_robot");
  ASSERT_TRUE(shutdown->wait_for_service(1s));
  auto result = shutdown->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  EXPECT_FALSE(result.get()->success);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(shutdown_requests_, 0);
}

