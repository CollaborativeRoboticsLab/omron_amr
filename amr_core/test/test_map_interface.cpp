#include "robot_interface_test_fixture.hpp"

TEST_F(DriverInterfaceLoopbackTest, MapInterfaceReadsStableSnapshot)
{
  enableMapBackend();
  amr_core::RobotSession session;
  ASSERT_TRUE(session.connect({ "127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "6MTX" }));
  amr_core::MapInterface map(session.client());
  const auto snapshot = map.snapshot();
  EXPECT_EQ(snapshot.name, "fixture.map");
  EXPECT_EQ(snapshot.content, "2D-Map\nDATA\n");
}

TEST_F(DriverInterfaceLoopbackTest, MapInterfaceUploadsStagedMap)
{
  enableMapBackend();
  ASSERT_TRUE(server_.addData("putFileInterleaved", "map upload", &map_upload_callback_, "file packets", "ack"));
  ASSERT_TRUE(server_.addData("getFile", "file readback", &map_download_callback_, "filename", "file packets"));
  amr_core::RobotSession session;
  ASSERT_TRUE(session.connect({ "127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "6MTX" }));
  amr_core::MapInterface map(session.client());
  const std::string content(40000, 'x');
  const auto name = map.upload(content);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(name, uploaded_map_name_);
    EXPECT_EQ(content, uploaded_map_);
  }
  EXPECT_NE(name, "fixture.map");
  EXPECT_EQ(map.getName(), "fixture.map");
}

TEST_F(DriverInterfaceLoopbackTest, MapInterfaceRejectsMismatchedReadback)
{
  enableMapBackend();
  ASSERT_TRUE(server_.addData("putFileInterleaved", "map upload", &map_upload_callback_, "file packets", "ack"));
  ASSERT_TRUE(server_.addData("getFile", "file readback", &map_download_callback_, "filename", "file packets"));
  corrupt_map_readback_ = true;
  amr_core::RobotSession session;
  ASSERT_TRUE(session.connect({ "127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "6MTX" }));
  amr_core::MapInterface map(session.client());
  EXPECT_THROW(map.upload("2D-Map\nDATA\n"), std::runtime_error);
}

TEST_F(DriverInterfaceLoopbackTest, MapInterfaceUploadsToSdkFileServer)
{
  enableMapBackend();
  std::array<char, 32> directory{};
  std::strcpy(directory.data(), "/tmp/amr-map-test-XXXXXX");
  ASSERT_NE(::mkdtemp(directory.data()), nullptr);
  ArServerFileFromClient upload_server(&server_, directory.data(), directory.data());
  ArServerFileToClient download_server(&server_, directory.data());
  ArServerFileLister listing_server(&server_, directory.data());
  amr_core::RobotSession session;
  ASSERT_TRUE(session.connect({ "127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "6MTX" }));
  amr_core::MapInterface map(session.client());
  const std::string content(40000, 'a');
  const auto name = map.upload(content, "empty_map_with_dock.map");
  EXPECT_EQ(name, "empty_map_with_dock.map");
  EXPECT_EQ(map.download(name), content);
  EXPECT_THROW(map.download("../unsafe.map"), std::invalid_argument);
  EXPECT_THROW(map.upload(content, "EMPTY_MAP_WITH_DOCK.map"), std::runtime_error);
  EXPECT_THROW(map.upload(content, "../unsafe.map"), std::invalid_argument);
  std::ifstream saved(std::filesystem::path(directory.data()) / name, std::ios::binary);
  const std::string actual((std::istreambuf_iterator<char>(saved)), std::istreambuf_iterator<char>());
  EXPECT_EQ(actual, content);
  saved.close();
  EXPECT_TRUE(std::filesystem::remove(std::filesystem::path(directory.data()) / name));
  EXPECT_TRUE(std::filesystem::remove(directory.data()));
}

TEST_F(DriverInterfaceLoopbackTest, CoreNodeMapServerUsesConfiguredServiceName)
{
  enableMapBackend();
  const auto port_argument = std::string("robot.port:=") + std::to_string(server_.getTcpPort());
  core_process_ = fork();
  ASSERT_GE(core_process_, 0);
  if (core_process_ == 0)
  {
    execl(AMR_CORE_EXECUTABLE, AMR_CORE_EXECUTABLE, "--ros-args", "-r", "__ns:=/map_test", "-p",
          "robot.ip:=127.0.0.1", "-p", port_argument.c_str(), "-p", "robot.user:=loopback", "-p",
          "robot.protocol:=6MTX", "-p", "status.publish:=false", "-p", "laser.main_laser.enabled:=false", "-p",
          "driver.publish_robot_tf:=false", "-p", "map.ros_service.enabled:=true", "-p",
          "map.ros_service.name:=custom_active_map", static_cast<char*>(nullptr));
    _exit(127);
  }
  auto service = node_->create_client<std_srvs::srv::Trigger>("/map_test/custom_active_map");
  ASSERT_TRUE(waitWithExecutor([&service]() { return service->service_is_ready(); }));
  auto result = service->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(waitWithExecutor([&result]() { return result.wait_for(0s) == std::future_status::ready; }));
  const auto response = result.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "fixture.map");
}

