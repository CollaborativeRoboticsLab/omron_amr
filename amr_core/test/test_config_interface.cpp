#include "robot_interface_test_fixture.hpp"

TEST_F(DriverInterfaceLoopbackTest, MapSelectionSendsCapturedTransactionAndRequiresReceipt)
{
  struct ConfigServer
  {
    ConfigServer() : callback(this, &ConfigServer::receive) {}
    void receive(ArServerClient* client, ArNetPacket* packet)
    {
      const auto* begin = reinterpret_cast<const std::uint8_t*>(packet->getBuf()) + packet->getReadLength();
      int count;
      {
        std::lock_guard<std::mutex> lock(mutex);
        request.assign(begin, begin + packet->getDataLength() - packet->getDataReadLength());
        count = receipt_count;
      }
      ArNetPacket response;
      response.uByteToBuf(1);
      response.uByteToBuf(0);
      response.uByte4ToBuf(count);
      client->sendPacketTcp(&response);
    }
    ArFunctor2C<ConfigServer, ArServerClient*, ArNetPacket*> callback;
    std::mutex mutex;
    std::vector<std::uint8_t> request;
    int receipt_count{ 2 };
  } config;
  server_.enforceProtocolVersion("6MTX");
  ASSERT_TRUE(server_.addData("setConfigValues", "configuration write", &config.callback, "transaction", "receipt"));
  amr_core::RobotSession session;
  ASSERT_TRUE(session.connect({ "127.0.0.1", static_cast<int>(server_.getTcpPort()), "", "", "6MTX" }));
  amr_core::ConfigurationInterface interface(session.client());
  ASSERT_TRUE(interface.setMapSelection("uc_room1.map", "empty_map_with_dock_nobom.map"));
  {
    std::lock_guard<std::mutex> lock(config.mutex);
    EXPECT_EQ(config.request, amr_core::mapSelectionRequest("uc_room1.map", "empty_map_with_dock_nobom.map"));
    config.receipt_count = 1;
  }
  EXPECT_FALSE(interface.setMapSelection("uc_room1.map", "empty_map_with_dock_nobom.map"));
}

