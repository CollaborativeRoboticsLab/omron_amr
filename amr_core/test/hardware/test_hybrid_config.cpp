#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "hybrid_config.hpp"
#include "../../calibration/deadzone_write.hpp"

namespace
{
using Bytes = std::vector<std::uint8_t>;

void number(Bytes& bytes, std::uint64_t value, std::size_t width)
{
  for (std::size_t index = 0; index < width; ++index)
  {
    bytes.push_back((value >> (8 * index)) & 255);
  }
}

void text(Bytes& bytes, const std::string& value)
{
  bytes.insert(bytes.end(), value.begin(), value.end());
  bytes.push_back(0);
}

Bytes body(const std::string& name = "TransNegVelMax")
{
  Bytes bytes{ 1, 0, 0, 0, 'P' };
  text(bytes, name);
  text(bytes, "");
  bytes.insert(bytes.end(), { 3, 'i' });
  number(bytes, static_cast<std::uint32_t>(-100), 4);
  number(bytes, static_cast<std::uint32_t>(-200), 4);
  number(bytes, static_cast<std::uint32_t>(-1), 4);
  bytes.insert(bytes.end(), { 0, 0, 0, 1, 0, 1, 0 });
  bytes.insert(bytes.end(), { 1, 2, 1 });
  number(bytes, static_cast<std::uint32_t>(-100), 4);
  bytes.insert(bytes.end(), { 0, 0, 0, 0 });
  return bytes;
}

Bytes chunk(const Bytes& content, std::size_t count = 1, std::size_t index = 0, std::size_t total = 0)
{
  Bytes bytes{ 1, 4 };
  text(bytes, "Robot config");
  bytes.push_back(0);
  number(bytes, count, 2);
  number(bytes, index, 2);
  number(bytes, total == 0 ? content.size() : total, 8);
  number(bytes, content.size(), 8);
  bytes.insert(bytes.end(), content.begin(), content.end());
  return bytes;
}
}

TEST(HybridConfig, EncodesNamedSectionRead)
{
  const std::vector<std::uint8_t> expected{
    1, 4, 'T', 'e', 'l', 'e', 'o', 'p', ' ', 'S', 'e', 't', 't', 'i', 'n', 'g', 's', 0, 0
  };
  EXPECT_EQ(amr_hardware::sectionRequest("Teleop Settings"), expected);
  const auto robot = amr_hardware::sectionRequest("Robot config");
  EXPECT_EQ(robot.at(0), 1);
  EXPECT_EQ(robot.at(1), 4);
  EXPECT_EQ(robot.size(), 16U);
}

TEST(HybridConfig, DecodesCapturedDeadzoneWriteTransaction)
{
  const Bytes captured{
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x04, 0x01,
    'R', 'o', 'b', 'o', 't', ' ', 'O', 'p', 'e', 'r', 'a', 't', 'i', 'o', 'n', 0,
    'T', 'e', 'l', 'e', 'o', 'p', ' ', 'S', 'e', 't', 't', 'i', 'n', 'g', 's', 0, 0,
    'T', 'r', 'a', 'n', 's', 'D', 'e', 'a', 'd', 'Z', 'o', 'n', 'e', 0,
    'd', '1', '0', 0, '5', 0, 0x01, 0x06, 0x01,
    'R', 'o', 'b', 'o', 't', ' ', 'O', 'p', 'e', 'r', 'a', 't', 'i', 'o', 'n', 0,
    'T', 'e', 'l', 'e', 'o', 'p', ' ', 'S', 'e', 't', 't', 'i', 'n', 'g', 's', 0, 0,
    'T', 'r', 'a', 'n', 's', 'D', 'e', 'a', 'd', 'Z', 'o', 'n', 'e', 0,
    0x02, 'f', 'a', 'l', 's', 'e', 0, 't', 'r', 'u', 'e', 0
  };
  amr_hardware::ConfigCursor cursor(captured);
  EXPECT_EQ(cursor.number(1), 1U);
  EXPECT_EQ(cursor.number(1), 0U);
  EXPECT_EQ(cursor.number(2), 1U);
  EXPECT_EQ(cursor.number(2), 0U);
  EXPECT_EQ(cursor.number(8), 122U);
  const auto length = cursor.number(8);
  EXPECT_EQ(length, cursor.remaining());
  EXPECT_EQ(cursor.number(4), 2U);
  for (const auto operation : { 4U, 6U })
  {
    EXPECT_EQ(cursor.number(1), 1U);
    EXPECT_EQ(cursor.number(1), operation);
    EXPECT_EQ(cursor.number(1), 1U);
    EXPECT_EQ(cursor.text(), "Robot Operation");
    EXPECT_EQ(cursor.text(), "Teleop Settings");
    EXPECT_EQ(cursor.text(), "");
    EXPECT_EQ(cursor.text(), "TransDeadZone");
    if (operation == 4)
    {
      EXPECT_EQ(cursor.number(1), static_cast<unsigned>('d'));
      EXPECT_EQ(cursor.text(), "10");
      EXPECT_EQ(cursor.text(), "5");
    }
    else
    {
      EXPECT_EQ(cursor.number(1), 2U);
      EXPECT_EQ(cursor.text(), "false");
      EXPECT_EQ(cursor.text(), "true");
    }
  }
  EXPECT_EQ(cursor.remaining(), 0U);
  const Bytes captured_reply{ 1, 0, 2, 0, 0, 0 };
  amr_hardware::ConfigCursor reply(captured_reply);
  EXPECT_EQ(reply.number(1), 1U);
  EXPECT_EQ(reply.number(1), 0U);
  EXPECT_EQ(reply.number(4), 2U);
  EXPECT_EQ(reply.remaining(), 0U);
  Bytes value_only(captured.begin(), captured.begin() + 82);
  value_only[6] = 60;
  value_only[14] = 60;
  value_only[22] = 1;
  EXPECT_EQ(amr_calibration::translationDeadzoneRequest(10, 5), value_only);
}

TEST(HybridConfig, RestrictsDeadzoneWriterToOneNumericChange)
{
  for (const auto target : { 1, 3, 5, 14 })
  {
    const auto request = amr_calibration::translationDeadzoneRequest(10, target);
    amr_hardware::ConfigCursor cursor(request);
    EXPECT_EQ(cursor.number(1), 1U);
    EXPECT_EQ(cursor.number(1), 0U);
    EXPECT_EQ(cursor.number(2), 1U);
    EXPECT_EQ(cursor.number(2), 0U);
    const auto total = cursor.number(8);
    const auto length = cursor.number(8);
    EXPECT_EQ(total, length);
    EXPECT_EQ(length, cursor.remaining());
    EXPECT_EQ(cursor.number(4), 1U);
    EXPECT_EQ(cursor.number(1), 1U);
    EXPECT_EQ(cursor.number(1), 4U);
    EXPECT_EQ(cursor.number(1), 1U);
    EXPECT_EQ(cursor.text(), "Robot Operation");
    EXPECT_EQ(cursor.text(), "Teleop Settings");
    EXPECT_EQ(cursor.text(), "");
    EXPECT_EQ(cursor.text(), "TransDeadZone");
    EXPECT_EQ(cursor.number(1), static_cast<unsigned>('d'));
    EXPECT_EQ(cursor.text(), "10");
    EXPECT_EQ(cursor.text(), std::to_string(target));
    EXPECT_EQ(cursor.remaining(), 0U);
  }
  for (const auto invalid : { -1, 0, 100, 101 })
  {
    EXPECT_THROW(amr_calibration::translationDeadzoneRequest(invalid, 14), std::invalid_argument);
    EXPECT_THROW(amr_calibration::translationDeadzoneRequest(10, invalid), std::invalid_argument);
  }
  EXPECT_THROW(amr_calibration::translationDeadzoneRequest(10, 10), std::invalid_argument);
  EXPECT_TRUE(amr_calibration::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0, 0 }));
  EXPECT_FALSE(amr_calibration::matchesSingleChangeReceipt({ 1, 0, 2, 0, 0, 0 }));
  EXPECT_FALSE(amr_calibration::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0 }));
  EXPECT_FALSE(amr_calibration::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0, 0, 0 }));
}

TEST(HybridConfig, GuardsDeadzoneWriteAndVerifiesAllCapturedParameters)
{
  for (const auto* scenario : { "success", "wrong-start", "moving", "preflight-change", "wrong-after",
                                "rotation-change", "property-change", "bad-receipt", "missing", "duplicate" })
  {
    const std::string mode(scenario);
    nlohmann::json snapshot{ { "complete", true }, { "read_only", true },
                             { "parameters", nlohmann::json::array() } };
    for (const auto& name : amr_hardware::driveParameters)
    {
      snapshot["parameters"].push_back({ { "section", "Teleop Settings" }, { "name", name },
          { "value", name == "TransDeadZone" ? 10 : 5 }, { "metadata_code_1", 0 } });
    }
    int reads = 0;
    int writes = 0;
    nlohmann::json report{ { "complete", false }, { "write_attempted", false } };
    const auto read = [&]() {
      auto result = snapshot;
      ++reads;
      for (auto& parameter : result["parameters"])
      {
        if (parameter["name"] == "TransDeadZone")
        {
          parameter["value"] = reads == 3 && mode != "wrong-after" ? 14 : 10;
          if (mode == "wrong-start") parameter["value"] = 5;
          if (mode == "preflight-change" && reads == 2) parameter["value"] = 11;
          if (mode == "property-change" && reads == 3) parameter["metadata_code_1"] = 1;
        }
        if (mode == "rotation-change" && reads == 3 && parameter["name"] == "RotDeadZone")
          parameter["value"] = 6;
      }
      if (mode == "missing") result["parameters"].erase(0);
      if (mode == "duplicate") result["parameters"].push_back(result["parameters"].at(0));
      return result;
    };
    const auto stationary = [&]() { return mode != "moving"; };
    const auto write = [&](const Bytes& encoded) {
      ++writes;
      EXPECT_EQ(encoded, amr_calibration::translationDeadzoneRequest(10, 14));
      return mode != "bad-receipt";
    };
    if (mode == "success")
    {
      EXPECT_NO_THROW(amr_calibration::changeTranslationDeadzone(10, 14, 5, read, stationary, write, report));
      EXPECT_TRUE(report.at("complete").get<bool>());
    }
    else
    {
      EXPECT_THROW(amr_calibration::changeTranslationDeadzone(10, 14, 5, read, stationary, write, report),
                   std::exception);
      EXPECT_FALSE(report.at("complete").get<bool>());
    }
    const bool preflight_failed = mode == "wrong-start" || mode == "moving" || mode == "preflight-change" ||
                                  mode == "missing" || mode == "duplicate";
    EXPECT_EQ(writes, preflight_failed ? 0 : 1) << mode;
  }
}

TEST(HybridCalibration, DISABLED_ApplyTranslationDeadzone)
{
  const auto* approval = std::getenv("AMR_APPLY_TRANSLATION_DEADZONE");
  if (approval == nullptr || std::string(approval) != "CONFIRMED")
  {
    GTEST_SKIP() << "Live configuration writes require separate explicit approval.";
  }
  std::array<std::string, 9> input;
  for (auto& field : input)
  {
    ASSERT_TRUE(static_cast<bool>(std::getline(std::cin, field))) << "Missing stdin settings (values withheld).";
    ASSERT_EQ(field.find('\r'), std::string::npos) << "Invalid stdin settings (values withheld).";
  }
  int port = 0;
  int expected = 0;
  int target = 0;
  int rotation = 0;
  const auto integer = [](const std::string& text) {
    std::size_t consumed = 0;
    const auto value = std::stoi(text, &consumed);
    if (consumed != text.size())
    {
      throw std::invalid_argument("Invalid integer settings (values withheld).");
    }
    return value;
  };
  ASSERT_NO_THROW({
    port = integer(input[1]);
    expected = integer(input[5]);
    target = integer(input[6]);
    rotation = integer(input[7]);
    amr_calibration::translationDeadzoneRequest(expected, target);
  });
  ASSERT_TRUE(port >= 1 && port <= 65535 && !input[0].empty() && !input[4].empty() &&
              rotation >= 1 && rotation <= 99 && input[8] == "APPLY_TRANSLATION_DEADZONE")
      << "Invalid calibration settings (values withheld).";
  nlohmann::json report{ { "complete", false }, { "write_attempted", false }, { "ready_for_motion", false } };
  Aria::init();
  ArLog::init(ArLog::None, ArLog::Terse, "", false, false, false);
  {
    amr_core::DriveClient client;
    try
    {
      client.enforceProtocolVersion(input[4].c_str());
      if (!client.configureTransport(32, 65536, 0.1) ||
          !client.blockingConnect(input[0].c_str(), port, false, input[2].c_str(),
                                   input[3].empty() ? nullptr : input[3].c_str()))
      {
        throw std::runtime_error("Connection or authentication failed.");
      }
      amr_calibration::changeTranslationDeadzone(client, expected, target, rotation, report);
    }
    catch (const std::exception&)
    {
      report["failure_reason"] = "Connection, preflight, receipt, or read-back failed. No retry.";
    }
    client.disconnect();
  }
  Aria::shutdown();
  std::cout << "CALIBRATION_RESULT " << report.dump() << std::endl;
  EXPECT_TRUE(report.at("complete").get<bool>()) << "Inspect sanitized calibration result; no automatic retry.";
}

TEST(HybridConfig, PreservesProtocolInfinitySentinelsWithoutAcceptingInvalidNumbers)
{
  for (const auto& encoded : { "INF", "-INF", "0", "-0.125" })
  {
    Bytes bytes;
    text(bytes, encoded);
    amr_hardware::ConfigCursor cursor(bytes);
    const auto value = cursor.scalar('d');
    EXPECT_TRUE(value.is_string() || value.is_number());
    EXPECT_EQ(cursor.remaining(), 0U);
  }
  for (const auto& encoded : { "nan", "garbage", "1extra", "", "1e999", "1e-999" })
  {
    Bytes bytes;
    text(bytes, encoded);
    amr_hardware::ConfigCursor cursor(bytes);
    EXPECT_THROW(cursor.scalar('d'), std::runtime_error);
  }
}

TEST(HybridConfig, DecodesCapturedSubnormalSourceValue)
{
  Bytes bytes;
  text(bytes, "4.67261e-310");
  amr_hardware::ConfigCursor cursor(bytes);
  const auto value = cursor.scalar('d').get<double>();
  EXPECT_GT(value, 0.0);
  EXPECT_LT(value, std::numeric_limits<double>::min());
  EXPECT_EQ(cursor.remaining(), 0U);
}

TEST(HybridConfig, RejectsUnrelatedSectionsAndEmbeddedTerminators)
{
  for (const auto& section : { std::string(), std::string("Network"), std::string("Teleop Settings\0extra", 21) })
  {
    EXPECT_THROW(amr_hardware::sectionRequest(section), std::invalid_argument);
  }
}

TEST(HybridConfig, DecodesSignedValuesBoundsAndSourceMetadata)
{
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(body()));
  ASSERT_TRUE(transfer.complete());
  const auto values = transfer.parameters();
  ASSERT_EQ(values.size(), 1U);
  EXPECT_EQ(values[0]["value"], -100);
  EXPECT_EQ(values[0]["minimum"], -200);
  EXPECT_EQ(values[0]["maximum"], -1);
  EXPECT_EQ(values[0]["source_value_1"], -100);
  EXPECT_FALSE(values[0].contains("source_value_2"));
  EXPECT_EQ(values[0]["scope_code"], 2);
}

TEST(HybridConfig, DecodesBothPresentSourceSlotsWithoutStringTerminators)
{
  auto content = body();
  content.resize(content.size() - 4);
  content.push_back(1);
  number(content, static_cast<std::uint32_t>(-150), 4);
  content.insert(content.end(), { 0, 0 });
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_EQ(transfer.parameters()[0]["source_value_2"], -150);
}

TEST(HybridConfig, ReassemblesAcrossParameterBoundaries)
{
  const auto content = body();
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(Bytes(content.begin(), content.begin() + 12), 2, 0, content.size()));
  EXPECT_FALSE(transfer.complete());
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
  transfer.accept(chunk(Bytes(content.begin() + 12, content.end()), 2, 1, content.size()));
  EXPECT_EQ(transfer.parameters()[0]["value"], -100);
}

TEST(HybridConfig, OmitsUnrelatedParameters)
{
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(body("NotADriveParameter")));
  EXPECT_TRUE(transfer.parameters().empty());
}

TEST(HybridConfig, PreservesUninterpretedMetadataCodes)
{
  auto content = body();
  content[content.size() - 2] = 3;
  content[content.size() - 1] = 4;
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_EQ(transfer.parameters()[0]["metadata_code_1"], 3);
  EXPECT_EQ(transfer.parameters()[0]["metadata_code_2"], 4);
}

TEST(HybridConfig, RejectsTruncatedAndUnsupportedRecords)
{
  const auto content = body();
  for (std::size_t length = 0; length < content.size(); ++length)
  {
    amr_hardware::SectionTransfer transfer("Robot config");
    EXPECT_THROW({ transfer.accept(chunk(Bytes(content.begin(), content.begin() + length)));
                   transfer.parameters(); }, std::runtime_error) << length;
  }
  auto unsupported = content;
  unsupported[4] = 'X';
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(unsupported));
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
}

TEST(HybridConfig, RejectsDuplicateOutOfOrderOversizedAndInconsistentChunks)
{
  const auto content = body();
  for (const auto& packet : { chunk(content, 0), chunk(content, 129), chunk(content, 2, 1),
                              chunk(content, 1, 0, 1048577), chunk(content, 1, 0, content.size() + 1) })
  {
    amr_hardware::SectionTransfer transfer("Robot config");
    EXPECT_THROW(transfer.accept(packet), std::runtime_error);
    EXPECT_THROW(transfer.accept(chunk(content)), std::runtime_error);
  }
  amr_hardware::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_THROW(transfer.accept(chunk(content)), std::runtime_error);
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
}