#include <gtest/gtest.h>

#include <limits>

#include "amr_core/utils/configuration_protocol.hpp"
#include "amr_core/utils/deadzone_calibration.hpp"
#include "amr_core/utils/robot_session.hpp"

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
}  // namespace

TEST(HybridConfig, RobotSessionRestoresRuntimeLifetimeOnException)
{
  const auto references = amr_core::detail::ariaRefCount();
  EXPECT_THROW({
    amr_core::RobotSession session;
    EXPECT_EQ(amr_core::detail::ariaRefCount(), references + 1);
    EXPECT_FALSE(session.client().isConnected());
    throw std::runtime_error("Injected operation failure");
  }, std::runtime_error);
  EXPECT_EQ(amr_core::detail::ariaRefCount(), references);
  const amr_core::RobotConnection invalid{ "127.0.0.1", 0, "user", "secret", "6MTX" };
  EXPECT_THROW(amr_core::withRobotSession(invalid, [](auto&) {}), std::invalid_argument);
  EXPECT_EQ(amr_core::detail::ariaRefCount(), references);
}

TEST(HybridConfig, SharedTelemetryDecodesSignedFieldsAndRejectsTruncation)
{
  Bytes payload;
  number(payload, 987, 2);
  number(payload, static_cast<std::uint32_t>(-1234), 4);
  number(payload, 5678, 4);
  number(payload, static_cast<std::uint16_t>(-179), 2);
  number(payload, static_cast<std::uint16_t>(-100), 2);
  number(payload, 25, 2);
  number(payload, static_cast<std::uint16_t>(-2), 2);
  payload.push_back(0);
  for (std::size_t length = 0; length <= payload.size(); ++length)
  {
    ArNetPacket packet;
    packet.dataToBuf(payload.data(), static_cast<int>(length));
    packet.finalizePacket();
    packet.resetRead();
    amr_core::DriveTelemetry telemetry;
    telemetry.x_mm = 999;
    const bool valid = amr_core::readDriveTelemetry(&packet, telemetry);
    EXPECT_EQ(valid, length == payload.size());
    if (valid)
    {
      EXPECT_EQ(telemetry.state_of_charge_tenths, 987);
      EXPECT_EQ(telemetry.x_mm, -1234);
      EXPECT_EQ(telemetry.y_mm, 5678);
      EXPECT_EQ(telemetry.heading_deg, -179);
      EXPECT_EQ(telemetry.forward_mm_s, -100);
      EXPECT_EQ(telemetry.angular_deg_s, 25);
      EXPECT_EQ(telemetry.lateral_mm_s, -2);
      EXPECT_EQ(telemetry.position(), (std::array<int, 3>{ -1234, 5678, -179 }));
      EXPECT_EQ(telemetry.toJson().at("forward_mm_s"), -100);
    }
    else EXPECT_EQ(telemetry.x_mm, 999);
  }
  amr_core::DriveTelemetry telemetry;
  EXPECT_FALSE(amr_core::readDriveTelemetry(nullptr, telemetry));
}

TEST(HybridConfig, SharedSafeDriveDecoderRejectsMalformedReplies)
{
  for (const auto value : { 0, 1, 2, 255 })
  {
    ArNetPacket packet;
    packet.uByteToBuf(static_cast<unsigned char>(value));
    packet.finalizePacket();
    packet.resetRead();
    std::uint8_t mode = 99;
    EXPECT_EQ(amr_core::readSafeDrive(&packet, mode), value <= 1);
    EXPECT_EQ(mode, value <= 1 ? value : 99);
    EXPECT_FALSE(amr_core::readSafeDrive(&packet, mode));
  }
  std::uint8_t mode = 99;
  EXPECT_FALSE(amr_core::readSafeDrive(nullptr, mode));
}

TEST(HybridConfig, ObservationDetectsSmallMotionWithoutProvingNoCreep)
{
  const nlohmann::json first{ { "x_mm", 0 },         { "y_mm", 0 },          { "heading_deg", 179 },
                              { "forward_mm_s", 0 }, { "angular_deg_s", 0 }, { "lateral_mm_s", 0 } };
  auto samples = nlohmann::json::array({ first, first });
  const auto stationary = amr_calibration::summarizeObservation(samples);
  EXPECT_FALSE(stationary.at("motion_evidence").get<bool>());
  EXPECT_FALSE(stationary.at("no_creep_proven").get<bool>());
  samples[1]["x_mm"] = 1;
  samples[1]["heading_deg"] = -179;
  samples[1]["forward_mm_s"] = 1;
  const auto moved = amr_calibration::summarizeObservation(samples);
  EXPECT_TRUE(moved.at("motion_evidence").get<bool>());
  EXPECT_EQ(moved.at("max_position_change_mm"), 1);
  EXPECT_EQ(moved.at("max_heading_change_deg"), 2);
  EXPECT_EQ(moved.at("peak_reported_speeds").at("forward_mm_s"), 1);
  EXPECT_FALSE(moved.at("no_creep_proven").get<bool>());
}

TEST(HybridConfig, ObservationRejectsInsufficientAndMalformedSamples)
{
  EXPECT_THROW(amr_calibration::summarizeObservation(nlohmann::json::array()), std::invalid_argument);
  EXPECT_THROW(amr_calibration::summarizeObservation(nlohmann::json::array({ 1, 2 })), std::invalid_argument);
  const nlohmann::json sample{ { "x_mm", 0 },         { "y_mm", 0 },          { "heading_deg", 0 },
                               { "forward_mm_s", 0 }, { "angular_deg_s", 0 }, { "lateral_mm_s", 0 } };
  for (const auto& invalid :
       { nlohmann::json(true), nlohmann::json(nullptr), nlohmann::json(std::numeric_limits<double>::infinity()) })
  {
    auto samples = nlohmann::json::array({ sample, sample });
    samples[1]["x_mm"] = invalid;
    EXPECT_THROW(amr_calibration::summarizeObservation(samples), std::invalid_argument);
  }
}

TEST(HybridConfig, CalibrationObservationChecksBothReadbacks)
{
  for (const auto& mode : { "success", "wrong-before", "wrong-after" })
  {
    int reads = 0;
    int observations = 0;
    const auto read = [&]() {
      ++reads;
      return nlohmann::json{ { "complete", true },
                             { "read_only", true },
                             { "parameters",
                               { { { "section", "Teleop Settings" },
                                   { "name", "TransDeadZone" },
                                   { "value", (std::string(mode) == "wrong-before" && reads == 1) ||
                                                      (std::string(mode) == "wrong-after" && reads == 2) ?
                                                  3 :
                                                  5 } },
                                 { { "section", "Teleop Settings" }, { "name", "RotDeadZone" }, { "value", 5 } } } } };
    };
    const auto observe = [&]() {
      ++observations;
      const nlohmann::json sample{ { "x_mm", 0 },         { "y_mm", 0 },          { "heading_deg", 0 },
                                   { "forward_mm_s", 0 }, { "angular_deg_s", 0 }, { "lateral_mm_s", 0 } };
      return nlohmann::json::array({ sample, sample });
    };
    nlohmann::json report;
    if (std::string(mode) == "success")
    {
      EXPECT_NO_THROW(amr_calibration::observeDeadzones(5, 5, read, observe, report));
      EXPECT_TRUE(report.at("complete").get<bool>());
      EXPECT_FALSE(report.at("odometry").at("no_creep_proven").get<bool>());
    }
    else
    {
      EXPECT_THROW(amr_calibration::observeDeadzones(5, 5, read, observe, report), std::runtime_error);
      EXPECT_FALSE(report.at("complete").get<bool>());
    }
    EXPECT_EQ(reads, std::string(mode) == "wrong-before" ? 1 : 2);
    EXPECT_EQ(observations, std::string(mode) == "wrong-before" ? 0 : 1);
    EXPECT_FALSE(report.at("ready_for_motion").get<bool>());
  }
}

TEST(HybridConfig, CalibrationWriteIsOnceAuditedAndIndependentlyVerified)
{
  for (const auto& scenario : { "success", "failed", "timeout", "bad-result", "wrong-readback", "no-readback" })
  {
    const std::string mode(scenario);
    int writes = 0;
    int reads = 0;
    std::vector<nlohmann::json> saved;
    nlohmann::json report = nlohmann::json::object();
    const auto apply = [&]() -> nlohmann::json {
      ++writes;
      EXPECT_EQ(saved.size(), 1U);
      EXPECT_EQ(saved.front().at("state"), "pending");
      if (mode == "timeout")
        throw std::runtime_error("Simulated timeout");
      if (mode == "bad-result")
        return nullptr;
      return { { "complete", mode != "failed" }, { "write_attempted", true } };
    };
    const auto readback = [&]() {
      ++reads;
      if (mode == "no-readback")
        throw std::runtime_error("Read-back unavailable");
      return nlohmann::json{ { "complete", true },
                             { "read_only", true },
                             { "parameters",
                               { { { "section", "Teleop Settings" },
                                   { "name", "TransDeadZone" },
                                   { "value", mode == "wrong-readback" ? 10 : 14 } },
                                 { { "section", "Teleop Settings" }, { "name", "RotDeadZone" }, { "value", 5 } } } } };
    };
    amr_calibration::applyTranslation(
        10, 14, 5, apply, readback, [&](const nlohmann::json& value) { saved.push_back(value); }, report);
    EXPECT_EQ(writes, 1);
    EXPECT_EQ(reads, 1);
    EXPECT_EQ(saved.size(), 3U);
    EXPECT_EQ(report.at("complete"), mode == "success");
    EXPECT_EQ(report.at("state"), mode == "success" ? "verified" : "failed_or_uncertain");
    EXPECT_FALSE(report.at("ready_for_motion").get<bool>());
    EXPECT_FALSE(report.at("persistence_verified").get<bool>());
  }
}

TEST(HybridConfig, CalibrationReadbackRejectsIncompleteDuplicateAndInvalidValues)
{
  const nlohmann::json valid{ { "complete", true },
                              { "read_only", true },
                              { "parameters",
                                { { { "section", "Teleop Settings" }, { "name", "TransDeadZone" }, { "value", 14 } },
                                  { { "section", "Teleop Settings" }, { "name", "RotDeadZone" }, { "value", 5 } } } } };
  EXPECT_EQ(amr_calibration::deadzoneValues(valid), (nlohmann::json{ { "TransDeadZone", 14 }, { "RotDeadZone", 5 } }));
  auto incomplete = valid;
  incomplete["complete"] = false;
  EXPECT_THROW(amr_calibration::deadzoneValues(incomplete), std::invalid_argument);
  auto duplicate = valid;
  duplicate["parameters"].push_back(duplicate["parameters"].front());
  EXPECT_THROW(amr_calibration::deadzoneValues(duplicate), std::invalid_argument);
  for (const auto& value : { nlohmann::json(true), nlohmann::json(-1), nlohmann::json(100) })
  {
    auto invalid = valid;
    invalid["parameters"][0]["value"] = value;
    EXPECT_THROW(amr_calibration::deadzoneValues(invalid), std::invalid_argument);
  }
}

TEST(HybridConfig, EncodesNamedSectionRead)
{
  const std::vector<std::uint8_t> expected{ 1,   4,   'T', 'e', 'l', 'e', 'o', 'p', ' ', 'S',
                                            'e', 't', 't', 'i', 'n', 'g', 's', 0,   0 };
  EXPECT_EQ(amr_core::sectionRequest("Teleop Settings"), expected);
  const auto robot = amr_core::sectionRequest("Robot config");
  EXPECT_EQ(robot.at(0), 1);
  EXPECT_EQ(robot.at(1), 4);
  EXPECT_EQ(robot.size(), 16U);
}

TEST(HybridConfig, DecodesCapturedDeadzoneWriteTransaction)
{
  const Bytes captured{ 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7a, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x04, 0x01, 'R',  'o',  'b',
                        'o',  't',  ' ',  'O',  'p',  'e',  'r',  'a',  't',  'i',  'o',  'n',  0,    'T',  'e',  'l',
                        'e',  'o',  'p',  ' ',  'S',  'e',  't',  't',  'i',  'n',  'g',  's',  0,    0,    'T',  'r',
                        'a',  'n',  's',  'D',  'e',  'a',  'd',  'Z',  'o',  'n',  'e',  0,    'd',  '1',  '0',  0,
                        '5',  0,    0x01, 0x06, 0x01, 'R',  'o',  'b',  'o',  't',  ' ',  'O',  'p',  'e',  'r',  'a',
                        't',  'i',  'o',  'n',  0,    'T',  'e',  'l',  'e',  'o',  'p',  ' ',  'S',  'e',  't',  't',
                        'i',  'n',  'g',  's',  0,    0,    'T',  'r',  'a',  'n',  's',  'D',  'e',  'a',  'd',  'Z',
                        'o',  'n',  'e',  0,    0x02, 'f',  'a',  'l',  's',  'e',  0,    't',  'r',  'u',  'e',  0 };
  amr_core::ConfigCursor cursor(captured);
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
  amr_core::ConfigCursor reply(captured_reply);
  EXPECT_EQ(reply.number(1), 1U);
  EXPECT_EQ(reply.number(1), 0U);
  EXPECT_EQ(reply.number(4), 2U);
  EXPECT_EQ(reply.remaining(), 0U);
  Bytes value_only(captured.begin(), captured.begin() + 82);
  value_only[6] = 60;
  value_only[14] = 60;
  value_only[22] = 1;
  EXPECT_EQ(amr_core::translationDeadzoneRequest(10, 5), value_only);
}

TEST(HybridConfig, RestrictsDeadzoneWriterToOneNumericChange)
{
  for (const auto target : { 1, 3, 5, 14 })
  {
    const auto request = amr_core::translationDeadzoneRequest(10, target);
    amr_core::ConfigCursor cursor(request);
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
    EXPECT_THROW(amr_core::translationDeadzoneRequest(invalid, 14), std::invalid_argument);
    EXPECT_THROW(amr_core::translationDeadzoneRequest(10, invalid), std::invalid_argument);
  }
  EXPECT_THROW(amr_core::translationDeadzoneRequest(10, 10), std::invalid_argument);
  EXPECT_TRUE(amr_core::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0, 0 }));
  EXPECT_FALSE(amr_core::matchesSingleChangeReceipt({ 1, 0, 2, 0, 0, 0 }));
  EXPECT_FALSE(amr_core::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0 }));
  EXPECT_FALSE(amr_core::matchesSingleChangeReceipt({ 1, 0, 1, 0, 0, 0, 0 }));
}

TEST(HybridConfig, ConfigurationInterfaceRejectsDisconnectedOperations)
{
  Aria::init();
  {
    amr_core::DriveClient client;
    amr_core::ConfigurationInterface configuration(client);
    EXPECT_THROW(configuration.getValues(), std::runtime_error);
    EXPECT_THROW(configuration.setTranslationDeadzone(10, 14), std::runtime_error);
    EXPECT_THROW(configuration.setTranslationDeadzone(10, 10), std::invalid_argument);
    EXPECT_FALSE(client.isConnected());
  }
  Aria::shutdown();
}

TEST(HybridConfig, GuardsDeadzoneWriteAndVerifiesAllCapturedParameters)
{
  for (const auto* scenario : { "success", "wrong-start", "moving", "preflight-change", "wrong-after",
                                "rotation-change", "property-change", "bad-receipt", "missing", "duplicate" })
  {
    const std::string mode(scenario);
    nlohmann::json snapshot{ { "complete", true }, { "read_only", true }, { "parameters", nlohmann::json::array() } };
    for (const auto& name : amr_core::driveParameters)
    {
      snapshot["parameters"].push_back({ { "section", "Teleop Settings" },
                                         { "name", name },
                                         { "value", name == "TransDeadZone" ? 10 : 5 },
                                         { "metadata_code_1", 0 } });
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
          if (mode == "wrong-start")
            parameter["value"] = 5;
          if (mode == "preflight-change" && reads == 2)
            parameter["value"] = 11;
          if (mode == "property-change" && reads == 3)
            parameter["metadata_code_1"] = 1;
        }
        if (mode == "rotation-change" && reads == 3 && parameter["name"] == "RotDeadZone")
          parameter["value"] = 6;
      }
      if (mode == "missing")
        result["parameters"].erase(0);
      if (mode == "duplicate")
        result["parameters"].push_back(result["parameters"].at(0));
      return result;
    };
    const auto stationary = [&]() { return mode != "moving"; };
    const auto write = [&](const Bytes& encoded) {
      ++writes;
      EXPECT_EQ(encoded, amr_core::translationDeadzoneRequest(10, 14));
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

TEST(HybridConfig, PreservesProtocolInfinitySentinelsWithoutAcceptingInvalidNumbers)
{
  for (const auto& encoded : { "INF", "-INF", "0", "-0.125" })
  {
    Bytes bytes;
    text(bytes, encoded);
    amr_core::ConfigCursor cursor(bytes);
    const auto value = cursor.scalar('d');
    EXPECT_TRUE(value.is_string() || value.is_number());
    EXPECT_EQ(cursor.remaining(), 0U);
  }
  for (const auto& encoded : { "nan", "garbage", "1extra", "", "1e999", "1e-999" })
  {
    Bytes bytes;
    text(bytes, encoded);
    amr_core::ConfigCursor cursor(bytes);
    EXPECT_THROW(cursor.scalar('d'), std::runtime_error);
  }
}

TEST(HybridConfig, DecodesCapturedSubnormalSourceValue)
{
  Bytes bytes;
  text(bytes, "4.67261e-310");
  amr_core::ConfigCursor cursor(bytes);
  const auto value = cursor.scalar('d').get<double>();
  EXPECT_GT(value, 0.0);
  EXPECT_LT(value, std::numeric_limits<double>::min());
  EXPECT_EQ(cursor.remaining(), 0U);
}

TEST(HybridConfig, RejectsUnrelatedSectionsAndEmbeddedTerminators)
{
  for (const auto& section : { std::string(), std::string("Network"), std::string("Teleop Settings\0extra", 21) })
  {
    EXPECT_THROW(amr_core::sectionRequest(section), std::invalid_argument);
  }
}

TEST(HybridConfig, DecodesSignedValuesBoundsAndSourceMetadata)
{
  amr_core::SectionTransfer transfer("Robot config");
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
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_EQ(transfer.parameters()[0]["source_value_2"], -150);
}

TEST(HybridConfig, ReassemblesAcrossParameterBoundaries)
{
  const auto content = body();
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(Bytes(content.begin(), content.begin() + 12), 2, 0, content.size()));
  EXPECT_FALSE(transfer.complete());
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
  transfer.accept(chunk(Bytes(content.begin() + 12, content.end()), 2, 1, content.size()));
  EXPECT_EQ(transfer.parameters()[0]["value"], -100);
}

TEST(HybridConfig, OmitsUnrelatedParameters)
{
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(body("NotADriveParameter")));
  EXPECT_TRUE(transfer.parameters().empty());
}

TEST(HybridConfig, PreservesUninterpretedMetadataCodes)
{
  auto content = body();
  content[content.size() - 2] = 3;
  content[content.size() - 1] = 4;
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_EQ(transfer.parameters()[0]["metadata_code_1"], 3);
  EXPECT_EQ(transfer.parameters()[0]["metadata_code_2"], 4);
}

TEST(HybridConfig, RejectsTruncatedAndUnsupportedRecords)
{
  const auto content = body();
  for (std::size_t length = 0; length < content.size(); ++length)
  {
    amr_core::SectionTransfer transfer("Robot config");
    EXPECT_THROW(
        {
          transfer.accept(chunk(Bytes(content.begin(), content.begin() + length)));
          transfer.parameters();
        },
        std::runtime_error)
        << length;
  }
  auto unsupported = content;
  unsupported[4] = 'X';
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(unsupported));
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
}

TEST(HybridConfig, RejectsDuplicateOutOfOrderOversizedAndInconsistentChunks)
{
  const auto content = body();
  for (const auto& packet : { chunk(content, 0), chunk(content, 129), chunk(content, 2, 1),
                              chunk(content, 1, 0, 1048577), chunk(content, 1, 0, content.size() + 1) })
  {
    amr_core::SectionTransfer transfer("Robot config");
    EXPECT_THROW(transfer.accept(packet), std::runtime_error);
    EXPECT_THROW(transfer.accept(chunk(content)), std::runtime_error);
  }
  amr_core::SectionTransfer transfer("Robot config");
  transfer.accept(chunk(content));
  EXPECT_THROW(transfer.accept(chunk(content)), std::runtime_error);
  EXPECT_THROW(transfer.parameters(), std::runtime_error);
}