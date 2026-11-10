#include "amr_core/utils/deadzone_calibration.hpp"
#include "amr_core/utils/robot_session.hpp"

#include <array>
#include <charconv>
#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help")
  {
    std::cout << "Usage: amr_deadzone_writer --apply-translation --confirm-write\n"
                 "Requires AMR_APPLY_TRANSLATION_DEADZONE=CONFIRMED and nine stdin lines:\n"
                 "host, port, user, password, protocol, expected translation, target translation,\n"
                 "expected rotation, APPLY_TRANSLATION_DEADZONE.\n"
                 "For --apply-rotation or --apply-timeout, set AMR_APPLY_TELEOP_SETTING=CONFIRMED;\n"
                 "stdin lines 6-9 are expected value, target value, expected unchanged dead-zone\n"
                 "(translation for rotation, rotation for timeout), and respectively\n"
                 "APPLY_ROTATION_DEADZONE or APPLY_TELEOP_TIMEOUT. One setting per run. Never drives.\n";
    return 0;
  }
  const std::string operation = argc >= 2 ? argv[1] : "";
  const bool translation = operation == "--apply-translation";
  const bool rotation_write = operation == "--apply-rotation";
  const bool timeout_write = operation == "--apply-timeout";
  const auto* approval = std::getenv(translation ? "AMR_APPLY_TRANSLATION_DEADZONE" : "AMR_APPLY_TELEOP_SETTING");
  if (argc != 3 || (!translation && !rotation_write && !timeout_write) ||
      std::string(argv[2]) != "--confirm-write" || approval == nullptr || std::string(approval) != "CONFIRMED")
  {
    std::cerr << "Explicit configuration-write confirmation required; no connection opened.\n";
    return 2;
  }
  std::array<std::string, 9> input;
  for (auto& field : input)
  {
    if (!std::getline(std::cin, field) || field.size() > 4096 ||
        field.find('\r') != std::string::npos || field.find('\0') != std::string::npos)
    {
      std::cerr << "Invalid stdin settings (values withheld); no connection opened.\n";
      return 2;
    }
  }
  const auto integer = [](const std::string& text) {
    int value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    {
      throw std::invalid_argument("Invalid integer settings");
    }
    return value;
  };
  const auto numeric = [](const std::string& text) {
    double value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !std::isfinite(value))
      throw std::invalid_argument("Invalid numeric settings");
    return value;
  };
  int port = 0;
  double expected = 0;
  double target = 0;
  int unchanged = 0;
  try
  {
    port = integer(input[1]);
    expected = timeout_write ? numeric(input[5]) : integer(input[5]);
    target = timeout_write ? numeric(input[6]) : integer(input[6]);
    unchanged = integer(input[7]);
    if (translation) amr_core::translationDeadzoneRequest(static_cast<int>(expected), static_cast<int>(target));
    else if (rotation_write) amr_core::rotationDeadzoneRequest(static_cast<int>(expected), static_cast<int>(target));
    else amr_core::teleopTimeoutRequest(expected, target);
    const std::string token = translation ? "APPLY_TRANSLATION_DEADZONE" :
                  rotation_write ? "APPLY_ROTATION_DEADZONE" : "APPLY_TELEOP_TIMEOUT";
    if (port < 1 || port > 65535 || input[0].empty() || input[4].empty() || unchanged < 1 || unchanged > 99 ||
      input[8] != token)
    {
      throw std::invalid_argument("Invalid calibration settings");
    }
  }
  catch (const std::exception&)
  {
    std::cerr << "Invalid calibration settings (values withheld); no connection opened.\n";
    return 2;
  }
  nlohmann::json report{ { "complete", false }, { "write_attempted", false }, { "ready_for_motion", false } };
  try
  {
    const amr_core::RobotConnection connection{ input[0], port, input[2], input[3], input[4] };
    amr_core::withRobotSession(connection, [&](amr_core::DriveClient& client) {
      if (translation)
        amr_calibration::changeTranslationDeadzone(client, static_cast<int>(expected), static_cast<int>(target),
                                                   unchanged, report);
      else
        amr_calibration::changeTeleopSetting(client, rotation_write ? "RotDeadZone" : "Timeout",
                                            expected, target, unchanged, report);
    });
  }
  catch (const std::exception&)
  {
    report["failure_reason"] = "Connection, preflight, receipt, or read-back failed. No retry.";
  }
  std::cout << "CALIBRATION_RESULT " << report.dump() << std::endl;
  return report.at("complete").get<bool>() ? 0 : 1;
}