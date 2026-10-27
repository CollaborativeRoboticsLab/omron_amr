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
                 "expected rotation, APPLY_TRANSLATION_DEADZONE. Never drives.\n";
    return 0;
  }
  const auto* approval = std::getenv("AMR_APPLY_TRANSLATION_DEADZONE");
  if (argc != 3 || std::string(argv[1]) != "--apply-translation" ||
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
  int port = 0;
  int expected = 0;
  int target = 0;
  int rotation = 0;
  try
  {
    port = integer(input[1]);
    expected = integer(input[5]);
    target = integer(input[6]);
    rotation = integer(input[7]);
    amr_core::translationDeadzoneRequest(expected, target);
    if (port < 1 || port > 65535 || input[0].empty() || input[4].empty() || rotation < 1 || rotation > 99 ||
        input[8] != "APPLY_TRANSLATION_DEADZONE")
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
      amr_calibration::changeTranslationDeadzone(client, expected, target, rotation, report);
    });
  }
  catch (const std::exception&)
  {
    report["failure_reason"] = "Connection, preflight, receipt, or read-back failed. No retry.";
  }
  std::cout << "CALIBRATION_RESULT " << report.dump() << std::endl;
  return report.at("complete").get<bool>() ? 0 : 1;
}