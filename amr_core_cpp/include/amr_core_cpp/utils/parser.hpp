#pragma once

#include <string>

/**
 * @brief Parses ARCL server messages and classifies them.
 *
 * Returns one of:
 *  - PASS (0)
 *  - FAIL (1)
 *  - CONTINUE (2)
 * Along with a human-readable message.
 */
class Parser
{
public:
  enum class Code : int
  {
    PASS = 0,
    FAIL = 1,
    CONTINUE = 2
  };

  struct Result
  {
    Code code;
    std::string message;
  };

  Parser() = default;

  /**
   * @brief Process a single ARCL server line and classify it.
   * @param input The input message.
   * @return Result classification and message.
   */
  Result process_arcl_server(const std::string& input)
  {
    if (input.find(" Docked") != std::string::npos)
    {
      return { Code::PASS, "Docked successfully." };
    }
    else if (input.find("Localized to point") != std::string::npos)
    {
      return { Code::PASS, "Localization complete." };
    }
    else if (input.find("Completed doing task") != std::string::npos)
    {
      return { Code::PASS, "Reached point" };
    }
    else if (input.find("Completed macro") != std::string::npos)
    {
      return { Code::PASS, "Macro execution complete" };
    }
    else if (input.find("Stopped") != std::string::npos)
    {
      return { Code::FAIL, "Command was interrupted." };
    }
    else if (input.find("Failed going to goal") != std::string::npos)
    {
      return { Code::FAIL, "Failed going to goal." };
    }
    else if (input.find("Interrupted") != std::string::npos)
    {
      return { Code::FAIL, "Interrupted." };
    }
    else if (input.find("Failed") != std::string::npos)
    {
      return { Code::FAIL, "Failed" };
    }
    else if (input.find("Error") != std::string::npos)
    {
      return { Code::FAIL, std::string("An Error has occurred:") + input };
    }
    else if (input.find("Going to") != std::string::npos)
    {
      return { Code::CONTINUE, input };
    }
    else if (input.find(" Docking ") != std::string::npos)
    {
      return { Code::CONTINUE, "Docking." };
    }
    else if (input.find("Will do task") != std::string::npos)
    {
      return { Code::CONTINUE, input };
    }
    else if (input.find("Executing macro") != std::string::npos)
    {
      return { Code::CONTINUE, input };
    }
    else
    {
      return { Code::CONTINUE, input };
    }
  }
};