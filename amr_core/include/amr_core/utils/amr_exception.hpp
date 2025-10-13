#pragma once

#include <stdexcept>
#include <string>

/**
 * @brief Custom exception class for AMR-related errors.
 */
struct amr_exception : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  explicit amr_exception(const std::string& what_arg) : std::runtime_error(what_arg)
  {
  }

  const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};