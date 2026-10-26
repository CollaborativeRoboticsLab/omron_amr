#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace amr_hardware
{
inline std::vector<std::uint8_t> sectionRequest(const std::string& section)
{
  if (section != "Robot config" && section != "Teleop Settings")
  {
    throw std::invalid_argument("Configuration section is not allowlisted");
  }
  std::vector<std::uint8_t> request{ 1, 4 };
  request.insert(request.end(), section.begin(), section.end());
  request.push_back(0);
  request.push_back(0);
  return request;
}

inline const std::set<std::string> driveParameters{
  "FullThrottleForwards", "FullThrottleBackwards", "RotAtStopped", "RotAtFullForwards", "RotAtFullBackwards",
  "TransDeadZone", "RotDeadZone", "TransVelMax", "TransNegVelMax", "TransAccel", "TransDecel",
  "RotVelMax", "RotAccel", "RotDecel", "TeleopTransVelMax", "TeleopTransNegVelMax", "TeleopTransAccel",
  "TeleopTransDecel", "TeleopRotVelMax", "TeleopRotAccel", "TeleopRotDecel", "Timeout"
};

class ConfigCursor
{
public:
  explicit ConfigCursor(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}

  std::size_t remaining() const { return bytes_.size() - offset_; }
  std::size_t offset() const { return offset_; }

  std::uint64_t number(std::size_t width)
  {
    if (width > 8 || remaining() < width)
    {
      throw std::runtime_error("Truncated configuration number");
    }
    std::uint64_t value = 0;
    for (std::size_t index = 0; index < width; ++index)
    {
      value |= std::uint64_t(bytes_[offset_++]) << (8 * index);
    }
    return value;
  }

  bool flag(const char* field = "value")
  {
    const auto value = number(1);
    if (value > 1)
    {
      throw std::runtime_error(std::string("Invalid configuration ") + field + " flag at body offset " +
               std::to_string(offset_ - 1));
    }
    return value != 0;
  }

  std::string text()
  {
    const auto begin = bytes_.begin() + offset_;
    const auto limit = begin + std::min<std::size_t>(remaining(), 16385);
    const auto end = std::find(begin, limit, 0);
    if (end == limit)
    {
      throw std::runtime_error("Unterminated or oversized configuration string");
    }
    offset_ += std::distance(begin, end) + 1;
    return std::string(begin, end);
  }

  nlohmann::json scalar(char type)
  {
    switch (type)
    {
      case 'b': return flag();
      case 'i':
      {
        const auto bits = static_cast<std::uint32_t>(number(4));
        std::int32_t value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
      }
      case 'o':
      {
        const auto bits = number(8);
        std::int64_t value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
      }
      case '8': return number(8);
      case 'd':
      {
        const auto encoded = text();
        if (encoded == "INF" || encoded == "-INF")
        {
          return encoded;
        }
        double value = 0.0;
        const auto parsed = std::from_chars(encoded.data(), encoded.data() + encoded.size(), value);
        if (parsed.ec != std::errc{} || parsed.ptr != encoded.data() + encoded.size() || !std::isfinite(value))
        {
          throw std::runtime_error("Invalid configuration floating-point value");
        }
        return value;
      }
      case 's': return text();
      case '.': return nullptr;
      default: throw std::runtime_error("Unsupported configuration scalar type");
    }
  }

private:
  const std::vector<std::uint8_t>& bytes_;
  std::size_t offset_ = 0;
};

inline nlohmann::json parameterRecord(ConfigCursor& cursor)
{
  nlohmann::json parameter;
  parameter["name"] = cursor.text();
  cursor.text();
  parameter["priority"] = cursor.number(1);
  const char type = static_cast<char>(cursor.number(1));
  parameter["wire_type"] = std::string(1, type);
  parameter["value"] = cursor.scalar(type);
  if (type == 'i' || type == 'o' || type == '8')
  {
    parameter["minimum"] = cursor.scalar(type);
    parameter["maximum"] = cursor.scalar(type);
  }
  else if (type == 'd')
  {
    parameter["minimum_text"] = cursor.text();
    parameter["maximum_text"] = cursor.text();
    cursor.number(4);
  }
  cursor.text();
  cursor.text();
  cursor.number(1);
  cursor.flag("serializable");
  const auto parent = cursor.text();
  if (!parent.empty())
  {
    throw std::runtime_error("Nested configuration parameters are unsupported");
  }
  const auto integer_type = cursor.number(1);
  if (type == 'i' && (integer_type == 5 || integer_type == 6))
  {
    throw std::runtime_error("Inconsistent configuration integer type");
  }
  cursor.text();
  if (cursor.flag("metadata-present"))
  {
    parameter["scope_code"] = cursor.number(1);
    const bool first_present = cursor.flag("source-1-present");
    if (first_present)
    {
      parameter["source_value_1"] = cursor.scalar(type);
    }
    else if (!cursor.text().empty())
    {
      throw std::runtime_error("Invalid absent source value placeholder");
    }
    const bool second_present = cursor.flag("source-2-present");
    if (second_present)
    {
      parameter["source_value_2"] = cursor.scalar(type);
    }
    else if (!cursor.text().empty())
    {
      throw std::runtime_error("Invalid absent source value placeholder");
    }
    parameter["metadata_code_1"] = cursor.number(1);
    parameter["metadata_code_2"] = cursor.number(1);
  }
  return parameter;
}

class SectionTransfer
{
public:
  explicit SectionTransfer(std::string section) : section_(std::move(section))
  {
    sectionRequest(section_);
  }

  bool complete() const { return expected_chunks_ != 0 && next_chunk_ == expected_chunks_; }

  void accept(const std::vector<std::uint8_t>& packet)
  {
    if (failed_)
    {
      throw std::runtime_error("Configuration transfer already failed");
    }
    try
    {
      ConfigCursor cursor(packet);
      if (complete() || cursor.number(1) != 1 || cursor.number(1) != 4 || cursor.text() != section_ ||
          cursor.number(1) != 0)
      {
        throw std::runtime_error("Unexpected configuration reply identity");
      }
      const auto chunks = cursor.number(2);
      const auto index = cursor.number(2);
      const auto total = cursor.number(8);
      const auto length = cursor.number(8);
      if (chunks == 0 || chunks > 128 || index != next_chunk_ || index >= chunks || total < 4 || total > 1048576 ||
          length != cursor.remaining() || length > total || body_.size() > total - length ||
          (expected_chunks_ != 0 && (chunks != expected_chunks_ || total != expected_bytes_)))
      {
        throw std::runtime_error("Invalid configuration chunk sequence or length");
      }
      expected_chunks_ = chunks;
      expected_bytes_ = total;
      body_.insert(body_.end(), packet.begin() + cursor.offset(), packet.end());
      ++next_chunk_;
      if (complete() && body_.size() != expected_bytes_)
      {
        throw std::runtime_error("Incomplete configuration body");
      }
    }
    catch (const std::exception&)
    {
      failed_ = true;
      throw;
    }
  }

  nlohmann::json parameters() const
  {
    if (failed_ || !complete())
    {
      throw std::runtime_error("Configuration section transfer did not complete");
    }
    ConfigCursor cursor(body_);
    const auto count = cursor.number(4);
    if (count == 0 || count > 2048)
    {
      throw std::runtime_error("Invalid configuration parameter count");
    }
    auto result = nlohmann::json::array();
    std::set<std::string> names;
    for (std::size_t index = 0; index < count; ++index)
    {
      if (cursor.number(1) != 'P')
      {
        throw std::runtime_error("Unsupported configuration record tag");
      }
      auto parameter = parameterRecord(cursor);
      const auto name = parameter.at("name").get<std::string>();
      if (!names.insert(name).second)
      {
        throw std::runtime_error("Duplicate configuration parameter");
      }
      if (driveParameters.count(name) != 0)
      {
        if (!parameter.at("value").is_number())
        {
          throw std::runtime_error("Drive configuration parameter is not numeric");
        }
        parameter["section"] = section_;
        result.push_back(std::move(parameter));
      }
    }
    if (cursor.remaining() != 0)
    {
      throw std::runtime_error("Trailing configuration section data");
    }
    return result;
  }

private:
  std::string section_;
  std::vector<std::uint8_t> body_;
  std::size_t expected_chunks_ = 0;
  std::size_t next_chunk_ = 0;
  std::size_t expected_bytes_ = 0;
  bool failed_ = false;
};
}