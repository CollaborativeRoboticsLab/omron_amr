#pragma once

#include <ArNetworking/ArNetworking.h>

#include <algorithm>
#include <cstddef>
#include <string>

namespace amr_core
{
class MapStream
{
public:
  bool accept(ArNetPacket* packet)
  {
    ++packets_;
    if (failed_ || !packet || !packet->isValid())
      return fail();
    if (packet->getDataLength() == packet->getDataReadLength()) return complete_ ? true : fail();
    if (packet->getDataLength() < packet->getDataReadLength()) return fail();
    const auto* begin = packet->getBuf() + packet->getReadLength();
    const auto* end = begin + packet->getDataLength() - packet->getDataReadLength();
    last_bytes_ = static_cast<std::size_t>(end - begin);
    last_terminators_ = static_cast<std::size_t>(std::count(begin, end, '\0'));
    const auto* terminator = std::find(begin, end, '\0');
    if (terminator != end - 1) return fail();
    if (complete_) return terminator == begin ? true : fail();
    if (terminator == begin)
    {
      if (content_.empty()) return fail();
      complete_ = true;
      return true;
    }
    const auto length = static_cast<std::size_t>(terminator - begin);
    if (length + 1 > max_size_ - content_.size() || std::find(begin, terminator, '\n') != terminator ||
        std::find(begin, terminator, '\r') != terminator)
      return fail();
    content_.append(begin, terminator);
    content_.push_back('\n');
    ++lines_;
    return true;
  }

  bool complete() const { return complete_ && !failed_; }
  bool failed() const { return failed_; }
  std::size_t lines() const { return lines_; }
  std::size_t packets() const { return packets_; }
  std::size_t lastBytes() const { return last_bytes_; }
  std::size_t lastTerminators() const { return last_terminators_; }
  const std::string& content() const { return content_; }

private:
  bool fail()
  {
    failed_ = true;
    return false;
  }
  static constexpr std::size_t max_size_ = 64 * 1024 * 1024;
  std::string content_;
  std::size_t lines_{ 0 };
  std::size_t packets_{ 0 };
  std::size_t last_bytes_{ 0 };
  std::size_t last_terminators_{ 0 };
  bool complete_{ false };
  bool failed_{ false };
};
}  // namespace amr_core