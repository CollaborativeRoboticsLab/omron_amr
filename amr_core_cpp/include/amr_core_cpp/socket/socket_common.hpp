#pragma once
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>

#include "amr_core_cpp/amr_exception.hpp"

namespace amr::net
{

// Set non-blocking on a fd
inline void set_nonblocking(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    throw amr_exception(std::string("fcntl(O_NONBLOCK) failed: ") + std::strerror(errno));
  }
}

// Close fd if valid and reset ref
inline void close_fd(int& fd)
{
  if (fd >= 0)
  {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
    fd = -1;
  }
}

// Read SO_ERROR from a connected socket
inline int get_so_error(int fd)
{
  int err = 0;
  socklen_t len = sizeof(err);
  if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) < 0)
    return errno;
  return err;
}

// Drain recv into buffer (append), non-blocking; returns true if peer closed
inline bool recv_append_nb(int fd, std::string& buf)
{
  char tmp[4096];
  for (;;)
  {
    ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
    if (n > 0)
    {
      buf.append(tmp, static_cast<size_t>(n));
    }
    else if (n == 0)
    {
      return true;  // peer closed
    }
    else
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return false;
      throw amr_exception(std::string("recv() failed: ") + std::strerror(errno));
    }
  }
}

// Send from buffer, erase sent prefix, non-blocking; returns false if would-block
inline bool send_erase_nb(int fd, std::string& buf)
{
  while (!buf.empty())
  {
    ssize_t n = ::send(fd, buf.data(), buf.size(), 0);
    if (n > 0)
    {
      buf.erase(0, static_cast<size_t>(n));
    }
    else if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return false;
      throw amr_exception(std::string("send() failed: ") + std::strerror(errno));
    }
    else
    {
      // n == 0: treat as closed
      throw amr_exception("send() returned 0 (connection closed)");
    }
  }
  return true;
}

// Extract CRLF-terminated lines, without CRLF
inline size_t extract_crlf_lines(std::string& buf, std::vector<std::string>& out)
{
  size_t count = 0;
  for (;;)
  {
    auto pos = buf.find("\r\n");
    if (pos == std::string::npos)
      break;
    out.emplace_back(buf.substr(0, pos));
    buf.erase(0, pos + 2);
    ++count;
  }
  return count;
}

}  // namespace amr::net