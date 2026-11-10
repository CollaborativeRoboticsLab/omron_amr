#pragma once

#include <Aria/Aria.h>
#include <ArNetworking/ArNetworking.h>

#include <cstdlib>
#include <mutex>

namespace amr_core
{
namespace detail
{
inline std::mutex& ariaMutex()
{
  static std::mutex mutex;
  return mutex;
}

inline int& ariaRefCount()
{
  static int ref_count = 0;
  return ref_count;
}

inline void configureAriaDirectory()
{
  if (std::getenv("ARIA") != nullptr)
  {
    return;
  }

#ifdef AMR_CORE_ARIA_DIR
  ::setenv("ARIA", AMR_CORE_ARIA_DIR, 0);
#endif
}
}  // namespace detail

class LibAriaRuntime
{
public:
  enum class Logging { Normal, Silent };

  explicit LibAriaRuntime(Logging logging = Logging::Normal)
  {
    std::lock_guard<std::mutex> lock(detail::ariaMutex());
    if (detail::ariaRefCount() == 0)
    {
      detail::configureAriaDirectory();
      if (logging == Logging::Silent)
      {
        ArLog::init(ArLog::None, ArLog::Terse, "", false, false, false);
      }
      Aria::init();
      if (logging == Logging::Normal)
      {
        ArLog::init(ArLog::StdOut, ArLog::Normal);
      }
    }
    ++detail::ariaRefCount();
  }

  ~LibAriaRuntime()
  {
    std::lock_guard<std::mutex> lock(detail::ariaMutex());
    if (detail::ariaRefCount() == 0)
    {
      return;
    }

    --detail::ariaRefCount();
    if (detail::ariaRefCount() == 0)
    {
      Aria::shutdown();
    }
  }

  LibAriaRuntime(const LibAriaRuntime&) = delete;
  LibAriaRuntime& operator=(const LibAriaRuntime&) = delete;
};
}  // namespace amr_core