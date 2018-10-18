#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#ifdef _WIN32
#include <process.h>
#define GETPID _getpid
#else
#include <unistd.h>
#define GETPID getpid
#endif

#define RESERVE(x) ( (void)x )

#include <chrono>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

/*
void set_now(builtin_interfaces::msg::Time & time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0)) {
    time.sec = time.nanosec = 0;
  } else {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}
*/

#endif  // _COMMON_HPP_
