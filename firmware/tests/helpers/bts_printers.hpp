#pragma once

#include "BTS72220.hpp"

#include <ostream>

namespace BTS {

// GoogleTest prints channel status by name instead of its numeric value
inline void PrintTo(Channel::Status status, std::ostream *os) {
  switch (status) {
  case Channel::Status::OFF:
    *os << "OFF";
    return;
  case Channel::Status::PERM_LOCK:
    *os << "PERM_LOCK";
    return;
  case Channel::Status::TEMP_LOCK:
    *os << "TEMP_LOCK";
    return;
  case Channel::Status::ERR:
    *os << "ERR";
    return;
  case Channel::Status::ON:
    *os << "ON";
    return;
  }
  *os << "Status(" << static_cast<int>(status) << ")";
}

} // namespace BTS
