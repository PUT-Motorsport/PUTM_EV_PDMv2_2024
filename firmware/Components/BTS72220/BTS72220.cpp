#include "BTS72220.hpp"

namespace BTS72220 {

bool Channel::update_current(uint16_t current_value) {
  current = current_value;
  if (current > threshold && status != Status::LOCK) {
    status = Status::ERR;
    return true;
  }
  return false;
}
} // namespace BTS72220