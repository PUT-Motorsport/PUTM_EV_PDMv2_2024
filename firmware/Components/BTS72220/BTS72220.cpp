#include "BTS72220.hpp"

namespace BTS72220 {

void Channel::update_current(uint32_t current_value, uint32_t tick_now) {
  current = current_value;
  switch (status) {
  case Status::ON: {
    if (current > threshold) {
      status = Status::ERR;
    }
    break;
  }
  case Status::TEMP_LOCK: {
    const uint32_t time_per_attempt_ms{5000};
    uint32_t time_since = tick_now - tick_last_attempt;
    if (retry_count >= MAX_RETRIES) {
      status = Status::PERM_LOCK;
    } else if (time_since > ((retry_count + 1) * time_per_attempt_ms)) {
      retry_count++;
      status = Status::ON;
    }
    break;
  }
  default:
    break;
  }
  return;
}

bool Ic::check_diag(uint8_t rx_value) {
  if (rx_value >> 6) {
    WRNDIAG wrndiag{rx_value};
    // DECODE
    return true;
  } else {
    STDDIAG stddiag{rx_value};
    // DECODE
    return false;
  }
}
} // namespace BTS72220