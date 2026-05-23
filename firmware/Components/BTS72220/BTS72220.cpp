#include "BTS72220.hpp"

namespace BTS {

void Channel::update_current(uint32_t current_value, uint32_t tick_now) {
  current = current_value;
  switch (status) {
  case Status::ON: {
    if (current > threshold) {
      tick_last_attempt = tick_now;
      status = Status::ERR;
    } else {
      // Zero retry_count after delay
    }
    break;
  }
  default:
    break;
  }
  return;
}

bool Channel::handle_overcurrent(uint32_t tick_now) {
  constexpr uint32_t time_per_attempt_ms{5000};

  switch (status) {
  case Status::ON: {
    return false;
  }

  case Status::OFF:
  case Status::ERR: {
    if (retry_count >= MAX_RETRIES) {
      status = Status::PERM_LOCK;
    } else {
      tick_last_attempt = tick_now;
      status = Status::TEMP_LOCK;
      retry_count++;
    }
    return true;
  }

  case Status::TEMP_LOCK: {
    uint32_t time_since = tick_now - tick_last_attempt;
    if (time_since > (retry_count * time_per_attempt_ms)) {
      status = Status::ON;
      return false;
    }
    return true;
  }

  case Status::PERM_LOCK: {
    return true;
  }
  }
  return true;
}

bool Ic::check_response(uint8_t rx_value) {
  if ((rx_value & DIAG_MASK) == WRNDIAG_MASK) {
    Wrndiag wrndiag{rx_value};
    (void)wrndiag; // DECODE
    return false;
  } else if ((rx_value & DIAG_MASK) == STDDIAG_MASK) {
    Stddiag stddiag{rx_value};
    if (stddiag.reg.TER) {
      status = Status::SLEEP;
      return true;
    }
    return false;
  } else
    return true;
}

bool Ic::check_err(uint8_t rx_value) {
  if ((rx_value & DIAG_MASK) == ERRDIAG_MASK) {
    Errdiag errdiag{rx_value};
    if (errdiag.reg.ERRn == 0)
      return false;

    for (std::size_t ch_idx{}; ch_idx < CHANNEL_COUNT; ch_idx++) {
      if (errdiag.reg.ERRn & (1 << ch_idx))
        channels.at(ch_idx).status = Channel::Status::ERR;
    }
    return true;
  }
  return false;
}
} // namespace BTS