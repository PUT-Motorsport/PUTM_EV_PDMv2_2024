#include "BTS72220.hpp"

namespace BTS72220 {

void Channel::update_current(uint32_t current_value, uint32_t tick_now) {
  current = current_value;
  switch (status) {
  case Status::ON: {
    if (current > threshold) {
      tick_last_attempt = tick_now;
      status = Status::ERR;
    }
    break;
  }
  default:
    break;
  }
  return;
}

bool Channel::handle_overcurrent(uint32_t tick_now) {
  const uint32_t time_per_attempt_ms{5000};

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
    if (time_since > ((retry_count + 1) * time_per_attempt_ms)) {
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
  if (rx_value >> 6) {
    WRNDIAG wrndiag{rx_value};
    (void)wrndiag; // DECODE
    return false;
  } else {
    STDDIAG stddiag{rx_value};
    if (stddiag.reg.TER) {
      status = Status::SLEEP;
      return true;
    }
    return false;
  }
}

bool Ic::check_err(uint8_t rx_value) {
  if (rx_value >> 6) {
    ERRDIAG errdiag{rx_value};
    if (errdiag.reg.ERRn == 0)
      return false;

    int channel_count{0};
    for (auto &channel : channels) {
      if (errdiag.reg.ERRn & (1 << channel_count))
        channel.status = Channel::Status::ERR;
      channel_count++;
    }
    return true;
  }
  return false;
}
} // namespace BTS72220