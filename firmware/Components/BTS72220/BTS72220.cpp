#include "BTS72220.hpp"

namespace BTS {

/*
Update current for ON channels, transition to ERR if current > threshold
*/
void Channel::update_current(uint32_t i_val_mA, uint32_t tick_now) {
  i_mA = i_val_mA;
  switch (status) {
  case Status::ON: {
    if (i_mA > i_threshold_mA) {
      tick_last_attempt = tick_now;
      update_status(Status::ERR);
      break;
    }
  }
  default:
    break;
  }
}

/*
Updates channel status with check for permanent channel lock
*/
bool Channel::update_status(Status new_status) {
  if (status == Status::PERM_LOCK)
    return true;

  status = new_status;
  return false;
}
/*
If channel status:
- ON - reset retry count after delay
- OFF - do nothing
- ERR - transition to TEMP_LOCK or PERM_LOCK based on retry count
- TEMP_LOCK - unlock channel after delay
- PERM_LOCK - do nothing
Returns if state other than ON
*/
bool Channel::handle_overcurrent(uint32_t tick_now) {
  constexpr uint32_t time_per_attempt_ms{5000};

  switch (status) {
  case Status::ON: {
    // Reset retry count after delay
    uint32_t time_since = tick_now - tick_last_attempt;
    if (time_since > (retry_count * time_per_attempt_ms)) {
      retry_count = 0;
    }
    return false;
  }

  case Status::OFF:
    return true;

  case Status::ERR: {
    if (retry_count >= MAX_RETRIES) {
      update_status(Status::PERM_LOCK);
    } else {
      tick_last_attempt = tick_now;
      update_status(Status::TEMP_LOCK);
      retry_count++;
    }
    return true;
  }

  case Status::TEMP_LOCK: {
    uint32_t time_since = tick_now - tick_last_attempt;
    if (time_since > (retry_count * time_per_attempt_ms)) {
      tick_last_attempt = tick_now;
      update_status(Status::ON);
      return false;
    }
    return true;
  }

  case Status::PERM_LOCK:
    return true;
  }
  return true;
}

/*
Works only if channel was turned OFF manually
*/
bool Channel::turn_on() {
  switch (status) {
  case Status::OFF:
  case Status::ON: {
    update_status(Status::ON);
    return false;
  }
  default:
    return true;
  }
}
/*
Turn off unconditionally
*/
void Channel::turn_off() { update_status(Status::OFF); }

/*
Decode STDDIAG and WRNDIAG registers, returns true if frame doesn't match
pattern
*/
bool Ic::check_response(uint8_t rx_value) {
  if ((rx_value & DIAG_MASK) == WRNDIAG_MASK) {
    Wrndiag wrndiag{rx_value};
    (void)wrndiag; // DECODE
    return false;
  } else if ((rx_value & DIAG_MASK) == STDDIAG_MASK) {
    Stddiag stddiag{rx_value};
    if (stddiag.reg.TER) {
      status = Status::SLEEP;
    }
    return false;
  } else
    return true;
}

/*
Decode ERRDIAG register, transition to ERR state based on ERRn bits,
returns true if frame doesn't match pattern
*/
bool Ic::check_err(uint8_t rx_value) {
  if ((rx_value & DIAG_MASK) == ERRDIAG_MASK) {
    Errdiag errdiag{rx_value};
    for (std::size_t ch_idx{}; ch_idx < CHANNEL_COUNT; ch_idx++) {
      if (errdiag.reg.ERRn & (1 << ch_idx))
        channels.at(ch_idx).update_status(Channel::Status::ERR);
    }
    return false;
  }
  return true;
}
} // namespace BTS