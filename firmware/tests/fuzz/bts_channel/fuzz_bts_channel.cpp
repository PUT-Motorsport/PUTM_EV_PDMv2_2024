#include "BTS72220.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

/*
Stateful fuzzer for BTS::Channel overcurrent state machine.

Input layout (little-endian, no alignment or packing assumptions):
  [0..1]  uint16 current threshold in mA
  then a sequence of operations, each starting with one opcode byte
  (opcode % OP_COUNT):
    UPDATE_CURRENT      + uint32 current in mA (uses current tick)
    ADVANCE_AND_HANDLE  + uint16 time delta in ms, then handle_overcurrent()
    TURN_ON
    TURN_OFF
    FORCE_ERR           update_status(ERR), as done by Ic::check_err()
Truncated trailing payload ends the sequence.

After every operation the harness checks state machine invariants and aborts
on violation, so libFuzzer reports the input as a crash.
*/

namespace {

using Status = BTS::Channel::Status;

enum Op : uint8_t {
  UPDATE_CURRENT,
  ADVANCE_AND_HANDLE,
  TURN_ON,
  TURN_OFF,
  FORCE_ERR,
  OP_COUNT,
};

class Reader {
public:
  Reader(const uint8_t *data, size_t size) : data{data}, size{size} {}

  bool read_u8(uint8_t &out) {
    if (pos + 1 > size)
      return false;
    out = data[pos++];
    return true;
  }

  bool read_u16(uint16_t &out) {
    if (pos + 2 > size)
      return false;
    out = static_cast<uint16_t>(static_cast<uint16_t>(data[pos]) |
                                (static_cast<uint16_t>(data[pos + 1]) << 8));
    pos += 2;
    return true;
  }

  bool read_u32(uint32_t &out) {
    if (pos + 4 > size)
      return false;
    out = static_cast<uint32_t>(data[pos]) |
          (static_cast<uint32_t>(data[pos + 1]) << 8) |
          (static_cast<uint32_t>(data[pos + 2]) << 16) |
          (static_cast<uint32_t>(data[pos + 3]) << 24);
    pos += 4;
    return true;
  }

private:
  const uint8_t *data;
  size_t size;
  size_t pos{};
};

const char *to_str(Status status) {
  switch (status) {
  case Status::OFF:
    return "OFF";
  case Status::PERM_LOCK:
    return "PERM_LOCK";
  case Status::TEMP_LOCK:
    return "TEMP_LOCK";
  case Status::ERR:
    return "ERR";
  case Status::ON:
    return "ON";
  }
  return "?";
}

[[noreturn]] void fail(const char *what, Status before, Status after) {
  std::fprintf(stderr, "Invariant violated: %s (status %s -> %s)\n", what,
               to_str(before), to_str(after));
  __builtin_trap();
}

#define CHECK(cond, what)                                                      \
  do {                                                                         \
    if (!(cond))                                                               \
      fail(what, before, after);                                               \
  } while (0)

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  Reader in{data, size};

  uint16_t threshold_mA{};
  if (!in.read_u16(threshold_mA))
    return 0;

  BTS::Channel channel{};
  channel.set_threshold(threshold_mA);

  uint32_t tick_now{};
  // Shadow state used by invariants
  uint32_t temp_lock_entries{};
  uint32_t temp_lock_tick{};

  uint8_t opcode{};
  while (in.read_u8(opcode)) {
    const Status before{channel.get_status()};

    switch (static_cast<Op>(opcode % OP_COUNT)) {
    case UPDATE_CURRENT: {
      uint32_t current_mA{};
      if (!in.read_u32(current_mA))
        return 0;
      channel.update_current(current_mA, tick_now);
      const Status after{channel.get_status()};

      if (before == Status::ON) {
        if (current_mA > threshold_mA)
          CHECK(after == Status::ERR,
                "ON channel with current over threshold must go to ERR");
        else
          CHECK(after == Status::ON,
                "ON channel with current under threshold must stay ON");
      } else {
        CHECK(after == before,
              "update_current must not change status of non-ON channel");
      }
      break;
    }

    case ADVANCE_AND_HANDLE: {
      uint16_t delta_ms{};
      if (!in.read_u16(delta_ms))
        return 0;
      tick_now += delta_ms; // wrap-around is intended, like HAL_GetTick()
      channel.handle_overcurrent(tick_now);
      const Status after{channel.get_status()};

      CHECK(after != Status::ERR, "ERR must be resolved by handle_overcurrent");

      if (before == Status::ERR && after == Status::TEMP_LOCK) {
        temp_lock_entries++;
        temp_lock_tick = tick_now;
      }
      if (before == Status::TEMP_LOCK && after == Status::ON) {
        CHECK(tick_now - temp_lock_tick > BTS::Channel::TIME_PER_ATTEMPT_MS,
              "TEMP_LOCK released before retry delay elapsed");
      }
      if (before != Status::PERM_LOCK && after == Status::PERM_LOCK) {
        CHECK(temp_lock_entries >= BTS::Channel::MAX_RETRIES,
              "PERM_LOCK entered before MAX_RETRIES attempts");
      }
      CHECK(before != Status::OFF || after == Status::OFF,
            "handle_overcurrent must not change OFF channel");
      break;
    }

    case TURN_ON: {
      channel.turn_on();
      const Status after{channel.get_status()};
      if (before == Status::ON || before == Status::OFF)
        CHECK(after == Status::ON, "turn_on must enable ON/OFF channel");
      else
        CHECK(after == before, "turn_on must not unlock locked/ERR channel");
      break;
    }

    case TURN_OFF: {
      channel.turn_off();
      const Status after{channel.get_status()};
      if (before == Status::PERM_LOCK)
        CHECK(after == Status::PERM_LOCK, "turn_off must not leave PERM_LOCK");
      else
        CHECK(after == Status::OFF, "turn_off must switch channel OFF");
      break;
    }

    case FORCE_ERR: {
      channel.update_status(Status::ERR);
      const Status after{channel.get_status()};
      if (before == Status::PERM_LOCK)
        CHECK(after == Status::PERM_LOCK, "PERM_LOCK must be permanent");
      else
        CHECK(after == Status::ERR, "update_status(ERR) must set ERR");
      break;
    }

    case OP_COUNT:
      break;
    }

    const Status after{channel.get_status()};
    CHECK(before != Status::PERM_LOCK || after == Status::PERM_LOCK,
          "PERM_LOCK must be permanent");
  }

  return 0;
}
