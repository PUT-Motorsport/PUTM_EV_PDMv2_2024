#pragma once
#include <cstdint>
#include <gmock/gmock.h>

class Led {
public:
  MOCK_METHOD(bool, update, (uint8_t channels_failed, uint32_t tick_now));
};
