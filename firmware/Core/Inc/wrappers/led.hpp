#pragma once

#include "BTS72220.hpp"
#include "main.h"
#include <cstdint>

/* Led blinking speed indicates each IC channels status:
  OFF - all channels OK
  ON - all channels ERROR
*/
class Led {
private:
  GPIO_TypeDef *port;
  const uint16_t pin;

  uint32_t toggle_time{};
  uint32_t last_toggle{};

public:
  Led(GPIO_TypeDef *new_port, uint16_t new_pin) : port{new_port}, pin{new_pin} {
    if (!port) {
      Error_Handler();
    }
  }

  /* Update single Led state based on failed channels count
   */
  bool update(uint8_t channels_failed, uint32_t tick_now) {
    if (channels_failed > BTS::Ic::CHANNEL_COUNT) {
      return true;
    } else if (channels_failed == BTS::Ic::CHANNEL_COUNT) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
      return false;
    } else if (channels_failed == 0) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
      return false;
    } else {
      toggle_time = 3000 / channels_failed;
      if (tick_now - last_toggle >= toggle_time) {
        last_toggle = tick_now;
        HAL_GPIO_TogglePin(port, pin);
      }
      return false;
    }
  }
};
