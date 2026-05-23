#pragma once

#include "BTS72220.hpp"
#include "PUTM_CAN_M.h"
#include "main.h"

#include <algorithm>
#include <array>
#include <span>

enum class System_name {
  INV2,
  INV1,
  RBOX_DIAG_BRAKE_L,
  TSAL_HV,

  DASH,
  SDC_ASMS,
  BRAKE_IR_AIR,
  FBOX,

  PC3,
  PC2,
  PC1,
  PC0,

  FAN2,
  PUMP2,
  PUMP1,
  FAN1,

  COUNT,
};

struct System {
  System_name name;
  uint16_t threshold;
};

/* Led blinking indicates each IC channels status:
  OFF - all channels OK



  ON - all channels ERROR
*/
class Led {
public:
  Led(GPIO_TypeDef *port, uint16_t pin) : port{port}, pin{pin} {};

  // Update single Led state based on failed channels count
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

private:
  GPIO_TypeDef *port;
  uint16_t pin;

  uint32_t toggle_time{};
  uint32_t last_toggle{};
};

class Temperature {
public:
  const uint8_t min;
  const uint8_t max;
  struct Values {
    uint8_t front_left{};
    uint8_t front_right{};
    uint8_t rear_left{};
    uint8_t rear_right{};
  };

  Temperature(const uint8_t min_temperature, const uint8_t max_temperature)
      : min{min_temperature}, max{max_temperature} {}

  bool check(uint8_t value) { return (value < min || value > max); }

  bool update(Values new_values) {
    values.front_left = new_values.front_left;
    values.front_right = new_values.front_right;
    values.rear_left = new_values.rear_left;
    values.rear_right = new_values.rear_right;

    if (check(values.front_left) || check(values.front_right) ||
        check(values.rear_left) || check(values.rear_right))
      return true;

    return false;
  }

private:
  Values values{};
};

// Base class that controls all ICs
class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  Pdu(std::array<Led, IC_COUNT> leds,
      const std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                       Pdu::IC_COUNT> &systems_data,
      Temperature inv_temperature, Temperature motor_temperature);

  const BTS::Channel &get_channel(System_name name) const;
  uint16_t get_total_current() const;
  PUTM_CAN_M_pdu_channnel_t get_can_pdu_channel_t() const;
  PUTM_CAN_M_pdu_data_t get_can_pdu_data_t() const;

  bool update_leds(uint32_t tick_now);
  bool update_fans(bool rtd_status, Temperature::Values inv_values,
                   Temperature::Values motor_values);
                   
  std::array<uint8_t, IC_COUNT>
  chain_tx_rx(const std::array<uint8_t, IC_COUNT> &tx);
  bool check_chain_responses(std::array<uint8_t, IC_COUNT> rx);
  bool check_chain_errors();
  bool init_chain();
  bool start_chain();
  bool set_channel_sense(uint8_t channel);
  void update_channel_currents(uint8_t channel,
                               const std::array<uint16_t, IC_COUNT> &voltages,
                               uint32_t tick_now);
  bool handle_overcurrent(uint32_t tick_now);

private:
  std::array<Led, IC_COUNT> leds;
  std::array<BTS::Ic, IC_COUNT> ics;
  std::array<size_t, static_cast<size_t>(System_name::COUNT)>
      systems_channel_map;

  Temperature inv_temperature;
  Temperature motor_temperature;
};