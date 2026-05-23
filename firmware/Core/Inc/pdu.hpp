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
  bool update(uint8_t channels_failed, uint32_t tick_now);

private:
  GPIO_TypeDef *port;
  const uint16_t pin;

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
  enum class Status : uint8_t {
    OK,
    TOO_LOW,
    TOO_HIGH,
  };

  Temperature(uint8_t min_temperature, uint8_t max_temperature)
      : min{min_temperature}, max{max_temperature} {}

  Status check(uint8_t value) const;

  void update(Values new_values);

  Status is_ok() const;

private:
  struct {
    Status front_left;
    Status front_right;
    Status rear_left;
    Status rear_right;
  } status;
};

// Base class that controls all ICs
class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  Pdu(std::array<Led, IC_COUNT> leds,
      const std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                       Pdu::IC_COUNT> &systems_data,
      Temperature inv_temperature, Temperature motor_temperature);

  uint16_t get_total_current() const;
  PUTM_CAN_M_pdu_channnel_t get_can_pdu_channel_t();
  PUTM_CAN_M_pdu_data_t get_can_pdu_data_t();

  bool update_leds(uint32_t tick_now);
  void update_fans(const bool &rtd_status,
                   const Temperature::Values &inv_values,
                   const Temperature::Values &motor_values);

  bool update_chain_diag(std::array<uint8_t, IC_COUNT> rx);
  bool update_chain_errors();
  bool init_chain();
  bool start_chain();
  bool set_channel_sense(uint8_t channel);
  void update_channel_currents(uint8_t channel,
                               const std::array<uint16_t, IC_COUNT> &voltages,
                               uint32_t tick_now);
  bool handle_overcurrent(uint32_t tick_now);

private:
  std::array<Led, IC_COUNT> leds;
  std::array<BTS::Ic, IC_COUNT> ics{};
  std::array<size_t, static_cast<size_t>(System_name::COUNT)>
      systems_channel_map;

  bool fan_temp_triggered{};
  Temperature inv_temperature;
  Temperature motor_temperature;

  BTS::Channel &get_channel(System_name name);
};