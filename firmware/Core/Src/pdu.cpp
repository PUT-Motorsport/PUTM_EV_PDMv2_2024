#include "pdu.hpp"

#include "BTS72220.hpp"
#include "spi.h"

// Converts adc voltage measured (in mV) to calibrated current value (in mA)
static uint16_t mV_to_mA(uint16_t u_mV, uint16_t k_ilis) {
  const uint16_t VOLTAGE_OFFSET{123};
  const uint16_t R_SENSE{1200};
  if (u_mV < VOLTAGE_OFFSET)
    return 0;
  return ((u_mV - VOLTAGE_OFFSET) * k_ilis / R_SENSE);
}

// Translate channel status to can frame channel data
static uint8_t ch_status_can(BTS::Channel ch) {
  switch (ch.get_status()) {
  case BTS::Channel::Status::OFF:
    return 0;
  case BTS::Channel::Status::ERR:
  case BTS::Channel::Status::ON:
    return 1;
  case BTS::Channel::Status::TEMP_LOCK:
    return 2;
  case BTS::Channel::Status::PERM_LOCK:
    return 3;
  }
  return 0;
}

// Transmit and receive 8-byte data for all ICs in daisy chain, this
// function flips data in array so each IC receives correct data index
template <size_t CHAIN_ICS>
std::array<uint8_t, CHAIN_ICS>
daisy_chain_txrx(const std::array<uint8_t, CHAIN_ICS> &tx) {
  uint8_t tx_buffer[CHAIN_ICS]{};
  std::reverse_copy(tx.begin(), tx.end(), tx_buffer);

  uint8_t rx_buffer[CHAIN_ICS]{};

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  for (size_t i{}; i < CHAIN_ICS; i++) {
    tx_buffer[i] = 0x00;
  }
  HAL_Delay(1);

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  std::array<uint8_t, CHAIN_ICS> rx{};
  std::reverse_copy(&(rx_buffer[0]), &(rx_buffer[CHAIN_ICS]), rx.begin());

  return rx;
}

/* Update single Led state based on failed channels count
 */
bool Led::update(uint8_t channels_failed, uint32_t tick_now) {
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

Temperature::Status Temperature::check(uint8_t value) const {
  if (value >= max)
    return Status::TOO_HIGH;
  else if (value < min)
    return Status::TOO_LOW;
  else
    return Status::OK;
}

void Temperature::update(Values values) {
  status.front_left = check(values.front_left);
  status.front_right = check(values.front_right);
  status.rear_left = check(values.rear_left);
  status.rear_right = check(values.rear_right);
}

Temperature::Status Temperature::is_ok() const {
  if (status.front_left == Status::TOO_HIGH ||
      status.front_right == Status::TOO_HIGH ||
      status.rear_left == Status::TOO_HIGH ||
      status.rear_right == Status::TOO_HIGH) {
    return Status::TOO_HIGH;
  } else if (status.front_left == Status::TOO_LOW &&
             status.front_right == Status::TOO_LOW &&
             status.rear_left == Status::TOO_LOW &&
             status.rear_right == Status::TOO_LOW) {
    return Status::TOO_LOW;
  } else
    return Status::OK;
}

/*
Sets current threshold for each ic channel and creates map for systems connected
to pdu, so channels are accessible with system name
*/
Pdu::Pdu(std::array<Led, IC_COUNT> leds,
         const std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                          Pdu::IC_COUNT> &systems_data,
         Temperature inv_temperature, Temperature motor_temperature)
    : leds{leds}, inv_temperature{inv_temperature},
      motor_temperature{motor_temperature} {

  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      auto system_data{systems_data.at(ic_idx).at(ch_idx)};

      systems_channel_map[static_cast<uint8_t>(system_data.name)] =
          ic_idx * BTS::Ic::CHANNEL_COUNT + ch_idx;
      ics.at(ic_idx).channels.at(ch_idx).set_threshold(
          system_data.i_threshold_mA);
    }
  }
}

/*
Provides access to channel with system name
*/
BTS::Channel &Pdu::get_channel(Sys_name name) {
  auto index{systems_channel_map.at(static_cast<size_t>(name))};
  auto ic_index{index / BTS::Ic::CHANNEL_COUNT};
  auto channel_index{index % BTS::Ic::CHANNEL_COUNT};
  return ics.at(ic_index).channels.at(channel_index);
}

uint16_t Pdu::get_total_current() const {
  uint16_t sum{};
  for (const auto &ic : ics) {
    for (const auto &channel : ic.channels) {
      sum += channel.get_current();
    }
  }
  return sum;
}

PUTM_CAN_M_pdu_channnel_t Pdu::get_can_pdu_channel_t() {
  return {
      .pc_status{std::max({ch_status_can(get_channel(Sys_name::PC0)),
                           ch_status_can(get_channel(Sys_name::PC1)),
                           ch_status_can(get_channel(Sys_name::PC2)),
                           ch_status_can(get_channel(Sys_name::PC3))})},
      .fan_status{std::max({ch_status_can(get_channel(Sys_name::FAN1)),
                            ch_status_can(get_channel(Sys_name::FAN2))})},
      .pump_status{std::max({ch_status_can(get_channel(Sys_name::PUMP1)),
                             ch_status_can(get_channel(Sys_name::PUMP2))})},
      .inverter_status{std::max({ch_status_can(get_channel(Sys_name::INV1)),
                                 ch_status_can(get_channel(Sys_name::INV2))})},
      .fbox_status{ch_status_can(get_channel(Sys_name::FBOX))},
      .sdc_status{ch_status_can(get_channel(Sys_name::SDC_ASMS))},
      .dash_status{ch_status_can(get_channel(Sys_name::DASH))},
      .tsal_hv_status{ch_status_can(get_channel(Sys_name::TSAL_HV))},
      .rbox_diagport_brake_l_status{
          ch_status_can(get_channel(Sys_name::RBOX_DIAG_BRAKE_L))},
      .brake_ir_air_status{ch_status_can(get_channel(Sys_name::BRAKE_IR_AIR))},
  };
}

PUTM_CAN_M_pdu_data_1_t Pdu::get_can_pdu_data_1() {
  return {
      .pc_current{get_channel(Sys_name::PC0).get_current() / 10 +
                  get_channel(Sys_name::PC1).get_current() / 10 +
                  get_channel(Sys_name::PC2).get_current() / 10 +
                  get_channel(Sys_name::PC3).get_current() / 10},
      .pump_current{get_channel(Sys_name::PUMP1).get_current() / 10 +
                    get_channel(Sys_name::PUMP2).get_current() / 10},
      .fan_current{get_channel(Sys_name::FAN1).get_current() / 10 +
                   get_channel(Sys_name::FAN2).get_current() / 10},
      .inverter_current{get_channel(Sys_name::INV1).get_current() / 10 +
                        get_channel(Sys_name::INV2).get_current() / 10},
  };
}

PUTM_CAN_M_pdu_data_2_t Pdu::get_can_pdu_data_2() {
  return {
      .fbox_current{get_channel(Sys_name::FBOX).get_current() / 10},
      .sdc_current{get_channel(Sys_name::SDC_ASMS).get_current() / 10},
      .total_current{get_total_current() / 10},
  };
}

// Count failed channels(with state other than ON) and update
// leds
bool Pdu::update_leds(uint32_t tick_now) {
  bool fail{false};
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    int led_err_count{0};
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      if (ics.at(ic_idx).channels.at(ch_idx).get_status() !=
          BTS::Channel::Status::ON) {
        led_err_count++;
      }
    }
    if (leds.at(ic_idx).update(led_err_count, tick_now)) {
      fail = true;
    }
  }
  return fail;
}

/*
Manages fan and pump channels based on temperature and rtd status received by
CAN.
*/
void Pdu::update_fans(const bool &rtd_status,
                      const Temperature::Values &inv_values,
                      const Temperature::Values &motor_values) {

  inv_temperature.update(inv_values);
  motor_temperature.update(motor_values);

  if (!fan_temp_triggered &&
      (inv_temperature.is_ok() == Temperature::Status::TOO_HIGH ||
       motor_temperature.is_ok() == Temperature::Status::TOO_HIGH)) {
    fan_temp_triggered = true;
  } else if (fan_temp_triggered &&
             (inv_temperature.is_ok() == Temperature::Status::TOO_LOW &&
              motor_temperature.is_ok() == Temperature::Status::TOO_LOW)) {
    fan_temp_triggered = false;
  }

  if (rtd_status || fan_temp_triggered) {
    get_channel(Sys_name::FAN1).turn_on();
    get_channel(Sys_name::FAN2).turn_on();
    get_channel(Sys_name::PUMP1).turn_on();
    get_channel(Sys_name::PUMP2).turn_on();
  } else {
    get_channel(Sys_name::FAN1).turn_off();
    get_channel(Sys_name::FAN2).turn_off();
    get_channel(Sys_name::PUMP1).turn_off();
    get_channel(Sys_name::PUMP2).turn_off();
  }
}

/*
- Checks BTS72220 STDDIAG and WRNDIAG frames received after sending commands
- Returns true if received frame doesn't match diag frames
*/
bool Pdu::update_chain_diag(std::array<uint8_t, IC_COUNT> rx) {
  bool has_error = false;
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    if (ics.at(ic_idx).check_response(rx.at(ic_idx))) {
      has_error = true;
    }
  }
  return has_error;
}

/*
- Sends ERRDIAG command and decodes received frame
- Disables channel based on ERRn bits
- Returns true if received frame doesn't match diag frames
*/
bool Pdu::update_chain_errors() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::ERRDIAG_CMD);

  auto rx{daisy_chain_txrx<IC_COUNT>(tx)};

  bool has_error = false;
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    if (ics.at(ic_idx).check_err(rx.at(ic_idx))) {
      has_error = true;
    }
  }
  return has_error;
}

/*
Sends DCR_ACTIVATE command to all ICs in daisy chain, transition from SLEEP to
STAND_BY state
*/
void Pdu::init_chain() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::DCR_ACTIVE);

  auto rx{daisy_chain_txrx<IC_COUNT>(tx)};
  update_chain_diag(rx);

  for (auto &ic : ics) {
    ic.status = BTS::Ic::Status::STAND_BY;
  }
}

/*
Sends OUT_READY command to all ICs in daisy chain, transition from STAND_BY to
ACTIVE state, activates all channels
*/
void Pdu::start_chain() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::OUT_READY);

  auto rx{daisy_chain_txrx<IC_COUNT>(tx)};
  update_chain_diag(rx);

  for (auto &ic : ics) {
    ic.status = BTS::Ic::Status::ACTIVE;
  }
}

/*
Sets channel current measurement, delay needed to stabilize output, returns true
for wrong channel id
*/
bool Pdu::set_channel_sense(uint8_t channel) {
  uint8_t dcr_channel{};
  switch (channel) {
  case 0: {
    dcr_channel = BTS::DCR_CHANNEL0;
    break;
  }
  case 1: {
    dcr_channel = BTS::DCR_CHANNEL1;
    break;
  }
  case 2: {
    dcr_channel = BTS::DCR_CHANNEL2;
    break;
  }
  case 3: {
    dcr_channel = BTS::DCR_CHANNEL3;
    break;
  }
  default: {
    return true;
  }
  }

  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(dcr_channel);

  auto rx{daisy_chain_txrx<IC_COUNT>(tx)};
  update_chain_diag(rx);
  return false;
}

/*
Updates channel currents and status, returns true for wrong channel id
*/
bool Pdu::update_channel_currents(
    uint8_t ch, const std::array<uint16_t, IC_COUNT> &voltages,
    uint32_t tick_now) {
  if (ch >= BTS::Ic::CHANNEL_COUNT)
    return true;

  uint16_t k_ilis{(ch == 0 || ch == 3) ? BTS::Ic::K_ILIS_5_5
                                       : BTS::Ic::K_ILIS_13_5};
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    BTS::Channel &channel{ics.at(ic_idx).channels.at(ch)};
    uint16_t i_mA{mV_to_mA(voltages.at(ic_idx), k_ilis)};
    channel.update_current(i_mA, tick_now);
  }
  return false;
}

/*
Closes channels with status other than ON, manages locked channels retry logic
*/
void Pdu::handle_overcurrent(uint32_t tick_now) {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::OUT_CLOSE);

  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      if (ics.at(ic_idx).channels.at(ch_idx).handle_overcurrent(tick_now) ==
          false)
        tx.at(ic_idx) |= 1 << ch_idx;
    }
  }

  auto rx{daisy_chain_txrx<IC_COUNT>(tx)};
  update_chain_diag(rx);
}