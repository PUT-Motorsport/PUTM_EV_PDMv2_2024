#include "pdu.hpp"

#include "BTS72220.hpp"
#include "spi.h"

// For individual channels 0 and 3 (returns 0.1A units as uint8_t) - do
// wyjebania
uint16_t mv_to_hma(uint16_t mv) {
  if (mv < 123)
    return 0;
  return (((mv - 123) * 1000) / 217 + 100);
}

// for channels 1 and 2
uint16_t mv_to_hma2(uint16_t mv) {
  if (mv < 123)
    return 0;
  return (((mv - 123) * 1000) / 482 + 50);
}

// Translate channel status to can frame data displayed on dash
uint8_t ch_status_can(const BTS::Channel ch) {
  switch (ch.status) {
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

Pdu::Pdu(std::array<Led, IC_COUNT> leds,
         const std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                          Pdu::IC_COUNT> &systems_data,
         Temperature inv_temperature, Temperature motor_temperature)
    : leds{leds[0], leds[1], leds[2], leds[3]},
      inv_temperature{inv_temperature}, motor_temperature{motor_temperature} {

  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      systems_channel_map[static_cast<uint8_t>(
          systems_data.at(ic_idx).at(ch_idx).name)] = ic_idx + ch_idx;

      ics.at(ic_idx).channels.at(ch_idx).set_threshold(
          systems_data.at(ic_idx).at(ch_idx).threshold);
    }
  }
}

const BTS::Channel &Pdu::get_channel(System_name name) const {
  size_t index = systems_channel_map.at(static_cast<size_t>(name));
  auto ic_index{index / IC_COUNT};
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

PUTM_CAN_M_pdu_channnel_t Pdu::get_can_pdu_channel_t() const {
  return {
      .pc_status{std::min({ch_status_can(get_channel(System_name::PC0)),
                           ch_status_can(get_channel(System_name::PC1)),
                           ch_status_can(get_channel(System_name::PC2)),
                           ch_status_can(get_channel(System_name::PC3))})},
      .fan_status{std::min({ch_status_can(get_channel(System_name::FAN1)),
                            ch_status_can(get_channel(System_name::FAN2))})},
      .pump_status{std::min({ch_status_can(get_channel(System_name::PUMP1)),
                             ch_status_can(get_channel(System_name::PUMP2))})},
      .inverter_status{
          std::min({ch_status_can(get_channel(System_name::INV1)),
                    ch_status_can(get_channel(System_name::INV2))})},
      .fbox_status{ch_status_can(get_channel(System_name::FBOX))},
      .sdc_status{ch_status_can(get_channel(System_name::SDC_ASMS))},
      .dash_status{ch_status_can(get_channel(System_name::DASH))},
      .tsal_hv_status{ch_status_can(get_channel(System_name::TSAL_HV))},
      .rbox_diagport_brake_l_status{
          ch_status_can(get_channel(System_name::RBOX_DIAG_BRAKE_L))},
      .brake_ir_air_status{
          ch_status_can(get_channel(System_name::BRAKE_IR_AIR))},
  };
}

PUTM_CAN_M_pdu_data_t Pdu::get_can_pdu_data_t() const {
  return {
      .pc_current{get_channel(System_name::PC0).get_current() +
                  get_channel(System_name::PC1).get_current() +
                  get_channel(System_name::PC2).get_current() +
                  get_channel(System_name::PC3).get_current()},
      .pump_current{get_channel(System_name::PUMP1).get_current() +
                    get_channel(System_name::PUMP2).get_current()},
      .fan_current{get_channel(System_name::FAN1).get_current() +
                   get_channel(System_name::FAN2).get_current()},
      .inverter_current{get_channel(System_name::INV1).get_current() +
                        get_channel(System_name::INV2).get_current()},
      .fbox_current{get_channel(System_name::FBOX).get_current()},
      .sdc_current{get_channel(System_name::SDC_ASMS).get_current()},
      .total_current{get_total_current()},
  };
}

// Count failed channels(with state other than ON) and update
// leds
bool Pdu::update_leds(uint32_t tick_now) {
  bool fail{false};
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    int led_err_count{0};
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      if (ics.at(ic_idx).channels.at(ch_idx).status !=
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

bool Pdu::update_fans(bool rtd_status, Temperature::Values inv_values,
                      Temperature::Values motor_values) {
  return (rtd_status || inv_temperature.update(inv_values) ||
          motor_temperature.update(motor_values));
}

// Transmit and receive 8-byte data for all ICs in daisy chain, this
// function flips data in array so each IC receives correct data index
std::array<uint8_t, Pdu::IC_COUNT>
Pdu::chain_tx_rx(const std::array<uint8_t, IC_COUNT> &tx) {
  uint8_t tx_buffer[IC_COUNT]{};
  std::reverse_copy(tx.begin(), tx.end(), tx_buffer);

  uint8_t rx_buffer[IC_COUNT]{};

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  for (int i{}; i < IC_COUNT; i++) {
    tx_buffer[i] = 0x00;
  }
  HAL_Delay(1);

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  std::array<uint8_t, IC_COUNT> rx{};
  std::reverse_copy(&(rx_buffer[0]), &(rx_buffer[IC_COUNT - 1]), rx.begin());

  return rx;
}

bool Pdu::check_chain_responses(std::array<uint8_t, IC_COUNT> rx) {
  bool has_error = false;
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    if (ics.at(ic_idx).check_response(rx.at(ic_idx))) {
      has_error = true;
    }
  }
  return has_error;
}

bool Pdu::check_chain_errors() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::ERRDIAG_CMD);

  auto rx{chain_tx_rx(tx)};

  bool has_error = false;
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    if (ics.at(ic_idx).check_err(rx.at(ic_idx))) {
      has_error = true;
    }
  }
  return has_error;
}

bool Pdu::init_chain() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::DCR_ACTIVE);

  auto rx{chain_tx_rx(tx)};
  if (check_chain_responses(rx))
    return true;

  for (auto &ic : ics) {
    ic.status = BTS::Ic::Status::STAND_BY;
  }
  return true;
}

bool Pdu::start_chain() {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::OUT_READY);

  auto rx{chain_tx_rx(tx)};
  if (check_chain_responses(rx))
    return true;

  for (auto &ic : ics) {
    ic.status = BTS::Ic::Status::ACTIVE;
  }
  return false;
}

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

  auto rx{chain_tx_rx(tx)};
  return check_chain_responses(rx);
}

void Pdu::update_channel_currents(
    uint8_t ch, const std::array<uint16_t, IC_COUNT> &voltages,
    uint32_t tick_now) {
  auto mV_to_mA{(ch == 0 || ch == 3) ? mv_to_hma : mv_to_hma2};
  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    BTS::Channel &channel{ics.at(ic_idx).channels.at(ch)};
    channel.update_current(mV_to_mA(voltages.at(ic_idx)), tick_now);
  }
}

bool Pdu::handle_overcurrent(uint32_t tick_now) {
  std::array<uint8_t, IC_COUNT> tx{};
  tx.fill(BTS::OUT_CLOSE);

  for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
    for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
      if (ics.at(ic_idx).channels.at(ch_idx).handle_overcurrent(tick_now) ==
          false)
        tx.at(ic_idx) |= 1 << ch_idx;
    }
  }

  auto rx{chain_tx_rx(tx)};
  return check_chain_responses(rx);
}