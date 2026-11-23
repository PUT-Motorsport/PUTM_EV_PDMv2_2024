#include "BTS72220.hpp"
#include "bts_printers.hpp"
#include "spi_daisy_chain.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <pdu.hpp>
#include <pdu_config.hpp>

class PduTest : public ::testing::Test {
protected:
  std::array<Led, Pdu::IC_COUNT> leds{};
  Spi_daisy_chain<Pdu::IC_COUNT> test_daisy_chain{};
  Pdu test_pdu{leds,
               test_daisy_chain,
               CONFIG::SYSTEM_DATA,
               {CONFIG::INV_MIN_TEMP, CONFIG::INV_MAX_TEMP},
               {CONFIG::MOTOR_MIN_TEMP, CONFIG::MOTOR_MAX_TEMP}};
};

/* --- Fans and pumps logic tests --- */

// Test temperatures derived from configured thresholds
constexpr uint8_t TEMP_MARGIN = 5;
static_assert(CONFIG::INV_MIN_TEMP >= TEMP_MARGIN &&
              CONFIG::MOTOR_MIN_TEMP >= TEMP_MARGIN);
static_assert(CONFIG::INV_MAX_TEMP <= UINT8_MAX - TEMP_MARGIN &&
              CONFIG::MOTOR_MAX_TEMP <= UINT8_MAX - TEMP_MARGIN);

constexpr uint8_t INV_TEMP_TO_STOP_FANS = CONFIG::INV_MIN_TEMP - TEMP_MARGIN;
constexpr uint8_t MOTOR_TEMP_TO_STOP_FANS =
    CONFIG::MOTOR_MIN_TEMP - TEMP_MARGIN;

constexpr uint8_t INV_TEMP_TO_START_FANS = CONFIG::INV_MAX_TEMP + TEMP_MARGIN;
constexpr uint8_t MOTOR_TEMP_TO_START_FANS =
    CONFIG::MOTOR_MAX_TEMP + TEMP_MARGIN;

constexpr uint8_t INV_TEMP_NEUTRAL =
    (CONFIG::INV_MIN_TEMP + CONFIG::INV_MAX_TEMP) / 2;
constexpr uint8_t MOTOR_TEMP_NEUTRAL =
    (CONFIG::MOTOR_MIN_TEMP + CONFIG::MOTOR_MAX_TEMP) / 2;

// All four sensors at the same temperature
constexpr Temperature::Values all_sensors(uint8_t temp) {
  return {temp, temp, temp, temp};
}

TEST_F(PduTest, CheckIfFansOnAfterRtdOn) {
  // Entering RTD should turn on fans and pumps regardless of temperatures
  test_pdu.update_fans(true, all_sensors(INV_TEMP_TO_STOP_FANS),
                       all_sensors(MOTOR_TEMP_TO_STOP_FANS));

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::ON);
}

TEST_F(PduTest, CheckIfFansOffAfterRtdOff) {
  // Initial condition: RTD on, fans on, temperature under limit
  test_pdu.update_fans(true, all_sensors(INV_TEMP_TO_STOP_FANS),
                       all_sensors(MOTOR_TEMP_TO_STOP_FANS));

  // Fans should turn off if RTD off and all temperatures below high temperature
  // limit
  test_pdu.update_fans(false, all_sensors(INV_TEMP_NEUTRAL),
                       all_sensors(MOTOR_TEMP_NEUTRAL));

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::OFF);
}

TEST_F(PduTest, CheckIfFansOnAfterAnyTemperatureHigh) {
  // If any temperature raises over limit, fans should turn on
  Temperature::Values motor_values = all_sensors(MOTOR_TEMP_NEUTRAL);
  motor_values.rear_right = MOTOR_TEMP_TO_START_FANS;
  test_pdu.update_fans(false, all_sensors(INV_TEMP_NEUTRAL), motor_values);

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::ON);
}

TEST_F(PduTest, CheckIfFansStillOnAfterAllTemperaturesNeutral) {
  // Initial condition: RTD off, fans on, temperature over limit
  test_pdu.update_fans(false, all_sensors(INV_TEMP_NEUTRAL),
                       all_sensors(MOTOR_TEMP_TO_START_FANS));

  // Even if all temperatures fall under high temperature limit, fans should
  // still be on
  test_pdu.update_fans(false, all_sensors(INV_TEMP_NEUTRAL),
                       all_sensors(MOTOR_TEMP_NEUTRAL));

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::ON);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::ON);
}

TEST_F(PduTest, CheckIfFansOffAfterAllTemperaturesLow) {
  // Initial condition: RTD off, fans on, temperature over limit
  test_pdu.update_fans(false, all_sensors(INV_TEMP_NEUTRAL),
                       all_sensors(MOTOR_TEMP_TO_START_FANS));

  // If all temperatures fall below low temperature limit, fans should turn off
  test_pdu.update_fans(false, all_sensors(INV_TEMP_TO_STOP_FANS),
                       all_sensors(MOTOR_TEMP_TO_STOP_FANS));

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::OFF);
}