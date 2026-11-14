#include "BTS72220.hpp"
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

constexpr uint8_t INV_TEMP_TO_STOP_FANS = 25;
constexpr uint8_t MOTOR_TEMP_TO_STOP_FANS = 25;

constexpr uint8_t INV_TEMP_TO_START_FANS = 45;
constexpr uint8_t MOTOR_TEMP_TO_START_FANS = 45;

constexpr uint8_t INV_TEMP_NEUTRAL = 35;
constexpr uint8_t MOTOR_TEMP_NEUTRAL = 35;

TEST_F(PduTest, CheckIfFansOnAfterRtdOn) {
  // Entering RTD should turn on fans and pumps regardless of temperatures
  test_pdu.update_fans(true, {INV_TEMP_TO_STOP_FANS},
                       {MOTOR_TEMP_TO_STOP_FANS});

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
  test_pdu.update_fans(true, {INV_TEMP_TO_STOP_FANS},
                       {MOTOR_TEMP_TO_STOP_FANS});

  // Fans should turn off if RTD off and all temperatures below high temperature
  // limit
  test_pdu.update_fans(false, {INV_TEMP_NEUTRAL}, {MOTOR_TEMP_NEUTRAL});

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
  test_pdu.update_fans(false, {INV_TEMP_NEUTRAL}, {MOTOR_TEMP_TO_START_FANS});

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
  test_pdu.update_fans(false, {INV_TEMP_NEUTRAL}, {MOTOR_TEMP_TO_START_FANS});

  // Even if all temperatures fall under high temperature limit, fans should
  // still be on
  test_pdu.update_fans(false, {INV_TEMP_NEUTRAL}, {MOTOR_TEMP_NEUTRAL});

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
  test_pdu.update_fans(false, {INV_TEMP_NEUTRAL}, {MOTOR_TEMP_TO_START_FANS});

  // If all temperatures fall below low temperature limit, fans should turn off
  test_pdu.update_fans(false, {INV_TEMP_TO_STOP_FANS},
                       {MOTOR_TEMP_TO_STOP_FANS});

  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::FAN2).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP1).get_status(),
            BTS::Channel::Status::OFF);
  EXPECT_EQ(test_pdu.get_channel(Sys_name::PUMP2).get_status(),
            BTS::Channel::Status::OFF);
}