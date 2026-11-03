#include "BTS72220.hpp"
#include "spi_daisy_chain.hpp"
#include <gtest/gtest.h>
#include <pdu.hpp>

class PduTest : public ::testing::Test {
protected:
  static constexpr uint16_t K_ILIS_13_5{2520};
  static constexpr uint16_t K_ILIS_5_5{1900};

  static constexpr std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                              Pdu::IC_COUNT>
      SYSTEM_DATA{{
          {{{Sys_name::INV2, 5000, K_ILIS_5_5},
            {Sys_name::INV1, 3000, K_ILIS_13_5},
            {Sys_name::RBOX_DIAG_BRAKE_L, 5000, K_ILIS_13_5},
            {Sys_name::TSAL_HV, 2000, K_ILIS_5_5}}},
          {{{Sys_name::DASH, 3000, K_ILIS_5_5},
            {Sys_name::SDC_ASMS, 1000, K_ILIS_13_5},
            {Sys_name::BRAKE_IR_AIR, 5000, K_ILIS_13_5},
            {Sys_name::FBOX, 5000, K_ILIS_5_5}}},
          {{{Sys_name::PC3, 4000, K_ILIS_5_5},
            {Sys_name::PC2, 5000, K_ILIS_13_5},
            {Sys_name::PC1, 5000, K_ILIS_13_5},
            {Sys_name::PC0, 4000, K_ILIS_5_5}}},
          {{{Sys_name::FAN2, 5000, K_ILIS_5_5},
            {Sys_name::PUMP2, 5000, K_ILIS_13_5},
            {Sys_name::PUMP1, 5000, K_ILIS_13_5},
            {Sys_name::FAN1, 5000, K_ILIS_5_5}}},
      }};

  std::array<Led, Pdu::IC_COUNT> leds{};
  Spi_daisy_chain<Pdu::IC_COUNT> test_daisy_chain{};
  Pdu test_pdu{leds, test_daisy_chain, SYSTEM_DATA, {30, 40}, {30, 40}};
};

TEST_F(PduTest, TotalCurrentIsZeroAtStartup) {
  ASSERT_EQ(test_pdu.get_total_current(), 0u);
}

TEST_F(PduTest, CheckIfFansOffAfterFirstUpdate) {
  test_pdu.update_fans(false, {35u}, {35u});
  ASSERT_EQ(test_pdu.get_can_pdu_channel_t().fan_status, 1u);
}

TEST_F(PduTest, CheckIfFansOnAfterRtd) {
  test_pdu.update_fans(true, {35u}, {35u});
  ASSERT_EQ(test_pdu.get_can_pdu_channel_t().fan_status, 1u);
}