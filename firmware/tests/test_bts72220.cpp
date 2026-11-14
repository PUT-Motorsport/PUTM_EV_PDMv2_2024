#include "BTS72220.hpp"
#include "pdu_config.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class BTS72220Test : public ::testing::Test {
protected:
  BTS::Channel test_channel{};

  static constexpr uint16_t TEST_CHANNEL_CURRENT_THRESHOLD_mA = 5000;

  void SetUp() override {
    test_channel.set_threshold(TEST_CHANNEL_CURRENT_THRESHOLD_mA);
  }
};

/* --- Overcurrent logic tests --- */

constexpr uint16_t CURRENT_OVER_THRESHOLD_mA = 5100;
constexpr uint16_t CURRENT_UNDER_THRESHOLD_mA = 4900;

TEST_F(BTS72220Test, IsChannelOnIfCurrentUnderThreshold) {
  uint32_t tick_now = 0;
  test_channel.update_current(CURRENT_UNDER_THRESHOLD_mA, tick_now);
  test_channel.handle_overcurrent(tick_now);

  EXPECT_EQ(test_channel.get_status(), BTS::Channel::Status::ON);
}

TEST_F(BTS72220Test, IsChannelLockedIfCurrentOverThreshold) {
  uint32_t tick_now = 0;
  test_channel.update_current(CURRENT_OVER_THRESHOLD_mA, tick_now);
  test_channel.handle_overcurrent(tick_now);

  EXPECT_THAT(test_channel.get_status(),
              testing::AnyOf(BTS::Channel::Status::TEMP_LOCK,
                             BTS::Channel::Status::PERM_LOCK));
}

TEST_F(BTS72220Test, IsChannelUnlockedAfterRetry) {
  // Initial conditions: channel locked after overcurrent
  uint32_t tick_now = 0;
  test_channel.update_current(CURRENT_OVER_THRESHOLD_mA, tick_now);
  test_channel.handle_overcurrent(tick_now);

  // Channel should unlock after configured time
  tick_now += (BTS::Channel::TIME_PER_ATTEMPT_MS + 1);
  test_channel.handle_overcurrent(tick_now);

  EXPECT_EQ(test_channel.get_status(), BTS::Channel::Status::ON);
}

TEST_F(BTS72220Test, IsChannelPermanentlyLockedAfterMaxRetries) {
  uint32_t tick_now = 0;
  for (uint32_t retry = 0; retry <= BTS::Channel::MAX_RETRIES; retry++) {
    EXPECT_EQ(test_channel.get_status(), BTS::Channel::Status::ON);
    test_channel.update_current(CURRENT_OVER_THRESHOLD_mA, tick_now);
    test_channel.handle_overcurrent(tick_now);
    tick_now += ((retry + 1) * (BTS::Channel::TIME_PER_ATTEMPT_MS + 1));
    test_channel.handle_overcurrent(tick_now);
  }

  EXPECT_EQ(test_channel.get_status(), BTS::Channel::Status::PERM_LOCK);
}