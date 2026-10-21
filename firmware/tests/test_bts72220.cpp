#include "BTS72220.hpp"
#include <gtest/gtest.h>

class BTS72220Test : public ::testing::Test {
protected:
  BTS::Ic test_ic{};
};

TEST_F(BTS72220Test, IsIcStatusSleepAtStartup) {
  ASSERT_EQ(test_ic.status, BTS::Ic::Status::SLEEP);
}

TEST_F(BTS72220Test, IsChannelTurnedOnAtStartup) {
  ASSERT_EQ(test_ic.channels[0].get_status(), BTS::Channel::Status::ON);
}

TEST_F(BTS72220Test, TryTurnOffChannel) {
  test_ic.channels[0].turn_off();
  ASSERT_EQ(test_ic.channels[0].get_status(), BTS::Channel::Status::OFF);
}
