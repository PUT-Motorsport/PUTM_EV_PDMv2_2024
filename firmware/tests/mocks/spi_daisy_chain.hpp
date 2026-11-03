#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <gmock/gmock.h>

template <size_t CHAIN_ICS> class Spi_daisy_chain {
public:
  MOCK_METHOD((std::array<uint8_t, CHAIN_ICS>), transfer,
              ((const std::array<uint8_t, CHAIN_ICS> &tx)));
};