#pragma once

#include "main.h"
#include <algorithm>
#include <array>

template <size_t CHAIN_ICS> class Spi_daisy_chain {
private:
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *nss_gpio_port;
  uint16_t nss_gpio_pin;

public:
  Spi_daisy_chain(SPI_HandleTypeDef *new_hspi, GPIO_TypeDef *new_nss_gpio_port,
                  uint16_t new_nss_gpio_pin)
      : hspi{new_hspi}, nss_gpio_port{new_nss_gpio_port},
        nss_gpio_pin{new_nss_gpio_pin} {
    if (!hspi || !nss_gpio_port) {
      Error_Handler();
    }
  }

  // Transmit and receive 8-byte data for all ICs in daisy chain, this
  // function flips data in array so each IC receives correct data index
  std::array<uint8_t, CHAIN_ICS>
  transfer(const std::array<uint8_t, CHAIN_ICS> &tx) {
    uint8_t tx_buffer[CHAIN_ICS]{};
    std::reverse_copy(tx.begin(), tx.end(), tx_buffer);

    uint8_t rx_buffer[CHAIN_ICS]{};

    HAL_GPIO_WritePin(nss_gpio_port, nss_gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
    HAL_GPIO_WritePin(nss_gpio_port, nss_gpio_pin, GPIO_PIN_SET);

    for (size_t i{}; i < CHAIN_ICS; i++) {
      tx_buffer[i] = 0x00;
    }
    HAL_Delay(1);

    HAL_GPIO_WritePin(nss_gpio_port, nss_gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
    HAL_GPIO_WritePin(nss_gpio_port, nss_gpio_pin, GPIO_PIN_SET);

    std::array<uint8_t, CHAIN_ICS> rx{};
    std::reverse_copy(&(rx_buffer[0]), &(rx_buffer[CHAIN_ICS]), rx.begin());

    return rx;
  }
};