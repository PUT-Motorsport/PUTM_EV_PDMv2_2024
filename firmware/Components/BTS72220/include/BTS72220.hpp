#pragma once

#include <array>
#include <cstdint>

namespace BTS72220 {

constexpr uint8_t OUT_READY{0x8F};    // 1000 1111	switching to ready mode
constexpr uint8_t OUT_READY1{0x9F};   // 1001 1111
constexpr uint8_t OUT_CLOSE{0x80};    // 1000 0000
constexpr uint8_t DCR_ACTIVE{0xF5};   // 1111 0101	switching to active mode
constexpr uint8_t DCR_SLEEP{0xFF};    // 1111 1111
constexpr uint8_t DCR_CHANNEL0{0xF8}; // 1111 1000
constexpr uint8_t DCR_CHANNEL1{0xF9}; // 1111 1001
constexpr uint8_t DCR_CHANNEL2{0xFA}; // 1111 1010
constexpr uint8_t DCR_CHANNEL3{0xFB}; // 1111 1011
// Diagnosis Registers - Read Commands
constexpr uint8_t WRNDIAG_CMD{0x01}; // 0000 0001
constexpr uint8_t STDDIAG_CMD{0x02}; // 0000 0010
constexpr uint8_t ERRDIAG_CMD{0x03}; // 0000 0011
// Configuration Registers - Read Commands
constexpr uint8_t OUT_READ_CMD{0x00};        // 0000 0000
constexpr uint8_t RCS_READ_CMD{0x08};        // 0000 1000
constexpr uint8_t SRC_READ_CMD{0x09};        // 0000 1001
constexpr uint8_t OCR_READ_CMD{0x04};        // 0000 0100
constexpr uint8_t RCD_READ_CMD{0x0A};        // 0000 1100
constexpr uint8_t KRC_READ_CMD{0x05};        // 0000 0101
constexpr uint8_t PCS_READ_CMD{0x0B};        // 0000 1101
constexpr uint8_t HWCR_READ_CMD{0x05};       // 0000 0110
constexpr uint8_t ICS_READ_CMD{0x0B};        // 0000 1110
constexpr uint8_t DCR_READ_CMD{0x07};        // 0000 0111
constexpr uint8_t CLOSE_CHANNEL_0_CMD{0x8E}; // 1000 1110
constexpr uint8_t CLOSE_CHANNEL_1_CMD{0x8D}; // 1000 1101
constexpr uint8_t CLOSE_CHANNEL_2_CMD{0x8B}; // 1000 1011
constexpr uint8_t CLOSE_CHANNEL_3_CMD{0x87}; // 1000 0111

union WRNDIAG {
  uint8_t value;
  struct {
    uint8_t RES : 4;
    uint8_t WRNn : 4;
  } reg;
};

union STDDIAG {
  uint8_t value;
  struct {
    uint8_t RES : 2;
    uint8_t TER : 1;
    uint8_t CSV : 1;
    uint8_t LHI : 1;
    uint8_t SLP : 1;
    uint8_t SBM : 1;
    uint8_t VSMON : 1;
  } reg;
};

union ERRDIAG {
  uint8_t value;
  struct {
    uint8_t RES : 4;
    uint8_t ERRn : 4;
  } reg;
};

class Channel {
public:
  enum class Status {
    OFF,
    ON,
    ERR, // Overcurrent / failure
    TEMP_LOCK,
    PERM_LOCK,
  };

  static constexpr uint32_t MAX_RETRIES{5};
  Status status{Status::ON};
  uint32_t retry_count{};
  uint32_t tick_last_attempt{};

  inline uint32_t get_current() { return current; }
  inline void set_threshold(uint16_t threshold_value) {
    threshold = threshold_value;
  }

  void update_current(uint32_t current_value, uint32_t tick_now);

private:
  uint32_t current{};
  uint32_t threshold{};
};

class Ic {
public:
  static constexpr uint8_t CHANNEL_COUNT{4};
  std::array<Channel, CHANNEL_COUNT> channels{};

  bool check_diag(uint8_t rx_value);

private:
};

} // namespace BTS72220