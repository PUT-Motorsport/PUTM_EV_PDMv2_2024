#include "pdu.hpp"

#include <cstddef>
#include <cstdint>

// Same values as in Core/Src/main.cpp
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

static std::array<Led, Pdu::IC_COUNT> leds{};
static Spi_daisy_chain<Pdu::IC_COUNT> test_daisy_chain{};
static Pdu test_pdu{leds, test_daisy_chain, SYSTEM_DATA, {30, 40}, {30, 40}};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    if (size != 8) return 0;

    // Decoding without alignment, structure packing, or endianness assumptions.
    std::array<uint16_t, Pdu::IC_COUNT> samples{};
    for (size_t i = 0; i < 4; ++i) {
        samples[i] = static_cast<uint16_t>(
            static_cast<uint16_t>(data[2 * i]) |
            (static_cast<uint16_t>(data[2 * i + 1]) << 8)
        );
    }

    // 0..4095 is the real ADC range. Higher values also help test the
    // function's robustness; they are not realistic physical ADC samples.
    test_pdu.update_channel_currents(0, samples, 0);
    return 0;
}
