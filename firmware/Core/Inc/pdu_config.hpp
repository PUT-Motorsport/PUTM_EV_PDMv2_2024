#include "pdu.hpp"

namespace CONFIG {

  // Temperature thresholds for turning off fans
  constexpr uint8_t INV_MIN_TEMP{30};
  constexpr uint8_t INV_MAX_TEMP{40};

  // Temperature thresholds for turning on fans
  constexpr uint8_t MOTOR_MIN_TEMP{30};
  constexpr uint8_t MOTOR_MAX_TEMP{40};

  // BTS7220 calibration values, check in datasheet(https://www.infineon.com/assets/row/public/documents/10/49/infineon-bts72220-4ese-datasheet-en.pdf)
  constexpr uint16_t K_ILIS_13_5{2520};
  constexpr uint16_t K_ILIS_5_5{1900};

  // Physical channels order with cutoff current values and calibration values
  constexpr std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                       Pdu::IC_COUNT>
      SYSTEM_DATA{{{{{Sys_name::INV2, 5000, K_ILIS_5_5},
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
                     {Sys_name::FAN1, 5000, K_ILIS_5_5}}}}};
}