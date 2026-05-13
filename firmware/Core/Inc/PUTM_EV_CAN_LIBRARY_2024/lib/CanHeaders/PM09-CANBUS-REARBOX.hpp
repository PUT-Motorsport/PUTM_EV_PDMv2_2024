#ifndef __REARBOX_H__
#define __REARBOX_H__

#include "can_ids.hpp"

namespace PUTM_CAN {

struct __attribute__((packed)) RearboxSafety {
    bool safety_rfu1 : 1;
    bool safety_rfu2 : 1;
    bool safety_asms : 1;
    bool safety_fw : 1;
    bool safety_hv : 1;
    bool safety_res : 1;
    bool safety_hvd : 1;
    bool safety_inv : 1;
    bool safety_wheel_fl : 1;
    bool safety_wheel_fr : 1;
    bool safety_wheel_rl : 1;
    bool safety_wheel_rr : 1;
};

struct __attribute__((packed)) RearboxTemperature {
    uint8_t mono_temperature;
    uint8_t coolant_temperature_in;
    uint8_t coolant_temperature_out;
    uint8_t oil_temperature_l;
    uint8_t oil_temperature_r;
};

struct __attribute__((packed)) RearboxMiscellaneous {
    uint8_t coolant_pressure_in;
    uint8_t coolant_pressure_out;
    uint16_t suspension_l;
    uint16_t suspension_r;
};

const uint8_t REARBOX_SAFETY_CAN_DLC = sizeof(RearboxSafety);
const uint8_t REARBOX_SAFETY_FREQUENCY = 10;

const uint8_t REARBOX_TEMPERATURE_CAN_DLC = sizeof(RearboxTemperature);
const uint8_t REARBOX_TEMPERATURE_FREQUENCY = 10;

const uint8_t REARBOX_MISCELLANEOUS_CAN_DLC = sizeof(RearboxMiscellaneous);
const uint8_t REARBOX_MISCELLANEOUS_FREQUENCY = 10;

#ifndef PUTM_USE_CAN_FD

const FDCAN_TxHeaderTypeDef can_tx_header_REARBOX_SAFETY{REARBOX_SAFETY_CAN_ID,  FDCAN_STANDARD_ID,  FDCAN_DATA_FRAME,
                                                         REARBOX_SAFETY_CAN_DLC, FDCAN_ESI_PASSIVE,  FDCAN_BRS_OFF,
                                                         FDCAN_CLASSIC_CAN,      FDCAN_NO_TX_EVENTS, 0};

const FDCAN_TxHeaderTypeDef can_tx_header_REARBOX_TEMPERATURE{REARBOX_TEMPERATURE_CAN_ID,  FDCAN_STANDARD_ID,  FDCAN_DATA_FRAME,
                                                              REARBOX_TEMPERATURE_CAN_DLC, FDCAN_ESI_PASSIVE,  FDCAN_BRS_OFF,
                                                              FDCAN_CLASSIC_CAN,           FDCAN_NO_TX_EVENTS, 0};

const FDCAN_TxHeaderTypeDef can_tx_header_REARBOX_MISCELLANEOUS{REARBOX_MISCELLANEOUS_CAN_ID,
                                                                FDCAN_STANDARD_ID,
                                                                FDCAN_DATA_FRAME,
                                                                REARBOX_MISCELLANEOUS_CAN_DLC,
                                                                FDCAN_ESI_PASSIVE,
                                                                FDCAN_BRS_OFF,
                                                                FDCAN_CLASSIC_CAN,
                                                                FDCAN_NO_TX_EVENTS,
                                                                0};
#endif

} // namespace PUTM_CAN
#endif // __REARBOX_H__
