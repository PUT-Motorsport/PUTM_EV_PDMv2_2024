#pragma once
#include "can_ids.hpp"

namespace PUTM_CAN {

// Structures used to pack data as in the datasheet
struct __attribute__((packed)) AmkFrontLeftStatus_t {
    unsigned int AMK_bReserve1 : 8;

    unsigned int AMK_bSystemReady : 1;
    unsigned int AMK_bError : 1;
    unsigned int AMK_bWarn : 1;
    unsigned int AMK_bQuitDcOn : 1;
    unsigned int AMK_bDcOn : 1;
    unsigned int AMK_bQuitInverterOn : 1;
    unsigned int AMK_bInverterOn : 1;
    unsigned int AMK_bDerating : 1;
};

struct __attribute__((packed)) amkFrontLeftControl_t {
    unsigned int AMK_bReserve1 : 8;

    unsigned int AMK_bInverterOn : 1;
    unsigned int AMK_bDcOn : 1;
    unsigned int AMK_bEnable : 1;
    unsigned int AMK_bErrorReset : 1;

    unsigned int AMK_bReserve2 : 4;
};

// ---------- inverter -> can device ----------
struct __attribute__((packed)) AmkFrontLeftActualValues1 {
    AmkFrontLeftStatus_t AMK_Status;
    int16_t AMK_ActualVelocity;
    int16_t AMK_TorqueCurrent;
    int16_t AMK_MagnetizingCurrent;
};

struct __attribute__((packed)) AmkFrontLeftActualValues2 {
    int16_t AMK_TempMotor;
    int16_t AMK_TempInverter;
    uint16_t AMK_ErrorInfo;
    int16_t AMK_TempIGBT;
};
// --------------------------------------------

// ---------- can device -> inverter ----------
struct __attribute__((packed)) AmkFrontLeftSetpoints1 {
    amkFrontLeftControl_t AMK_Control;
    int16_t AMK_TargetVelocity;
    int16_t AMK_TorqueLimitPositiv;
    int16_t AMK_TorqueLimitNegativ;
};
// --------------------------------------------

// n = {1, 2, 5, 6} (inverter 1, 2 ,3 ,4)

constexpr uint8_t FRONT_LEFT_AMK_ACTUAL_VALUES_1_CAN_DLC = sizeof(AmkFrontLeftActualValues1);
constexpr uint8_t FRONT_LEFT_AMK_ACTUAL_VALUES_2_CAN_DLC = sizeof(AmkFrontLeftActualValues2);
constexpr uint8_t FRONT_LEFT_AMK_SETPOINTS_1_CAN_DLC = sizeof(AmkFrontLeftSetpoints1);

const FDCAN_TxHeaderTypeDef can_tx_header_AMK_FRONT_LEFT_SETPOINTS{FRONT_LEFT_AMK_SETPOINTS_1_CAN_ID,
                                                                   FDCAN_STANDARD_ID,
                                                                   FDCAN_DATA_FRAME,
                                                                   FRONT_LEFT_AMK_SETPOINTS_1_CAN_DLC,
                                                                   FDCAN_ESI_PASSIVE,
                                                                   FDCAN_BRS_OFF,
                                                                   FDCAN_CLASSIC_CAN,
                                                                   FDCAN_NO_TX_EVENTS,
                                                                   0};

} // namespace PUTM_CAN
