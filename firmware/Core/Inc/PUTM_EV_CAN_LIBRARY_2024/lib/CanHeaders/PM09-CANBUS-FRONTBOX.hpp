#pragma once
#include "can_ids.hpp"

namespace PUTM_CAN {

enum struct FboxStates : uint8_t {
    Undefined,
    Normal_operation,
    Sensor_Implausiblity,
    Left_sensor_out_of_range_lower_bound,
    Left_sensor_out_of_range_upper_bound,
    Right_sensor_out_of_range_lower_bound,
    Right_sensor_out_of_range_upper_bound,
};

struct __attribute__((packed)) DriverInput {
    uint16_t pedalPosition; // pedal position form 0 – 1000 where 0 means pedal not pressed
    uint16_t brakePressureFront;
    uint16_t brakePressureRear;
    uint16_t steeringWheelPosition;
};

struct __attribute__((packed)) FrontData {
    bool sense_left_kill : 1;
    bool sense_right_kill : 1;
    bool sense_driver_kill : 1;
    bool sense_inertia : 1;
    bool sense_bspd : 1;
    bool sense_overtravel : 1;
    bool sense_right_wheel : 1;
    bool is_braking : 1;
    bool apps : 1;
    uint8_t apps_implausibility;
    int16_t frontLeftSuspension;
    int16_t frontRightSuspension;
};

const uint8_t DRIVER_INPUT_CAN_DLC = sizeof(DriverInput);
const uint8_t DRIVER_INPUT_FREQUENCY = 100;

const uint8_t FRONT_DATA_CAN_DLC = sizeof(FrontData);
const uint8_t FRONT_DATA_FREQUENCY = 10;

const FDCAN_TxHeaderTypeDef can_tx_header_DRIVER_INPUT{
    DRIVER_INPUT_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, DRIVER_INPUT_CAN_DLC, FDCAN_ESI_PASSIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};

const FDCAN_TxHeaderTypeDef can_tx_header_FRONT_DATA{
    FRONT_DATA_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, DRIVER_INPUT_CAN_DLC, FDCAN_ESI_PASSIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};

} // namespace PUTM_CAN
