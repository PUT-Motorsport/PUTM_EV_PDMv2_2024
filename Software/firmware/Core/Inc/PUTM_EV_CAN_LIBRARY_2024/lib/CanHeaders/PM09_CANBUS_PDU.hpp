#pragma once

#include "can_ids.hpp"

namespace PUTM_CAN {

struct __attribute__((packed)) PduChannel {
 uint32_t pc_status : 2;
 uint32_t fan_status : 2; 
 uint32_t pump_status : 2;
 uint32_t inverter_status : 2;
 uint32_t fbox_status : 2;
 uint32_t sdc_status : 2;
 uint32_t dash_status : 2;
 uint32_t tsal_hv_status : 2;
 uint32_t rbox_diagport_brake_l_status : 2;
 uint32_t brake_ir_air_status : 2;

};

const uint8_t PDU_CHANNEL_CAN_DLC = sizeof(PduChannel);
const uint8_t PDU_CHANNEL_FREQUENCY = 50;

const FDCAN_TxHeaderTypeDef can_tx_header_PDU_CHANNEL{
    PDU_CHANNEL_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, PDU_CHANNEL_CAN_DLC, FDCAN_ESI_PASSIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};

struct __attribute__((packed)) PduData {
 uint32_t pc_current : 8;
 uint32_t pump_current : 8; 
 uint32_t fan_current : 8;
 uint32_t inverter_current : 8;
 uint32_t fbox_current : 8;
 uint32_t sdc_current : 8;
 uint32_t total_current : 16;
};

const uint8_t PDU_DATA_CAN_DLC = sizeof(PduData);
const uint8_t PDU_DATA_FREQUENCY = 50;

const FDCAN_TxHeaderTypeDef can_tx_header_PDU_DATA{
    PDU_DATA_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, PDU_DATA_CAN_DLC, FDCAN_ESI_PASSIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};
}