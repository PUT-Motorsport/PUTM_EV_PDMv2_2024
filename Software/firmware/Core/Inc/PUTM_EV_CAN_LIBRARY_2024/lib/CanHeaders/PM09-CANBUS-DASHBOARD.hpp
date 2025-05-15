#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "can_ids.hpp"

namespace PUTM_CAN {

struct __attribute__((packed)) Dashboard {
    bool ready_to_drive_button : 1;
    bool ts_activation_button : 1;
    bool user_button : 1;
};

const uint8_t DASHBOARD_CAN_DLC = sizeof(Dashboard);
const uint8_t DASHBOARD_FREQUENCY = 100;

#ifndef PUTM_USE_CAN_FD

const FDCAN_TxHeaderTypeDef can_tx_header_DASHBOARD{
    DASHBOARD_CAN_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, DASHBOARD_CAN_DLC, FDCAN_ESI_PASSIVE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN, FDCAN_NO_TX_EVENTS, 0};

#endif

} // namespace PUTM_CAN
#endif // __DASHBOARD_H__
