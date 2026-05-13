/*
 * PM09-CANBUS-BMS-HV.hpp
 *
 *  Created on: Jun 28, 2024
 *      Author: ketirange
 */

#ifndef INC_PUTM_EV_CAN_LIBRARY_2024_LIB_CANHEADERS_PM09_CANBUS_BMS_HV_HPP_
#define INC_PUTM_EV_CAN_LIBRARY_2024_LIB_CANHEADERS_PM09_CANBUS_BMS_HV_HPP_

#include <cstdint>

namespace PUTM_CAN {

struct __attribute__((packed)) BMS_HV_main
{
    uint16_t voltage_sum;
    int16_t current;
    uint8_t temp_max;
    uint8_t temp_avg;  // in Celsius
    uint16_t soc : 10; // state of charge
    bool ok : 1;
	bool precharge : 1;
};

const uint8_t BMS_HV_MAIN_CAN_DLC = sizeof(BMS_HV_main);

const FDCAN_TxHeaderTypeDef can_tx_header_BMS_HV_MAIN =
{
    BMS_HV_MAIN_CAN_ID,
	FDCAN_STANDARD_ID,
	FDCAN_DATA_FRAME,
	BMS_HV_MAIN_CAN_DLC,
	FDCAN_ESI_PASSIVE,
	FDCAN_BRS_OFF,
	FDCAN_CLASSIC_CAN,
	FDCAN_NO_TX_EVENTS,
	0
};
} // namespace PUTM_CAN
#endif /* INC_PUTM_EV_CAN_LIBRARY_2024_LIB_CANHEADERS_PM09_CANBUS_BMS_HV_HPP_ */
