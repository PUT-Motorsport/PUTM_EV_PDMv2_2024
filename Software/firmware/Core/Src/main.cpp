/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


//*
//fuse4
//fuse3
//fuse2
//fuse1

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "PUTM_EV_CAN_LIBRARY_2024/lib/can_interface.hpp"

//---------------------------------------------------------------
//  SYSTEM OVERVIEW
//---------------------------------------------------------------
//
//
//  HARDWARE STRUCTURE:
//
//  - 4 × BTS72220 smart high-side switches (called fuse1–fuse4, top to bottom)
//  - Each IC controls 4 output channels → total of 16 channels
//  - Current is measured via 4 ADC channels (1 per IC)
//
//  CHANNEL INDEX MAPPING:
//
//  - IC index 0 = fuse1 (bottom), IC 1 = fuse2, IC 2 = fuse3, IC 3 = fuse4 (top)
//  - In `fuse_currents[IC][CH]`, the IC index reflects this order
//  - In `tx_buffer` and `rx_buffer` (5 bytes):
//      tx_buffer[0] = common command
//      tx_buffer[1] = fuse4 (IC3)
//      tx_buffer[2] = fuse3 (IC2)
//      tx_buffer[3] = fuse2 (IC1)
//      tx_buffer[4] = fuse1 (IC0)
//    ⚠️ This means buffer index = 4 - IC index
//
//  ADC CHANNEL MAPPING:
//
//  - ADC buffer layout is reversed:
//      adc_buffer[3] = IC0 (fuse1)
//      adc_buffer[2] = IC1 (fuse2)
//      adc_buffer[1] = IC2 (fuse3)
//      adc_buffer[0] = IC3 (fuse4)
//
//  CHANNEL CONTROL:
//
//  - System enters READY mode on startup, then ACTIVE
//  - If current exceeds threshold, the corresponding channel is disabled via SPI
//  - After 5s, disabled channels are retried if current drops below threshold
//
//  LOGICAL OUTPUTS:
//
//  - Physical channels grouped into 10 logical outputs (e.g. pc, pump, fan, inverter)
//  - Each logical group has a 2-bit status: ON = 0, OFF = 1, ERROR = 2
//  - Status is aggregated → if any subchannel fails, the whole group = ERROR
//
//  CAN COMMUNICATION:
//
//  - Logical statuses sent via `PUTM_CAN::PduChannel` every 40 ms
//  - Currents (summed per group) sent via `PUTM_CAN::PduData` every 200 ms



/*
 * LED DEBUGGING SYSTEM FOR 4 BTS72220 CONTROLLERS (16 CHANNELS)
 *
 * NORMAL OPERATION:
 * - All LEDs blink at 1 Hz → System OK
 *
 * SINGLE CHANNEL FAILURE (1 ERROR ON A CONTROLLER):
 * - The LED of the faulty controller blinks in a specific pattern:
 *   - 1 blink → Channel 0 failure
 *   - 2 blinks → Channel 1 failure
 *   - 3 blinks → Channel 2 failure
 *   - 4 blinks → Channel 3 failure
 * - This pattern repeats continuously with a short pause.
 *
 * MULTIPLE CHANNEL FAILURES ON ONE CONTROLLER:
 * - The corresponding LED blinks rapidly at 5 Hz.
 *
 * MULTIPLE CONTROLLERS FAILING:
 * - Each affected controller's LED blinks rapidly at 5 Hz.
 *
 * CRITICAL FAILURE (ALL CONTROLLERS FAILING):
 * - All 4 LEDs stay solid ON.
 *
 * SYSTEM IN STANDBY/RESET:
 * - All LEDs stay OFF.
 *
 * HOW TO INTERPRET THE LEDS:
 * - Example: If LED2 blinks 3 times, pauses, then repeats → Controller 2, Channel 2 failure.
 * - Example: If LED4 blinks fast (5 Hz) → Controller 4 has multiple channel failures.
 * - Example: If all LEDs are ON → System critical failure.
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 4
#define __VREFANALOG_VOLTAGE__ 3300
#define OUT_READY 0x8F // 1000 1111		switching to ready mode
#define OUT_READY1 0x9F // 1001 1111
#define OUT_CLOSE 0x80 // 1000 0000
#define DCR_ACTIVE 0xF5 // 1111 0101	switching to active mode
#define DCR_SLEEP 0xFF // 1111 1111
#define DCR_CHANNEL0 0xF8 // 1111 1000
#define DCR_CHANNEL1 0xF9 // 1111 1001
#define DCR_CHANNEL2 0xFA // 1111 1010
#define DCR_CHANNEL3 0xFB // 1111 1011
// Diagnosis Registers - Read Commands
#define WRNDIAG 0x01// 0000 0001
#define STDDIAG 0x02// 0000 0010
#define ERRDIAG 0x03// 0000 0011
// Configuration Registers - Read Commands
#define OUT_READ 0x00 // 0000 0000
#define RCS_READ 0x08 // 0000 1000
#define SRC_READ 0x09 // 0000 1001
#define OCR_READ 0x04 // 0000 0100
#define RCD_READ 0x0A // 0000 1100
#define KRC_READ 0x05 // 0000 0101
#define PCS_READ 0x0B // 0000 1101
#define HWCR_READ 0x05 // 0000 0110
#define ICS_READ 0x0B // 0000 1110
#define DCR_READ 0x07 // 0000 0111
//Led initialisation
#define LED1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15

#define MAX_RETRIES 2

#define CLOSE_CHANNEL_0 0x8E//1000 1110
#define CLOSE_CHANNEL_1 0x8D//1000 1101
#define CLOSE_CHANNEL_2 0x8B//1000 1011
#define CLOSE_CHANNEL_3 0x87//1000 0111

#define IC_COUNT 4
#define CHANNEL_COUNT 4

typedef enum {
    STATUS_ON  = 0,  // Channel ON
    STATUS_OFF = 1,  // Channel OFF
    STATUS_ERR = 2   // Overcurrent / failure
} ChannelStatus;

typedef struct {
    ChannelStatus pc_status;
    ChannelStatus fan_status;
    ChannelStatus pump_status;
    ChannelStatus inverter_status;
    ChannelStatus fbox_status;
    ChannelStatus sdc_status;
    ChannelStatus dash_status;
    ChannelStatus tsal_hv_status;
    ChannelStatus rbox_diagport_brake_l_status;
    ChannelStatus brake_ir_air_status;
} SystemStatus;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//CAN
uint32_t can_pdu_channel_tick = 0;
uint32_t can_pdu_data_tick = 0;
bool CanErrorCommunication = false;

//ADC
uint16_t adc_buffer[ADC_BUF_SIZE];
uint8_t adc_ready = 0;

uint8_t tx_buffer[5];
uint8_t rx_buffer[5];

uint8_t after_first_loop = 0;



uint8_t fuse_currents[IC_COUNT][CHANNEL_COUNT]; // Stored in 0.1A units (hMA) | IC index 0 = fuse1 (bottom), 3 = fuse4 (top)
uint8_t channel_states[IC_COUNT] = {0x0F, 0x0F, 0x0F, 0x0F}; // All channels ON

// old retry variables
bool any_channel_closed = false;
uint32_t last_shutdown_time = 0;

//new retry variables
uint8_t channel_disabled[IC_COUNT][CHANNEL_COUNT] = {0};
uint8_t channel_retry_count[IC_COUNT][CHANNEL_COUNT] = {0};
uint32_t channel_last_attempt[IC_COUNT][CHANNEL_COUNT] = {0};


uint8_t thresholds[IC_COUNT][CHANNEL_COUNT] = {
    {20, 20, 30, 50},	// IC0  2.0A, 2.0A, 3.0A, 5.0A
    {20, 20, 30, 50},	// IC1
    {20, 20, 30, 50},	// IC2
    {20, 20, 30, 50} 	// IC3
};


bool RTD_status;
uint8_t rearRightInverterTemperature;  // Range 20-100
uint8_t rearLeftInverterTemperature;   // Range 20-100
uint8_t rearRightMotorTemperature;     // Range 20-130
uint8_t rearLeftMotorTemperature;      // Range 20-130
uint8_t frontRightInverterTemperature; // Range 20-100
uint8_t frontLeftInverterTemperature;  // Range 20-100
uint8_t frontRightMotorTemperature;    // Range 20-130
uint8_t frontLeftMotorTemperature;     // Range 20-130
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int get_tx_index(uint8_t ic_index)
{
    return 1 + ic_index;
}


// For individual channels (returns 0.1A units as uint8_t)
uint8_t mv_to_hma(uint32_t mv) {
    if (mv < 93) return 0;
    return (uint8_t)(((mv - 93) / 217.0f) * 10.0f);
}

void handle_overcurrent(uint8_t ic_index, uint8_t channel_number, uint8_t threshold)
{
    if (fuse_currents[ic_index][channel_number] > threshold && after_first_loop && RTD_status)
    {
        int tx_index = get_tx_index(ic_index);

        channel_states[ic_index] &= ~(1 << channel_number);
        channel_disabled[ic_index][channel_number] = 1;
        channel_retry_count[ic_index][channel_number] = 0;
        channel_last_attempt[ic_index][channel_number] = HAL_GetTick();

        for (int i = 0; i < 5; i++) tx_buffer[i] = DCR_ACTIVE;
        tx_buffer[tx_index] = 0x80 | (channel_states[ic_index] & 0x0F);

        HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
        HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
    }
}

// old function
//void handle_overcurrent(uint8_t ic_index, uint8_t channel_number, uint8_t threshold)
//{
//    if (fuse_currents[ic_index][channel_number] > threshold && after_first_loop)
//    {
//    	 int tx_index = get_tx_index(ic_index);
//
//    	        //Mark channel OFF in state
//    	        channel_states[ic_index] &= ~(1 << channel_number); // IC index 0 = fuse1 (bottom), 3 = fuse4 (top)
//    	        //track the time since the first channel is closed
//    	        any_channel_closed = true;
//    	        last_shutdown_time = HAL_GetTick();
//    	        //Build OUT register byte from updated state
//    	        for (int i = 0; i < 5; i++) tx_buffer[i] = DCR_ACTIVE;
//    	        tx_buffer[tx_index] = 0x80 | (channel_states[ic_index] & 0x0F); // OUT command
//
//    	        HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
//    	        HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
//    	        HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
//
//    }
//}

ChannelStatus reduce_status(ChannelStatus a, ChannelStatus b) {
    if (a == STATUS_ERR || b == STATUS_ERR) return STATUS_ERR;
    if (a == STATUS_OFF || b == STATUS_OFF) return STATUS_OFF;
    return STATUS_ON;
}

SystemStatus get_system_status_from_channels() {
    SystemStatus status;

    // IC0 = fuse1, IC1 = fuse2, IC2 = fuse3, IC3 = fuse4

    // Get individual channel states
    auto ch = [](uint8_t ic, uint8_t ch) -> ChannelStatus {
        return (channel_states[ic] & (1 << ch)) ? STATUS_ON :
               (fuse_currents[ic][ch] > thresholds[ic][ch]) ? STATUS_ERR : STATUS_OFF;
    };


    // PC: IC0 ch0-3
    ChannelStatus pc = ch(0,0);
    pc = reduce_status(pc, ch(0,1));
    pc = reduce_status(pc, ch(0,2));
    pc = reduce_status(pc, ch(0,3));
    status.pc_status = pc;

    // FAN: IC1 ch0, ch2
    status.fan_status = reduce_status(ch(1,0), ch(1,2));

    // PUMP: IC1 ch1, ch3
    status.pump_status = reduce_status(ch(1,1), ch(1,3));

    // INVERTER: IC2 ch0, ch1
    status.inverter_status = reduce_status(ch(2,0), ch(2,1));

    // FBOX: IC2 ch2
    status.fbox_status = ch(2,2);

    // SDC: IC2 ch3
    status.sdc_status = ch(2,3);

    // DASH: IC3 ch0
    status.dash_status = ch(3,0);

    // TSAL_HV: IC3 ch1
    status.tsal_hv_status = ch(3,1);

    // RBOX_DIAGPORT_BRAKE_L: IC3 ch2
    status.rbox_diagport_brake_l_status = ch(3,2);

    // BRAKE_IR_AIR: IC3 ch3
    status.brake_ir_air_status = ch(3,3);

    return status;
}

// summing of the current for CAN
uint32_t current_sum(std::initializer_list<std::pair<uint8_t, uint8_t>> list) {
    uint32_t sum = 0;
    for (auto [ic, ch] : list) {
        sum += fuse_currents[ic][ch];
    }
    return sum;
}

//void send_test_message() {
//    FDCAN_TxHeaderTypeDef txHeader;
//    uint8_t txData[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
//
//    txHeader.Identifier = 0x123;
//    txHeader.IdType = FDCAN_STANDARD_ID;
//    txHeader.TxFrameType = FDCAN_DATA_FRAME;
//    txHeader.DataLength = FDCAN_DLC_BYTES_8;
//    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    txHeader.MessageMarker = 0;
//
//    for (int i = 0; i < 10; i++) {
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
//        HAL_Delay(10); // short delay to separate bursts
//    }
//}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  auto st1 = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//  st1 = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
//  st1 = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_FULL, 0);

  static FDCAN_FilterTypeDef sFilterConfig =
  {
  	.IdType = FDCAN_STANDARD_ID,
  	.FilterIndex = 0,
  	.FilterType = FDCAN_FILTER_MASK,
  	.FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  	.FilterID1 = 0,
  	.FilterID2 = 0
  };

  auto st2 = HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();  // Only once here
    }

  // tx_buffer[1] = fuse4 (IC3), ..., tx_buffer[4] = fuse1 (IC0)

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  //sleep -> ready
  tx_buffer[0] = OUT_READY;
  tx_buffer[1] = OUT_READY;
  tx_buffer[2] = OUT_READY;
  tx_buffer[3] = OUT_READY;
  tx_buffer[4] = OUT_READY;
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
  //ready -> active
  tx_buffer[0] = DCR_ACTIVE;
  tx_buffer[1] = DCR_ACTIVE; 		//fuse 4
  tx_buffer[2] = DCR_ACTIVE;		//fuse 3
  tx_buffer[3] = DCR_ACTIVE;		//fuse 2
  tx_buffer[4] = DCR_ACTIVE;		//fuse 1
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);

  can_pdu_channel_tick = HAL_GetTick();
  can_pdu_data_tick    = HAL_GetTick();



  HAL_Delay(100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint32_t now = HAL_GetTick();

	  //CAN
	  if(PUTM_CAN::can.get_pc_new_data()){
		  auto pc_data=PUTM_CAN::can.get_pc_main_data();
		  RTD_status = pc_data.rtd;
	  }
	  if(PUTM_CAN::can.get_pc_temperature_data_new_data()){
	 		 auto pc_temp=PUTM_CAN::can.get_pc_temperature_data();

	 		 rearRightInverterTemperature = pc_temp.rearRightInverterTemperature; // Range 20-100
	 		 rearLeftInverterTemperature = pc_temp.rearLeftInverterTemperature;  // Range 20-100
	 		 rearRightMotorTemperature = pc_temp.rearRightMotorTemperature;     // Range 20-130
	 		 rearLeftMotorTemperature= pc_temp.rearLeftMotorTemperature;      // Range 20-130
	 		 frontRightInverterTemperature = pc_temp.frontRightInverterTemperature; // Range 20-100
	 		 frontLeftInverterTemperature = pc_temp.frontLeftInverterTemperature;  // Range 20-100
	 		 frontRightMotorTemperature = pc_temp.frontRightMotorTemperature;    // Range 20-130
	 		 frontLeftMotorTemperature = pc_temp.frontLeftMotorTemperature;     // Range
	 	  }


	  SystemStatus currentStatus = get_system_status_from_channels();

	  PUTM_CAN::PduChannel pdu_channel{
	      .pc_status = currentStatus.pc_status,
	      .fan_status = currentStatus.fan_status,
	      .pump_status = currentStatus.pump_status,
	      .inverter_status = currentStatus.inverter_status,
	      .fbox_status = currentStatus.fbox_status,
	      .sdc_status = currentStatus.sdc_status,
	      .dash_status = currentStatus.dash_status,
	      .tsal_hv_status = currentStatus.tsal_hv_status,
	      .rbox_diagport_brake_l_status = currentStatus.rbox_diagport_brake_l_status,
	      .brake_ir_air_status = currentStatus.brake_ir_air_status
	  };


	  PUTM_CAN::PduData pdu_data {
	      .pc_current = current_sum({{0, 0}, {0, 1}, {0, 2}, {0, 3}}),               // IC0 ch0–3
	      .pump_current = current_sum({{1, 1}, {1, 3}}),                             // IC1 ch1, ch3
	      .fan_current = current_sum({{1, 0}, {1, 2}}),                              // IC1 ch0, ch2
	      .inverter_current = current_sum({{2, 0}, {2, 1}}),                         // IC2 ch0, ch1
	      .fbox_current = fuse_currents[2][2],                                      // IC2 ch2
	      .sdc_current = fuse_currents[2][3],                                       // IC2 ch3
	      .total_current = current_sum({                                           // All 16 channels
	          {0, 0}, {0, 1}, {0, 2}, {0, 3},
	          {1, 0}, {1, 1}, {1, 2}, {1, 3},
	          {2, 0}, {2, 1}, {2, 2}, {2, 3},
	          {3, 0}, {3, 1}, {3, 2}, {3, 3}
	      })
	  };


	  auto pdu_data_msg = PUTM_CAN::Can_tx_message<PUTM_CAN::PduData>(pdu_data, PUTM_CAN::can_tx_header_PDU_DATA);
	  auto pdu_channel_msg = PUTM_CAN::Can_tx_message<PUTM_CAN::PduChannel>(pdu_channel, PUTM_CAN::can_tx_header_PDU_CHANNEL);

	  // co 40ms wysyłamy
	  if (now >= can_pdu_channel_tick)
	  {
	      auto status_channel = pdu_channel_msg.send(hfdcan1);
	      can_pdu_channel_tick = now + 40; // 40 ms
	      CanErrorCommunication = (status_channel == HAL_OK) ? 0 : 1;
	  }

	  // co 200ms wysyłamy
	  if (now >= can_pdu_data_tick)
	  {
	      auto status_data = pdu_data_msg.send(hfdcan1);
	      can_pdu_data_tick = now + 200; // 200 ms
	      CanErrorCommunication = (status_data == HAL_OK) ? 0 : 1;
	  }


	  // tx_buffer[1] = fuse4 (IC3), ..., tx_buffer[4] = fuse1 (IC0)
	  // OPTIONAL: Read diagnostic registers to check for critical errors
	  uint8_t tx_buffer_diag[4] = {ERRDIAG, 0, 0, 0};
	  uint8_t rx_buffer_diag[4];

	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer_diag, rx_buffer_diag, 4, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

	  // If all controllers report a major failure, turn all LEDs ON permanently
	  if ((rx_buffer_diag[1] & 0xFF) == 0xFF &&
	      (rx_buffer_diag[2] & 0xFF) == 0xFF &&
	      (rx_buffer_diag[3] & 0xFF) == 0xFF &&
	      (rx_buffer_diag[4] & 0xFF) == 0xFF)
	  {
	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

	      while (1);  // Stop execution if all controllers fail
	  }

	  // If no failure, continue blinking
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);



//current sense mode activation ----------

	  // tx_buffer[1] = fuse4 (IC3), ..., tx_buffer[4] = fuse1 (IC0)
	  // ADC channels (reverse mapped):
	  // adc_buffer[3] = IC0 (fuse1)
	  // adc_buffer[2] = IC1 (fuse2)
	  // adc_buffer[1] = IC2 (fuse3)
	  // adc_buffer[0] = IC3 (fuse4)

	  // set channel 0 - 7A // value fuse = 217,4*current + 93
	  tx_buffer[0] = DCR_CHANNEL0;
	  tx_buffer[1] = DCR_CHANNEL0; //closes fuse4
	  tx_buffer[2] = DCR_CHANNEL0;
	  tx_buffer[3] = DCR_CHANNEL0;
	  tx_buffer[4] = DCR_CHANNEL0;
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  // check current on channel 0 - 7A
	  fuse_currents[0][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0 - CH0
	  fuse_currents[1][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1 - CH0
	  fuse_currents[2][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2 - CH0
	  fuse_currents[3][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3 - CH0

	  // set channel 1 - 4A // value fuse = 217,4*current + 93
	  tx_buffer[0] = DCR_CHANNEL1;
	  tx_buffer[1] = DCR_CHANNEL1;
	  tx_buffer[2] = DCR_CHANNEL1;
	  tx_buffer[3] = DCR_CHANNEL1;
	  tx_buffer[4] = DCR_CHANNEL1;
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  // check current on channel 1 - 4A
	  fuse_currents[0][1] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
	  fuse_currents[1][1] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
	  fuse_currents[2][1] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
	  fuse_currents[3][1] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3

	  // set channel 2 - 4A // value fuse = 217,4*current + 93
	  tx_buffer[0] = DCR_CHANNEL2;
	  tx_buffer[1] = DCR_CHANNEL2;
	  tx_buffer[2] = DCR_CHANNEL2;
	  tx_buffer[3] = DCR_CHANNEL2;
	  tx_buffer[4] = DCR_CHANNEL2;
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  // check current on channel 2 - 4A
	  fuse_currents[0][2] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
	  fuse_currents[1][2] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
	  fuse_currents[2][2] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
	  fuse_currents[3][2] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3


	  // set channel 3 - 7A // value fuse = 217,4*current + 93
	  tx_buffer[0] = DCR_CHANNEL3;
	  tx_buffer[1] = DCR_CHANNEL3;
	  tx_buffer[2] = DCR_CHANNEL3;
	  tx_buffer[3] = DCR_CHANNEL3;
	  tx_buffer[4] = DCR_CHANNEL3;
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  // check current on channel 3 - 7A // value fuse = 217,4*current + 93
	  fuse_currents[0][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
	  fuse_currents[1][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
	  fuse_currents[2][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
	  fuse_currents[3][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3
	  //activating each channel individually by calling a function



	  HAL_Delay(100);

	  tx_buffer[0] = OUT_READ;
	  tx_buffer[1] = OUT_READ;
	  tx_buffer[2] = OUT_READ;
	  tx_buffer[3] = OUT_READ;
	  tx_buffer[4] = OUT_READ;
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);

	  if (!RTD_status) {
	      uint8_t ic = 1; // IC1 only
	      uint8_t new_state = 0x00;

	      bool fan_on = rearLeftInverterTemperature >= 40 ||
	                    rearRightInverterTemperature >= 40 ||
	                    frontLeftInverterTemperature >= 40 ||
	                    frontRightInverterTemperature >= 40;

	      bool fan_off = rearLeftInverterTemperature <= 30 &&
	                     rearRightInverterTemperature <= 30 &&
	                     frontLeftInverterTemperature <= 30 &&
	                     frontRightInverterTemperature <= 30;

	      bool pump_on = rearLeftMotorTemperature >= 40 ||
	                     rearRightMotorTemperature >= 40;

	      bool pump_off = rearLeftMotorTemperature <= 30 &&
	                      rearRightMotorTemperature <= 30;

	      if (fan_on) new_state |= (1 << 0) | (1 << 2);
	      if (pump_on) new_state |= (1 << 1) | (1 << 3);

	      for (uint8_t ch = 0; ch < 4; ch++) {
	          if (new_state & (1 << ch)) {
	              if (!channel_disabled[ic][ch]) {
	                  channel_states[ic] |= (1 << ch);
	              }
	          } else {
	              channel_states[ic] &= ~(1 << ch);
	          }
	      }

	      uint8_t tx_index = get_tx_index(ic);
	      for (int i = 0; i < 5; i++) tx_buffer[i] = DCR_ACTIVE;
	      tx_buffer[tx_index] = 0x80 | (channel_states[ic] & 0x0F);

	      HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	      HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	      HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	  }


	  //retry logic
	  for (uint8_t ic = 0; ic < IC_COUNT; ic++) {
	      for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
	          handle_overcurrent(ic, ch, thresholds[ic][ch]);
	      }
	  }
	  //retry logic
	  for (uint8_t ic = 0; ic < IC_COUNT; ic++) {
	      for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
	          if (channel_disabled[ic][ch] && RTD_status) {
	              uint32_t now = HAL_GetTick();
	              uint32_t time_since = now - channel_last_attempt[ic][ch];

	              if ((channel_retry_count[ic][ch] == 0 && time_since >= 5000) ||
	                  (channel_retry_count[ic][ch] == 1 && time_since >= 10000))
	              {
	                  uint8_t adc_index = 3 - ic;
	                  fuse_currents[ic][ch] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
	                      __VREFANALOG_VOLTAGE__, adc_buffer[adc_index], ADC_RESOLUTION12b));

	                  if (fuse_currents[ic][ch] <= thresholds[ic][ch]) {
	                      channel_states[ic] |= (1 << ch);
	                      channel_disabled[ic][ch] = 0;
	                  }

	                  channel_retry_count[ic][ch]++;
	                  channel_last_attempt[ic][ch] = now;

	                  uint8_t tx_index = get_tx_index(ic);
	                  for (int i = 0; i < 5; i++) tx_buffer[i] = DCR_ACTIVE;
	                  tx_buffer[tx_index] = 0x80 | (channel_states[ic] & 0x0F);

	                  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
	                  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
	                  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
	              }

	              if (channel_retry_count[ic][ch] >= 2) {
	                  channel_disabled[ic][ch] = 0;
	              }
	          }
	      }
	  }

//previous retry logic
//	  if (any_channel_closed && (HAL_GetTick() - last_shutdown_time >= 5000))
//	  {
//	      any_channel_closed = false; // reset the flag
//
//	      for (uint8_t ic = 0; ic < IC_COUNT; ic++)
//	      {
//	          uint8_t adc_index = 3 - ic;
//	          for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++)
//	          {
//	              fuse_currents[ic][ch] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
//	                  __VREFANALOG_VOLTAGE__, adc_buffer[adc_index], ADC_RESOLUTION12b));
//
//	              if (fuse_currents[ic][ch] <= thresholds[ic][ch])
//	              {
//	                  channel_states[ic] |= (1 << ch); // enable channel
//	              }
//	          }
//
//	          // Send updated state
//	          uint8_t tx_index = get_tx_index(ic);
//	          for (int i = 0; i < 5; i++) tx_buffer[i] = DCR_ACTIVE;
//	          tx_buffer[tx_index] = 0x80 | (channel_states[ic] & 0x0F);
//
//	          HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);
//	          HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
//	          HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);
//	      }
//	  }


	 	  after_first_loop = 1;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
