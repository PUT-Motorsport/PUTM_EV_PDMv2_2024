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
// fuse4
// fuse3
// fuse2
// fuse1

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BTS72220.hpp"
#include <array>

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
//  - IC index 0 = fuse1 (bottom), IC 1 = fuse2, IC 2 = fuse3, IC 3 = fuse4
//  (top)
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
//  - If current exceeds threshold, the corresponding channel is disabled via
//  SPI
//  - After 5s, disabled channels are retried if current drops below threshold
//
//  LOGICAL OUTPUTS:
//
//  - Physical channels grouped into 10 logical outputs (e.g. pc, pump, fan,
//  inverter)
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
 * - Example: If LED2 blinks 3 times, pauses, then repeats → Controller 2,
 * Channel 2 failure.
 * - Example: If LED4 blinks fast (5 Hz) → Controller 4 has multiple channel
 * failures.
 * - Example: If all LEDs are ON → System critical failure.
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
constexpr uint8_t ADC_BUF_SIZE{4};
constexpr uint32_t __VREFANALOG_VOLTAGE__{3300};

constexpr uint32_t MAX_RETRIES{5};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
using namespace BTS72220;
class Led {
public:
  Led(GPIO_TypeDef *port, uint16_t pin) : port{port}, pin{pin} {};

private:
  const GPIO_TypeDef *port;
  const uint16_t pin;

  uint32_t blink_counter{};
  uint32_t blink_phase{};
  uint32_t last_toggle{};
  uint32_t pause_time{};
};

class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  void update_led_status(uint32_t now) {
    bool all_failed{true};

    for (auto &ic : ics) {
      uint8_t failed_channels{};
      for (auto channel : ic.channels) {
        if (channel.get_status() == Channel::Status::ERR) {
          failed_channels++;
        }
      }
    }
  }

  uint32_t update_total_current() {
    uint32_t sum{};
    for (auto &ic : ics) {
      for (auto channel : ic.channels) {
        sum +=
            channel
                .get_current(); // If channel is off, current is 0 → no effect
      }
    }
    total_current = sum;
    return sum;
  }

private:
  std::array<Led, IC_COUNT> leds;
  std::array<Ic, IC_COUNT> ics{};
  struct {
    const Channel::Status &pc_status;
    const Channel::Status &fan_status;
    const Channel::Status &pump_status;
    const Channel::Status &inverter_status;
    const Channel::Status &fbox_status;
    const Channel::Status &sdc_status;
    const Channel::Status &dash_status;
    const Channel::Status &tsal_hv_status;
    const Channel::Status &rbox_diagport_brake_l_status;
    const Channel::Status &brake_ir_air_status;
  } system_status;

  uint32_t total_current{};
};

// ADC
uint16_t adc_buffer[ADC_BUF_SIZE];
uint8_t adc_ready = 0;

uint8_t tx_buffer[5];
uint8_t rx_buffer[5];

bool after_first_loop{false};

static constexpr struct {
  uint32_t INV2{50};
  uint32_t INV1{30};
  uint32_t RBOX_DIAG_BRAKE_L{50};
  uint32_t TSAL_HV{20};

  uint32_t DASH{30};
  uint32_t SDC_ASMS{10};
  uint32_t BRAKE_IR_AIR{50};
  uint32_t FBOX{50};

  uint32_t PC4{40};
  uint32_t PC3{50};
  uint32_t PC2{50};
  uint32_t PC1{40};

  uint32_t FAN2{50};
  uint32_t PUMP2{50};
  uint32_t PUMP1{50};
  uint32_t FAN1{50};
} I_MAX; // Current thresholds

std::array<std::array<uint32_t, Ic::CHANNEL_COUNT>, Pdu::IC_COUNT> thresholds{{
    {I_MAX.INV2, I_MAX.INV1, I_MAX.RBOX_DIAG_BRAKE_L, I_MAX.TSAL_HV}, // IC0
    {I_MAX.DASH, I_MAX.SDC_ASMS, I_MAX.BRAKE_IR_AIR, I_MAX.FBOX},     // IC1
    {I_MAX.PC4, I_MAX.PC3, I_MAX.PC2, I_MAX.PC1},                     // IC2
    {I_MAX.FAN2, I_MAX.PUMP2, I_MAX.PUMP1, I_MAX.FAN1},               // IC3
}};

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
int get_tx_index(uint8_t ic_index) { return 1 + ic_index; }

// For individual channels 0 and 3 (returns 0.1A units as uint8_t)
uint8_t mv_to_hma(uint32_t mv) {
  if (mv < 123)
    return 0;
  return (uint8_t)(((mv - 123) / 217.0f) * 10.0f);
}

// for channels 1 and 2
uint8_t mv_to_hma2(uint32_t mv) {
  if (mv < 123)
    return 0;
  return (uint8_t)(((mv - 123) / 482.0f) * 10.0f);
}

void handle_overcurrent(uint8_t ic_index, uint8_t channel_number,
                        uint8_t threshold) {
  if (fuse_currents[ic_index][channel_number] > threshold && after_first_loop &&
      !channel_permanently_disabled[ic_index][channel_number]) {
    int tx_index = get_tx_index(ic_index);

    channel_states[ic_index] &= ~(1 << channel_number);
    channel_disabled[ic_index][channel_number] = 1;
    channel_last_attempt[ic_index][channel_number] = HAL_GetTick();

    channel_last_attempt[ic_index][channel_number] = HAL_GetTick();

    for (int i = 0; i < 5; i++)
      tx_buffer[i] = DCR_ACTIVE;
    tx_buffer[tx_index] = 0x80 | (channel_states[ic_index] & 0x0F);

    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
  }
}

ChannelStatus reduce_status(ChannelStatus a, ChannelStatus b) {
  if (a == STATUS_LOCK || b == STATUS_LOCK)
    return STATUS_LOCK;
  if (a == STATUS_ERR || b == STATUS_ERR)
    return STATUS_ERR;
  if (a == STATUS_OFF || b == STATUS_OFF)
    return STATUS_OFF;
  return STATUS_ON;
}

bool is_temp_high() {
  return rearRightInverterTemperature >= 40 ||
         rearLeftInverterTemperature >= 40 || rearRightMotorTemperature >= 40 ||
         rearLeftMotorTemperature >= 40 ||
         frontRightInverterTemperature >= 40 ||
         frontLeftInverterTemperature >= 40 ||
         frontRightMotorTemperature >= 40 || frontLeftMotorTemperature >= 40;
}

bool is_temp_low() {
  return rearRightInverterTemperature < 30 &&
         rearLeftInverterTemperature < 30 && rearRightMotorTemperature < 30 &&
         rearLeftMotorTemperature < 30 && frontRightInverterTemperature < 30 &&
         frontLeftInverterTemperature < 30 && frontRightMotorTemperature < 30 &&
         frontLeftMotorTemperature < 30;
}

void update_fan_channel_logic() {
  // Update state based on RTD
  //    if (RTD_status) {
  if (RTD_status) {
    fan_forced_on_by_rtd = true;
  } else {
    fan_forced_on_by_rtd = false;

    if (!fan_temp_triggered && is_temp_high()) {
      fan_temp_triggered = true;
    } else if (fan_temp_triggered && is_temp_low()) {
      fan_temp_triggered = false;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler(); // Only once here
  }

  // tx_buffer[1] = fuse4 (IC3), ..., tx_buffer[4] = fuse1 (IC0)

  HAL_GPIO_WritePin(LHI_1_GPIO_Port, LHI_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LHI_2_GPIO_Port, LHI_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LHI_3_GPIO_Port, LHI_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LHI_4_GPIO_Port, LHI_4_Pin, GPIO_PIN_RESET);
  // sleep -> ready
  tx_buffer[0] = OUT_READY;
  tx_buffer[1] = OUT_READY;
  tx_buffer[2] = OUT_READY;
  tx_buffer[3] = OUT_READY;
  tx_buffer[4] = OUT_READY;
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);
  // ready -> active
  tx_buffer[0] = DCR_ACTIVE;
  tx_buffer[1] = DCR_ACTIVE; // fuse 4
  tx_buffer[2] = DCR_ACTIVE; // fuse 3
  tx_buffer[3] = DCR_ACTIVE; // fuse 2
  tx_buffer[4] = DCR_ACTIVE; // fuse 1
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer), 100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_SIZE);

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    uint32_t now = HAL_GetTick();

    // // CAN
    // if (PUTM_CAN::can.get_pc_new_data()) {
    //   auto pc_data = PUTM_CAN::can.get_pc_main_data();
    //   RTD_status = pc_data.rtd;
    // }

    // if (PUTM_CAN::can.get_pc_temperature_data_new_data()) {
    //   auto pc_temp = PUTM_CAN::can.get_pc_temperature_data();

    //   rearRightInverterTemperature =
    //       pc_temp.rearRightInverterTemperature; // Range 20-100
    //   rearLeftInverterTemperature =
    //       pc_temp.rearLeftInverterTemperature; // Range 20-100
    //   rearRightMotorTemperature =
    //       pc_temp.rearRightMotorTemperature; // Range 20-130
    //   rearLeftMotorTemperature =
    //       pc_temp.rearLeftMotorTemperature; // Range 20-130
    //   frontRightInverterTemperature =
    //       pc_temp.frontRightInverterTemperature; // Range 20-100
    //   frontLeftInverterTemperature =
    //       pc_temp.frontLeftInverterTemperature; // Range 20-100
    //   frontRightMotorTemperature =
    //       pc_temp.frontRightMotorTemperature; // Range 20-130
    //   frontLeftMotorTemperature = pc_temp.frontLeftMotorTemperature; // Range
    // }

    // SystemStatus currentStatus = get_system_status_from_channels();

    // PUTM_CAN::PduChannel pdu_channel{
    //     .pc_status = currentStatus.pc_status,
    //     .fan_status = currentStatus.fan_status,
    //     .pump_status = currentStatus.pump_status,
    //     .inverter_status = currentStatus.inverter_status,
    //     .fbox_status = currentStatus.fbox_status,
    //     .sdc_status = currentStatus.sdc_status,
    //     .dash_status = currentStatus.dash_status,
    //     .tsal_hv_status = currentStatus.tsal_hv_status,
    //     .rbox_diagport_brake_l_status =
    //         currentStatus.rbox_diagport_brake_l_status,
    //     .brake_ir_air_status = currentStatus.brake_ir_air_status};

    // PUTM_CAN::PduData pdu_data{
    //     .pc_current =
    //         current_sum({{1, 0}, {1, 1}, {1, 2}, {1, 3}}), // IC0 ch0–3
    //     .pump_current = current_sum({{0, 1}, {0, 2}}),     // IC1 ch1, ch3
    //     .fan_current = current_sum({{0, 0}, {0, 3}}),      // IC1 ch0, ch2
    //     .inverter_current = current_sum({{3, 0}, {3, 1}}), // IC2 ch0, ch1
    //     .fbox_current = fuse_currents[2][3],               // IC2 ch2
    //     .sdc_current = fuse_currents[2][1],                // IC2 ch3
    //     .total_current = total_current_calc(fuse_currents)};

    pdu.update_total_current();
    // auto pdu_data_msg = PUTM_CAN::Can_tx_message<PUTM_CAN::PduData>(
    //     pdu_data, PUTM_CAN::can_tx_header_PDU_DATA);
    // auto pdu_channel_msg = PUTM_CAN::Can_tx_message<PUTM_CAN::PduChannel>(
    //     pdu_channel, PUTM_CAN::can_tx_header_PDU_CHANNEL);

    // // co 40ms wysyłamy
    // if (now >= can_pdu_channel_tick) {
    //   auto status_channel = pdu_channel_msg.send(hfdcan1);
    //   can_pdu_channel_tick = now + 40; // 40 ms
    //   CanErrorCommunication = (status_channel == HAL_OK) ? 0 : 1;
    // }

    // // co 200ms wysyłamy
    // if (now >= can_pdu_data_tick) {
    //   auto status_data = pdu_data_msg.send(hfdcan1);
    //   can_pdu_data_tick = now + 200; // 200 ms
    //   CanErrorCommunication = (status_data == HAL_OK) ? 0 : 1;
    // }

    update_fan_channel_logic();

    // tx_buffer[1] = fuse4 (IC3), ..., tx_buffer[4] = fuse1 (IC0)
    // OPTIONAL: Read diagnostic registers to check for critical errors
    uint8_t tx_buffer_diag[4] = {ERRDIAG, 0, 0, 0};
    uint8_t rx_buffer_diag[4];

    HAL_SPI_TransmitReceive(&hspi1, tx_buffer_diag, rx_buffer_diag, 4, 100);

    // set channel 0 - 7A // value fuse = 217,4*current + 93
    tx_buffer[0] = DCR_CHANNEL0;
    tx_buffer[1] = DCR_CHANNEL0; // closes fuse4
    tx_buffer[2] = DCR_CHANNEL0;
    tx_buffer[3] = DCR_CHANNEL0;
    tx_buffer[4] = DCR_CHANNEL0;
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    // check current on channel 0 - 7A
    fuse_currents[0][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0 - CH0
    fuse_currents[1][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1 - CH0
    fuse_currents[2][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2 - CH0
    fuse_currents[3][0] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3 - CH0

    // set channel 1 - 4A // value fuse = 217,4*current + 93
    tx_buffer[0] = DCR_CHANNEL1;
    tx_buffer[1] = DCR_CHANNEL1;
    tx_buffer[2] = DCR_CHANNEL1;
    tx_buffer[3] = DCR_CHANNEL1;
    tx_buffer[4] = DCR_CHANNEL1;
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    // check current on channel 1 - 4A
    fuse_currents[0][1] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
    fuse_currents[1][1] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
    fuse_currents[2][1] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
    fuse_currents[3][1] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3

    // set channel 2 - 4A // value fuse = 217,4*current + 93
    tx_buffer[0] = DCR_CHANNEL2;
    tx_buffer[1] = DCR_CHANNEL2;
    tx_buffer[2] = DCR_CHANNEL2;
    tx_buffer[3] = DCR_CHANNEL2;
    tx_buffer[4] = DCR_CHANNEL2;
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    // check current on channel 2 - 4A
    fuse_currents[0][2] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
    fuse_currents[1][2] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
    fuse_currents[2][2] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
    fuse_currents[3][2] = mv_to_hma2(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3

    // set channel 3 - 7A // value fuse = 217,4*current + 93
    tx_buffer[0] = DCR_CHANNEL3;
    tx_buffer[1] = DCR_CHANNEL3;
    tx_buffer[2] = DCR_CHANNEL3;
    tx_buffer[3] = DCR_CHANNEL3;
    tx_buffer[4] = DCR_CHANNEL3;
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    // check current on channel 3 - 7A // value fuse = 217,4*current + 93
    fuse_currents[0][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[3], ADC_RESOLUTION12b)); // IC0
    fuse_currents[1][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[2], ADC_RESOLUTION12b)); // IC1
    fuse_currents[2][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[1], ADC_RESOLUTION12b)); // IC2
    fuse_currents[3][3] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[0], ADC_RESOLUTION12b)); // IC3
    // activating each channel individually by calling a function

    HAL_Delay(100);

    tx_buffer[0] = OUT_READ;
    tx_buffer[1] = OUT_READ;
    tx_buffer[2] = OUT_READ;
    tx_buffer[3] = OUT_READ;
    tx_buffer[4] = OUT_READ;
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);

    // closing logic
    //  Step 1: Overcurrent handling (disables channels if needed)
    for (uint8_t ic = 0; ic < IC_COUNT; ic++) {
      for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        handle_overcurrent(ic, ch, thresholds[ic][ch]);
      }
    }

    // Step 2: Retry logic
    for (uint8_t ic = 0; ic < IC_COUNT; ic++) {
      for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        if (channel_disabled[ic][ch] && !channel_permanently_disabled[ic][ch]) {
          uint32_t now = HAL_GetTick();
          uint32_t time_since = now - channel_last_attempt[ic][ch];

          if ((channel_retry_count[ic][ch] == 0 && time_since >= 5000) ||
              (channel_retry_count[ic][ch] == 1 && time_since >= 10000) ||
              (channel_retry_count[ic][ch] == 2 && time_since >= 15000) ||
              (channel_retry_count[ic][ch] == 3 && time_since >= 20000) ||
              (channel_retry_count[ic][ch] == 4 && time_since >= 25000)) {

            uint8_t adc_index = 3 - ic;
            fuse_currents[ic][ch] = mv_to_hma(__HAL_ADC_CALC_DATA_TO_VOLTAGE(
                __VREFANALOG_VOLTAGE__, adc_buffer[adc_index],
                ADC_RESOLUTION12b));

            if (fuse_currents[ic][ch] <= thresholds[ic][ch]) {
              if (channel_retry_count[ic][ch] + 1 < MAX_RETRIES) {
                channel_states[ic] |= (1 << ch);
                channel_disabled[ic][ch] = 0;
              }
            }

            channel_retry_count[ic][ch]++;
            channel_last_attempt[ic][ch] = now;

            uint8_t tx_index = get_tx_index(ic);
            for (int i = 0; i < 5; i++)
              tx_buffer[i] = DCR_ACTIVE;
            tx_buffer[tx_index] = 0x80 | (channel_states[ic] & 0x0F);

            HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
          }

          // Final retry -> permanently disable and force off
          if (channel_retry_count[ic][ch] >= MAX_RETRIES) {
            channel_disabled[ic][ch] = 1;
            channel_permanently_disabled[ic][ch] = 1;
            channel_states[ic] &= ~(1 << ch); // 🔒 Force OFF
          }
        }
      }
    }
    // Apply fan logic to IC1 (index 1)
    if (fan_forced_on_by_rtd || fan_temp_triggered) {
      for (uint8_t ch = 0; ch < 4; ch++) {
        if (!channel_disabled[0][ch] && !channel_permanently_disabled[0][ch]) {
          channel_states[0] |= (1 << ch); // turn ON if allowed
        }
      }
    } else {
      for (uint8_t ch = 0; ch < 4; ch++) {
        channel_states[0] &= ~(1 << ch); // turn OFF unconditionally
      }
    }

    // Step 3: Send updated channel states to all ICs (only once)
    for (uint8_t ic = 0; ic < IC_COUNT; ic++) {
      uint8_t tx_index = get_tx_index(ic);
      for (int i = 0; i < 5; i++)
        tx_buffer[i] = DCR_ACTIVE;
      tx_buffer[tx_index] = 0x80 | (channel_states[ic] & 0x0F);

      HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 5, 100);
    }

    update_led_status(HAL_GetTick());

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
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
