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
#include "stm32g0xx_hal.h"
#include <array>
#include <span>

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

// For individual channels 0 and 3 (returns 0.1A units as uint8_t)
uint32_t mv_to_hma(uint32_t mv) {
  if (mv < 123)
    return 0;
  return ((mv - 123) / 217.0f) * 10.0f;
}

// for channels 1 and 2
uint32_t mv_to_hma2(uint32_t mv) {
  if (mv < 123)
    return 0;
  return ((mv - 123) / 482.0f) * 10.0f;
}

using namespace BTS72220;
class Led {
public:
  const GPIO_TypeDef *port{nullptr};
  const uint16_t pin{0};

  Led(GPIO_TypeDef *port, uint16_t pin) : port{port}, pin{pin} {};

private:
  uint32_t blink_counter{};
  uint32_t blink_phase{};
  uint32_t last_toggle{};
  uint32_t pause_time{};
};

class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  Pdu(std::array<Led, IC_COUNT> leds,
      std::array<std::array<uint32_t, Ic::CHANNEL_COUNT>, Pdu::IC_COUNT>
          thresholds)
      : leds{leds} {
    int ic_count{};
    for (auto &ic : ics) {
      int channel_count{0};
      for (auto &channel : ic.channels) {
        channel.set_threshold(thresholds[ic_count][channel_count]);
        channel_count++;
      }
      ic_count++;
    }
  }

  bool init_ics() {
    HAL_GPIO_WritePin(LHI_1_GPIO_Port, LHI_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_2_GPIO_Port, LHI_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_3_GPIO_Port, LHI_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_4_GPIO_Port, LHI_4_Pin, GPIO_PIN_RESET);

    uint8_t tx_buffer[IC_COUNT + 1]{};
    tx_buffer[0] = OUT_READY;
    tx_buffer[1] = OUT_READY;
    tx_buffer[2] = OUT_READY;
    tx_buffer[3] = OUT_READY;
    tx_buffer[4] = OUT_READY;
    uint8_t rx_buffer[IC_COUNT + 1]{};
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    int ic_count{1};
    for (auto &ic : ics) {
      if (ic.check_diag(rx_buffer[ic_count])) {
        return true;
      }
      ic_count++;
    }
    return false;
  }

  bool start_ics() {
    uint8_t tx_buffer[IC_COUNT + 1]{};
    tx_buffer[0] = DCR_ACTIVE;
    tx_buffer[1] = DCR_ACTIVE;
    tx_buffer[2] = DCR_ACTIVE;
    tx_buffer[3] = DCR_ACTIVE;
    tx_buffer[4] = DCR_ACTIVE;
    uint8_t rx_buffer[IC_COUNT + 1]{};
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    return false;
  }

  bool set_channel_sense(uint8_t channel) {
    uint8_t dcr_channel{};
    switch (channel) {
    case 0: {
      dcr_channel = DCR_CHANNEL0;
      break;
    }
    case 1: {
      dcr_channel = DCR_CHANNEL1;
      break;
    }
    case 2: {
      dcr_channel = DCR_CHANNEL2;
      break;
    }
    case 3: {
      dcr_channel = DCR_CHANNEL3;
      break;
    }
    default: {
      return true;
    }
    }

    uint8_t tx_buffer[IC_COUNT + 1]{};
    tx_buffer[0] = dcr_channel;
    tx_buffer[1] = dcr_channel;
    tx_buffer[2] = dcr_channel;
    tx_buffer[3] = dcr_channel;
    tx_buffer[4] = dcr_channel;
    uint8_t rx_buffer[IC_COUNT + 1]{};
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    HAL_Delay(5);
    return false;
  }
  void update_channel_currents(uint8_t channel,
                               std::span<uint16_t, ADC_BUF_SIZE> adc_buffer) {
    int ic_count{0};
    for (auto &ic : ics) {
      uint32_t mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(
          __VREFANALOG_VOLTAGE__, adc_buffer[(IC_COUNT - 1) - ic_count],
          ADC_RESOLUTION12b);

      uint32_t current_val = (channel == 0 || channel == 3) ? mv_to_hma(mv) : mv_to_hma2(mv);

      ic.channels[channel].update_current(current_val, HAL_GetTick());
      ic_count++;
    }
  }

  void update_led_status(uint32_t now) {
    bool all_failed{true};

    for (auto &ic : ics) {
      uint8_t failed_channels{};
      for (auto channel : ic.channels) {
        if (channel.status == Channel::Status::ERR) {
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

  bool handle_overcurrent() {
    uint8_t tx_buffer[IC_COUNT + 1]{};
    tx_buffer[0] = OUT_CLOSE;
    tx_buffer[1] = OUT_CLOSE;
    tx_buffer[2] = OUT_CLOSE;
    tx_buffer[3] = OUT_CLOSE;
    tx_buffer[4] = OUT_CLOSE;

    int ic_count{0};
    for (auto &ic : ics) {
      int channel_count{0};
      for (auto &channel : ic.channels) {
        if (channel.status == Channel::Status::ERR) {
          channel.tick_last_attempt = HAL_GetTick();
          channel.status = Channel::Status::TEMP_LOCK;
        } else if (channel.status == Channel::Status::ON) {
          tx_buffer[IC_COUNT - ic_count] |= 1 << channel_count;
        }
        channel_count++;
      }
      ic_count++;
    }

    uint8_t rx_buffer[IC_COUNT + 1]{};
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                                100) != HAL_OK)
      return true;

    return false;
  }

private:
  std::array<Led, IC_COUNT> leds;
  std::array<Ic, IC_COUNT> ics{};
  struct {
    Channel::Status pc;
    Channel::Status fan;
    Channel::Status pump;
    Channel::Status inverter;
    Channel::Status fbox;
    Channel::Status sdc;
    Channel::Status dash;
    Channel::Status tsal_hv;
    Channel::Status rbox_diagport_brake_l;
    Channel::Status brake_ir_air;
  } system_status;

  uint32_t total_current{};
};

class Temperature {
public:
  const uint8_t min;
  const uint8_t max;

  Temperature(const uint8_t min_temperature, const uint8_t max_temperature)
      : min{min_temperature}, max{max_temperature} {}

  bool check(uint8_t value) {
    if (value < min || value > max)
      return true;
    return false;
  }

  bool update(uint8_t front_left_value, uint8_t front_right_value,
              uint8_t rear_left_value, uint8_t rear_right_value) {
    front_left = front_left_value;
    front_right = front_right_value;
    rear_left = rear_left_value;
    rear_right = rear_right_value;
    if (check(front_left) || check(front_right) || check(rear_left) ||
        check(rear_right)) {
      return true;
    }
    return false;
  }

private:
  uint8_t front_left{};
  uint8_t front_right{};
  uint8_t rear_left{};
  uint8_t rear_right{};
};

/* USER CODE END PV */

/* Private function prototypes
 * -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
 * ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
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

  std::array<std::array<uint32_t, Ic::CHANNEL_COUNT>, Pdu::IC_COUNT> thresholds{
      {
          {I_MAX.INV2, I_MAX.INV1, I_MAX.RBOX_DIAG_BRAKE_L, I_MAX.TSAL_HV},
          {I_MAX.DASH, I_MAX.SDC_ASMS, I_MAX.BRAKE_IR_AIR, I_MAX.FBOX},
          {I_MAX.PC4, I_MAX.PC3, I_MAX.PC2, I_MAX.PC1},
          {I_MAX.FAN2, I_MAX.PUMP2, I_MAX.PUMP1, I_MAX.FAN1},
      }};

  std::array<Led, Pdu::IC_COUNT> leds{{{LED1_GPIO_Port, LED1_Pin},
                                       {LED2_GPIO_Port, LED2_Pin},
                                       {LED3_GPIO_Port, LED3_Pin},
                                       {LED4_GPIO_Port, LED4_Pin}}};
  Pdu pdu{leds, thresholds};
  Temperature inv_temperature{20, 100};
  Temperature motor_temperature{20, 130};

  pdu.init_ics();

  bool RTD_status;

  bool after_first_loop{false};
  uint16_t adc_buffer[ADC_BUF_SIZE];
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_SIZE);

  pdu.start_ics();

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint8_t tx_buffer_diag[4] = {ERRDIAG_CMD, 0, 0, 0};
    uint8_t rx_buffer_diag[4];

    HAL_SPI_TransmitReceive(&hspi1, tx_buffer_diag, rx_buffer_diag, 4, 100);

    for (int i{}; i < Ic::CHANNEL_COUNT; i++) {
      pdu.set_channel_sense(i);
      pdu.update_channel_currents(i, adc_buffer);
    }

    HAL_Delay(100);

    if (after_first_loop)
      pdu.handle_overcurrent();

    pdu.update_total_current();

    // update_led_status(HAL_GetTick());

    after_first_loop = true;

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
  /* User can add his own implementation to report the HAL error return state
   */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
