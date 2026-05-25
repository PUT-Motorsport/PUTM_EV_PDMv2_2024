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
#include "PUTM_CAN_M.h"
#include "can_driver.hpp"
#include "pdu.hpp"

#include <algorithm>
#include <array>
#include <span>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
constexpr uint8_t ADC_BUF_SIZE{4};
constexpr uint32_t __VREFANALOG_VOLTAGE__{3300};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

template <size_t BUF_SIZE>
std::array<uint16_t, BUF_SIZE>
adc_to_mV(std::span<volatile uint16_t, BUF_SIZE> adc_buffer) {
  std::array<uint16_t, BUF_SIZE> voltages_mV{};
  for (size_t ch{}; ch < BUF_SIZE; ch++) {
    voltages_mV.at(ch) = __HAL_ADC_CALC_DATA_TO_VOLTAGE(
        __VREFANALOG_VOLTAGE__, adc_buffer[ch], ADC_RESOLUTION12b);
  }
  return voltages_mV;
}

volatile bool rtd_status{};

void can_pc_main_data_cb(const PUTM_CAN_M_pc_main_data_t &pc_main_data) {
  rtd_status = pc_main_data.rtd;
}

volatile Temperature::Values inv_temperature_values{};
volatile Temperature::Values motor_temperature_values{};

void can_pc_temperature_data_cb(
    const PUTM_CAN_M_pc_temperature_data_t &pc_temperature_data) {
  inv_temperature_values.front_left =
      pc_temperature_data.front_left_inverter_temperature;
  inv_temperature_values.front_right =
      pc_temperature_data.front_right_inverter_temperature;
  inv_temperature_values.rear_left =
      pc_temperature_data.rear_left_inverter_temperature;
  inv_temperature_values.rear_right =
      pc_temperature_data.rear_right_inverter_temperature;

  motor_temperature_values.front_left =
      pc_temperature_data.front_left_motor_temperature;
  motor_temperature_values.front_right =
      pc_temperature_data.front_right_motor_temperature;
  motor_temperature_values.rear_left =
      pc_temperature_data.rear_left_motor_temperature;
  motor_temperature_values.rear_right =
      pc_temperature_data.rear_right_motor_temperature;
}

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

  putm_ev_can::CanDriver can_m;
  can_m.RegisterCallback<PUTM_CAN_M_pc_main_data_t>(
      PUTM_CAN_M_PC_MAIN_DATA_FRAME_ID, can_pc_main_data_cb);
  can_m.RegisterCallback<PUTM_CAN_M_pc_temperature_data_t>(
      PUTM_CAN_M_PC_TEMPERATURE_DATA_FRAME_ID, can_pc_temperature_data_cb);

  constexpr uint32_t CAN_PDU_DATA_PERIOD{200};
  constexpr uint32_t CAN_PDU_CHANNEL_PERIOD{40};

  uint32_t can_pdu_data_tick{};
  uint32_t can_pdu_channel_tick{};

  constexpr uint8_t INV_MIN_TEMP{30};
  constexpr uint8_t INV_MAX_TEMP{40};
  constexpr uint8_t MOTOR_MIN_TEMP{30};
  constexpr uint8_t MOTOR_MAX_TEMP{40};

  constexpr std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                       Pdu::IC_COUNT>
      SYSTEM_DATA{{{{{Sys_name::INV2, 5000},
                     {Sys_name::INV1, 3000},
                     {Sys_name::RBOX_DIAG_BRAKE_L, 5000},
                     {Sys_name::TSAL_HV, 2000}}},

                   {{{Sys_name::DASH, 3000},
                     {Sys_name::SDC_ASMS, 1000},
                     {Sys_name::BRAKE_IR_AIR, 5000},
                     {Sys_name::FBOX, 5000}}},

                   {{{Sys_name::PC3, 4000},
                     {Sys_name::PC2, 5000},
                     {Sys_name::PC1, 5000},
                     {Sys_name::PC0, 4000}}},

                   {{{Sys_name::FAN2, 5000},
                     {Sys_name::PUMP2, 5000},
                     {Sys_name::PUMP1, 5000},
                     {Sys_name::FAN1, 5000}}}}};

  const std::array<Led, Pdu::IC_COUNT> LEDS{{{LED3_GPIO_Port, LED3_Pin},
                                             {LED0_GPIO_Port, LED0_Pin},
                                             {LED1_GPIO_Port, LED1_Pin},
                                             {LED2_GPIO_Port, LED2_Pin}}};

  static Pdu pdu{LEDS,
                 SYSTEM_DATA,
                 {INV_MIN_TEMP, INV_MAX_TEMP},
                 {MOTOR_MIN_TEMP, MOTOR_MAX_TEMP}};

  bool local_rtd{};
  Temperature::Values local_inv, local_motor;
  bool after_first_loop{false};

  pdu.init_chain();
  HAL_Delay(1);

  static volatile uint16_t adc_buffer[ADC_BUF_SIZE];
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_SIZE);

  pdu.start_chain();
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t tick_now{HAL_GetTick()};

    pdu.update_chain_errors();

    for (int i{}; i < BTS::Ic::CHANNEL_COUNT; i++) {
      pdu.set_channel_sense(i);
      HAL_Delay(1);
      pdu.update_channel_currents(i, adc_to_mV<ADC_BUF_SIZE>(adc_buffer),
                                  tick_now);
    }
    __disable_irq();
    local_rtd = rtd_status;
    local_inv.front_left = inv_temperature_values.front_left;
    local_inv.front_right = inv_temperature_values.front_right;
    local_inv.rear_left = inv_temperature_values.rear_left;
    local_inv.rear_right = inv_temperature_values.rear_right;
    local_motor.front_left = motor_temperature_values.front_left;
    local_motor.front_right = motor_temperature_values.front_right;
    local_motor.rear_left = motor_temperature_values.rear_left;
    local_motor.rear_right = motor_temperature_values.rear_right;
    __enable_irq();
    pdu.update_fans(local_rtd, local_inv, local_motor);

    if (after_first_loop)
      pdu.handle_overcurrent(tick_now);
    pdu.update_leds(tick_now);

    if (tick_now - can_pdu_channel_tick > CAN_PDU_CHANNEL_PERIOD) {
      can_m.Send(PUTM_CAN_M_PDU_CHANNNEL_FRAME_ID, pdu.get_can_pdu_channel_t());
      can_pdu_channel_tick = tick_now;
    }
    if (tick_now - can_pdu_data_tick > CAN_PDU_DATA_PERIOD) {
      can_m.Send(PUTM_CAN_M_PDU_DATA_FRAME_ID, pdu.get_can_pdu_data_t());
      can_pdu_data_tick = tick_now;
    }
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
