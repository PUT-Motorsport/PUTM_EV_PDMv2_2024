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
#include "PUTM_CAN_M.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BTS72220.hpp"
#include "can_driver.hpp"

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
enum class System_name {
  INV2,
  INV1,
  RBOX_DIAG_BRAKE_L,
  TSAL_HV,

  DASH,
  SDC_ASMS,
  BRAKE_IR_AIR,
  FBOX,

  PC3,
  PC2,
  PC1,
  PC0,

  FAN2,
  PUMP2,
  PUMP1,
  FAN1,

  COUNT,
};

struct System {
  System_name name;
  uint16_t threshold;
};

// For individual channels 0 and 3 (returns 0.1A units as uint8_t) - do
// wyjebania
uint16_t mv_to_hma(uint16_t mv) {
  if (mv < 123)
    return 0;
  return (((mv - 123) * 1000) / 217 + 100);
}

// for channels 1 and 2
uint16_t mv_to_hma2(uint16_t mv) {
  if (mv < 123)
    return 0;
  return (((mv - 123) * 1000) / 482 + 50);
}

// Translate channel status to can frame data displayed on dash
uint8_t ch_status_can(const BTS::Channel ch) {
  switch (ch.status) {
  case BTS::Channel::Status::OFF:
    return 0;
  case BTS::Channel::Status::ERR:
  case BTS::Channel::Status::ON:
    return 1;
  case BTS::Channel::Status::TEMP_LOCK:
    return 2;
  case BTS::Channel::Status::PERM_LOCK:
    return 3;
  }
  return 0;
}

/* Led blinking indicates each IC channels status:
  OFF - all channels OK



  ON - all channels ERROR
*/
class Led {
public:
  Led(GPIO_TypeDef *port, uint16_t pin) : port{port}, pin{pin} {};

  // Update single Led state based on failed channels count
  bool update(uint8_t channels_failed, uint32_t tick_now) {
    if (channels_failed > BTS::Ic::CHANNEL_COUNT) {
      return true;
    } else if (channels_failed == BTS::Ic::CHANNEL_COUNT) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
      return false;
    } else if (channels_failed == 0) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
      return false;
    } else {
      toggle_time = 3000 / channels_failed;
      if (tick_now - last_toggle >= toggle_time) {
        last_toggle = tick_now;
        HAL_GPIO_TogglePin(port, pin);
      }
      return false;
    }
  }

private:
  GPIO_TypeDef *port;
  uint16_t pin;

  uint32_t toggle_time{};
  uint32_t last_toggle{};
};

class Temperature {
public:
  const uint8_t min;
  const uint8_t max;
  struct Values {
    uint8_t front_left{};
    uint8_t front_right{};
    uint8_t rear_left{};
    uint8_t rear_right{};
  };

  Temperature(const uint8_t min_temperature, const uint8_t max_temperature)
      : min{min_temperature}, max{max_temperature} {}

  bool check(uint8_t value) { return (value < min || value > max); }

  bool update(Values new_values) {
    values.front_left = new_values.front_left;
    values.front_right = new_values.front_right;
    values.rear_left = new_values.rear_left;
    values.rear_right = new_values.rear_right;

    if (check(values.front_left) || check(values.front_right) ||
        check(values.rear_left) || check(values.rear_right))
      return true;

    return false;
  }

private:
  Values values{};
};

// Base class that controls all ICs
class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  Pdu(std::array<Led, IC_COUNT> leds,
      const std::array<std::array<System, BTS::Ic::CHANNEL_COUNT>,
                       Pdu::IC_COUNT> &systems_data,
      Temperature inv_temperature, Temperature motor_temperature)
      : leds{leds[0], leds[1], leds[2], leds[3]},
        inv_temperature{inv_temperature}, motor_temperature{motor_temperature} {

    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
        systems_channel_map[static_cast<uint8_t>(
            systems_data.at(ic_idx).at(ch_idx).name)] = ic_idx + ch_idx;

        ics.at(ic_idx).channels.at(ch_idx).set_threshold(
            systems_data.at(ic_idx).at(ch_idx).threshold);
      }
    }
  }

  const BTS::Channel &get_channel(System_name name) {
    size_t index = systems_channel_map.at(static_cast<size_t>(name));
    auto ic_index{index / IC_COUNT};
    auto channel_index{index % BTS::Ic::CHANNEL_COUNT};
    return ics.at(ic_index).channels.at(channel_index);
  }

  PUTM_CAN_M_pdu_channnel_t get_can_pdu_channel_t() {
    return {
        .pc_status{std::min({ch_status_can(get_channel(System_name::PC0)),
                             ch_status_can(get_channel(System_name::PC1)),
                             ch_status_can(get_channel(System_name::PC2)),
                             ch_status_can(get_channel(System_name::PC3))})},
        .fan_status{std::min({ch_status_can(get_channel(System_name::FAN1)),
                              ch_status_can(get_channel(System_name::FAN2))})},
        .pump_status{
            std::min({ch_status_can(get_channel(System_name::PUMP1)),
                      ch_status_can(get_channel(System_name::PUMP2))})},
        .inverter_status{
            std::min({ch_status_can(get_channel(System_name::INV1)),
                      ch_status_can(get_channel(System_name::INV2))})},
        .fbox_status{ch_status_can(get_channel(System_name::FBOX))},
        .sdc_status{ch_status_can(get_channel(System_name::SDC_ASMS))},
        .dash_status{ch_status_can(get_channel(System_name::DASH))},
        .tsal_hv_status{ch_status_can(get_channel(System_name::TSAL_HV))},
        .rbox_diagport_brake_l_status{
            ch_status_can(get_channel(System_name::RBOX_DIAG_BRAKE_L))},
        .brake_ir_air_status{
            ch_status_can(get_channel(System_name::BRAKE_IR_AIR))},
    };
  }

  PUTM_CAN_M_pdu_data_t get_can_pdu_data_t() {
    return {
        .pc_current{get_channel(System_name::PC0).get_current() +
                    get_channel(System_name::PC1).get_current() +
                    get_channel(System_name::PC2).get_current() +
                    get_channel(System_name::PC3).get_current()},
        .pump_current{get_channel(System_name::PUMP1).get_current() +
                      get_channel(System_name::PUMP2).get_current()},
        .fan_current{get_channel(System_name::FAN1).get_current() +
                     get_channel(System_name::FAN2).get_current()},
        .inverter_current{get_channel(System_name::INV1).get_current() +
                          get_channel(System_name::INV2).get_current()},
        .fbox_current{get_channel(System_name::FBOX).get_current()},
        .sdc_current{get_channel(System_name::SDC_ASMS).get_current()},
        .total_current{get_total_current()},
    };
  }

  // Count failed channels(with state other than ON) and update
  // leds
  bool update_leds(uint32_t tick_now) {
    bool fail{false};
    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      int led_err_count{0};
      for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
        if (ics.at(ic_idx).channels.at(ch_idx).status !=
            BTS::Channel::Status::ON) {
          led_err_count++;
        }
      }
      if (leds.at(ic_idx).update(led_err_count, tick_now)) {
        fail = true;
      }
    }
    return fail;
  }

  bool update_fans(bool rtd_status, Temperature::Values inv_values,
                   Temperature::Values motor_values) {
    return (rtd_status || inv_temperature.update(inv_values) ||
            motor_temperature.update(motor_values));
  }

  // Transmit and receive 8-byte data for all ICs in daisy chain, this
  // function flips data in array so each IC receives correct data index
  std::array<uint8_t, IC_COUNT> chain_send(std::array<uint8_t, IC_COUNT> tx) {
    uint8_t tx_buffer[IC_COUNT]{};
    std::reverse_copy(tx.begin(), tx.end(), tx_buffer);

    uint8_t rx_buffer[IC_COUNT]{};

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                            100);
    for (int i{}; i < IC_COUNT; i++) {
      tx_buffer[i] = 0x00;
    }
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                            100);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

    std::array<uint8_t, IC_COUNT> rx{};
    std::reverse_copy(&(rx_buffer[0]), &(rx_buffer[IC_COUNT - 1]), rx.begin());

    return rx;
  }

  bool check_chain_responses(std::array<uint8_t, IC_COUNT> rx) {
    bool has_error = false;
    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      if (ics.at(ic_idx).check_response(rx.at(ic_idx))) {
        has_error = true;
      }
    }
    return has_error;
  }

  bool check_chain_errors() {
    std::array<uint8_t, IC_COUNT> tx{};
    tx.fill(BTS::ERRDIAG_CMD);

    auto rx{chain_send(tx)};

    bool has_error = false;
    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      if (ics.at(ic_idx).check_err(rx.at(ic_idx))) {
        has_error = true;
      }
    }
    return has_error;
  }

  bool init_chain() {
    std::array<uint8_t, IC_COUNT> tx{};
    tx.fill(BTS::DCR_ACTIVE);

    auto rx{chain_send(tx)};
    if (check_chain_responses(rx))
      return true;

    for (auto &ic : ics) {
      ic.status = BTS::Ic::Status::STAND_BY;
    }
    return true;
  }

  bool start_chain() {
    std::array<uint8_t, IC_COUNT> tx{};
    tx.fill(BTS::OUT_READY);

    auto rx{chain_send(tx)};
    if (check_chain_responses(rx))
      return true;

    for (auto &ic : ics) {
      ic.status = BTS::Ic::Status::ACTIVE;
    }
    return false;
  }

  bool set_channel_sense(uint8_t channel) {
    uint8_t dcr_channel{};
    switch (channel) {
    case 0: {
      dcr_channel = BTS::DCR_CHANNEL0;
      break;
    }
    case 1: {
      dcr_channel = BTS::DCR_CHANNEL1;
      break;
    }
    case 2: {
      dcr_channel = BTS::DCR_CHANNEL2;
      break;
    }
    case 3: {
      dcr_channel = BTS::DCR_CHANNEL3;
      break;
    }
    default: {
      return true;
    }
    }

    std::array<uint8_t, IC_COUNT> tx{};
    tx.fill(dcr_channel);

    auto rx{chain_send(tx)};
    return check_chain_responses(rx);
  }

  void update_channel_currents(uint8_t channel,
                               std::span<uint16_t, ADC_BUF_SIZE> adc_buffer,
                               uint32_t tick_now) {
    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      uint16_t mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(
          __VREFANALOG_VOLTAGE__, adc_buffer[ic_idx], ADC_RESOLUTION12b);
      uint16_t current_val =
          (channel == 0 || channel == 3) ? mv_to_hma(mv) : mv_to_hma2(mv);
      ics.at(ic_idx).channels.at(channel).update_current(current_val, tick_now);
    }
  }

  uint16_t get_total_current() {
    uint16_t sum{};
    for (auto &ic : ics) {
      for (auto channel : ic.channels) {
        sum += channel.get_current(); // If channel is off, current is 0 → no
      }
    }
    return sum;
  }

  bool handle_overcurrent(uint32_t tick_now) {
    std::array<uint8_t, IC_COUNT> tx{
        BTS::OUT_CLOSE,
        BTS::OUT_CLOSE,
        BTS::OUT_CLOSE,
        BTS::OUT_CLOSE,
    };

    for (size_t ic_idx{}; ic_idx < IC_COUNT; ic_idx++) {
      for (size_t ch_idx{}; ch_idx < BTS::Ic::CHANNEL_COUNT; ch_idx++) {
        if (ics.at(ic_idx).channels.at(ch_idx).handle_overcurrent(tick_now) ==
            false)
          tx.at(ic_idx) |= 1 << ch_idx;
      }
    }

    auto rx{chain_send(tx)};
    return check_chain_responses(rx);
  }

private:
  std::array<Led, IC_COUNT> leds;
  std::array<BTS::Ic, IC_COUNT> ics;
  std::array<size_t, static_cast<size_t>(System_name::COUNT)>
      systems_channel_map;

  Temperature inv_temperature;
  Temperature motor_temperature;
};

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
      SYSTEM_DATA{{{{{System_name::INV2, 5000},
                     {System_name::INV1, 3000},
                     {System_name::RBOX_DIAG_BRAKE_L, 5000},
                     {System_name::TSAL_HV, 2000}}},

                   {{{System_name::DASH, 3000},
                     {System_name::SDC_ASMS, 1000},
                     {System_name::BRAKE_IR_AIR, 5000},
                     {System_name::FBOX, 5000}}},

                   {{{System_name::PC3, 4000},
                     {System_name::PC2, 5000},
                     {System_name::PC1, 5000},
                     {System_name::PC0, 4000}}},

                   {{{System_name::FAN2, 5000},
                     {System_name::PUMP2, 5000},
                     {System_name::PUMP1, 5000},
                     {System_name::FAN1, 5000}}}}};

  const std::array<Led, Pdu::IC_COUNT> LEDS{{{LED3_GPIO_Port, LED3_Pin},
                                             {LED0_GPIO_Port, LED0_Pin},
                                             {LED1_GPIO_Port, LED1_Pin},
                                             {LED2_GPIO_Port, LED2_Pin}}};

  static Pdu pdu{LEDS,
                 SYSTEM_DATA,
                 {INV_MIN_TEMP, INV_MAX_TEMP},
                 {MOTOR_MIN_TEMP, MOTOR_MAX_TEMP}};

  bool after_first_loop{false};

  pdu.init_chain();
  HAL_Delay(1);

  static uint16_t adc_buffer[ADC_BUF_SIZE];
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_SIZE);

  pdu.start_chain();
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t tick_now{HAL_GetTick()};

    pdu.check_chain_errors();

    for (int i{}; i < BTS::Ic::CHANNEL_COUNT; i++) {
      pdu.set_channel_sense(i);
      HAL_Delay(1);
      pdu.update_channel_currents(i, adc_buffer, tick_now);
    }

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
