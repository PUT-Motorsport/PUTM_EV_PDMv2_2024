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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_gpio.h"
#include "stm32g0xx_hal_spi.h"
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
constexpr uint32_t MAX_RETRIES{5};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// For individual channels 0 and 3 (returns 0.1A units as uint8_t) - do
// wyjebania
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

/* Led blinking indicates each IC channels status:
  OFF - all channels OK



  ON - all channels ERROR
*/
class Led {
public:
  Led(GPIO_TypeDef *port, const uint16_t pin) : port{port}, pin{pin} {};

  // Update single Led state based on failed channels count
  bool update(uint8_t channels_failed, uint32_t tick_now) {
    if (channels_failed > Ic::CHANNEL_COUNT) {
      return true;
    } else if (channels_failed == Ic::CHANNEL_COUNT) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
      return false;
    } else if (channels_failed == 0) {
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
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
  GPIO_TypeDef *port{nullptr};
  const uint16_t pin{0};

  uint32_t toggle_time{};
  uint32_t last_toggle{};
};

// Base class that controls all ICs
class Pdu {
public:
  static constexpr uint8_t IC_COUNT{4};

  Pdu(std::array<Led, IC_COUNT> leds,
      std::array<std::array<uint16_t, Ic::CHANNEL_COUNT>, Pdu::IC_COUNT>
          thresholds)
      : leds{leds},
        ics{thresholds[0], thresholds[1], thresholds[2], thresholds[3]} {}

  // Count failed channels(with state other than ON) and update
  // leds
  bool update_leds(uint32_t tick_now) {
    int ic_count{0};
    bool fail{false};
    for (auto &ic : ics) {
      int led_err_count{0};
      for (auto &channel : ic.channels) {
        if (channel.status != Channel::Status::ON) {
          led_err_count++;
        }
      }
      if (leds.at(ic_count).update(led_err_count, tick_now)) {
        fail = true;
      }
    }
    return fail;
  }

  // Returns lower status of two channels
  Channel::Status reduce_status(const Channel::Status a,
                                const Channel::Status b) {
    auto a_val{static_cast<uint8_t>(a)};
    auto b_val{static_cast<uint8_t>(b)};
    uint8_t out_val{};

    if (a_val > b_val) {
      out_val = b_val;
    } else {
      out_val = a_val;
    }

    return static_cast<Channel::Status>(out_val);
  }

  // Updates all data that is needed for CAN
  void update_system_status() {
    system_data.fan_status = reduce_status(fan1().status, fan2().status);
    system_data.pump_status = reduce_status(pump1().status, pump2().status);
    system_data.pc_status = reduce_status(
        pc1().status,
        reduce_status(pc2().status, reduce_status(pc3().status, pc4().status)));
    system_data.dash_status = dash().status;
    system_data.sdc_status = sdc_asms().status;
    system_data.brake_ir_air_status = brake_ir_air().status;
    system_data.fbox_status = fbox().status;
    system_data.inverter_status = reduce_status(inv1().status, inv2().status);

    system_data.rbox_diag_brake_l_status = rbox_diag_brake_l().status;
    system_data.tsal_hv_status = tsal_hv().status;

    system_data.fan_current = fan1().get_current() + fan2().get_current();
    system_data.pump_current = pump1().get_current() + pump2().get_current();
    system_data.pc_current = pc1().get_current() + pc2().get_current() +
                             pc3().get_current() + pc4().get_current();
    system_data.sdc_current = sdc_asms().get_current();
    system_data.fbox_current = fbox().get_current();
    system_data.inverter_current = inv1().get_current() + inv2().get_current();

    system_data.total_current = update_total_current();
  }

  // Transmit and receive 8-byte data for all ICs in daisy chain, this function
  // flips data in array so each IC receives correct data index
  std::array<uint8_t, IC_COUNT> chain_send(std::array<uint8_t, IC_COUNT> tx) {
    uint8_t tx_buffer[IC_COUNT]{};
    int tx_count{tx.max_size() - 1};
    for (auto tx_val : tx) {
      tx_buffer[tx_count] = tx_val;
      tx_count--;
    }
    uint8_t rx_buffer[IC_COUNT]{};

    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                            100);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, sizeof(rx_buffer),
                            100);

    std::array<uint8_t, IC_COUNT> rx{};
    int rx_count{tx.max_size() - 1};
    for (auto &rx_val : rx) {
      rx_val = rx_buffer[rx_count];
      rx_count--;
    }
    return rx;
  }

  bool check_chain_responses(std::array<uint8_t, IC_COUNT> rx) {
    bool has_error = false;
    int ic_count{0};
    for (auto &ic : ics) {
      if (ic.check_response(rx.at(ic_count))) {
        has_error = true;
      }
      ic_count++;
    }
    return has_error;
  }

  bool check_chain_errors() {
    std::array<uint8_t, IC_COUNT> tx{
        ERRDIAG_CMD,
        ERRDIAG_CMD,
        ERRDIAG_CMD,
        ERRDIAG_CMD,
    };
    auto rx{chain_send(tx)};

    bool has_error = false;
    int ic_count{0};
    for (auto &ic : ics) {
      if (ic.check_err(rx.at(ic_count))) {
        has_error = true;
      }
      ic_count++;
    }
    return has_error;
  }

  bool init_chain() {
    HAL_GPIO_WritePin(LHI_1_GPIO_Port, LHI_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_2_GPIO_Port, LHI_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_3_GPIO_Port, LHI_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LHI_4_GPIO_Port, LHI_4_Pin, GPIO_PIN_RESET);

    std::array<uint8_t, IC_COUNT> tx{
        DCR_ACTIVE,
        DCR_ACTIVE,
        DCR_ACTIVE,
        DCR_ACTIVE,
    };
    auto rx{chain_send(tx)};
    if (check_chain_responses(rx))
      return true;

    for (auto &ic : ics) {
      ic.status = Ic::Status::STAND_BY;
    }
    return true;
  }

  bool start_chain() {
    std::array<uint8_t, IC_COUNT> tx{
        OUT_READY,
        OUT_READY,
        OUT_READY,
        OUT_READY,
    };
    auto rx{chain_send(tx)};
    if (check_chain_responses(rx))
      return true;

    for (auto &ic : ics) {
      ic.status = Ic::Status::ACTIVE;
    }
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

    std::array<uint8_t, IC_COUNT> tx{
        dcr_channel,
        dcr_channel,
        dcr_channel,
        dcr_channel,
    };

    auto rx{chain_send(tx)};
    return check_chain_responses(rx);
  }

  void update_channel_currents(uint8_t channel,
                               std::span<uint16_t, ADC_BUF_SIZE> adc_buffer,
                               uint32_t tick_now) {
    int ic_count{0};
    for (auto &ic : ics) {
      uint16_t mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(
          __VREFANALOG_VOLTAGE__, adc_buffer[ic_count], ADC_RESOLUTION12b);

      uint16_t current_val =
          (channel == 0 || channel == 3) ? mv_to_hma(mv) : mv_to_hma2(mv);

      ic.channels[channel].update_current(current_val, tick_now);
      ic_count++;
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
    system_data.total_current = sum;
    return sum;
  }

  bool handle_overcurrent(uint16_t tick_now) {
    std::array<uint8_t, IC_COUNT> tx{
        OUT_CLOSE,
        OUT_CLOSE,
        OUT_CLOSE,
        OUT_CLOSE,
    };

    int ic_count{0};
    for (auto &ic : ics) {
      int channel_count{0};
      for (auto &channel : ic.channels) {
        if (channel.handle_overcurrent(tick_now) == false)
          tx.at(ic_count) |= 1 << channel_count;
        channel_count++;
      }
      ic_count++;
    }

    auto rx{chain_send(tx)};
    return check_chain_responses(rx);
  }

private:
  std::array<Led, IC_COUNT> leds;
  std::array<Ic, IC_COUNT> ics;

  struct {
    Channel::Status pc_status{};
    Channel::Status fan_status{};
    Channel::Status pump_status{};
    Channel::Status inverter_status{};
    Channel::Status fbox_status{};
    Channel::Status sdc_status{};
    Channel::Status dash_status{};
    Channel::Status tsal_hv_status{};
    Channel::Status rbox_diag_brake_l_status{};
    Channel::Status brake_ir_air_status{};

    uint16_t pc_current{};
    uint16_t pump_current{};
    uint16_t fan_current{};
    uint16_t inverter_current{};
    uint16_t fbox_current{};
    uint16_t sdc_current{};

    uint16_t total_current{};
  } system_data;

  Channel &inv2() { return ics[0].channels[0]; }
  Channel &inv1() { return ics[0].channels[1]; }
  Channel &rbox_diag_brake_l() { return ics[0].channels[2]; }
  Channel &tsal_hv() { return ics[0].channels[3]; }

  Channel &dash() { return ics[1].channels[0]; }
  Channel &sdc_asms() { return ics[1].channels[1]; }
  Channel &brake_ir_air() { return ics[1].channels[2]; }
  Channel &fbox() { return ics[1].channels[3]; }

  Channel &pc4() { return ics[2].channels[0]; }
  Channel &pc3() { return ics[2].channels[1]; }
  Channel &pc2() { return ics[2].channels[2]; }
  Channel &pc1() { return ics[2].channels[3]; }

  Channel &fan2() { return ics[3].channels[0]; }
  Channel &pump2() { return ics[3].channels[1]; }
  Channel &pump1() { return ics[3].channels[2]; }
  Channel &fan1() { return ics[3].channels[3]; }
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
    uint16_t INV2{50};
    uint16_t INV1{30};
    uint16_t RBOX_DIAG_BRAKE_L{50};
    uint16_t TSAL_HV{20};

    uint16_t DASH{30};
    uint16_t SDC_ASMS{10};
    uint16_t BRAKE_IR_AIR{50};
    uint16_t FBOX{50};

    uint16_t PC4{40};
    uint16_t PC3{50};
    uint16_t PC2{50};
    uint16_t PC1{40};

    uint16_t FAN2{50};
    uint16_t PUMP2{50};
    uint16_t PUMP1{50};
    uint16_t FAN1{50};
  } I_MAX; // Current thresholds

  std::array<std::array<uint16_t, Ic::CHANNEL_COUNT>, Pdu::IC_COUNT> thresholds{
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

  static Pdu pdu{leds, thresholds};
  Temperature inv_temperature{20, 100};
  Temperature motor_temperature{20, 130};

  bool RTD_status;

  bool after_first_loop{false};
  uint16_t adc_buffer[ADC_BUF_SIZE];

  pdu.init_chain();
  HAL_Delay(1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_SIZE);

  pdu.start_chain();
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t tick_now{HAL_GetTick()};
    pdu.check_chain_errors();

    for (int i{}; i < Ic::CHANNEL_COUNT; i++) {
      pdu.set_channel_sense(i);
      HAL_Delay(1);
      pdu.update_channel_currents(i, adc_buffer, tick_now);
    }

    if (after_first_loop)
      pdu.handle_overcurrent(tick_now);

    pdu.update_leds(tick_now);
    pdu.update_system_status();

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
