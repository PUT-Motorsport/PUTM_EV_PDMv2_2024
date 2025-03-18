/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FUSE1_Pin GPIO_PIN_0
#define FUSE1_GPIO_Port GPIOA
#define FUSE2_Pin GPIO_PIN_1
#define FUSE2_GPIO_Port GPIOA
#define FUSE3_Pin GPIO_PIN_2
#define FUSE3_GPIO_Port GPIOA
#define FUSE4_Pin GPIO_PIN_3
#define FUSE4_GPIO_Port GPIOA
#define SPI1_SS_Pin GPIO_PIN_4
#define SPI1_SS_GPIO_Port GPIOA
#define LHI_3_Pin GPIO_PIN_0
#define LHI_3_GPIO_Port GPIOB
#define LHI_4_Pin GPIO_PIN_1
#define LHI_4_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOA
#define LHI_1_Pin GPIO_PIN_2
#define LHI_1_GPIO_Port GPIOD
#define LHI_2_Pin GPIO_PIN_3
#define LHI_2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
