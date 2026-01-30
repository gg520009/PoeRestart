/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define led_Pin GPIO_PIN_0
#define led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RELAY_CLOSE()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define RELAY_OPEN()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)

//#define DCDC_ENABLE()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
//#define DCDC_DISABLE()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define DCDC_ENABLE()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define DCDC_DISABLE()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)

#define Powerkeyin_Pin GPIO_PIN_2
#define Powerkeyin_GPIO_Port GPIOA

#define LB16F1_LED_ON()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define LB16F1_LED_OFF()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

#define AP_ON()             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define AP_OFF()            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

//#define MY_DE_BUG
#ifdef MY_DE_BUG
#define DEBUG_PIN_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
#define DEBUG_PIN_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#else
#define DEBUG_PIN_HIGH()
#define DEBUG_PIN_LOW()
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
