/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CANT
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
#define GPIO_LED_Pin GPIO_PIN_13
#define GPIO_LED_GPIO_Port GPIOC
#define GPIO_KEY_Pin GPIO_PIN_0
#define GPIO_KEY_GPIO_Port GPIOA
#define EXTI_VL_2_Pin GPIO_PIN_0
#define EXTI_VL_2_GPIO_Port GPIOB
#define EXTI_VL_2_EXTI_IRQn EXTI0_IRQn
#define GPIO_SHDN_VL_2_Pin GPIO_PIN_1
#define GPIO_SHDN_VL_2_GPIO_Port GPIOB
#define EXTI_INA_2_Pin GPIO_PIN_12
#define EXTI_INA_2_GPIO_Port GPIOB
#define EXTI_INA_2_EXTI_IRQn EXTI15_10_IRQn
#define EXTI_INA_1_Pin GPIO_PIN_3
#define EXTI_INA_1_GPIO_Port GPIOB
#define EXTI_INA_1_EXTI_IRQn EXTI3_IRQn
#define EXTI_VL_1_Pin GPIO_PIN_4
#define EXTI_VL_1_GPIO_Port GPIOB
#define EXTI_VL_1_EXTI_IRQn EXTI4_IRQn
#define GPIO_SHDN_VL_1_Pin GPIO_PIN_5
#define GPIO_SHDN_VL_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
