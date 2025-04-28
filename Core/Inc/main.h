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
#include "stm32g4xx_hal.h"

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
#define EMER_VALVE_Pin GPIO_PIN_0
#define EMER_VALVE_GPIO_Port GPIOB
#define EMER_VALVE_EXTI_IRQn EXTI0_IRQn
#define SOLENOID_IN_Pin GPIO_PIN_1
#define SOLENOID_IN_GPIO_Port GPIOB
#define SOLENOID_IN_EXTI_IRQn EXTI1_IRQn
#define FC_FAULT_Pin GPIO_PIN_2
#define FC_FAULT_GPIO_Port GPIOB
#define FC_FAULT_EXTI_IRQn EXTI2_IRQn
#define Solenoid_OUT_Pin GPIO_PIN_11
#define Solenoid_OUT_GPIO_Port GPIOB
#define Emergency_Valve_Pin GPIO_PIN_12
#define Emergency_Valve_GPIO_Port GPIOB
#define Pressure_IN_Pin GPIO_PIN_13
#define Pressure_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
