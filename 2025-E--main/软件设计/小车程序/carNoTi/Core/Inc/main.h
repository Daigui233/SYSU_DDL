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
#include "stm32h7xx_hal.h"

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
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOE
#define Key3_Pin GPIO_PIN_13
#define Key3_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_1
#define PWM2_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_6
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_7
#define E1B_GPIO_Port GPIOA
#define ccd_clk_Pin GPIO_PIN_5
#define ccd_clk_GPIO_Port GPIOC
#define ccd_si_Pin GPIO_PIN_0
#define ccd_si_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_9
#define PWMA_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOE
#define PWMC_Pin GPIO_PIN_13
#define PWMC_GPIO_Port GPIOE
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_11
#define PWM4_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_8
#define SW2_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_9
#define SW1_GPIO_Port GPIOD
#define Key2_Pin GPIO_PIN_10
#define Key2_GPIO_Port GPIOD
#define Key1_Pin GPIO_PIN_11
#define Key1_GPIO_Port GPIOD
#define E2A_Pin GPIO_PIN_12
#define E2A_GPIO_Port GPIOD
#define E2B_Pin GPIO_PIN_13
#define E2B_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOD
#define YunTaiOk_Pin GPIO_PIN_7
#define YunTaiOk_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
