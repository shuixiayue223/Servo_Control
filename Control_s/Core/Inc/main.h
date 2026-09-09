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
#include "stm32f1xx_hal.h"

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
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define CURRENT_SENSE_Pin GPIO_PIN_0
#define CURRENT_SENSE_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_1
#define VBUS_SENSE_GPIO_Port GPIOA
#define MOS_TEMP_1_Pin GPIO_PIN_2
#define MOS_TEMP_1_GPIO_Port GPIOA
#define MOS_TEMP_2_Pin GPIO_PIN_3
#define MOS_TEMP_2_GPIO_Port GPIOA
#define AS5600_ADC_Pin GPIO_PIN_4
#define AS5600_ADC_GPIO_Port GPIOA
#define MOTOR_A_PWM_L_Pin GPIO_PIN_13
#define MOTOR_A_PWM_L_GPIO_Port GPIOB
#define MOTOR_B_PWM_L_Pin GPIO_PIN_14
#define MOTOR_B_PWM_L_GPIO_Port GPIOB
#define MOTOR_A_PWM_H_Pin GPIO_PIN_8
#define MOTOR_A_PWM_H_GPIO_Port GPIOA
#define MOTOR_B_PWM_H_Pin GPIO_PIN_9
#define MOTOR_B_PWM_H_GPIO_Port GPIOA
#define AS5600_SCL_Pin GPIO_PIN_6
#define AS5600_SCL_GPIO_Port GPIOB
#define AS5600_SDA_Pin GPIO_PIN_7
#define AS5600_SDA_GPIO_Port GPIOB
#define PWM_IN_Pin GPIO_PIN_8
#define PWM_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
