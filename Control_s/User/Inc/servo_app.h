#ifndef SERVO_APP_H
#define SERVO_APP_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>

bool ServoApp_Init(ADC_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c,
  CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *motor_tim, TIM_HandleTypeDef *pwm_tim,
  GPIO_TypeDef *led_port, uint16_t led_pin);
void ServoApp_Process(void);
void ServoApp_CAN_RxCallback(CAN_HandleTypeDef *hcan);
void ServoApp_TIM_IC_Callback(TIM_HandleTypeDef *htim);
void ServoApp_EmergencyShutdown(void);
#endif
