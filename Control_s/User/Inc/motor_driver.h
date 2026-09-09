#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef MotorDriver_Init(TIM_HandleTypeDef *htim);
void MotorDriver_Enable(void);
void MotorDriver_Disable(void);
void MotorDriver_Set(int16_t command_permille, uint16_t maximum_permille);
#endif
