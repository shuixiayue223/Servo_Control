#ifndef PWM_INPUT_H
#define PWM_INPUT_H
#include "servo_config.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint16_t pulse_us, period_us; uint32_t last_ms; bool valid; } PWMInputData;
HAL_StatusTypeDef PWMInput_Init(TIM_HandleTypeDef *htim, ServoConfig *config);
void PWMInput_CaptureCallback(TIM_HandleTypeDef *htim);
void PWMInput_Update(uint32_t now_ms);
int32_t PWMInput_PositionMdeg(void);
const PWMInputData *PWMInput_Get(void);
#endif
