#ifndef SERVO_CAN_H
#define SERVO_CAN_H
#include "servo_control.h"
#include "stm32f1xx_hal.h"

HAL_StatusTypeDef ServoCAN_Init(CAN_HandleTypeDef *hcan, ServoControl *control, ServoConfig *config);
void ServoCAN_RxCallback(CAN_HandleTypeDef *hcan);
void ServoCAN_Process(uint32_t now_ms);
void ServoCAN_Telemetry(uint32_t now_ms);
#endif
