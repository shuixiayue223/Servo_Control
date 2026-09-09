#ifndef ANALOG_SENSORS_H
#define ANALOG_SENSORS_H
#include "servo_config.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define SERVO_ADC_CHANNELS 4U
typedef struct {
  uint16_t raw[SERVO_ADC_CHANNELS];
  int32_t current_ma;
  int32_t current_instant_ma;
  uint32_t bus_mv;
  int16_t temp1_cdeg;
  int16_t temp2_cdeg;
  int32_t analog_angle_mdeg;
  uint16_t current_zero_adc;
  uint16_t current_raw_min;
  uint16_t current_raw_max;
  bool current_calibrated;
  bool temp1_valid;
  bool temp2_valid;
  bool angle_valid;
} AnalogSensorData;

HAL_StatusTypeDef AnalogSensors_Init(ADC_HandleTypeDef *hadc, ServoConfig *config);
void AnalogSensors_Update(void);
void AnalogSensors_CalibrateCurrent(void);
void AnalogSensors_ResetCurrentExtrema(void);
const AnalogSensorData *AnalogSensors_Get(void);
#endif
