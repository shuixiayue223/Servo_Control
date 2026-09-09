#ifndef AS5600_H
#define AS5600_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t raw;
  int32_t angle_mdeg;
  uint8_t status;
  uint8_t agc;
  uint16_t magnitude;
  uint32_t errors;
  uint32_t last_update_ms;
  bool valid;
  bool magnet_ok;
} AS5600_Data;

void AS5600_Init(I2C_HandleTypeDef *hi2c);
bool AS5600_Update(uint32_t now_ms);
const AS5600_Data *AS5600_Get(void);
#endif
