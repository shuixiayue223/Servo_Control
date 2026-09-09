#include "as5600.h"
#include <string.h>

#define AS5600_ADDR (0x36U << 1)
static I2C_HandleTypeDef *s_i2c;
static AS5600_Data s_data;

static bool Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  return s_i2c != 0 && HAL_I2C_Mem_Read(s_i2c, AS5600_ADDR, reg,
    I2C_MEMADD_SIZE_8BIT, data, length, 3U) == HAL_OK;
}

void AS5600_Init(I2C_HandleTypeDef *hi2c)
{
  memset(&s_data, 0, sizeof(s_data));
  s_i2c = hi2c;
}

bool AS5600_Update(uint32_t now_ms)
{
  uint8_t angle[2], status, extra[3];
  if (s_i2c == 0) return false;
	//1
  if (HAL_I2C_IsDeviceReady(s_i2c, AS5600_ADDR, 1U, 1U) != HAL_OK) {
    s_data.valid = false; s_data.errors++; return false;
  }
	
  if (!Read(0x0CU, angle, 2U)) {
    s_data.valid = false; s_data.errors++; return false;
  }
  s_data.raw = (uint16_t)((((uint16_t)angle[0] & 0x0FU) << 8) | angle[1]);
  s_data.angle_mdeg = (int32_t)(((uint32_t)s_data.raw * 360000UL) / 4096UL);
  if (!Read(0x0BU, &status, 1U)) {
    s_data.valid = false; s_data.errors++; return false;
  }
  s_data.status = status;
  s_data.magnet_ok = (status & 0x20U) != 0U;
  if (Read(0x1AU, extra, 3U)) {
    s_data.agc = extra[0];
    s_data.magnitude = (uint16_t)((((uint16_t)extra[1] & 0x0FU) << 8) | extra[2]);
  }
  s_data.last_update_ms = now_ms;
  s_data.valid = s_data.magnet_ok;
  return s_data.valid;
}

const AS5600_Data *AS5600_Get(void) { return &s_data; }
