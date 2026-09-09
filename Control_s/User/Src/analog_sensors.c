#include "analog_sensors.h"
#include <math.h>
#include <string.h>

static ADC_HandleTypeDef *s_adc;
static ServoConfig *s_cfg;
static volatile uint16_t s_dma[SERVO_ADC_CHANNELS];
static AnalogSensorData s_data;
static float s_filtered;
static uint32_t s_calib_acc;
static uint32_t s_calib_n;
static uint32_t s_zero_track_acc;
static uint16_t s_zero_track_n;

#define CURRENT_ZERO_TRACK_THRESHOLD_MA 80
#define CURRENT_ZERO_TRACK_SAMPLES 256U

static void ResetCurrentExtrema(uint16_t raw)
{
  s_data.current_raw_min = raw;
  s_data.current_raw_max = raw;
}

static int16_t Ntc(uint16_t raw, bool *valid)
{
  float r, inv_t, cdeg;
  if (raw < 8U || raw > 4087U) { *valid = false; return 0; }
  r = (float)s_cfg->ntc_pullup_ohm * raw / (4095.0f - raw);
  inv_t = 1.0f / 298.15f + logf(r / (float)s_cfg->ntc_r0_ohm) / (float)s_cfg->ntc_beta;
  cdeg = (1.0f / inv_t - 273.15f) * 100.0f;
  *valid = cdeg >= -4000.0f && cdeg <= 15000.0f;
  return *valid ? (int16_t)cdeg : 0;
}

HAL_StatusTypeDef AnalogSensors_Init(ADC_HandleTypeDef *hadc, ServoConfig *cfg)
{
  HAL_StatusTypeDef result;
  if (hadc == 0 || cfg == 0) return HAL_ERROR;
  s_adc = hadc; s_cfg = cfg; memset(&s_data, 0, sizeof(s_data));
  result = HAL_ADCEx_Calibration_Start(s_adc);
  if (result != HAL_OK) return result;
  result = HAL_ADC_Start_DMA(s_adc, (uint32_t *)(void *)s_dma, SERVO_ADC_CHANNELS);
  if (result == HAL_OK) AnalogSensors_CalibrateCurrent();
  return result;
}

void AnalogSensors_CalibrateCurrent(void)
{
  s_calib_acc = 0U; s_calib_n = 0U;
  s_zero_track_acc = 0U; s_zero_track_n = 0U;
  s_data.current_calibrated = false;
  if (s_cfg != 0) s_data.current_zero_adc = (uint16_t)(1650UL * 4095UL / (uint32_t)s_cfg->adc_vref_mv);
  ResetCurrentExtrema(s_data.current_zero_adc);
}

void AnalogSensors_Update(void)
{
  uint32_t i, node_mv, sensor_mv;
  int32_t delta;
  int64_t uv, angle;
  if (s_cfg == 0) return;
  for (i = 0U; i < SERVO_ADC_CHANNELS; i++) s_data.raw[i] = s_dma[i];
  if (!s_data.current_calibrated) {
    s_calib_acc += s_data.raw[0];
    if (++s_calib_n >= 4096U) {
      s_data.current_zero_adc = (uint16_t)(s_calib_acc / 4096U);
      s_data.current_calibrated = true;
      s_filtered = 0.0f;
      s_zero_track_acc = 0U; s_zero_track_n = 0U;
      ResetCurrentExtrema(s_data.raw[0]);
    }
  }
  if (s_data.current_calibrated) {
    int32_t quiet_ma = s_data.current_ma < 0 ? -s_data.current_ma : s_data.current_ma;
    if (quiet_ma <= CURRENT_ZERO_TRACK_THRESHOLD_MA) {
      s_zero_track_acc += s_data.raw[0];
      if (++s_zero_track_n >= CURRENT_ZERO_TRACK_SAMPLES) {
        uint16_t avg = (uint16_t)(s_zero_track_acc / CURRENT_ZERO_TRACK_SAMPLES);
        int32_t zero_error = (int32_t)avg - (int32_t)s_data.current_zero_adc;
        if (zero_error > 1) s_data.current_zero_adc++;
        else if (zero_error < -1) s_data.current_zero_adc--;
        s_zero_track_acc = 0U;
        s_zero_track_n = 0U;
      }
    } else {
      s_zero_track_acc = 0U;
      s_zero_track_n = 0U;
    }
  }
  if (s_data.raw[0] < s_data.current_raw_min) s_data.current_raw_min = s_data.raw[0];
  if (s_data.raw[0] > s_data.current_raw_max) s_data.current_raw_max = s_data.raw[0];
  delta = (int32_t)s_data.raw[0] - s_data.current_zero_adc;
  uv = (int64_t)delta * s_cfg->adc_vref_mv * 1000LL / 4095LL;
  s_data.current_instant_ma = (int32_t)(uv * 1000LL / s_cfg->current_uv_per_a);
  s_filtered += 0.05f * ((float)s_data.current_instant_ma - s_filtered);
  s_data.current_ma = (int32_t)s_filtered;
  node_mv = (uint32_t)s_data.raw[1] * (uint32_t)s_cfg->adc_vref_mv / 4095UL;
  s_data.bus_mv = (uint32_t)((uint64_t)node_mv * (uint32_t)s_cfg->bus_divider_x1000 / 1000ULL);
  s_data.temp1_cdeg = Ntc(s_data.raw[2], &s_data.temp1_valid);
  /* 小舵机板只有一路 MOS 温度采样，第二路沿用同一结果，兼容现有控制逻辑。 */
  s_data.temp2_cdeg = s_data.temp1_cdeg;
  s_data.temp2_valid = s_data.temp1_valid;
  node_mv = (uint32_t)s_data.raw[3] * (uint32_t)s_cfg->adc_vref_mv / 4095UL;
  sensor_mv = node_mv * 2U;
  angle = (int64_t)sensor_mv * 360000LL / s_cfg->as5600_analog_fs_mv;
  s_data.angle_valid = sensor_mv <= (uint32_t)s_cfg->as5600_analog_fs_mv * 105U / 100U;
  if (angle < 0) angle = 0; if (angle > 359999) angle = 359999;
  s_data.analog_angle_mdeg = (int32_t)angle;
}

void AnalogSensors_ResetCurrentExtrema(void) { ResetCurrentExtrema(s_data.raw[0]); }
const AnalogSensorData *AnalogSensors_Get(void) { return &s_data; }
