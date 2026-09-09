#include "pwm_input.h"
#include <string.h>
static TIM_HandleTypeDef *s_tim; static ServoConfig *s_cfg;
static volatile PWMInputData s_data; static uint16_t s_rise, s_prev_rise, s_period;
static bool s_falling, s_have_rise; static uint8_t s_good;

HAL_StatusTypeDef PWMInput_Init(TIM_HandleTypeDef *htim, ServoConfig *cfg)
{
  if (htim == 0 || cfg == 0) return HAL_ERROR;
  s_tim = htim; s_cfg = cfg; memset((void *)&s_data, 0, sizeof(s_data));
  __HAL_TIM_SET_CAPTUREPOLARITY(s_tim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
  return HAL_TIM_IC_Start_IT(s_tim, TIM_CHANNEL_3);
}

void PWMInput_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint16_t cap, width; bool ok;
  if (htim != s_tim || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3) return;
  cap = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
  if (!s_falling) {
    s_rise = cap; if (s_have_rise) s_period = (uint16_t)(cap - s_prev_rise);
    s_prev_rise = cap; s_have_rise = true; s_falling = true;
    __HAL_TIM_SET_CAPTUREPOLARITY(s_tim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
    return;
  }
  width = (uint16_t)(cap - s_rise);
  ok = s_have_rise && width >= 500U && width <= 2500U && s_period >= 10000U && s_period <= 30000U;
  if (ok) {
    s_data.pulse_us = width; s_data.period_us = s_period; s_data.last_ms = HAL_GetTick();
    if (s_good < 3U) s_good++; s_data.valid = s_good >= 3U;
  } else { s_good = 0U; s_data.valid = false; }
  s_falling = false;
  __HAL_TIM_SET_CAPTUREPOLARITY(s_tim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
}

void PWMInput_Update(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - s_data.last_ms) > (uint32_t)s_cfg->pwm_timeout_ms) {
    s_data.valid = false; s_good = 0U;
  }
}

int32_t PWMInput_PositionMdeg(void)
{
  int32_t p, span;
  if (!s_data.valid) return 0;
  p = s_data.pulse_us; if (p < s_cfg->pwm_min_us) p = s_cfg->pwm_min_us;
  if (p > s_cfg->pwm_max_us) p = s_cfg->pwm_max_us;
  span = s_cfg->pwm_max_us - s_cfg->pwm_min_us;
  return s_cfg->position_min_mdeg + (int32_t)((int64_t)(p - s_cfg->pwm_min_us) *
    (s_cfg->position_max_mdeg - s_cfg->position_min_mdeg) / span);
}
const PWMInputData *PWMInput_Get(void) { return (const PWMInputData *)&s_data; }
