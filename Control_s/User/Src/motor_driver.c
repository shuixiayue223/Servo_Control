#include "motor_driver.h"
static TIM_HandleTypeDef *s_tim;

HAL_StatusTypeDef MotorDriver_Init(TIM_HandleTypeDef *htim)
{
  HAL_StatusTypeDef r;
  if (htim == 0) return HAL_ERROR;
  s_tim = htim;
  __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_2, 0U);
  r = HAL_TIM_PWM_Start(s_tim, TIM_CHANNEL_1);
  if (r == HAL_OK) r = HAL_TIMEx_PWMN_Start(s_tim, TIM_CHANNEL_1);
  if (r == HAL_OK) r = HAL_TIM_PWM_Start(s_tim, TIM_CHANNEL_2);
  if (r == HAL_OK) r = HAL_TIMEx_PWMN_Start(s_tim, TIM_CHANNEL_2);
  __HAL_TIM_MOE_DISABLE(s_tim);
  return r;
}

/* brake 模式说明：
 * Disable 不再清 MOE，而是把 OC1M/OC2M 设为 010（Force Inactive，
 * 即 OCxREF=0）。STM32F103 互补输出下：OC=0（HIN=0 上管关），
 * OCN=1（LIN=1 下管开）。H 桥低侧 FET 全开，电机两端短接到 GND，
 * 机械动能经线圈电阻 + FET Rds(on) 消耗掉，几百 ms 内停住。
 * 关键：保持 MOE=1，否则 STM32F1 的 MOE=0 会让输出变 Hi-Z（coast）。 */
void MotorDriver_Enable(void)
{
  if (s_tim == 0) return;
  /* 退出 brake：恢复 OC1M/OC2M = 110 (PWM Mode 1) */
  s_tim->Instance->CCMR1 &= ~((0x7UL << 4) | (0x7UL << 12));
  s_tim->Instance->CCMR1 |= ((0x6UL << 4) | (0x6UL << 12));
  __HAL_TIM_MOE_ENABLE(s_tim);
}

void MotorDriver_Disable(void)
{
  if (s_tim == 0) return;
  /* 进入 brake：OC1M/OC2M = 010 (Force Inactive)，OCxREF=0 */
  s_tim->Instance->CCMR1 &= ~((0x7UL << 4) | (0x7UL << 12));
  s_tim->Instance->CCMR1 |= ((0x2UL << 4) | (0x2UL << 12));
  /* 关键：不清 MOE。MOE 状态由 Enable/Init 单独管理。 */
}

void MotorDriver_Set(int16_t command, uint16_t maximum)
{
  int32_t half, delta, a, b; uint32_t period;
  if (s_tim == 0) return;
  if (maximum > 950U) maximum = 950U;
  if (command > (int16_t)maximum) command = (int16_t)maximum;
  if (command < -(int16_t)maximum) command = -(int16_t)maximum;
  if (command == 0) {
    __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_2, 0U);
    return;
  }
  period = __HAL_TIM_GET_AUTORELOAD(s_tim) + 1U;
  half = (int32_t)(period / 2U); delta = (int32_t)command * half / 1000;
  a = half + delta; b = half - delta;
  if (a < 1) a = 1; if (b < 1) b = 1;
  if (a >= (int32_t)period) a = (int32_t)period - 1;
  if (b >= (int32_t)period) b = (int32_t)period - 1;
  __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_1, (uint32_t)a);
  __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_2, (uint32_t)b);
}
