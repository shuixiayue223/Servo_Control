#include "servo_config.h"
#include <string.h>

void ServoConfig_Defaults(ServoConfig *c)
{
  if (c == 0) return;
  memset(c, 0, sizeof(*c));
  c->node_id = 1; c->telemetry_ms = 200;
  c->position_min_mdeg = 0; c->position_max_mdeg = 180000;
  c->sensor_direction = 1; c->motor_direction = 1;
  /* 实际电机：低速舵机/机械臂用（额定24V/15W，296 RPM空载，0.5N·m，0.86A，减速比27:1）
   * 母线：12V/24V 通用（用户实测两者功率扭矩转速基本一致）
   * 母线分压：21kΩ + 1kΩ → 总衰减 22 倍 → bus_divider_x1000=22000
   * 采样：单颗 2mΩ + INA181A1 (20V/V) → 40mV/A = 40000 µV/A
   * 编码器：AS5600（5V 供电，I2C 接口） */
  c->max_current_ma = 1500; c->trip_current_ma = 2500;
  c->max_speed_mdps = 1800000; c->max_power_mw = 30000;
  c->max_duty_permille = 950;
  c->temp_derate_cdeg = 7000; c->temp_shutdown_cdeg = 9000;
  c->temp_recover_cdeg = 6500; c->bus_uv_mv = 7500; c->bus_ov_mv = 30000;
  c->position_kp_x1000 = 4000; c->speed_kp_x1000 = 100000;
  c->speed_ki_x1000 = 3000;    c->current_kp_x1000 = 200000;   /* 电流环 P 项 100：200mA 误差 → 18‰ duty，足够驱动电机；之前 18 太小输出 0.003‰ 截断为 0 */
  c->current_ki_x1000 = 30000; c->position_deadband_mdeg = 500;
  c->can_timeout_ms = 5000; c->pwm_timeout_ms = 100;
  c->pwm_min_us = 1000; c->pwm_max_us = 2000;
  c->angle_source = 0; c->angle_crosscheck_mdeg = 10000;
  c->adc_vref_mv = 3300; c->as5600_analog_fs_mv = 5000;
  c->bus_divider_x1000 = 22000; c->current_uv_per_a = 40000;
  c->ntc_beta = 3950; c->ntc_r0_ohm = 10000; c->ntc_pullup_ohm = 4700;
}

bool ServoConfig_Validate(const ServoConfig *c)
{
  if (c == 0) return false;
  return c->node_id >= 1 && c->node_id <= 63 && c->telemetry_ms >= 10 &&
    c->position_min_mdeg < c->position_max_mdeg &&
    (c->sensor_direction == 1 || c->sensor_direction == -1) &&
    (c->motor_direction == 1 || c->motor_direction == -1) &&
    c->max_current_ma >= 100 && c->max_current_ma <= c->trip_current_ma &&
    c->trip_current_ma <= 60000 && c->max_speed_mdps > 0 &&
    c->max_duty_permille >= 10 && c->max_duty_permille <= 950 &&
    c->temp_recover_cdeg < c->temp_derate_cdeg &&
    c->temp_derate_cdeg < c->temp_shutdown_cdeg &&
    c->bus_uv_mv < c->bus_ov_mv && c->can_timeout_ms >= 20 &&
    c->pwm_timeout_ms >= 20 && c->pwm_min_us < c->pwm_max_us &&
    c->angle_source >= 0 && c->angle_source <= 2 && c->adc_vref_mv > 0 &&
    c->as5600_analog_fs_mv > 0 && c->current_uv_per_a > 0 && c->ntc_beta > 0;
}

#define CFG_CASE(ID, FIELD) case ID: *value = c->FIELD; return true
bool ServoConfig_Get(const ServoConfig *c, uint16_t id, int32_t *value)
{
  if (c == 0 || value == 0) return false;
  switch (id) {
    CFG_CASE(SERVO_PARAM_NODE_ID,node_id); CFG_CASE(SERVO_PARAM_TELEMETRY_MS,telemetry_ms);
    CFG_CASE(SERVO_PARAM_POS_MIN_MDEG,position_min_mdeg); CFG_CASE(SERVO_PARAM_POS_MAX_MDEG,position_max_mdeg);
    CFG_CASE(SERVO_PARAM_POS_OFFSET_MDEG,position_offset_mdeg); CFG_CASE(SERVO_PARAM_SENSOR_DIR,sensor_direction);
    CFG_CASE(SERVO_PARAM_MOTOR_DIR,motor_direction); CFG_CASE(SERVO_PARAM_MAX_CURRENT_MA,max_current_ma);
    CFG_CASE(SERVO_PARAM_TRIP_CURRENT_MA,trip_current_ma); CFG_CASE(SERVO_PARAM_MAX_SPEED_MDPS,max_speed_mdps);
    CFG_CASE(SERVO_PARAM_MAX_POWER_MW,max_power_mw); CFG_CASE(SERVO_PARAM_MAX_DUTY_PERMILLE,max_duty_permille);
    CFG_CASE(SERVO_PARAM_TEMP_DERATE_CDEG,temp_derate_cdeg); CFG_CASE(SERVO_PARAM_TEMP_SHUTDOWN_CDEG,temp_shutdown_cdeg);
    CFG_CASE(SERVO_PARAM_TEMP_RECOVER_CDEG,temp_recover_cdeg); CFG_CASE(SERVO_PARAM_BUS_UV_MV,bus_uv_mv);
    CFG_CASE(SERVO_PARAM_BUS_OV_MV,bus_ov_mv); CFG_CASE(SERVO_PARAM_POS_KP_X1000,position_kp_x1000);
    CFG_CASE(SERVO_PARAM_SPEED_KP_X1000,speed_kp_x1000); CFG_CASE(SERVO_PARAM_SPEED_KI_X1000,speed_ki_x1000);
    CFG_CASE(SERVO_PARAM_CURRENT_KP_X1000,current_kp_x1000); CFG_CASE(SERVO_PARAM_CURRENT_KI_X1000,current_ki_x1000);
    CFG_CASE(SERVO_PARAM_POS_DEADBAND_MDEG,position_deadband_mdeg); CFG_CASE(SERVO_PARAM_CAN_TIMEOUT_MS,can_timeout_ms);
    CFG_CASE(SERVO_PARAM_PWM_TIMEOUT_MS,pwm_timeout_ms); CFG_CASE(SERVO_PARAM_PWM_MIN_US,pwm_min_us);
    CFG_CASE(SERVO_PARAM_PWM_MAX_US,pwm_max_us); CFG_CASE(SERVO_PARAM_ANGLE_SOURCE,angle_source);
    CFG_CASE(SERVO_PARAM_ANGLE_CROSSCHECK_MDEG,angle_crosscheck_mdeg); CFG_CASE(SERVO_PARAM_ADC_VREF_MV,adc_vref_mv);
    CFG_CASE(SERVO_PARAM_AS5600_FS_MV,as5600_analog_fs_mv); CFG_CASE(SERVO_PARAM_BUS_DIVIDER_X1000,bus_divider_x1000);
    CFG_CASE(SERVO_PARAM_CURRENT_UV_PER_A,current_uv_per_a); CFG_CASE(SERVO_PARAM_NTC_BETA,ntc_beta);
    CFG_CASE(SERVO_PARAM_NTC_R0_OHM,ntc_r0_ohm); CFG_CASE(SERVO_PARAM_NTC_PULLUP_OHM,ntc_pullup_ohm);
    default: return false;
  }
}

#define SET_CASE(ID, FIELD) case ID: trial.FIELD = value; break
bool ServoConfig_Set(ServoConfig *c, uint16_t id, int32_t value)
{
  ServoConfig trial;
  if (c == 0) return false;
  trial = *c;
  switch (id) {
    SET_CASE(SERVO_PARAM_NODE_ID,node_id); SET_CASE(SERVO_PARAM_TELEMETRY_MS,telemetry_ms);
    SET_CASE(SERVO_PARAM_POS_MIN_MDEG,position_min_mdeg); SET_CASE(SERVO_PARAM_POS_MAX_MDEG,position_max_mdeg);
    SET_CASE(SERVO_PARAM_POS_OFFSET_MDEG,position_offset_mdeg); SET_CASE(SERVO_PARAM_SENSOR_DIR,sensor_direction);
    SET_CASE(SERVO_PARAM_MOTOR_DIR,motor_direction); SET_CASE(SERVO_PARAM_MAX_CURRENT_MA,max_current_ma);
    SET_CASE(SERVO_PARAM_TRIP_CURRENT_MA,trip_current_ma); SET_CASE(SERVO_PARAM_MAX_SPEED_MDPS,max_speed_mdps);
    SET_CASE(SERVO_PARAM_MAX_POWER_MW,max_power_mw); SET_CASE(SERVO_PARAM_MAX_DUTY_PERMILLE,max_duty_permille);
    SET_CASE(SERVO_PARAM_TEMP_DERATE_CDEG,temp_derate_cdeg); SET_CASE(SERVO_PARAM_TEMP_SHUTDOWN_CDEG,temp_shutdown_cdeg);
    SET_CASE(SERVO_PARAM_TEMP_RECOVER_CDEG,temp_recover_cdeg); SET_CASE(SERVO_PARAM_BUS_UV_MV,bus_uv_mv);
    SET_CASE(SERVO_PARAM_BUS_OV_MV,bus_ov_mv); SET_CASE(SERVO_PARAM_POS_KP_X1000,position_kp_x1000);
    SET_CASE(SERVO_PARAM_SPEED_KP_X1000,speed_kp_x1000); SET_CASE(SERVO_PARAM_SPEED_KI_X1000,speed_ki_x1000);
    SET_CASE(SERVO_PARAM_CURRENT_KP_X1000,current_kp_x1000); SET_CASE(SERVO_PARAM_CURRENT_KI_X1000,current_ki_x1000);
    SET_CASE(SERVO_PARAM_POS_DEADBAND_MDEG,position_deadband_mdeg); SET_CASE(SERVO_PARAM_CAN_TIMEOUT_MS,can_timeout_ms);
    SET_CASE(SERVO_PARAM_PWM_TIMEOUT_MS,pwm_timeout_ms); SET_CASE(SERVO_PARAM_PWM_MIN_US,pwm_min_us);
    SET_CASE(SERVO_PARAM_PWM_MAX_US,pwm_max_us); SET_CASE(SERVO_PARAM_ANGLE_SOURCE,angle_source);
    SET_CASE(SERVO_PARAM_ANGLE_CROSSCHECK_MDEG,angle_crosscheck_mdeg); SET_CASE(SERVO_PARAM_ADC_VREF_MV,adc_vref_mv);
    SET_CASE(SERVO_PARAM_AS5600_FS_MV,as5600_analog_fs_mv); SET_CASE(SERVO_PARAM_BUS_DIVIDER_X1000,bus_divider_x1000);
    SET_CASE(SERVO_PARAM_CURRENT_UV_PER_A,current_uv_per_a); SET_CASE(SERVO_PARAM_NTC_BETA,ntc_beta);
    SET_CASE(SERVO_PARAM_NTC_R0_OHM,ntc_r0_ohm); SET_CASE(SERVO_PARAM_NTC_PULLUP_OHM,ntc_pullup_ohm);
    default: return false;
  }
  if (!ServoConfig_Validate(&trial)) return false;
  *c = trial;
  return true;
}
