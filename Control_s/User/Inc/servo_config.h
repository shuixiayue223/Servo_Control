#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SERVO_PARAM_NODE_ID = 0x0001,
  SERVO_PARAM_TELEMETRY_MS = 0x0002,
  SERVO_PARAM_POS_MIN_MDEG = 0x0010,
  SERVO_PARAM_POS_MAX_MDEG = 0x0011,
  SERVO_PARAM_POS_OFFSET_MDEG = 0x0012,
  SERVO_PARAM_SENSOR_DIR = 0x0013,
  SERVO_PARAM_MOTOR_DIR = 0x0014,
  SERVO_PARAM_MAX_CURRENT_MA = 0x0020,
  SERVO_PARAM_TRIP_CURRENT_MA = 0x0021,
  SERVO_PARAM_MAX_SPEED_MDPS = 0x0022,
  SERVO_PARAM_MAX_POWER_MW = 0x0023,
  SERVO_PARAM_MAX_DUTY_PERMILLE = 0x0024,
  SERVO_PARAM_TEMP_DERATE_CDEG = 0x0030,
  SERVO_PARAM_TEMP_SHUTDOWN_CDEG = 0x0031,
  SERVO_PARAM_TEMP_RECOVER_CDEG = 0x0032,
  SERVO_PARAM_BUS_UV_MV = 0x0033,
  SERVO_PARAM_BUS_OV_MV = 0x0034,
  SERVO_PARAM_POS_KP_X1000 = 0x0040,
  SERVO_PARAM_SPEED_KP_X1000 = 0x0041,
  SERVO_PARAM_SPEED_KI_X1000 = 0x0042,
  SERVO_PARAM_CURRENT_KP_X1000 = 0x0043,
  SERVO_PARAM_CURRENT_KI_X1000 = 0x0044,
  SERVO_PARAM_POS_DEADBAND_MDEG = 0x0045,
  SERVO_PARAM_CAN_TIMEOUT_MS = 0x0050,
  SERVO_PARAM_PWM_TIMEOUT_MS = 0x0051,
  SERVO_PARAM_PWM_MIN_US = 0x0052,
  SERVO_PARAM_PWM_MAX_US = 0x0053,
  SERVO_PARAM_ANGLE_SOURCE = 0x0060,
  SERVO_PARAM_ANGLE_CROSSCHECK_MDEG = 0x0061,
  SERVO_PARAM_ADC_VREF_MV = 0x0062,
  SERVO_PARAM_AS5600_FS_MV = 0x0063,
  SERVO_PARAM_BUS_DIVIDER_X1000 = 0x0064,
  SERVO_PARAM_CURRENT_UV_PER_A = 0x0065,
  SERVO_PARAM_NTC_BETA = 0x0066,
  SERVO_PARAM_NTC_R0_OHM = 0x0067,
  SERVO_PARAM_NTC_PULLUP_OHM = 0x0068
} ServoParamId;

typedef struct {
  int32_t node_id;
  int32_t telemetry_ms;
  int32_t position_min_mdeg;
  int32_t position_max_mdeg;
  int32_t position_offset_mdeg;
  int32_t sensor_direction;
  int32_t motor_direction;
  int32_t max_current_ma;
  int32_t trip_current_ma;
  int32_t max_speed_mdps;
  int32_t max_power_mw;
  int32_t max_duty_permille;
  int32_t temp_derate_cdeg;
  int32_t temp_shutdown_cdeg;
  int32_t temp_recover_cdeg;
  int32_t bus_uv_mv;
  int32_t bus_ov_mv;
  int32_t position_kp_x1000;
  int32_t speed_kp_x1000;
  int32_t speed_ki_x1000;
  int32_t current_kp_x1000;
  int32_t current_ki_x1000;
  int32_t position_deadband_mdeg;
  int32_t can_timeout_ms;
  int32_t pwm_timeout_ms;
  int32_t pwm_min_us;
  int32_t pwm_max_us;
  int32_t angle_source; /* 0=I2C, 1=analog, 2=cross-check */
  int32_t angle_crosscheck_mdeg;
  int32_t adc_vref_mv;
  int32_t as5600_analog_fs_mv;
  int32_t bus_divider_x1000;
  int32_t current_uv_per_a;
  int32_t ntc_beta;
  int32_t ntc_r0_ohm;
  int32_t ntc_pullup_ohm;
} ServoConfig;

void ServoConfig_Defaults(ServoConfig *config);
bool ServoConfig_Validate(const ServoConfig *config);
bool ServoConfig_Get(const ServoConfig *config, uint16_t id, int32_t *value);
bool ServoConfig_Set(ServoConfig *config, uint16_t id, int32_t value);

#endif
