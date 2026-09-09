#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H
#include "servo_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { SERVO_MODE_DISABLED=0, SERVO_MODE_CURRENT=1, SERVO_MODE_SPEED=2,
  SERVO_MODE_POSITION=3, SERVO_MODE_PWM_INPUT=4 } ServoMode;
typedef enum { SERVO_STATE_BOOT=0, SERVO_STATE_READY=1, SERVO_STATE_ACTIVE=2,
  SERVO_STATE_DERATING=3, SERVO_STATE_FAULT=4 } ServoState;

#define SERVO_FAULT_ESTOP           (1UL<<0)
#define SERVO_FAULT_OVERCURRENT     (1UL<<1)
#define SERVO_FAULT_OVERTEMP        (1UL<<2)
#define SERVO_FAULT_TEMP_SENSOR     (1UL<<3)
#define SERVO_FAULT_BUS_UV          (1UL<<4)
#define SERVO_FAULT_BUS_OV          (1UL<<5)
#define SERVO_FAULT_POSITION        (1UL<<6)
#define SERVO_FAULT_MAGNET          (1UL<<7)
#define SERVO_FAULT_ANGLE_MISMATCH  (1UL<<8)
#define SERVO_FAULT_CAN_TIMEOUT     (1UL<<9)
#define SERVO_FAULT_PWM_TIMEOUT     (1UL<<10)
#define SERVO_FAULT_ADC_NOT_READY   (1UL<<11)
#define SERVO_FAULT_CAN_OVERFLOW    (1UL<<12)
#define SERVO_FAULT_CONFIG          (1UL<<13)

typedef struct {
  int32_t position_mdeg;
  int32_t current_ma;
  int32_t current_instant_ma;
  uint32_t bus_mv;
  int16_t temp1_cdeg, temp2_cdeg;
  bool position_valid, i2c_valid, analog_valid, magnet_ok;
  bool temp1_valid, temp2_valid, current_calibrated, angle_mismatch, pwm_valid;
} ServoFeedback;

typedef struct {
  ServoState state; ServoMode mode; bool enabled;
  int32_t target, position_mdeg, speed_mdps, current_ma, target_current_ma;
  uint32_t bus_mv, faults, command_age_ms;
  int16_t temp1_cdeg, temp2_cdeg, duty_permille;
  uint16_t derate_permille;
  bool i2c_valid, analog_valid, pwm_valid, current_calibrated;
} ServoTelemetry;

typedef struct {
  ServoConfig *config; ServoFeedback feedback; ServoTelemetry telemetry;
  uint32_t last_command_ms, timeout_ms, last_speed_ms;
  int32_t last_position_mdeg; float speed_filtered, speed_integral, current_integral;
  uint8_t overcurrent_count;
} ServoControl;

void ServoControl_Init(ServoControl *control, ServoConfig *config);
void ServoControl_SetFeedback(ServoControl *control, const ServoFeedback *feedback);
bool ServoControl_Command(ServoControl *control, ServoMode mode, int32_t target, uint32_t timeout_ms);
void ServoControl_Update(ServoControl *control, uint32_t now_ms, int32_t pwm_position_mdeg);
void ServoControl_Disable(ServoControl *control);
void ServoControl_EStop(ServoControl *control);
bool ServoControl_ClearFaults(ServoControl *control);
void ServoControl_AddFault(ServoControl *control, uint32_t fault);
#endif
