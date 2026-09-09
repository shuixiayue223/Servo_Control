#include "servo_app.h"
#include "analog_sensors.h"
#include "as5600.h"
#include "motor_driver.h"
#include "pwm_input.h"
#include "servo_can.h"
#include "servo_config.h"
#include "servo_control.h"
#include <string.h>

static ServoConfig s_cfg;static ServoControl s_ctrl;static GPIO_TypeDef*s_led_port;static uint16_t s_led_pin;
static uint32_t s_control_tick,s_angle_tick;static bool s_ready;
static int32_t Diff(int32_t a,int32_t b){int32_t d=a-b;while(d>180000)d-=360000;while(d< -180000)d+=360000;return d<0?-d:d;}

bool ServoApp_Init(ADC_HandleTypeDef*adc,I2C_HandleTypeDef*i2c,CAN_HandleTypeDef*can,
 TIM_HandleTypeDef*motor,TIM_HandleTypeDef*pwm,GPIO_TypeDef*led_port,uint16_t led_pin)
{
  s_ready=false;s_led_port=led_port;s_led_pin=led_pin;ServoConfig_Defaults(&s_cfg);
  if(MotorDriver_Init(motor)!=HAL_OK)return false;ServoControl_Init(&s_ctrl,&s_cfg);
  if(AnalogSensors_Init(adc,&s_cfg)!=HAL_OK)return false;AS5600_Init(i2c);
  if(PWMInput_Init(pwm,&s_cfg)!=HAL_OK)return false;if(ServoCAN_Init(can,&s_ctrl,&s_cfg)!=HAL_OK)return false;
  s_control_tick=s_angle_tick=HAL_GetTick();s_ready=true;return true;
}

void ServoApp_Process(void)
{
  uint32_t now;ServoFeedback f;const AnalogSensorData*a;const AS5600_Data*d;const PWMInputData*p;
  if(!s_ready){MotorDriver_Disable();return;}now=HAL_GetTick();ServoCAN_Process(now);
  if(now-s_angle_tick>=5U){s_angle_tick=now;AS5600_Update(now);}
  if(now-s_control_tick>=1U){s_control_tick=now;AnalogSensors_Update();PWMInput_Update(now);a=AnalogSensors_Get();d=AS5600_Get();p=PWMInput_Get();memset(&f,0,sizeof(f));
    f.i2c_valid=d->valid;f.analog_valid=a->angle_valid;f.magnet_ok=d->magnet_ok;f.angle_mismatch=d->valid&&a->angle_valid&&Diff(d->angle_mdeg,a->analog_angle_mdeg)>s_cfg.angle_crosscheck_mdeg;
    if(s_cfg.angle_source==0){f.position_mdeg=d->angle_mdeg;f.position_valid=d->valid;}
    else if(s_cfg.angle_source==1){f.position_mdeg=a->analog_angle_mdeg;f.position_valid=a->angle_valid;}
    else{f.position_mdeg=d->angle_mdeg;f.position_valid=d->valid&&a->angle_valid&&!f.angle_mismatch;}
    f.current_ma=a->current_ma;f.current_instant_ma=a->current_instant_ma;f.bus_mv=a->bus_mv;f.temp1_cdeg=a->temp1_cdeg;f.temp2_cdeg=a->temp2_cdeg;
    f.temp1_valid=a->temp1_valid;f.temp2_valid=a->temp2_valid;f.current_calibrated=a->current_calibrated;f.pwm_valid=p->valid;
    ServoControl_SetFeedback(&s_ctrl,&f);ServoControl_Update(&s_ctrl,now,PWMInput_PositionMdeg());
    if(s_led_port){GPIO_PinState state=s_ctrl.telemetry.faults?GPIO_PIN_SET:(((now/(s_ctrl.telemetry.enabled?100U:500U))&1U)?GPIO_PIN_SET:GPIO_PIN_RESET);HAL_GPIO_WritePin(s_led_port,s_led_pin,state);}
  }
  ServoCAN_Telemetry(now);
}
void ServoApp_CAN_RxCallback(CAN_HandleTypeDef*hcan){ServoCAN_RxCallback(hcan);}
void ServoApp_TIM_IC_Callback(TIM_HandleTypeDef*htim){PWMInput_CaptureCallback(htim);}
void ServoApp_EmergencyShutdown(void){ServoControl_Disable(&s_ctrl);}
