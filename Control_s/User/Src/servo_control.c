#include "servo_control.h"
#include "motor_driver.h"
#include <math.h>
#include <string.h>

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }
static int32_t Clamp32(int32_t v,int32_t lo,int32_t hi){return v<lo?lo:(v>hi?hi:v);}
static float Clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static int32_t Wrap(int32_t v){while(v>=180000)v-=360000;while(v< -180000)v+=360000;return v;}
static int32_t LimitCurrentStep(int32_t prev, int32_t target, int32_t rise_step, int32_t fall_step)
{
  int32_t step;
  if (target == prev) return target;
  step = (Abs32(target) > Abs32(prev) && ((target >= 0 && prev >= 0) || (target <= 0 && prev <= 0))) ? rise_step : fall_step;
  if (target > prev + step) return prev + step;
  if (target < prev - step) return prev - step;
  return target;
}
static bool NeedsPosition(ServoMode m){return m==SERVO_MODE_SPEED||m==SERVO_MODE_POSITION||m==SERVO_MODE_PWM_INPUT;}

#define SPEED_ESTIMATE_INTERVAL_MS 50U
#define SPEED_START_ASSIST_MDPS 3000
#define SPEED_START_ASSIST_MIN_CURRENT_MA 900
#define SPEED_START_ASSIST_MAX_SPEED_MDPS 2000

//小舵机板只有一个温度检测，这里是冗余
static uint16_t Derate(const ServoControl *c)
{
  int32_t hot, span;
  if (!c->feedback.temp1_valid || !c->feedback.temp2_valid) return 0U;
  hot = c->feedback.temp1_cdeg > c->feedback.temp2_cdeg ? c->feedback.temp1_cdeg : c->feedback.temp2_cdeg;
  if (hot <= c->config->temp_derate_cdeg) return 1000U;
  if (hot >= c->config->temp_shutdown_cdeg) return 0U;
  span = c->config->temp_shutdown_cdeg - c->config->temp_derate_cdeg;
  return (uint16_t)((int64_t)(c->config->temp_shutdown_cdeg-hot)*1000LL/span);
}

static void ResetLoops(ServoControl *c)
{
  c->speed_integral=0.0f; c->current_integral=0.0f;
  c->telemetry.target_current_ma=0; c->telemetry.duty_permille=0;
}

void ServoControl_Init(ServoControl *c, ServoConfig *cfg)
{
  if(c==0||cfg==0)return; memset(c,0,sizeof(*c)); c->config=cfg;
  c->telemetry.state=SERVO_STATE_READY; c->telemetry.mode=SERVO_MODE_DISABLED;
  c->timeout_ms=(uint32_t)cfg->can_timeout_ms; MotorDriver_Disable();
}

void ServoControl_SetFeedback(ServoControl *c,const ServoFeedback *f)
{
  if(c==0||f==0)return; c->feedback=*f;
  c->telemetry.position_mdeg=Wrap(f->position_mdeg*c->config->sensor_direction+c->config->position_offset_mdeg);
  c->telemetry.current_ma=f->current_ma; c->telemetry.bus_mv=f->bus_mv;
  c->telemetry.temp1_cdeg=f->temp1_cdeg; c->telemetry.temp2_cdeg=f->temp2_cdeg;
  c->telemetry.i2c_valid=f->i2c_valid; c->telemetry.analog_valid=f->analog_valid;
  c->telemetry.pwm_valid=f->pwm_valid; c->telemetry.current_calibrated=f->current_calibrated;
}

bool ServoControl_Command(ServoControl *c,ServoMode mode,int32_t target,uint32_t timeout)
{
  if(c==0||mode<SERVO_MODE_CURRENT||mode>SERVO_MODE_PWM_INPUT||c->telemetry.faults!=0U)return false;
  if(!c->feedback.current_calibrated||!c->feedback.temp1_valid||!c->feedback.temp2_valid||
     (NeedsPosition(mode)&&!c->feedback.position_valid))return false;
  if(!c->telemetry.enabled||c->telemetry.mode!=mode)ResetLoops(c);
  c->telemetry.mode=mode;c->telemetry.target=target;c->telemetry.enabled=true;
  c->last_command_ms=HAL_GetTick();c->timeout_ms=timeout?timeout:(uint32_t)c->config->can_timeout_ms;
  return true;
}

void ServoControl_Disable(ServoControl *c)
{
  if(c==0)return;
	MotorDriver_Disable();
	c->telemetry.enabled=false;
  c->telemetry.mode=SERVO_MODE_DISABLED;
	c->telemetry.state=c->telemetry.faults?SERVO_STATE_FAULT:SERVO_STATE_READY;
	ResetLoops(c);
}
void ServoControl_AddFault(ServoControl *c,uint32_t f){if(c){c->telemetry.faults|=f;ServoControl_Disable(c);}}
void ServoControl_EStop(ServoControl *c){ServoControl_AddFault(c,SERVO_FAULT_ESTOP);}

bool ServoControl_ClearFaults(ServoControl *c)
{
  int16_t hot;
  if(c==0||c->telemetry.enabled||!c->feedback.current_calibrated||!c->feedback.temp1_valid||!c->feedback.temp2_valid)return false;
  hot=c->feedback.temp1_cdeg>c->feedback.temp2_cdeg?c->feedback.temp1_cdeg:c->feedback.temp2_cdeg;
  if(hot>=c->config->temp_recover_cdeg||Abs32(c->feedback.current_instant_ma)>c->config->max_current_ma)return false;
  c->telemetry.faults=0U;c->telemetry.state=SERVO_STATE_READY;return true;
}

void ServoControl_Update(ServoControl *c,uint32_t now,int32_t pwm_position)
{
  uint32_t dt; int32_t delta,hot,current_limit,speed_target=0,current_target=0,pos_error;
  uint16_t derate,max_duty; float speed_error,current_error,duty;
  if(c==0)return;
  dt=now-c->last_speed_ms;
  /* 低速时 12bit 编码器在 10ms 窗口内只有 0/1 个计数跳变，速度会抖；拉长到 50ms 提升分辨率。 */
  if(dt>=SPEED_ESTIMATE_INTERVAL_MS){delta=Wrap(c->telemetry.position_mdeg-c->last_position_mdeg);
    c->speed_filtered+=0.25f*((float)delta*1000.0f/(float)dt-c->speed_filtered);
    c->telemetry.speed_mdps=(int32_t)c->speed_filtered;c->last_position_mdeg=c->telemetry.position_mdeg;c->last_speed_ms=now;}
  c->telemetry.command_age_ms=now-c->last_command_ms;derate=Derate(c);c->telemetry.derate_permille=derate;
  if(!ServoConfig_Validate(c->config)){ServoControl_AddFault(c,SERVO_FAULT_CONFIG);return;}
  if(!c->feedback.temp1_valid||!c->feedback.temp2_valid){ServoControl_AddFault(c,SERVO_FAULT_TEMP_SENSOR);return;}
  hot=c->feedback.temp1_cdeg>c->feedback.temp2_cdeg?c->feedback.temp1_cdeg:c->feedback.temp2_cdeg;
  if(hot>=c->config->temp_shutdown_cdeg){ServoControl_AddFault(c,SERVO_FAULT_OVERTEMP);return;}
  if(Abs32(c->feedback.current_instant_ma)>=c->config->trip_current_ma){if(++c->overcurrent_count>=3U){ServoControl_AddFault(c,SERVO_FAULT_OVERCURRENT);return;}}
  else c->overcurrent_count=0U;
  if(c->telemetry.enabled&&c->feedback.bus_mv<(uint32_t)c->config->bus_uv_mv){ServoControl_AddFault(c,SERVO_FAULT_BUS_UV);return;}
  if(c->feedback.bus_mv>(uint32_t)c->config->bus_ov_mv){ServoControl_AddFault(c,SERVO_FAULT_BUS_OV);return;}
  if(c->telemetry.enabled&&c->telemetry.mode!=SERVO_MODE_PWM_INPUT&&c->telemetry.command_age_ms>c->timeout_ms){
    /* 软超时：先停机刹车，等下一条命令接管；不锁故障，避免摇杆换向被卡住。 */
    ServoControl_Disable(c);
    return;
  }
  if(c->telemetry.enabled&&c->telemetry.mode==SERVO_MODE_PWM_INPUT&&!c->feedback.pwm_valid){ServoControl_AddFault(c,SERVO_FAULT_PWM_TIMEOUT);return;}
  if(!c->telemetry.enabled){MotorDriver_Disable();return;}
  if(NeedsPosition(c->telemetry.mode)&&!c->feedback.position_valid){ServoControl_AddFault(c,SERVO_FAULT_POSITION);return;}
  current_limit=(int32_t)((int64_t)c->config->max_current_ma*derate/1000LL);
  if(c->feedback.bus_mv>1000U){int32_t p=(int32_t)((int64_t)c->config->max_power_mw*1000LL/c->feedback.bus_mv);if(p<current_limit)current_limit=p;}
  max_duty=(uint16_t)((int64_t)c->config->max_duty_permille*derate/1000LL);
  if(c->telemetry.mode==SERVO_MODE_CURRENT)current_target=Clamp32(c->telemetry.target,-current_limit,current_limit);
  else {
    if(c->telemetry.mode==SERVO_MODE_SPEED)speed_target=Clamp32(c->telemetry.target,-c->config->max_speed_mdps,c->config->max_speed_mdps);
    else {
			if(c->telemetry.mode==SERVO_MODE_PWM_INPUT)c->telemetry.target=pwm_position;
      c->telemetry.target=Clamp32(c->telemetry.target,c->config->position_min_mdeg,c->config->position_max_mdeg);
      pos_error=c->telemetry.target-c->telemetry.position_mdeg;if(Abs32(pos_error)<=c->config->position_deadband_mdeg)pos_error=0;
      speed_target=Clamp32((int32_t)((int64_t)pos_error*c->config->position_kp_x1000/1000LL),-c->config->max_speed_mdps,c->config->max_speed_mdps);}
    speed_error=((float)speed_target-c->telemetry.speed_mdps)/1000.0f;
    c->speed_integral=Clampf(c->speed_integral+speed_error*(float)c->config->speed_ki_x1000/1000.0f*0.001f,-(float)current_limit,(float)current_limit);
    current_target=Clamp32((int32_t)(speed_error*(float)c->config->speed_kp_x1000/1000.0f+c->speed_integral),-current_limit,current_limit);
  }
  /* 低速起步补偿：12bit 编码器在低速下速度误差很容易被量化吃掉，给一点最小起转电流。 */
  if(c->telemetry.mode==SERVO_MODE_SPEED&&Abs32(speed_target)>=SPEED_START_ASSIST_MDPS&&
     Abs32(c->telemetry.speed_mdps)<=SPEED_START_ASSIST_MAX_SPEED_MDPS&&
     Abs32(current_target)<SPEED_START_ASSIST_MIN_CURRENT_MA){
    current_target=(speed_target<0)?-SPEED_START_ASSIST_MIN_CURRENT_MA:SPEED_START_ASSIST_MIN_CURRENT_MA;
  }
  if(c->telemetry.mode!=SERVO_MODE_CURRENT){
    int32_t rise_slew=current_limit/12;
    int32_t fall_slew=current_limit/3;
    if(rise_slew<80)rise_slew=80;
    if(rise_slew>180)rise_slew=180;
    if(fall_slew<250)fall_slew=250;
    if(fall_slew>600)fall_slew=600;
    current_target=LimitCurrentStep(c->telemetry.target_current_ma,current_target,rise_slew,fall_slew);
  }
  c->telemetry.target_current_ma=current_target;
  if(Abs32(current_target)<50||max_duty==0U){
    c->current_integral=0.0f;
    c->telemetry.duty_permille=0;
    MotorDriver_Disable();
    c->telemetry.state=derate<1000U?SERVO_STATE_DERATING:SERVO_STATE_ACTIVE;
    return;
  }
  //电流环算法
  else {current_error=((float)Abs32(current_target)-(float)Abs32(c->feedback.current_ma))/1000.0f;
    c->current_integral=Clampf(c->current_integral+current_error*(float)c->config->current_ki_x1000/1000.0f*0.001f,0.0f,(float)max_duty);
    duty=Clampf(current_error*(float)c->config->current_kp_x1000/1000.0f+c->current_integral,0.0f,(float)max_duty);
    if(current_target<0)duty=-duty;}
  c->telemetry.duty_permille=(int16_t)(duty*c->config->motor_direction);
  MotorDriver_Set(c->telemetry.duty_permille,max_duty);MotorDriver_Enable();
  c->telemetry.state=derate<1000U?SERVO_STATE_DERATING:SERVO_STATE_ACTIVE;
}
