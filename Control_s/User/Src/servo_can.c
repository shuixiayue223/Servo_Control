#include "servo_can.h"
#include "analog_sensors.h"
#include <string.h>
#define QLEN 16U
typedef struct { CAN_RxHeaderTypeDef h; uint8_t d[8]; } Frame;
static CAN_HandleTypeDef *s_can;static ServoControl *s_ctrl;static ServoConfig *s_cfg;
static volatile Frame s_q[QLEN];static volatile uint8_t s_head,s_tail;
static uint32_t s_fast,s_slow,s_heart;
static uint16_t U16(const uint8_t*d){return(uint16_t)(d[0]|((uint16_t)d[1]<<8));}
static int32_t I32(const uint8_t*d){return(int32_t)((uint32_t)d[0]|((uint32_t)d[1]<<8)|((uint32_t)d[2]<<16)|((uint32_t)d[3]<<24));}
static void W16(uint8_t*d,uint16_t v){d[0]=(uint8_t)v;d[1]=(uint8_t)(v>>8);}
static void W32(uint8_t*d,uint32_t v){d[0]=(uint8_t)v;d[1]=(uint8_t)(v>>8);d[2]=(uint8_t)(v>>16);d[3]=(uint8_t)(v>>24);}
static int16_t S16(int32_t v){return v>32767?32767:(v< -32768?-32768:(int16_t)v);}
static bool Send(uint16_t id,uint8_t*d){CAN_TxHeaderTypeDef h;uint32_t box;if(HAL_CAN_GetTxMailboxesFreeLevel(s_can)==0U)return false;
  memset(&h,0,sizeof(h));h.StdId=id;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;return HAL_CAN_AddTxMessage(s_can,&h,d,&box)==HAL_OK;}
static void Ack(uint8_t status,uint8_t op,uint8_t seq){uint8_t d[8]={0};d[0]=status;d[1]=op;d[2]=seq;d[3]=(uint8_t)s_ctrl->telemetry.state;W32(&d[4],s_ctrl->telemetry.faults);Send((uint16_t)(0x140+s_cfg->node_id),d);}


HAL_StatusTypeDef ServoCAN_Init(CAN_HandleTypeDef *hcan,ServoControl *ctrl,ServoConfig *cfg)
{
  CAN_FilterTypeDef f;HAL_StatusTypeDef r;uint32_t now;if(hcan==0||ctrl==0||cfg==0)return HAL_ERROR;
  s_can=hcan;s_ctrl=ctrl;s_cfg=cfg;s_head=s_tail=0;memset(&f,0,sizeof(f));
  f.FilterBank=0;f.FilterMode=CAN_FILTERMODE_IDMASK;f.FilterScale=CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh=0;f.FilterMaskIdHigh=0;f.FilterMaskIdLow=0x0006;f.FilterFIFOAssignment=CAN_FILTER_FIFO1;f.FilterActivation=ENABLE;f.SlaveStartFilterBank=14;
  r=HAL_CAN_ConfigFilter(s_can,&f);if(r==HAL_OK)r=HAL_CAN_Start(s_can);
  if(r==HAL_OK)r=HAL_CAN_ActivateNotification(s_can,CAN_IT_RX_FIFO1_MSG_PENDING);
  now=HAL_GetTick();s_fast=now+cfg->telemetry_ms;s_slow=now+500U;s_heart=now+1007U;return r;
}

void ServoCAN_RxCallback(CAN_HandleTypeDef *hcan)
{
  Frame f;uint8_t n;if(hcan!=s_can||HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&f.h,f.d)!=HAL_OK)return;
  n=(uint8_t)((s_head+1U)%QLEN);if(n==s_tail){ServoControl_AddFault(s_ctrl,SERVO_FAULT_CAN_OVERFLOW);return;}s_q[s_head]=f;s_head=n;
}

void ServoCAN_Process(uint32_t now)
{
  Frame f;uint16_t base,id;uint8_t node,op,seq,status;int32_t target,value;bool broadcast;
  (void)now;
  while(s_tail!=s_head){f=s_q[s_tail];s_tail=(uint8_t)((s_tail+1U)%QLEN);
    if(f.h.IDE!=CAN_ID_STD||f.h.RTR!=CAN_RTR_DATA||f.h.DLC!=8U)continue;
    id=(uint16_t)f.h.StdId;base=(uint16_t)(id&0x7C0U);node=(uint8_t)(id&0x3FU);broadcast=node==0U;
    if(!broadcast&&node!=(uint8_t)s_cfg->node_id)continue;
    if(base==0x080U){if(f.d[0]!=0xA5U||f.d[1]!=0x5AU)continue;if(f.d[3]==0U)ServoControl_EStop(s_ctrl);else if(f.d[3]==1U)ServoControl_Disable(s_ctrl);if(!broadcast)Ack(0,f.d[3],f.d[2]);}
    else if(base==0x100U){op=f.d[0];seq=f.d[2];target=I32(&f.d[4]);status=0;
      if(broadcast&&op!=0U)continue;
      if(op==0U)ServoControl_Disable(s_ctrl);
      else if(op>=1U&&op<=4U){if(op==3U&&(f.d[1]&1U))target+=s_ctrl->telemetry.position_mdeg;if(!ServoControl_Command(s_ctrl,(ServoMode)op,target,(uint32_t)f.d[3]*10U))status=5U;}
      else if(op==5U){if(!ServoControl_ClearFaults(s_ctrl))status=5U;}
      else if(op==6U){if(s_ctrl->telemetry.enabled)status=5U;else s_cfg->position_offset_mdeg-=s_ctrl->telemetry.position_mdeg;}
      else if(op==7U){if(s_ctrl->telemetry.enabled)status=5U;else AnalogSensors_CalibrateCurrent();}
      else if(op!=8U)status=2U;if(!broadcast)Ack(status,op,seq);}
    else if(base==0x180U&&!broadcast){op=f.d[0];seq=f.d[1];value=I32(&f.d[4]);status=0;
      if(op==0U){if(!ServoConfig_Get(s_cfg,U16(&f.d[2]),&value))status=3U;}
      else if(op==1U){if(s_ctrl->telemetry.enabled)status=5U;else if(!ServoConfig_Set(s_cfg,U16(&f.d[2]),value))status=4U;}
      else if(op==3U){if(s_ctrl->telemetry.enabled)status=5U;else ServoConfig_Defaults(s_cfg);}
      else status=2U;{uint8_t d[8];d[0]=status;d[1]=seq;W16(&d[2],U16(&f.d[2]));W32(&d[4],(uint32_t)value);Send((uint16_t)(0x1C0+s_cfg->node_id),d);}}
  }
}

void ServoCAN_Telemetry(uint32_t now)
{
  ServoTelemetry*t=&s_ctrl->telemetry;const AnalogSensorData*a;uint8_t d[8]={0};int32_t v;
  if((int32_t)(now-s_fast)>=0){W32(&d[0],(uint32_t)t->position_mdeg);v=t->speed_mdps/100;if(v>32767)v=32767;if(v< -32768)v=-32768;W16(&d[4],(uint16_t)(int16_t)v);
    v=t->current_ma/10;if(v>32767)v=32767;if(v< -32768)v=-32768;W16(&d[6],(uint16_t)(int16_t)v);Send((uint16_t)(0x200+s_cfg->node_id),d);
    a=AnalogSensors_Get();W16(&d[0],a->raw[0]);W16(&d[2],a->current_zero_adc);W16(&d[4],(uint16_t)S16(a->current_instant_ma));W16(&d[6],(uint16_t)S16(a->current_ma));Send((uint16_t)(0x300+s_cfg->node_id),d);
    W16(&d[0],(uint16_t)S16(t->target_current_ma));W16(&d[2],(uint16_t)t->duty_permille);W16(&d[4],a->current_raw_min);W16(&d[6],a->current_raw_max);Send((uint16_t)(0x340+s_cfg->node_id),d);AnalogSensors_ResetCurrentExtrema();
    s_fast=now+s_cfg->telemetry_ms;}
  if((int32_t)(now-s_slow)>=0){W16(&d[0],(uint16_t)(t->bus_mv>65535U?65535U:t->bus_mv));W16(&d[2],(uint16_t)t->temp1_cdeg);W16(&d[4],(uint16_t)t->temp2_cdeg);W16(&d[6],t->derate_permille);Send((uint16_t)(0x240+s_cfg->node_id),d);
    W32(&d[0],t->faults);d[4]=(uint8_t)t->state;d[5]=(uint8_t)t->mode;d[6]=(uint8_t)((t->enabled?1U:0U)|(t->pwm_valid?2U:0U)|(t->i2c_valid?4U:0U)|(t->analog_valid?8U:0U)|(t->current_calibrated?64U:0U));d[7]=(uint8_t)(t->command_age_ms>2550U?255U:t->command_age_ms/10U);Send((uint16_t)(0x280+s_cfg->node_id),d);s_slow=now+500U;}
  if((int32_t)(now-s_heart)>=0){W32(&d[0],now/1000U);W16(&d[4],0x0100U);d[6]=(uint8_t)s_cfg->node_id;d[7]=1U;Send((uint16_t)(0x2C0+s_cfg->node_id),d);s_heart=now+1000U;}
}
