# 舵机业务模块接入说明

本目录是独立业务代码，不修改任何 CubeMX 生成文件。`User/Inc` 加入头文件搜索路径，`User/Src` 中全部 `.c` 加入 Keil 工程即可。

## 一、先在 CubeMX 调整

当前 `H BRIDGE.ioc` 中 ADC 配置仍为3个序列且三个序列都是 `ADC_CHANNEL_0`，必须先修正，否则DMA缓冲区没有母线、温度和AS5600模拟量数据。

### ADC1 + DMA（必须）

Regular Conversion 数量设置为5，顺序固定为：

1. Rank 1：IN0 / PA0 / CURRENT_SENSE
2. Rank 2：IN1 / PA1 / VBUS_SENSE
3. Rank 3：IN2 / PA2 / MOS_TEMP_1
4. Rank 4：IN4 / PA4 / AS5600_ADC（小舵机板不再使用 MOS_TEMP_2）
5. Rank 5：IN4 / PA4 / AS5600_ADC

设置：Scan Conversion Enabled、Continuous Conversion Enabled、采样时间建议71.5 cycles。ADC1 DMA设置为Circular、Memory Increment、Peripheral/Memory Half Word。

### TIM1全桥PWM（必须）

当前 `.ioc` 为约2 kHz，建议改为：

- Prescaler = 0
- Counter Period = 3599
- PWM频率 = 72 MHz / 3600 = 20 kHz
- CH1+CH1N、CH2+CH2N均为PWM Generation
- Dead Time = 36（72 MHz、CKD_DIV1时约500 ns，最终以示波器和MOSFET开关时间确认）
- Break输入未接硬件时保持Disable；Automatic Output保持Disable

### TIM4 PWM输入（现配置可用）

- Prescaler = 71，计数频率1 MHz
- Period = 65535
- CH3 Direct Input，Rising edge，Filter可保持8
- 打开TIM4 global interrupt

### CAN（必须确认）

- Normal mode，500 kbit/s，打开CAN RX FIFO1 interrupt
- 当前 Prescaler=8、BS1=5、BS2=3 可得到500 kbit/s
- 更推荐 Prescaler=4、BS1=13、BS2=4、SJW=1，采样点约77.8%
- 建议开启Automatic Bus-Off和Automatic Retransmission

### I2C1

100 kHz或400 kHz均可。本模块使用阻塞式HAL读取，不依赖I2C事件中断。硬件必须确认PB6/PB7存在上拉电阻。

修改后点击 `GENERATE CODE`，确认生成 `Core/Inc`、`Core/Src` 和完整Keil工程。

## 二、Keil添加模块

1. 打开生成后的 `MDK-ARM/H BRIDGE.uvprojx`。
2. 在工程树右键 Target，选择 `Add Group`，命名为 `User/Servo`。
3. 右键该Group，选择 `Add Existing Files to Group`。
4. 添加 `User/Src` 下全部8个 `.c`：
   - `servo_config.c`
   - `analog_sensors.c`
   - `as5600.c`
   - `motor_driver.c`
   - `pwm_input.c`
   - `servo_control.c`
   - `servo_can.c`
   - `servo_app.c`
5. `Options for Target → C/C++ → Include Paths` 添加：`..\User\Inc`
6. 不需要把 `.h` 作为源文件编译。

再次用CubeMX生成代码后，CubeMX可能重写Keil工程文件；如果User Group消失，重新执行上述Keil添加步骤即可，`User`目录中的源码不会受影响。

## 三、main.c中添加

只在CubeMX保留的 `USER CODE` 区域中写入。

### USER CODE BEGIN Includes

```c
#include "servo_app.h"
```

### USER CODE BEGIN 2（所有MX_xxx_Init之后）

```c
if (!ServoApp_Init(&hadc1,
                   &hi2c1,
                   &hcan,
                   &htim1,
                   &htim4,
                   STATUS_LED_GPIO_Port,
                   STATUS_LED_Pin))
{
  Error_Handler();
}
```

### while(1)循环

```c
while (1)
{
  ServoApp_Process();
}
```

### USER CODE BEGIN 4：中断回调

```c
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  ServoApp_CAN_RxCallback(hcan_ptr);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim_ptr)
{
  ServoApp_TIM_IC_Callback(htim_ptr);
}
```

### Error_Handler

在 `__disable_irq()` 之前增加：

```c
ServoApp_EmergencyShutdown();
```

HardFault、BusFault等异常处理也建议在死循环之前调用 `ServoApp_EmergencyShutdown()`，确保TIM1 MOE立即关闭。

## 四、模块行为

- 上电自动启动ADC DMA，并在电机未使能时采128个样本校准INA181的1.65 V零点。
- AS5600地址为0x36，每5 ms读取一次I2C角度和磁铁状态。
- AS5600模拟量按外部1/2分压恢复为0～5 V，再映射0～360°。
- PWM输入默认1000～2000 µs映射到配置的位置最小值～最大值，连续3帧有效后接管。
- 控制环周期1 ms，支持电流、速度、位置和物理PWM输入模式。
- 温度达到70°C开始线性降额，90°C关断；默认限流只有3 A，便于首次联调。
- CAN Protocol v1：标准帧、11 bit、500 kbit/s；节点默认1。
- 本版参数只保存在RAM，复位后恢复默认值。PARAM SAVE尚未实现，因此不需要修改链接区或预留Flash。

## 五、首次通电

先断开电机并使用限流电源。确认四路门极PWM互补关系和死区后，再连接空载电机。首先核对电流零点、母线电压、两路温度、I2C角度和模拟角度；方向不正确时修改 `sensor_direction` 或 `motor_direction`，不要直接提高电流限制。
