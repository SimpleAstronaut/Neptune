# Neptune

## 1.硬件基础

四麦轮底盘

### 1.1 主控板

大疆A板 STM32F427IIH

### 1.2 遥控器和接收器

DT7&DR16 2.4GHz

DR16内置在大疆A板上，位于USART1_RX(PB7)

usart dma接收到buff数组，利用官方解码代码解码，可以考虑双缓存

```c
#define RX_BUFFER_SIZE 20
uint8_t rxBuffer1[RX_BUFFER_SIZE];
uint8_t rxBuffer2[RX_BUFFER_SIZE];
HAL_UARTEx_ReeiveToIdle_DMA(&huart1, rxBuffer1, RX_BUFFER_SIZE);
```

### 1.3 IMU

使用板载内置IMU:IMU_INT(PB8)

MPU6500和IST8310

磁力计数据干扰大，禁用了磁力计数据的算法

SPI协议DMA接收，参考[【STM32】HAL库 STM32CubeMX教程十四---SPI_cubemx spi-CSDN博客](https://blog.csdn.net/as480133937/article/details/105849607)

### 1.4 电机和电调

未知，猜测为c620或c610搭配m3508电机

## 2.控制逻辑

DBUS接收USART1_RX(PB7)

### 2.1 停止模式

激活方式：同时向上推s1和s2

关闭方式：同时下拉s1和s2

关闭整车输出，仅接收遥控器s1和s2输入

### 2.2 全向模式

激活方式：同时下拉s1和s2

全向模式：遥控器
左手控制车的全向移动，遥控器右手控制车身旋转。

![](file://C:\Users\escsc\AppData\Roaming\marktext\images\2024-11-29-23-46-37-image.png?msec=1734068791282)

通道2 3控制全向移动

通道0控制旋转速度和方向：往右正值顺时针，往左负值逆时针

### 2.3 小陀螺模式

激活方式：S1拉上S2拉下

底盘以角速度x匀速旋转

通道2 3控制全向移动，

通道0控制正方向，与全向模式功能相似

### 2.4 运行流程

上电默认进入停止状态，需要保证s1和s2保持上拉状态

## 3.控制模块

### 3.1接收遥控器

DBUS协议，接收到DR16，从PB8 USART1_RX <mark>用</mark>dma接收到buff

参考 https://github.com/RoboMaster/RoboRTS-Firmware

在dbus.c下面，我们可以看到官方的解码函数，其完成的工作包括——数据的分离和拼接；为了防止遥控器数据的零漂，设置了一个正负5的死区；数据溢出的处理

```c
static void get_dr16_data(rc_device_t rc_dev, uint8_t *buff)
{

  memcpy(&(rc_dev->last_rc_info), &rc_dev->rc_info, sizeof(struct rc_info));

  rc_info_t rc = &rc_dev->rc_info;

  //satori：这里完成的是数据的分离和拼接，减去1024是为了让数据的中间值变为0
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  //satori:防止数据零漂，设置正负5的死区
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;

  //satori:防止数据溢出
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(struct rc_info));
    return ;
  }

  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
  rc->mouse.y = buff[8] | (buff[9] << 8);
  rc->mouse.z = buff[10] | (buff[11] << 8);

  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
  rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}
```

找到对应的数据结构rc_info，其内容和手册上的内容完全一一对应，可以看到官方代码为了方便后续代码的编写，在键盘的数据处使用了联合体。

```c
struct rc_info
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
};
```

关于遥控器数据的接收，我们可以直接去寻找解码函数在何处被调用，按照

*get_dr16_data -> rc_device_date_update -> dr16_rx_data_by_uart -> dr16_rx_callback -> dr16_uart_rx_data_handle -> USART1_IRQHandler*

可以发现其是在串口1的接收中断中被调用的，DMA初始化函数如下

```c
void dr16_uart_init(void)
{
  UART_Receive_DMA_No_IT(&huart1, dr16_uart_rx_buff, DR16_RX_BUFFER_SIZE);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
```

接收处理函数如下

抛除一些HAL库为了驱动DMA进行的一些寄存器操作，其实我们可以看到整个DMA接收逻辑实际上还是很简单的，即通过**dr16_uart_rx_buff**这个数组存储DMA获取的数据，每次完成接收之后对接收数据的长度进行判断，如果确认了是18个字节则判定为合法数据，传入**dr16_rx_callback**进行下一步的处理。

虽然官方的代码为了各种目的进行了很多层的封装，但是实际上在dma接收处理函数中进行数据合法性的判断之后，已经可以直接将数据送入解码函数了。

```c
uint32_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */
    if ((DR16_RX_BUFFER_SIZE - huart->hdmarx->Instance->NDTR) == DR16_DATA_LEN)
    {
      if (dr16_rx_callback != NULL)
      {
        dr16_rx_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
      }

      if (dr16_forword_callback != NULL)
      {
        dr16_forword_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
      }
    }

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RX_BUFFER_SIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
  return 0;
}
```

### 3.2 陀螺仪

引用imu模块，**有可能需要采用禁用磁力计的IMU算法**

初始化imu mpu6500和磁强计ist3810

```c
mpu_device_init(void);
```

循环调用mpu_get_data()函数，从mpu_data结构体的gx gy gz获取方向数控

```c
mpu_get_data();

typedef struct
{
    int16_t ax;                        //    加速度（acc）的x                
    int16_t ay;                        //    加速度（acc）的y    4            
    int16_t az;                        //    加速度（acc）的z                

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;                        //陀螺仪（gyro）的x
    int16_t gy;                        //陀螺仪（gyro）的y
    int16_t gz;                        //陀螺仪（gyro）的z

    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;

    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;

    int16_t mx_offset;
    int16_t my_offset;
    int16_t mz_offset;
} mpu_data_t;
```

**官方车代码里面同样有解算的代码，并且有多种算法。下面这段代码是一个禁用了磁力计数据的算法，也是我自己以前移植到自己的工程里进行过测试的，由于当时我发现磁力计读取到的数据干扰很大，于是选择了禁用了磁力计数据的算法，只使用加速度计和陀螺仪进行数据融合。其中invSqrt是运用牛顿迭代法快速求平方根，是用于归一化处理的，官方给的注释在我看来已经已到位了，因此在这里不去加更多的注脚。**

```c
void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = sensor->wx;
  gy = sensor->wy;
  gz = sensor->wz;
  ax = sensor->ax;
  ay = sensor->ay;
  az = sensor->az;
  mx = sensor->mx;
  my = sensor->my;
  mz = sensor->mz;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  atti->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll     -pi----pi
  atti->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // pitch    -pi/2----pi/2
  atti->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw      -pi----pi
}
```

除了读取寄存器值进行解算之外，MPU6500的姿态数据也可以通过芯片自身集成的DMP模块获取，有兴趣的同学可以自己加以研究。

最后说一下我自己使用板载陀螺仪进行解算的体验，实际解算出来的角速度数据和角度数据都是比较可靠的，波动也很小，我将板子静止放了10分钟左右，只观测到了极小的累积误差。

### 3.3 计算输出速度值

参考 [RoboMaster电控入门（5）麦克纳姆轮底盘控制 - sasasatori - 博客园](https://www.cnblogs.com/sasasatori/p/11720959.html)

大疆官方示例代码解算函数

```c
/**
  * @brief mecanum glb_chassis velocity decomposition.F:forword; B:backword; L:left; R:right
  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
void mecanum_calculate(struct mecanum *mec)
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_bl;
    static float rotate_ratio_br;
    static float wheel_rpm_ratio;

    rotate_ratio_fr = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_fl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f - mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset - mec->param.rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((mec->param.wheelbase + mec->param.wheeltrack) / 2.0f + mec->param.rotate_x_offset + mec->param.rotate_y_offset) / RADIAN_COEF;

    wheel_rpm_ratio = 60.0f / (mec->param.wheel_perimeter * MOTOR_DECELE_RATIO);

    MEC_VAL_LIMIT(mec->speed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
    MEC_VAL_LIMIT(mec->speed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
    MEC_VAL_LIMIT(mec->speed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //deg/s

    float wheel_rpm[4];
    float max = 0;

    wheel_rpm[0] = (-mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (mec->speed.vx - mec->speed.vy - mec->speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] = (mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-mec->speed.vx + mec->speed.vy - mec->speed.vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabs(wheel_rpm[i]) > max)
        {
            max = fabs(wheel_rpm[i]);
        }
    }

    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }
    memcpy(mec->wheel_rpm, wheel_rpm, 4 * sizeof(float));
}
```

### 3.4 PID

考虑到pid的复用性，单独开发pid模块

使用位置式pid

参考大疆官方pid代码

[RoboRTS-Firmware/components/algorithm/pid.c at 8cffd936b503eef819fa05265cf0162710c9cb3b · RoboMaster/RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware/blob/8cffd936b503eef819fa05265cf0162710c9cb3b/components/algorithm/pid.c)

```c
/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "sys.h"
#include "pid.h"

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
    {
        *a = ABS_MAX;
    }
    if (*a < -ABS_MAX)
    {
        *a = -ABS_MAX;
    }
}

static void pid_param_init(
    struct pid *pid,
    float maxout,
    float integral_limit,
    float kp,
    float ki,
    float kd)
{

    pid->param.integral_limit = integral_limit;
    pid->param.max_out = maxout;

    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;
}

/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(struct pid *pid, float kp, float ki, float kd)
{
    pid->param.p = kp;
    pid->param.i = ki;
    pid->param.d = kd;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float pid_calculate(struct pid *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err = set - get;
    if ((pid->param.input_max_err != 0) && (fabs(pid->err) > pid->param.input_max_err))
    {
        return 0;
    }

    pid->pout = pid->param.p * pid->err;
    pid->iout += pid->param.i * pid->err;
    pid->dout = pid->param.d * (pid->err - pid->last_err);

    abs_limit(&(pid->iout), pid->param.integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->param.max_out);

    if (pid->enable == 0)
    {
        pid->out = 0;
    }

    return pid->out;
}
/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void pid_struct_init(
    struct pid *pid,
    float maxout,
    float integral_limit,

    float kp,
    float ki,
    float kd)
{
    pid->enable = 1;
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;

    pid->f_param_init(pid, maxout, integral_limit, kp, ki, kd);
    pid->f_pid_reset(pid, kp, ki, kd);
}
```

### 3.5 电机输出

调用模块4计算输出值，pid计算出电流值闭环控制电机

参考[RoboRTS-Firmware/components/modules/chassis.c at 8cffd936b503eef819fa05265cf0162710c9cb3b · RoboMaster/RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware/blob/8cffd936b503eef819fa05265cf0162710c9cb3b/components/modules/chassis.c)

写出该文件类似物

## 4.程序架构

如图
