/****************************************************************************
 *  Neptune imu module
 *
 *  使用imu mpu6500和磁强计ist3810,板载内置IMU:IMU_INT(PB8)
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "main.h"
#include "imu.h"


/***************************************************************** 
  * 函数功能: rc接收初始化
  * 输入参数: TODO
  * 返 回 值: 无
  ****************************************************************/
void imu_init(void)
{
		mpu_device_init();
}


/***************************************************************** 
  * 函数功能: 加速度积分
  * 输入参数: TODO
  * 返 回 值:
  ****************************************************************/
//TODO