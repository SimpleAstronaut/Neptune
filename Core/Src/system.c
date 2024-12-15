/****************************************************************************
 *  Neptune system module
 *
 *  主控模块,获取各模块数据并计算电流值
 *  Author: SimpleAstronaut
 *  2024-12-2
 ***************************************************************************/


//引入各模块
//#include "main.h"
#include "rc.h"
#include "mes.h"
#include "pid.h"
#include "imu.h"
#include "mecanum.h"
#include "system.h"
#include "math.h"
#include "imu_data_analysis.h"

struct atti atti1;
struct atti atti2;
struct ctar ctar1;

struct chassis_info chassis;

PID_t pid1;
PID_t pid2;
PID_t pid3;
PID_t pid4;
PID_t pos;

//int mode = 0;						//模式检测
//int vtx = 0;						//target x
//int vty = 0;						//target y
//int vtw = 0;						//target w
//int v1, v2, v3, v4;			//定义四个电机转速
int dyaw;								//yaw差值
//int tyaw = 0;						//初始化target yaw
int ifMode3 = 0;				//判断是否为小陀螺
int ifEnable = 0;				//正方向是否开启
//int check = 0;					//断连保护标志位
int ifFirstChange = 0;	//是否为第一次

//float ctar = 0;					//已弃用 正方向target yaw

float tKp = 0.05f;			
float tKi = 0.00f;

float texInt = 0.0f;
float teyInt = 0.0f;
float tezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/***************************************************************** 
  * 函数功能: 系统初始化,包括rc imu mes模块 TODO:PID
  * 输入参数: UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, CAN_HandleTypeDef *hcan
  * 返 回 值: 无
	* 主函数中调用
  ****************************************************************/
void system_init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim)
{
		/*rc_init(huart);
		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
		imu_init();
		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);*/
		

		//CAN相关配置
		CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

		

		//串口使能
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer1, RX_BUFFER_SIZE);
		
		//PID初始化
		PID_Init(&pid1, 60, 19, 100, 20000.0, 20000, 5000);
		PID_Init(&pid2, 60, 19, 100, 20000.0, 20000, 5000);
		PID_Init(&pid3, 60, 19, 100, 20000.0, 20000, 5000);
		PID_Init(&pid4, 60, 19, 100, 20000.0, 20000, 5000);
		PID_Init(&pos, 60, 0, 100, 20000.0, 1800, 1800);
		//定时器使能
		HAL_TIM_Base_Start_IT(htim);
		//__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
		//imu_init();
		mpu_device_init();
		
		/* 底盘初始化 */
		chassis.vtx = 0;
		chassis.vty = 0;
		chassis.vtw = 0;
		chassis.spin_dir = 0;
		chassis.tyaw = 0;
		chassis.mode = 0;
		chassis.check = 0;
		for(int i = 0; i <= 5; i++)
		{
				chassis.rpm[i] = 0;
		}
}


/***************************************************************** 
  * 函数功能: 系统循环
  * 输入参数: TODO
  * 返 回 值: 无
	* 主函数while中调用
  *************** *************************************************/
void system_loop(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, CAN_HandleTypeDef *hcan)
{
		//mpu_get_data();
		if(rc.sw1 != 0 && rc.sw2 != 0)
		{
				if(rc.sw1 == 1)
				{
						chassis.check = 0;
						chassis.mode = 1;
						ctar1.ifEnable = 0;
						/*set_current(hcan, 0x200, 0.1*(PID_General_Cal(&pid1, 0, mes1.speed)), 
																			0.1*(PID_General_Cal(&pid1, 0, mes2.speed)), 
																			0.1*(PID_General_Cal(&pid1, 0, mes3.speed)), 
																			0.1*(PID_General_Cal(&pid1, 0, mes4.speed))
																																									);*/
						set_current(hcan, 0x200, 0, 0, 0, 0);
						chassis.tyaw = IMU.quaternion.yaw;
						ifMode3 = 0;
						//CAN_SendMessage(0x200, mes1.setCurrent, 0, 0, 0);
						//停止模式
				}
				if(rc.sw1 == 3)
				{	
						chassis.check = 0;
						ctar1.ifEnable = 0;
						chassis.mode = 2;
						
						chassis.vty = (rc.ch3 - 1024) * 2;
						chassis.vtx = (rc.ch2 - 1024) * 2;
						chassis.vtw = (rc.ch0 - 1024) * 2;

						int t = 0;				//方向系数
						ifEnable = 0;
						if((rc.ch0 - 1024) > 0)
						{
								chassis.spin_dir = 1;
								t = 1;
								ifEnable = 1;
						}
						if((rc.ch0 - 1024) < 0)
						{
								chassis.spin_dir = -1;
								t = -1;
								ifEnable = 1;
						}
						if((rc.ch0 - 1024) == 0)
						{
								chassis.spin_dir = 0;
								t = 1;
								//tyaw = IMU.quaternion.yaw;
								ifEnable = 0;
						}
						
						chassis.vtw = 10 * ( 0 - 0.1 * POSPID_General_Cal(&pos, chassis.tyaw, IMU.quaternion.yaw));

						dyaw = chassis.tyaw - IMU.quaternion.yaw;  

						//vtx = vtx * cos(dyaw) - vty * sin(dyaw);
						//vty = vtx * sin(dyaw) + vty * cos(dyaw);

						if(ifMode3 == 1)
						{
								//tyaw = IMU.quaternion.yaw;
						}

						/*
						if(rc.ch0 != 1024)
						{
								tyaw = IMU.quaternion.yaw;
						}
						else
						{
								vtw = 10 * ( 0 - 0.1 * POSPID_General_Cal(&pos, tyaw, IMU.quaternion.yaw));
						}*/

						int vtx1, vty1;

						vtx1 = chassis.vtx * cos(dyaw * 3.14/180) - chassis.vty * sin(dyaw * 3.14/180);
						vty1 = chassis.vtx * sin(dyaw * 3.14/180) + chassis.vty * cos(dyaw * 3.14/180);

						chassis.rpm[1] = vtx1 + vty1 + chassis.vtw;
						chassis.rpm[2] = vtx1 - vty1 + chassis.vtw;
						chassis.rpm[3] = -vtx1 + vty1 + chassis.vtw;
						chassis.rpm[4] = -vtx1 - vty1 + chassis.vtw;
						/*
						v1 = vtx1 + vty1 + vtw;
						v2 = +vtx1 - vty1 + vtw;
						v3 = -vtx1 + vty1 + vtw;
						v4 = -vtx1 - vty1 + vtw;
						*/
						
						set_current(hcan, 0x200, 0.1 * (PID_General_Cal(&pid1, 2*chassis.rpm[1], mes1.speed)), 
																			0.1 * (PID_General_Cal(&pid2, 2*chassis.rpm[2], mes2.speed)), 
																			0.1 * (PID_General_Cal(&pid3, 2*chassis.rpm[3], mes3.speed)), 
																			0.1 * (PID_General_Cal(&pid4, 2*chassis.rpm[4], mes4.speed))
																																									);
						//set_current(hcan, 0x200, 0.1*(PID_General_Cal(&pid1, v1, mes1.speed)), 0, 0, 0);
						//全向模式
						ifMode3 = 0;
				}
				if(rc.sw1 == 2)
				{
						//vtw = 1300;
						ctar1.ifEnable = 1;
						chassis.mode = 3;
						int ang = 0;
						chassis.check = 0;
						float vtx1, vty1;
						//vty = sin(rc.ch2 - 1024) + cos(rc.ch3 - 1024);
						//vtx = cos(rc.ch2 - 1024) - sin(rc.ch3 - 1024);
						chassis.vty = (rc.ch3 - 1024) * 1;
						chassis.vtx = (rc.ch2 - 1024) * 1;

						int t = 0;				//方向系数
						ifEnable = 0;
						if((rc.ch0 - 1024) > 0)
						{
								chassis.spin_dir = 1;
								t = 1;
								ifEnable = 1;
						}
						if((rc.ch0 - 1024) < 0)
						{
								chassis.spin_dir = -1;
								t = -1;
								ifEnable = 1;
						}
						if((rc.ch0 - 1024) == 0)
						{
								chassis.spin_dir = 0;
								t = 1;
								//tyaw = IMU.quaternion.yaw;
								ifEnable = 0;
						}
						
						if (ifFirstChange)
						{
								//tyaw = ctar;
						}
						//tyaw = ctar;
						
						dyaw = chassis.tyaw - IMU.quaternion.yaw; 
						//dyaw= -dyaw;

						vtx1 = chassis.vtx * cos(dyaw * 3.14/180) - chassis.vty * sin(dyaw * 3.14/180);
						vty1 = chassis.vtx * sin(dyaw * 3.14/180) + chassis.vty * cos(dyaw * 3.14/180);

						chassis.vtx = 2 * chassis.vtx;
						chassis.vty = 2 * chassis.vty;
	
						//vtx = vty * sin(dyaw * 3.14/180) + vtx * cos(dyaw * 3.14/180);
						//vty = vty * cos(dyaw * 3.14/180) - vtx * sin(dyaw * 3.14/180);

						chassis.vtw = 2000;
						//vtw = 0;
						chassis.rpm[1] = vtx1 + vty1 + chassis.vtw;
						chassis.rpm[2] = vtx1 - vty1 + chassis.vtw;
						chassis.rpm[3] = -vtx1 + vty1 + chassis.vtw;
						chassis.rpm[4] = -vtx1 - vty1 + chassis.vtw;
						set_current(hcan, 0x200, 0.1 * (PID_General_Cal(&pid1, chassis.rpm[1], mes1.speed)), 
																			0.1 * (PID_General_Cal(&pid2, chassis.rpm[2], mes2.speed)), 
																			0.1 * (PID_General_Cal(&pid3, chassis.rpm[3], mes3.speed)), 
																			0.1 * (PID_General_Cal(&pid4, chassis.rpm[4], mes4.speed))
																																									);
						ifMode3 = 1;
						//tyaw = IMU.quaternion.yaw;
						//小陀螺模式
				}
				else
				{
						chassis.check = 1;
				}
		}
}



/***************************************************************** 
  * 函数功能: AHRS互补滤波陀螺仪解算
  * 输入参数: TODO
  * 返 回 值: 无
	* 已弃用 舍不得删留着看看
  ****************************************************************/
void mahony_ahrs_updateIMU()
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
	float twoKi = 0.01f;
	float sampleFreq = 1000;    //采样频率

  float gx = mpu_data.gx;
  float gy = mpu_data.gy;
  float gz = mpu_data.gz;
  float ax = mpu_data.ax;
  float ay = mpu_data.ay;
  float az = mpu_data.az;
  float mx = mpu_data.mx;
  float my = mpu_data.my;
  float mz = mpu_data.mz;
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
    if (tKi > 0.0f)
    {/*
      exInt += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
      eyInt += twoKi * halfey * (1.0f / sampleFreq);
      ezInt += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;*/
    }
    else
    {
      texInt = 0.0f; // prevent integral windup
      teyInt = 0.0f;
      tezInt = 0.0f;
    }

    // Apply proportional feedback
    gx += tKp * halfex;
    gy += tKp * halfey;
    gz += tKp * halfez;
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
  atti1.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll     -pi----pi
  atti1.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // pitch    -pi/2----pi/2
  atti1.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw      -pi----pi
}



/***************************************************************** 
  * 函数功能: 牛顿迭代法快速求平方根
  * 输入参数: TODO
  * 返 回 值: 无
	* system_loop中调用
  ****************************************************************/
float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;          // get bits for floating value
    i =  0x5f375a86 - (i>>1);    // gives initial guess
    x = *(float*)&i;            // convert bits back to float
    x = x * (1.5f - xhalf*x*x); // Newton step
    return x;
}



/***************************************************************** 
  * 函数功能: 停止
  * 输入参数: TODO
  * 返 回 值: 无
	* 没用
  ****************************************************************/
void stop()
{
		//set_current(hcan, 0x200, 0, 0, 0, 0);
}

