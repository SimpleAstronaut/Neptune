#include "math.h"

#include "imu_data_analysis.h"
#include "ist8310_reg.h" 

#include "main.h"
#include "mpu6500_reg.h"
#include "spi.h"
#include "quaternion.h"
#include "string.h"

static uint8_t        		tx, rx;
uint8_t               		ist_buff[6];                         
IMU_T                 		IMU={0};
uint8_t 									id = 0;
float                 		Init_G = 0.0;
long 											a_offset_total[3];
long 											m_offset_total[3];
mpu_data_t            		mpu_data;

/**
  * @brief  将一个字节的数据写入指定的寄存器
  * @param  reg:  拟填写的注册地址
  *         data: 要写入的数据
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
		rx = SPI5_ReadWriteByte(tx);
	  tx = data;
		rx = SPI5_ReadWriteByte(tx);
	  MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  从指定的寄存器中读取一个字节的数据
  * @param  reg: 要读取的寄存器地址
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
		rx = SPI5_ReadWriteByte(tx);
		rx = SPI5_ReadWriteByte(tx);
  	MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  从指定寄存器读取数据字节
  * @param  reg: 数据写入地址
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t  regAddr, uint8_t* pData, uint8_t len)
{
		  int i;
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
		rx = SPI5_ReadWriteByte(tx);
	  for(i = 0; i < len; i++)
	  {
      tx = regAddr | 0x80;		
			*pData = SPI5_ReadWriteByte(tx);	
			pData++;
			regAddr++;
	  }
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  设置imu 6500陀螺仪测量范围
  * @param  fsr: range(0,?50dps;1,?00dps;2,?000dps;3,?000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  设置imu 6050/6500加速测量范围
  * @param  fsr: range(0,?g;1,?g;2,?g;3,?6g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

/**
	* @brief  获取MPU6500的偏移数据
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i = 0;	
	int offset_tmp[6] = {0};
	uint8_t mpu_buff_offset[14] = {0};
		
	HAL_Delay(20);
	for (i=0; i<100;i++)
	{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff_offset, 14);
		
		offset_tmp[0] += (int16_t)(mpu_buff_offset[0] << 8 | mpu_buff_offset[1]);
		offset_tmp[1] += (int16_t)(mpu_buff_offset[2] << 8 | mpu_buff_offset[3]);
		offset_tmp[2] += (int16_t)(mpu_buff_offset[4] << 8 | mpu_buff_offset[5]);
		offset_tmp[3] += (int16_t)(mpu_buff_offset[8] << 8 | mpu_buff_offset[9]);
		offset_tmp[4] += (int16_t)(mpu_buff_offset[10] << 8 | mpu_buff_offset[11]);
		offset_tmp[5] += (int16_t)(mpu_buff_offset[12] << 8 | mpu_buff_offset[13]);
		
		HAL_Delay(5);
	}
	
	mpu_data.ax_offset = (int16_t)(offset_tmp[0] / 100);
	mpu_data.ay_offset = (int16_t)(offset_tmp[1] / 100);
	mpu_data.az_offset = (int16_t)(offset_tmp[2] / 100);
	mpu_data.gx_offset = (int16_t)(offset_tmp[3] / 100);
	mpu_data.gy_offset = (int16_t)(offset_tmp[4] / 100);
	mpu_data.gz_offset = (int16_t)(offset_tmp[5] / 100);
	
	IMU.norm_g = sqrt(mpu_data.ax_offset*mpu_data.ax_offset+mpu_data.ay_offset*mpu_data.ay_offset+mpu_data.az_offset*mpu_data.az_offset);
	IMU.Gravity = IMU.norm_g / 4096.0f;
	
	if(IMU.Gravity > 1.13f || IMU.Gravity < 0.87f)
	NVIC_SystemReset();
	
	HAL_Delay(20);

}

/**
	* @brief  初始化imu mpu6500和磁强计ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_device_init(void)
{
	HAL_Delay(100);
	float quaternion_Init[2] 				 = {0};
	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_SIGNAL_PATH_RESET, 0x07	},   /*陀螺仪、加速度计、温度计复位*/
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */
	HAL_Delay(50);
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(10);
	}


//	ist8310_init();
//	HAL_Delay(10);	
	
//	while(Test_Temp());	
	
	mpu_offset_call();
	HAL_Delay(5);	

	quaternion_Init[0] = asin(mpu_data.ax_offset / IMU.norm_g);
	quaternion_Init[1] = asin(mpu_data.ay_offset / IMU.norm_g);
	quaternion_init(quaternion_Init[0], quaternion_Init[1]);
 
}

/**
	* @brief  获取imu数据
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
		uint8_t mpu_buff[14] = {0};
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0]  << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2]  << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4]  << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6]  << 8 | mpu_buff[7];
    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
		
//    ist8310_get_data(ist_buff);
//    memcpy(&mpu_data.mx, ist_buff, 6);
		
    IMU.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
		
		IMU.acc.x = mpu_data.ax / 4096.0f * 0.29;
		if(IMU.acc.x>IMU.Gravity)  IMU.acc.x=IMU.Gravity;
		if(IMU.acc.x<-IMU.Gravity)  IMU.acc.x=-IMU.Gravity;		
		IMU.acc.y = mpu_data.ay / 4096.0f * 0.29;
		if(IMU.acc.y>IMU.Gravity)  IMU.acc.y=IMU.Gravity;
		if(IMU.acc.y<-IMU.Gravity)  IMU.acc.y=-IMU.Gravity;				
		IMU.acc.z = mpu_data.az / 4096.0f * 0.29;
		if(IMU.acc.z>IMU.Gravity)  IMU.acc.z=IMU.Gravity;
		if(IMU.acc.z<-IMU.Gravity)  IMU.acc.z=-IMU.Gravity;		
		
		IMU.acc.x = Filter_one(IMU.acc.x,0.1f);
		IMU.acc.y = Filter_one(IMU.acc.y,0.1f);
		IMU.acc.z = Filter_one(IMU.acc.z,0.1f);		
		
		IMU.gyro.x = mpu_data.gx / 16.384f * 0.29;
//		if(IMU.gyro.x>GYRO_MAX)		IMU.gyro.x = GYRO_MAX;
		IMU.gyro.y = mpu_data.gy / 16.384f * 0.29;		
//		if(IMU.gyro.y>GYRO_MAX)		IMU.gyro.y = GYRO_MAX;		
		IMU.gyro.z = mpu_data.gz / 16.384f * 0.29;
//		if(IMU.gyro.z>GYRO_MAX)		IMU.gyro.z = GYRO_MAX;
		
		IMU.gyro.x = Filter_one(IMU.gyro.x,0.1f);
		IMU.gyro.y = Filter_one(IMU.gyro.y,0.1f);
		IMU.gyro.z = Filter_one(IMU.gyro.z,0.1f);	
		if(abs(IMU.gyro.z) <= 0.1)
		{
				IMU.gyro.z = 0;
		}

		imu_attitude_update();

}

void imu_attitude_update(void)	//	IMU-姿态-更新
{
	float dt = 0.001;
	IMU.eular_acc.pitch = asin(mpu_data.ay_offset/IMU.norm_g)*57.29578f;	
	
	IMU.Yaw_Ingetral = IMU.Yaw_Ingetral + IMU.gyro.z*dt;	
	IMU.Pit_Ingetral = IMU.Pit_Ingetral + IMU.gyro.x*dt;
	IMU.Rol_Ingetral = IMU.Rol_Ingetral + IMU.gyro.y*dt;

	if(IMU.gyro.z == 0)
	{
			//IMU.Yaw_Ingetral = 0;
	}
	
	imuUpdate(IMU.acc, IMU.gyro, 0.001);	
}

float Filter_one(float data,float Kp)  //一节滤波
{
	static float last_data = 0;
	static uint8_t first_flag = 1;
	float res = 0;
	
	if(first_flag)
	{
		first_flag = 0;
		res = data;
	}
	else
	{
		res = (1.0f - Kp)*data + Kp*last_data;
	}
	last_data = res;
	
	return res;
}

uint8_t SPI5_ReadWriteByte(uint8_t Txdata)
{
	uint8_t Rxdata;
	//SPI_HandleTypeDef hspi5 = getspi();
	HAL_SPI_TransmitReceive(&hspi5 , &Txdata , &Rxdata , 1 , 1000);
	return Rxdata;
}


