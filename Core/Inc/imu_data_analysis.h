#ifndef  	__IMU_DATA_ANALYSIS_H__
#define 	__IMU_DATA_ANALYSIS_H__

#include "main.h"

#define IST8310
#define MPU_NSS_LOW     	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET)
#define MPU_NSS_HIGH      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET)
#define abs(x) 						((x)>0? (x):(-(x)))
#define GYRO_MAX					1000.0f

typedef struct
{
	float x;					//��x��
	float y;					//��y��
	float z;					//��z��
}Axis3f;						//����������ٶ�

typedef struct
{
	int16_t ax;						//	���ٶȣ�acc����x				
	int16_t ay;						//	���ٶȣ�acc����y	4			
	int16_t az;						//	���ٶȣ�acc����z				

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;						//�����ǣ�gyro����x
	int16_t gy;						//�����ǣ�gyro����y
	int16_t gz;						//�����ǣ�gyro����z
	
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

typedef struct
{
	struct
	{
		float pitch;
		float yaw;
		float roll;
	}quaternion;				//��Ԫ��
	
		struct
	{
		float pitch;
		float yaw;
		float roll;
	}eular_acc;					//ŷ���ǵļ��ٶ�
	
	struct
	{
	  int16_t x;
	  int16_t y;
	  int16_t z;
	}m;	
	
  Axis3f gyro;					//������
  Axis3f acc;						//

	float Yaw_Ingetral;
	float Pit_Ingetral;
	float Rol_Ingetral;		
	float temp;
	
	float norm_g;
	float Gravity;				//����
	
}IMU_T;



#ifdef __cplusplus
 extern "C" {
#endif
	 
void mpu_get_data(void);
void  mpu_device_init(void);
void mpu_offset_call(void);
void imu_attitude_update(void);
float Filter_one(float data,float Kp);  //һ���˲�
uint8_t SPI5_ReadWriteByte(uint8_t Txdata);
extern IMU_T  IMU; 
extern mpu_data_t            		mpu_data;
#ifdef __cplusplus
 }
#endif
#endif

