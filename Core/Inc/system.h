#include "main.h"
void system_init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim);
void system_loop(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, CAN_HandleTypeDef *hcan);
void mahony_ahrs_updateIMU();
void stop(void);
float invSqrt(float x);
extern PID_t pid1;
extern PID_t pid2;
extern PID_t pid3;
extern PID_t pid4;
extern PID_t pos;

struct atti
{
		float roll;
		float pitch;
		float yaw;
};

struct ctar
{
		int ifEnable;
		float tar;
};

struct chassis_info
{
		float vtx;
		float vty;
		float vtw;
		
		int spin_dir;	 //��ת���� 1˳ʱ�� -1��ʱ��
		float tyaw;

		int mode;
		int check;					//����������־λ
		int stop;						//������־λ

		float rpm[5];				//��ת�� rpm[0]��ʾ�Ƿ���
};

extern struct atti atti2;
extern struct ctar ctar1;
extern struct chassis_info chassis;
extern int ifEnable;
extern float ctar;
//extern int tyaw;
//extern int check;
