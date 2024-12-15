/****************************************************************************
 *  Neptune timer module
 *
 *  ��ʱ��: SimpleAstronaut
 *  2024-12-13
 ***************************************************************************/


#include "main.h"
#include "system.h"
#include "mes.h"


int i = 0;
int p = 0;

//int count1 = 0;
//int stopt = 1;

/***************************************************************** 
  * ��������: ��ʱ���ж�
  * �������: TIM_HandleTypeDef
  * �� �� ֵ: ��
	* 1ms��ʱ���ж�
  ****************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

		//��ʱ���ж�1ms
		if(htim == (&htim2))
		{	
				if(ifEnable == 1)
				{
						i++;
						if(i == 10)
						{
								//10ms��ʱ��
								switch(chassis.spin_dir){
										case 1  :
													//˳ʱ��
													chassis.tyaw--;
													break; 
										case -1  :
													chassis.tyaw++;
													//��ʱ��
													break; 
								}
								/*chassis.tyaw++;
								if(chassis.tyaw > 180)
								{
										chassis.tyaw = -chassis.tyaw;
								}*/
								i = 0;
						}
				}
				if(stopt)
				{
						set_current(&hcan1, 0x200, 0, 0, 0, 0);
				}
				p++;
				if(p >= 150)
				{
						//����ֹͣ����
						p = 0;
						stopt = 1;
						
				}
				//mpu_get_data();
				//mahony_ahrs_updateIMU();
		}
}
