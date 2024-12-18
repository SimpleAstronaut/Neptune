/****************************************************************************
 *  Neptune timer module
 *
 *  定时器: SimpleAstronaut
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
  * 函数功能: 定时器中断
  * 输入参数: TIM_HandleTypeDef
  * 返 回 值: 无
	* 1ms定时器中断
  ****************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

		//定时器中断1ms
		if(htim == (&htim2))
		{	
				if(ifEnable == 1)
				{
						i++;
						if(i == 10)
						{
								//10ms定时器
								switch(chassis.spin_dir){
										case 1  :
													//顺时针
													chassis.tyaw--;
													break; 
										case -1  :
													chassis.tyaw++;
													//逆时针
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
						//心跳停止保护
						p = 0;
						stopt = 1;
						
				}
				//mpu_get_data();
				//mahony_ahrs_updateIMU();
		}
}
