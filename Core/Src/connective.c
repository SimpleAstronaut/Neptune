/****************************************************************************
 *  Neptune connective module
 *
 *  连接模块
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/


#include "main.h"
#include "rc.h"
#include "system.h"
#include "mes.h"


/***************************************************************** 
  * 函数功能: dma接收中断
  * 输入参数: UART_HandleTypeDef *huart, uint16_t Size
  * 返 回 值: 无
	* 双缓存
  ****************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

		//判断来自usart1 -> 遥控器接收数据
		if (huart->Instance == USART1)
		{
				stopt = 0;
				if(count1 == 0)
				{
						rc_receive(huart, Size);
						__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //清除DMA传输完成标注
						HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer2, RX_BUFFER_SIZE);
						system_loop(&huart1, &hspi5, &hcan1);
						count1++;
				}
				else
				{
						rc_receive(huart, Size);
						__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //清除DMA传输完成标注
						HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer1, RX_BUFFER_SIZE);
						system_loop(&huart1, &hspi5, &hcan1);
						count1 = 0;
				}
				//rc_receive(huart, Size);
				//__HAL_UART_CLEAR_IDLEFLAG(&huart1); //清空IDLE标志位
				//__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //清除DMA传输完成标注
				//HAL_UART_DMAStop(&huart1); //停止DMA接收
				//system_loop();
				//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer1, RX_BUFFER_SIZE);
				
		}
}


/***************************************************************** 
  * 函数功能: CAN接收中断
  * 输入参数: CAN_HandleTypeDef *hcan
  * 返 回 值: 无
	* mes_receive
  ****************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		mes_receive(hcan);
}
