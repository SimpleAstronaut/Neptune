/****************************************************************************
 *  Neptune connective module
 *
 *  ����ģ��
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/


#include "main.h"
#include "rc.h"
#include "system.h"
#include "mes.h"


/***************************************************************** 
  * ��������: dma�����ж�
  * �������: UART_HandleTypeDef *huart, uint16_t Size
  * �� �� ֵ: ��
	* ˫����
  ****************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

		//�ж�����usart1 -> ң������������
		if (huart->Instance == USART1)
		{
				stopt = 0;
				if(count1 == 0)
				{
						rc_receive(huart, Size);
						__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //���DMA������ɱ�ע
						HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer2, RX_BUFFER_SIZE);
						system_loop(&huart1, &hspi5, &hcan1);
						count1++;
				}
				else
				{
						rc_receive(huart, Size);
						__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //���DMA������ɱ�ע
						HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer1, RX_BUFFER_SIZE);
						system_loop(&huart1, &hspi5, &hcan1);
						count1 = 0;
				}
				//rc_receive(huart, Size);
				//__HAL_UART_CLEAR_IDLEFLAG(&huart1); //���IDLE��־λ
				//__HAL_DMA_CLEAR_FLAG(&huart1,DMA_FLAG_TCIF2_6); //���DMA������ɱ�ע
				//HAL_UART_DMAStop(&huart1); //ֹͣDMA����
				//system_loop();
				//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer1, RX_BUFFER_SIZE);
				
		}
}


/***************************************************************** 
  * ��������: CAN�����ж�
  * �������: CAN_HandleTypeDef *hcan
  * �� �� ֵ: ��
	* mes_receive
  ****************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		mes_receive(hcan);
}
