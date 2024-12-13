/****************************************************************************
 *  Neptune rc module
 *
 *  ʹ��DT7��DR16 2.4GHz,����DR16�����ڴ�A��,λ��USART1_RX(PB7)
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "main.h"
#include "rc.h"



int count = 0;                				//����˫����ļ�����
uint8_t rxBuffer1[RX_BUFFER_SIZE];    //����˫����
uint8_t rxBuffer2[RX_BUFFER_SIZE];	

struct rc_info rc;
		

/***************************************************************** 
  * ��������: rc���ճ�ʼ��
  * �������: UART_HandleTypeDef *huart
  * �� �� ֵ: ��
  ****************************************************************/
void rc_init(UART_HandleTypeDef *huart)
{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer1, RX_BUFFER_SIZE);
}


/***************************************************************** 
  * ��������: rc���պ���
  * �������: ��dma�жϷ�������ͬ
  * �� �� ֵ: ��
	* ��dma�����жϺ����е���
  ****************************************************************/
void rc_receive(UART_HandleTypeDef *huart, uint16_t Size)
{
		get_dr16_data(rxBuffer1);
		/*
		if(count == 0)
		{
				get_dr16_data(rxBuffer1);
				count++;
				HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer2, RX_BUFFER_SIZE);
		}
		else
		{
				get_dr16_data(rxBuffer2);
				count = 0;
				HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer1, RX_BUFFER_SIZE);
		}*/
}


/***************************************************************** 
  * ��������: ����DR16����
  * �������: TODO
  * �� �� ֵ: ��
	* ��rc_receive�����б�����
  ****************************************************************/
void get_dr16_data(uint8_t *buff)
{
  rc.ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
  //rc.ch1 -= 1024;
  rc.ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  //rc.ch2 -= 1024;
  rc.ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  //rc.ch3 -= 1024;
  rc.ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  //rc.ch4 -= 1024;
  rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc.sw2 = (buff[5] >> 4) & 0x0003;

  if(rc.ch0 <= 5 && rc.ch0 >= -5)
    rc.ch0 = 0;
  if(rc.ch1 <= 5 && rc.ch1 >= -5)
    rc.ch1 = 0;
  if(rc.ch2 <= 5 && rc.ch2 >= -5)
    rc.ch2 = 0;
  if(rc.ch3 <= 10 && rc.ch3 >= -10)
    rc.ch3 = 0;

}
