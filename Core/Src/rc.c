/****************************************************************************
 *  Neptune rc module
 *
 *  使用DT7和DR16 2.4GHz,其中DR16内置于大疆A板,位于USART1_RX(PB7)
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "main.h"
#include "rc.h"



int count = 0;                				//定义双缓存的计数器
uint8_t rxBuffer1[RX_BUFFER_SIZE];    //定义双缓存
uint8_t rxBuffer2[RX_BUFFER_SIZE];	

struct rc_info rc;
		

/***************************************************************** 
  * 函数功能: rc接收初始化
  * 输入参数: UART_HandleTypeDef *huart
  * 返 回 值: 无
  ****************************************************************/
void rc_init(UART_HandleTypeDef *huart)
{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rxBuffer1, RX_BUFFER_SIZE);
}


/***************************************************************** 
  * 函数功能: rc接收函数
  * 输入参数: 与dma中断服务函数相同
  * 返 回 值: 无
	* 在dma接收中断函数中调用
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
  * 函数功能: 处理DR16数据
  * 输入参数: TODO
  * 返 回 值: 无
	* 在rc_receive函数中被调用
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
