#include "stm32f4xx_hal.h"

#define RX_BUFFER_SIZE 20  						//定义缓存长度

void rc_init(UART_HandleTypeDef *huart);
void rc_receive(UART_HandleTypeDef *huart, uint16_t Size);
void get_dr16_data(uint8_t *buff);

struct rc_info
{
  /* 左右摇杆信息 */
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  /* s1 s2开关信息 */
  uint8_t sw1;
  uint8_t sw2;
};											//定义rc信息缓存

extern uint8_t rxBuffer1[RX_BUFFER_SIZE];    //定义双缓存
extern uint8_t rxBuffer2[RX_BUFFER_SIZE];

extern struct rc_info rc; 	

