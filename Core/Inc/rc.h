#include "stm32f4xx_hal.h"

#define RX_BUFFER_SIZE 20  						//���建�泤��

void rc_init(UART_HandleTypeDef *huart);
void rc_receive(UART_HandleTypeDef *huart, uint16_t Size);
void get_dr16_data(uint8_t *buff);

struct rc_info
{
  /* ����ҡ����Ϣ */
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  /* s1 s2������Ϣ */
  uint8_t sw1;
  uint8_t sw2;
};											//����rc��Ϣ����

extern uint8_t rxBuffer1[RX_BUFFER_SIZE];    //����˫����
extern uint8_t rxBuffer2[RX_BUFFER_SIZE];

extern struct rc_info rc; 	

