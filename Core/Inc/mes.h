
#include "stdint.h"
#include "main.h"

//������ݽṹ��
struct mes
{
	  uint8_t speedh8;       //ת��h8
		uint8_t speedl8;       //ת��l8
		int16_t speed;         //ת��
		int16_t current;       //����ֵ
		int16_t tem;           //����¶�
		int16_t eng;           //�Ƕ�
		int16_t setCurrent;    //���õ������ֵ
};

void mes_receive(CAN_HandleTypeDef *hcan);
void set_current(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

extern struct mes mes1;					 //����mesʵ�建��
extern struct mes mes2;					 //����mesʵ�建��
extern struct mes mes3;					 //����mesʵ�建��
extern struct mes mes4;					 //����mesʵ�建��

