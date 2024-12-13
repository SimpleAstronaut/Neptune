
#include "stdint.h"
#include "main.h"

//电机数据结构体
struct mes
{
	  uint8_t speedh8;       //转速h8
		uint8_t speedl8;       //转速l8
		int16_t speed;         //转速
		int16_t current;       //电流值
		int16_t tem;           //电机温度
		int16_t eng;           //角度
		int16_t setCurrent;    //设置电机电流值
};

void mes_receive(CAN_HandleTypeDef *hcan);
void set_current(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t current1, int16_t current2, int16_t current3, int16_t current4);

extern struct mes mes1;					 //定义mes实体缓存
extern struct mes mes2;					 //定义mes实体缓存
extern struct mes mes3;					 //定义mes实体缓存
extern struct mes mes4;					 //定义mes实体缓存

