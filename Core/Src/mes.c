/****************************************************************************
 *  Neptune rc module
 *
 *  电调和电机不详,先写着玩
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "main.h"
#include "mes.h"


CAN_RxHeaderTypeDef RxHeader; // 接收的消息头
uint8_t RxData[8];

struct mes mes1;					 //定义mes实体缓存
struct mes mes2;					 //定义mes实体缓存
struct mes mes3;					 //定义mes实体缓存
struct mes mes4;					 //定义mes实体缓存


/*
void chassis_init(void)
{
		
}*/

void set_current(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // 构造数据帧
    TxData[0] = (current1 >> 8) & 0xFF;
    TxData[1] = current1 & 0xFF;
    TxData[2] = (current2 >> 8) & 0xFF;
    TxData[3] = current2 & 0xFF;
    TxData[4] = (current3 >> 8) & 0xFF;
    TxData[5] = current3 & 0xFF;
    TxData[6] = (current4 >> 8) & 0xFF;
    TxData[7] = current4 & 0xFF;

    // 设置CAN帧头
    TxHeader.StdId = StdId; // 标准ID
    TxHeader.ExtId = 0; // 扩展ID
    TxHeader.RTR = CAN_RTR_DATA; // 数据帧
    TxHeader.IDE = CAN_ID_STD; // 标准ID
    TxHeader.DLC = 8; // 数据长度为8字节
    TxHeader.TransmitGlobalTime = DISABLE; // 禁用全局时间戳

    // 发送CAN消息
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 1) {
    } // 等待有可用的发送邮箱
    if(HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        // 错误处理
    }
}


void mes_receive(CAN_HandleTypeDef *hcan)
{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
				switch (RxHeader.StdId) {
            case 0x201: // 1
                mes1.speed = (uint16_t)(RxData[2] << 8) | RxData[3];
                break;
            case 0x202: // 2
                mes2.speed = (uint16_t)(RxData[2] << 8) | RxData[3];
                break;
            case 0x203: // 3
                mes3.speed = (uint16_t)(RxData[2] << 8) | RxData[3];
                break;
            case 0x204: // 4
                mes4.speed = (uint16_t)(RxData[2] << 8) | RxData[3];
                break;
            default:
                // 处理未知ID
                break;
        }
    }
    else
    {
        // 错误处理
        Error_Handler();
    }		
}
