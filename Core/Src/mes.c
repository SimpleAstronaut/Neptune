/****************************************************************************
 *  Neptune rc module
 *
 *  ����͵������,��д����
 *  Author: SimpleAstronaut
 *  2024-12-1
 ***************************************************************************/

#include "main.h"
#include "mes.h"


CAN_RxHeaderTypeDef RxHeader; // ���յ���Ϣͷ
uint8_t RxData[8];

struct mes mes1;					 //����mesʵ�建��
struct mes mes2;					 //����mesʵ�建��
struct mes mes3;					 //����mesʵ�建��
struct mes mes4;					 //����mesʵ�建��


/*
void chassis_init(void)
{
		
}*/

void set_current(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // ��������֡
    TxData[0] = (current1 >> 8) & 0xFF;
    TxData[1] = current1 & 0xFF;
    TxData[2] = (current2 >> 8) & 0xFF;
    TxData[3] = current2 & 0xFF;
    TxData[4] = (current3 >> 8) & 0xFF;
    TxData[5] = current3 & 0xFF;
    TxData[6] = (current4 >> 8) & 0xFF;
    TxData[7] = current4 & 0xFF;

    // ����CAN֡ͷ
    TxHeader.StdId = StdId; // ��׼ID
    TxHeader.ExtId = 0; // ��չID
    TxHeader.RTR = CAN_RTR_DATA; // ����֡
    TxHeader.IDE = CAN_ID_STD; // ��׼ID
    TxHeader.DLC = 8; // ���ݳ���Ϊ8�ֽ�
    TxHeader.TransmitGlobalTime = DISABLE; // ����ȫ��ʱ���

    // ����CAN��Ϣ
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 1) {
    } // �ȴ��п��õķ�������
    if(HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        // ������
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
                // ����δ֪ID
                break;
        }
    }
    else
    {
        // ������
        Error_Handler();
    }		
}
