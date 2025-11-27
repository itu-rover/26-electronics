#include "main.h"
#include "stm32g4xx_hal.h"
#include "stdint.h"

void Int2HexArray(uint8_t TxData[8], int32_t rpm)
{
	int8_t i = 0;
	for(i = 3; i >= 0; i--){
		TxData[i] = 0xFF & rpm;
		rpm = rpm >> 8;
	}
}

void Int2HexArray_Pos(uint8_t TxData[8], int32_t pos)
{
	for (int i = 0; i < 4; i++) {
	    TxData[3 - i] = (pos >> (8 * i)) & 0xFF;
	}
}

void VESC_RUN_RPM(int32_t rpm, uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan)
{
	uint8_t TxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	Int2HexArray(TxData, rpm);
	TxHeader->Identifier = 0x300 | can_id;
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, TxHeader, TxData) != HAL_OK){
		//Error_Handler();
	}
}

void VESC_RUN_POS(int32_t pos, uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan)
{
	pos = pos * 1000000;
	uint8_t TxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	Int2HexArray_Pos(TxData, pos);
	TxHeader->Identifier = 0x400 | can_id;
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, TxHeader, TxData) != HAL_OK){
		Error_Handler();
	}
}


void VESC_RUN_HANDBRAKE(uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan)
{
	uint8_t TxData[8] = {0x00, 0x00, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00};
	TxHeader->Identifier = 0x200 | can_id;
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, TxHeader, TxData) != HAL_OK){
		Error_Handler();
	}
}


void VESC_CAN_CB(uint32_t WHs[4], FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t 			  RxData[8];

	if((RxFifo1ITs && FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET){
			if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK){
				Error_Handler();
			}
			if ((RxHeader.Identifier & 0xE00) == 0xE00){
				switch(RxHeader.Identifier & 0x0FF)
				{
				case 0x1E:
					WHs[0] = ((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | ((uint32_t)RxData[3]);
					break;
				case 0x32:
					WHs[1] = ((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | ((uint32_t)RxData[3]);
					break;
				case 0x33:
					WHs[2] = ((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | ((uint32_t)RxData[3]);
					break;
				case 0x77:
					WHs[3] = ((uint32_t)RxData[0] << 24) | ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | ((uint32_t)RxData[3]);
					break;
				}
				WHs[3] += 1;
			}
			if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
				Error_Handler();
			}
		}
}
