#ifndef INC_VESC_CAN_COM_H
#define INC_VESC_CAN_COM_H


#include "main.h"
#include "stm32g4xx_hal.h"
#include "stdint.h"

void VESC_RUN_RPM(int32_t rpm, uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan);
void VESC_RUN_HANDBRAKE(uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan);
void Int2HexArray(uint8_t TxData[8], int32_t rpm);
void VESC_CAN_CB(uint32_t WHs[4], FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs);
void Int2HexArray_Pos(uint8_t TxData[8], int32_t pos);
void VESC_RUN_POS(int32_t pos, uint8_t can_id, FDCAN_TxHeaderTypeDef* TxHeader, FDCAN_HandleTypeDef* hfdcan);

#endif
