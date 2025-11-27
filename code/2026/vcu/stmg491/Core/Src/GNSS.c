/*
 * GNSS.c
 *
 *  Created on: Apr 9, 2025
 *      Author: Turk
 *
 */
#include"main.h"
#include"GNSS.h"


extern UART_HandleTypeDef huart3;

uint8_t GNSS_PVTmsgSync(uint8_t *gnss_rxBuff)
{
	static uint8_t sync_status = 0;
	if (sync_status == 0) {
		if (gnss_rxBuff[0] == 181) {
			sync_status = 1;
			HAL_UART_Receive_DMA(&huart3, gnss_rxBuff + 1, 1);
			return 0;
		}
		HAL_UART_Receive_DMA(&huart3, gnss_rxBuff, 1);
	} else if (sync_status == 1) {
		if (gnss_rxBuff[1] == 98) {
			sync_status = 2;
			HAL_UART_Receive_DMA(&huart3, gnss_rxBuff + 2, 98);
			return 0;
		}
		sync_status = 0;
		HAL_UART_Receive_DMA(&huart3, gnss_rxBuff, 1);
	} else {
		if (gnss_rxBuff[0] == 181 && gnss_rxBuff[1] == 98)
			return 1;
		sync_status = 0;
		HAL_UART_Receive_DMA(&huart3, gnss_rxBuff, 1);
	}
	return 0;
}

void GNSS_PVTmsgParse(uint8_t* gnss_rxBuff, uint8_t* gnss_parsedRxData)
{
	uint8_t offSet;

	// Latitude
	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_LAT_OFFSET;
	gnss_parsedRxData[0] = gnss_rxBuff[offSet];
	gnss_parsedRxData[1] = gnss_rxBuff[offSet + 1];
	gnss_parsedRxData[2] = gnss_rxBuff[offSet + 2];
	gnss_parsedRxData[3] = gnss_rxBuff[offSet + 3];

	// Longitude
	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_LON_OFFSET;
	gnss_parsedRxData[4] = gnss_rxBuff[offSet];
	gnss_parsedRxData[5] = gnss_rxBuff[offSet + 1];
	gnss_parsedRxData[6] = gnss_rxBuff[offSet + 2];
	gnss_parsedRxData[7] = gnss_rxBuff[offSet + 3];

	// COG
	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_COG_OFFSET;
	gnss_parsedRxData[8] = gnss_rxBuff[offSet];
	gnss_parsedRxData[9] = gnss_rxBuff[offSet+1];
	gnss_parsedRxData[10] = gnss_rxBuff[offSet+2];
	gnss_parsedRxData[11] = gnss_rxBuff[offSet+3];

	// SOG
	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_SOG_OFFSET;
	gnss_parsedRxData[12] = gnss_rxBuff[offSet];
	gnss_parsedRxData[13] = gnss_rxBuff[offSet + 1];
	gnss_parsedRxData[14] = gnss_rxBuff[offSet + 2];
	gnss_parsedRxData[15] = gnss_rxBuff[offSet + 3];


}


void GNSS_PVTmsgDisplay(uint8_t* gnss_rxBuff, GNSS_Data_t* GNSS_Data)
{
	uint8_t offSet;
	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_LON_OFFSET;
	GNSS_Data->lon = (int8_t)gnss_rxBuff[offSet] + ((int16_t)gnss_rxBuff[offSet+1]<<8) + ((int32_t)gnss_rxBuff[offSet+2]<<16) + ((int32_t)gnss_rxBuff[offSet+3]<<24);

	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_LAT_OFFSET;
	GNSS_Data->lat = (int8_t)gnss_rxBuff[offSet] + ((int16_t)gnss_rxBuff[offSet+1]<<8) + ((int32_t)gnss_rxBuff[offSet+2]<<16) + ((int32_t)gnss_rxBuff[offSet+3]<<24);

	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_SOG_OFFSET;
	GNSS_Data->sog = (int8_t)gnss_rxBuff[offSet] + ((int16_t)gnss_rxBuff[offSet+1]<<8) + ((int32_t)gnss_rxBuff[offSet+2]<<16) + ((int32_t)gnss_rxBuff[offSet+3]<<24);

	offSet = UBX_FRAME_HEADER_SIZE + UBX_PAYLOAD_COG_OFFSET;
	GNSS_Data->cog = (int8_t)gnss_rxBuff[offSet] + ((int16_t)gnss_rxBuff[offSet+1]<<8) + ((int32_t)gnss_rxBuff[offSet+2]<<16) + ((int32_t)gnss_rxBuff[offSet+3]<<24);
}
