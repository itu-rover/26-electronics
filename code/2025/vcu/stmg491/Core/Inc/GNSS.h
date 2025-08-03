/*
 * GNSS.h
 *
 *  Created on: Apr 9, 2025
 *      Author: Turk
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#define UBX_FRAME_HEADER_SIZE   6
#define UBX_PAYLOAD_LON_OFFSET  24
#define UBX_PAYLOAD_LAT_OFFSET  28
#define UBX_PAYLOAD_SOG_OFFSET  60
#define UBX_PAYLOAD_COG_OFFSET  64

typedef struct
{
	int32_t lon;
	int32_t lat;
	int32_t sog;
	int32_t cog;

}GNSS_Data_t;

uint8_t	GNSS_PVTmsgSync(uint8_t *gnss_rxBuff);
void	GNSS_PVTmsgParse(uint8_t* gnss_rxBuff, uint8_t* gnss_parsedRxData);
void 	GNSS_PVTmsgDisplay(uint8_t* gnss_rxBuff, GNSS_Data_t* GNSS_Data);

#endif /* INC_GNSS_H_ */
