/*
 * soil-sensor.h
 *
 *  Created on: June 10, 2026
 *      Author: BERKEKELES
 */

#ifndef INC_soil_sensor_H_


#include "stm32f3xx.h"

//01 03 00 12 00 04 E4 0C

extern const uint8_t Soil_Basic_TxBuffer[]; // make it const

extern float Moisture;
extern float Temp;
extern uint32_t Cond;
extern float pH;


// 0x01 0x03 0x00 0x12 0x00 0x09 25 C9  | 01 03 DC 07 00 01 


// 01 03 08 00 00 01 0e 00 00 00 00 fd c7


// 01 03 08 02 73 01 09 27 10 27 10 9a 54  (in salty water)

// 01 03 08 | 02 73 | 01 09 | 27 10 | 27 10 |


//01 03 00 1E 00 03 65 CD

//extern uint8_t NPK_TxBuffer[8] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x03, 0x65, 0xcd}; // doesn't work , make it const

//

#define INC_soil_sensor_H_
#define Soil_Basic_TxBuffer_size sizeof(Soil_Basic_TxBuffer)
#define NPK_TxBuffer_size sizeof(NPK_TxBuffer)

#define Soil_Basic_RxBuffer_size 13
#define NPK_RxBuffer_size 11

uint8_t createRawSoilReading();


#endif /* INC_soil_sensor_H_ */



