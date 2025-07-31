/*
 * MICS-4514.h
 *
 *  Created on: May 10, 2025
 *      Author: ALPEREN
 */

#ifndef INC_MICS_4514_H_
#define INC_MICS_4514_H_



#endif /* INC_MICS_4514_H_ */


#include "stm32f3xx.h"

#define NO2_R0 					735
#define MICS_4514_ADDRESS	    0x75<<1
#define OX_REGISTER_HIGH    	0x04
#define OX_REGISTER_LOW     	0x05
#define WAKE_UP_MODE			0x01
#define POWER_MODE_REGISTER     0x0A

float Read_NO2 ();
