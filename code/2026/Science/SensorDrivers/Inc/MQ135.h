/*
 * MQ135.h
 *
 *  Created on: May 10, 2025
 *      Author: ALPEREN
 */

#ifndef INC_MQ135_H_
#define INC_MQ135_H_

#include "stm32f3xx.h"

#define MQ135_ADC_MAX_VALUE		4095.0
#define MQ135_RL				10000.0f		//Load Resistor (?)
#define MQ135_R0				9685.6f
#define MQ135_ADC_VREF			3.3f

float Read_NH3 ();

#endif /* INC_MQ135_H_ */



