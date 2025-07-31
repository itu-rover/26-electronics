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
#define MQ135_RL				1		//Load Resistor (?)
#define MQ135_R0				4.5
#define MQ135_ADC_VREF			5

float Read_NH3 ();

#endif /* INC_MQ135_H_ */



