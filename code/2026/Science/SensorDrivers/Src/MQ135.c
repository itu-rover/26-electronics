/*
 * Mq135.c
 *
 *  Created on: May 10, 2025
 *      Author: ALPEREN
 */

#include "MQ135.h"
#include <math.h>

extern ADC_HandleTypeDef hadc3;

float Read_NH3 (){

	uint16_t ADC_Val;
	float  p, Vout, ppm, RS;

	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 200);
	ADC_Val = HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);

	Vout = ADC_Val * MQ135_ADC_VREF / MQ135_ADC_MAX_VALUE;
	RS = ((float)MQ135_ADC_VREF / Vout - 1) * MQ135_RL;

	p = ((log10(RS/MQ135_R0) - 0.2) / -0.415);
	ppm = pow(10.0,p);

	return ppm;
}



