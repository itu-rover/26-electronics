/*
 * MQ4.c
 *
 *  Created on: May 11, 2025
 *      Author: ALPEREN
 */

/*Measures CH4*/

/*Includes------------------------------------------*/
#include "MQ4.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;



/*
 * 	Equation derived from the graph  in datasheet using linear fit :
 * 	https://www.winsen-sensor.com/d/files/MQ-4.pdf
 *
 *  log(RS/R0) = -0.5991 log(ppm) + 0.8316
 *  log(ppm) = (log(RS/R0) - 0.8316) / -0.5991 = (0.8316 - log(RS/R0)) / 0.5991
 *  ppm = pow(10,  (0.8316 - log(RS/R0)) / 0.5991)
 *
 */

float Read_CH4() {

	uint16_t ADC_Val;
	float  Vout, ppm, RS;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 200);
	ADC_Val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	Vout = ADC_Val * MQ4_ADC_VREF / (float)MQ4_ADC_MAX_VALUE;
	RS = ((float) MQ4_ADC_VREF / Vout - 1) * MQ4_RL;

	ppm = pow(10,  (0.8316f - log(RS/(float)MQ4_R0)) / 0.5991f);

	return ppm;
}






