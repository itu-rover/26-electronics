/*
 * MQ4.h
 *
 *  Created on: May 11, 2025
 *      Author: ALPEREN
 */

#ifndef INC_MQ4_H_
#define INC_MQ4_H_

/*
 * Measures CH4
 */

/*Includes ---------------------------------------------------*/
#include "stm32f3xx.h"


#define MQ4_ADC_MAX_VALUE			4095.0f
#define MQ4_RL					    4700.0f      //(?)
#define MQ4_R0					    472.8f //460 - 490 (RS0)
#define MQ4_ADC_VREF				3.3f
#define CH4_BUFF_SIZE				1



float Read_CH4();




#endif /* INC_MQ4_H_ */
