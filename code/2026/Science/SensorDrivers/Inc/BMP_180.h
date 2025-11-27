/*
 * BMP_180.h
 *
 *  Created on: May 15, 2025
 *      Author: ALPEREN
 */

#ifndef INC_BMP_180_H_
#define INC_BMP_180_H_


#include "stm32f3xx.h"


#define BMP180_ADDRESS			0xEE
#define CALIB_ADDRESS			0xAA
#define CTRL_MEAS				0XF4
#define OUT_MSB					0xF6
#define OUT_LSB					0xF7
#define OUT_XLSB				0xF8
#define OSS						1U



void read_calib_data(void);
uint16_t get_ut (void);
float get_temp (void);
uint32_t get_up(void);
long get_press(void);





#endif /* INC_BMP_180_H_ */
