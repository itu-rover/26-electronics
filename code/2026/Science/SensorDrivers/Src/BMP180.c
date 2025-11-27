/*
 * BMP180.c
 *
 *  Created on: May 15, 2025
 *      Author: ALPEREN
 */

/*
 * Datasheet:
 * https://www.mouser.com.tr/datasheet/2/783/BST-BMP180-DS000-1509579.pdf
 */

#include "BMP_180.h"
#include <math.h>


extern I2C_HandleTypeDef hi2c1;

/*EEPROM CALIBRATION*/
short AC1;
short AC2;
short AC3;
unsigned short AC4;
unsigned short AC5;
unsigned short AC6;
short B1;
short B2;
short MB;
short MC;
short MD;
long UT;
long UP;
long X1;
long X2;
long X3;
long B3;
long B5;
unsigned long B4;
long B6;
unsigned long B7;
/*-----------------------------------------------------------------------*/

uint8_t calib_data [22];





void read_calib_data(void) {
//	HAL_I2C_Master_Transmit(&hi2c1, BMP180_ADDRESS, CALIB_ADDRESS, 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(&hi2c1, BMP180_ADDRESS, calib_data, 22, HAL_MAX_DELAY);

	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, CALIB_ADDRESS, 1, calib_data, 22, HAL_MAX_DELAY);

	AC1 = ((uint16_t)calib_data[0]<<8)  | calib_data[1];
	AC2 = ((uint16_t)calib_data[2]<<8)  | calib_data[3];
	AC3 = ((uint16_t)calib_data[4]<<8)  | calib_data[5];
	AC4 = ((uint16_t)calib_data[6]<<8)  | calib_data[7];
	AC5 = ((uint16_t)calib_data[8]<<8)  | calib_data[9];
	AC6 = ((uint16_t)calib_data[10]<<8) | calib_data[11];
	B1  = ((uint16_t)calib_data[12]<<8) | calib_data[13];
	B2  = ((uint16_t)calib_data[14]<<8) | calib_data[15];
	MB  = ((uint16_t)calib_data[16]<<8) | calib_data[17];
	MC  = ((uint16_t)calib_data[18]<<8) | calib_data[19];
	MD  = ((uint16_t)calib_data[20]<<8) | calib_data[21];
}


uint16_t get_ut (void) {
	uint8_t data = 0x2E;
	uint8_t ut_data[2] = {0};
	uint16_t temp;

	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CTRL_MEAS, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, OUT_MSB, 1, ut_data, 2, HAL_MAX_DELAY);

	temp = ((uint16_t)ut_data[0]<<8) | ut_data[1];

	return temp;
}



float get_temp(void) {
    long temp;

//    UT = get_ut();

    X1 = ((UT - (long)AC6) * (long)AC5) >> 15;
    X2 = ((long)MC << 11) / (X1 + (long)MD);
    B5 = X1 + X2;
    temp = (B5 + 8) >> 4;

    return temp / 10.0f;
}



uint32_t get_up(void) {
	uint8_t data = 0x34 + (OSS<<6);
	uint8_t up_data[3];
	uint32_t up;

	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CTRL_MEAS, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(5);

	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, OUT_MSB, 1, up_data, 3, HAL_MAX_DELAY);

	up = ((uint32_t)(up_data[0]<<16) + (uint32_t)(up_data[1]<<8) + up_data[2]) >> (8-OSS);
	return up;
}



long get_press(void) {
    long p;
    B6 = B5 - 4000;

    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)AC1 * 4 + X3) << OSS) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (5000 >> OSS);

    if (B7 < 0x80000000)
        p = (B7 << 1) / B4;
    else
        p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    return p;  // pascal
}
