/*
 * MICS-4514.c
 *
 *  Created on: May 10, 2025
 *      Author: ALPEREN
 */

#include "MICS-4514.h"

extern I2C_HandleTypeDef hi2c1;

float Read_NO2 () {

	uint8_t ox_data[6], reg;
	uint8_t tx_buf[2] = {POWER_MODE_REGISTER, WAKE_UP_MODE};
	uint16_t oxADC, powerADC, Rs;
	float ppm;

	reg = OX_REGISTER_HIGH;
	HAL_I2C_Master_Transmit(&hi2c1, MICS_4514_ADDRESS, tx_buf, 2, HAL_MAX_DELAY);

    HAL_I2C_Master_Transmit(&hi2c1, MICS_4514_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_Delay(200);
	HAL_I2C_Master_Receive(&hi2c1, MICS_4514_ADDRESS, ox_data, 6, HAL_MAX_DELAY);

	oxADC = ((uint16_t)ox_data[0]<<8) | (uint16_t)ox_data[1];
	powerADC = ((uint16_t)ox_data[4]<<8) | (uint16_t)ox_data[5];

    Rs = powerADC - oxADC;
    ppm = ((float)Rs / (float)NO2_R0 - 0.045) / 6.13;

    return ppm;
}
