/*
 * DTH11.h
 *
 *  Created on: Feb 24, 2025
 *      Author: ALPEREN
 */

#ifndef DTH11_H_
#define DTH11_H_



#endif /* DTH11_H_ */

#include "stm32f3xx.h"

#define DHT11_GPIO_PORT		GPIOC
#define DHT11_GPIO_PIN		GPIO_PIN_8

void Delay_us(uint32_t us);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start();
uint8_t DHT11_Check_Response();
uint8_t DHT11_Read_Byte();
//void DHT11_Read_Data();



