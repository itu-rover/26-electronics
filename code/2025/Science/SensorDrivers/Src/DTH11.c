
#include "DTH11.h"

extern TIM_HandleTypeDef htim2;

void Delay_us(uint32_t us) {
	HAL_TIM_Base_Start(&htim2);
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);  // Get current counter value

    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);  // Wait for required delay
}



// Function to set pin as output
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Function to set pin as input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start() {
    Set_Pin_Output(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET);
    Delay_us(40);
    Set_Pin_Input(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
}

// Function to check DHT11 response
uint8_t DHT11_Check_Response() {
    uint8_t Response = 0;
    Delay_us(40);			//we are in the middle of 80 us low

    if (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == 0) {
        Delay_us(80);       //we are in the middle of 80 us high
        if (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == 1) Response = 1;
        else Response = 0;
    }

    while ((HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)));	//wait for low
    return Response;
}

// Function to read a single byte
uint8_t DHT11_Read_Byte() {
    uint8_t data, i;
    for (i = 0; i < 8; i++) {

        while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == 0);	//wait for high (50 us low at the start of transmisson)
        Delay_us(40);

        if (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == 0) {
            data &= ~(1 << (7 - i));
        }
        else {
            data |= (1 << (7 - i));
        }
        while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == 1);	//wait for low
    }
    return data;
}



