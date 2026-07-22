
#include "soil-sensor.h"
#include <stdio.h>

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;


float Moisture = -1;
float Temp = -1;
uint32_t Cond;
float pH = -1;

uint32_t N;
uint32_t P;
uint32_t K;


const uint8_t Soil_Basic_TxBuffer[8] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x04, 0xe4, 0x0c};
// 15 24

//uint8_t MODBUS_Decode(uint8_t *buffer){}

uint8_t MODBUS_Soil_Basic_Decode(uint8_t *buffer){
    uint8_t Slave_ID = buffer[1];
    uint8_t Function_Type = buffer[2];
    uint8_t ByteCount = buffer[3];

    if (Slave_ID != 1 || Function_Type != 3 || ByteCount != 8){ // ERR
        return 1;
    }

    uint16_t raw_Moisture = buffer[4] * 256 + buffer[5];
    uint16_t raw_Temp = buffer[6] * 256 + buffer[7];
    uint16_t raw_Cond = buffer[8] * 256 + buffer[9];
    uint16_t raw_pH = buffer[10] * 256  + buffer[11];

    //implement checksum 

    Moisture = raw_Moisture / 10;
    Temp = raw_Temp / 10;
    Cond = raw_Cond;
    pH = raw_pH / 10;
}

uint8_t MODBUS_NPK_Decode(uint8_t *buffer){
    
    uint8_t Slave_ID = buffer[0];
    uint8_t Function_Type = buffer[1];
    uint8_t ByteCount = buffer[2];

    if (Slave_ID != 1 || Function_Type != 3 || ByteCount != 6){ // ERR
        return 1;
    }

    uint16_t raw_N = buffer[3] * 256 + buffer[4];
    uint16_t raw_P = buffer[5] * 256 + buffer[6];
    uint16_t raw_K = buffer[7] * 256 + buffer[8];

    //implement check sum

    N = raw_N;
    P = raw_P;
    K = raw_K;

}

uint16_t MODBUS_CRC_CheckSum(uint8_t *buffer, uint16_t length){
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc; // Note: Modbus standard sends Low-byte first
}

uint8_t createRawSoilReading(){
    //creates new Raw reading by reading values from the data 
    // 0 return value means it created 2 new raw reading succ, 1 return value means it couldn't create 1st new raw read, 2 return value means it couldn't create 2nd new raw read, 3 return value means it couldn't create 1st and 2nd raw read.
    // Transmit 1 by UART
    // Recieve 1 by UART
    uint8_t buffer[100] = {0x00};

    //uint8_t final_buffer[100] = {0};

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
    HAL_UART_Transmit(&huart5, Soil_Basic_TxBuffer, Soil_Basic_TxBuffer_size, 100);
        
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
    HAL_UART_Receive(&huart5, buffer, Soil_Basic_RxBuffer_size, 2000);

    //HAL_UART_Transmit(&huart2, buffer, Soil_Basic_RxBuffer_size, 2000);


    MODBUS_Soil_Basic_Decode(buffer);

    //Transmit 2 by UART
    //Recieve 2 by UART
    //HAL_UART_Transmit(&huart5, NPK_TxBuffer, NPK_TxBuffer_size, 100);
    //HAL_UART_Receive(&huart5, buffer, NPK_RxBuffer_size, 200);

    //MODBUS_NPK_Decode(buffer);    

    //write a global var or return the string arr give 2 byte for all raw values 
    //uint32_t buffer_size = sprintf(buffer, "ehe:%5.1f %5.1f %5lu %5.1fehe", Moisture, Temp, Cond, pH);
    //HAL_UART_Transmit(&huart2, buffer, buffer_size, 50);
    //HAL_UART_Transmit(&huart2, buffer, Soil_Basic_RxBuffer_size, 200);

    return 0;
}
     
