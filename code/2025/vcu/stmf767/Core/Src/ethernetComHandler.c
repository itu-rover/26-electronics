/*
 * ethernetComHandler.c
 *
 *  Created on: Apr 1, 2025
 *      Author: kadir
 *
 *      Bir for döngüsü, function pointer üzerinden bit sayarak istenilen bitlerin fonksiyonuna girer.
 *      Böylelikle tek bir pakette birden fazla istek veya komut atanabilir.
 *
 *      processCount değişkeni, tek pakette aktarılan birden fazla veride verilerin sırasını
 *      belirlemek için bulunur. Bir nevi son elemanı belirten pointer olarak bulunur.
 *
 *      ethBuf verisi, istenilen fonksiyonlarda processCount ile beraber birikerek,
 *      bu fonksiyon dizninde sonuncu elemana kadar toplanıp, en son gönderilir.
 *
 */
#include "stm32f7xx_hal.h"
#include "ethernetComHandler.h"
//#include "udpClientRAW.h"
#include <stdio.h>
#include <string.h>
#include "common4defines.h"

DataPacket myPacket;
//char ethBuf[MAX_PAYLOAD_SIZE];
//char txBuff[MAX_PAYLOAD_SIZE];
//BitOperationHandler bitOperations[MAX_BIT_OPERATIONS] = { NULL };

// Data Parsing Function

uint8_t processCount = 0;
uint8_t i_eth = 0;

/* 	Mesaj paketi uzerinden secilebilen fonksiyonlar
	Zaman icinde ek fonksiyonlar eklenebilir.
	Dizinin #ncı elemanı ile mesajın #ncı bitinin "1" olması fonksiyona girdirir	*/
void (*bitOperations[16])(uint8_t* payload, uint16_t length) = {
    drivetrain_motorCommand, roboticarm_motorCommand, science_motorCommand, gnssData,
    ledState, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL
};


/*	Veri değişmeme durumunu kontrol etmek için	*/
uint32_t last_msg_tick;
uint16_t last_ts = 0;
volatile uint8_t failsafe_triggered = 0;

uint32_t last_msg_tick;

uint8_t default_driveWheel[9] = {'1','0','0','0','1','0','0','0','0'};
uint8_t default_roboticArm[6] = {'5','5','5','5','5','5'};

uint8_t feedbackCounter;


void parseData(const uint8_t *data, uint16_t length)
{
    if (length < 4) {
        return;  // Invalid data length
    }


    // Parse the packet
    myPacket.timeStamp = (data[0] << 8) | data[1];
    myPacket.pID = (data[2] << 8) | data[3];
    extern uint16_t currentTs;

    currentTs = myPacket.timeStamp;



    uint16_t payloadSize = length - (sizeof(myPacket.pID) + sizeof(myPacket.timeStamp));
    memcpy(myPacket.payload, &data[4], payloadSize);

    comProcessHandler(payloadSize);

}

void comProcessHandler(uint16_t length)
{
	//packet->timeStamp; Karsilastirma icin kullanilacak

	/* Gelen paket doğru mu diye bakmak için Flag */
	uint8_t flag = 0;
	processCount = 0;
	feedbackCounter = 0;


	//extern DataPacket myPacket;

	uint16_t process = myPacket.pID;

	if((failsafe_triggered == 0) || (HAL_GetTick() - last_msg_tick) < 500){
		for (i_eth = 0; i_eth < 16; i_eth++) {
			if ((process >> i_eth) & 1){
				if (bitOperations[i_eth] != NULL) {
					bitOperations[i_eth](myPacket.payload, length);
					flag = 1;
				}
			}
		}
	}



    udpClient_send();
    if (flag == 0){
    	snprintf(ethBuf, sizeof(ethBuf), "WrongPackage");
    }
    return;
}

// Motor Command Handlers
void drivetrain_motorCommand(uint8_t* payload, uint16_t length){
	//*(payload + 8) = "\0"; zaten burası \0
	//snprintf(ethBuf, sizeof(ethBuf), "%s","drivetrain_motorCommand");
	memcpy(txBuff + processCount, (char*)payload, DRIVETRAIN_COMMAND_SIZE);
	//snprintf(txBuff + processCount, length + 1, "%s", (char*)payload);
	//memcpy(ethBuf + 4, &rxBuff, 32);


	/*ethBuf[0] = 0;
	ethBuf[1] = 0;
	ethBuf[2] |= 0x1 << (i_eth - 8);
	ethBuf[3] |= 0x1 << i_eth;*/

	memcpy(ethBuf + timeStampSize + processIdSize + feedbackCounter, &rxBuff[DRIVETRAIN_FEEDBACK_SIZE], DRIVETRAIN_FEEDBACK_SIZE);

	feedbackCounter += DRIVETRAIN_FEEDBACK_SIZE;
	processCount += DRIVETRAIN_COMMAND_SIZE;


	/* WDT baslatilacak 200ms. Eger timestamp degismezse durma komutu	*/
	/* Her atlanılan timestamp icin de bir sayac eklenilecek			*/
	return;
}

void roboticarm_motorCommand(uint8_t* payload, uint16_t length)
{
	memcpy(txBuff + processCount, (char*)(payload + processCount) , ARM_COMMAND_SIZE);

	/*ethBuf[0] = 0;
	ethBuf[1] = 0;
	ethBuf[2] |= 0x1 << (i_eth - 8);
	ethBuf[3] |= 0x1 << i_eth;*/

	memcpy(ethBuf + timeStampSize + processIdSize + feedbackCounter, &rxBuff[DRIVETRAIN_FEEDBACK_SIZE], ARM_FEEDBACK_SIZE);
	feedbackCounter += ARM_FEEDBACK_SIZE;
	processCount += ARM_COMMAND_SIZE;

	//snprintf(txBuff + DRIVETRAIN_COMMAND_SIZE, length + 1, "%s", (char*)payload);
}

void science_motorCommand(uint8_t* payload, uint16_t length)
{


	memcpy(uartTransmit, (char*)(payload + processCount) , SCIENCE_COMMAND_SIZE);
	/* 	Geliştirme Aşamasında	*/
	/*ethBuf[0] = 0;
	ethBuf[1] = 0;
	ethBuf[2] |= 0x1 << (i_eth - 8);
	ethBuf[3] |= 0x1 << i_eth;*/

	/* Bilim Komutları */
	for(int n = 0; n < SCIENCE_COMMAND_SIZE; n++){
		uart7_write_char(uartTransmit[n]);
	}

	uart7_receive_bytes(science_buffer, 39);

	memcpy(ethBuf + timeStampSize + processIdSize + feedbackCounter, science_buffer, SCIENCE_FEEDBACK_SIZE);
	feedbackCounter += SCIENCE_FEEDBACK_SIZE;
	processCount += SCIENCE_COMMAND_SIZE;

	//snprintf(ethBuf, sizeof(ethBuf), "science_motorCommand");
}

void gnssData(uint8_t* payload, uint16_t length)
{
	/*ethBuf[0] = 0;
	ethBuf[1] = 0;
	ethBuf[2] |= 0x1 << (i_eth - 8);
	ethBuf[3] |= 0x1 << i_eth;*/

	memcpy(ethBuf + timeStampSize + processIdSize + feedbackCounter, &rxBuff[DRIVETRAIN_FEEDBACK_SIZE + ARM_FEEDBACK_SIZE], GNSS_SIZE);
	feedbackCounter += GNSS_SIZE;
}

void ledState(uint8_t* payload, uint16_t length){
	HAL_GPIO_WritePin(GPIOD, 0x1000, *(payload + processCount)-48);	//D12
	HAL_GPIO_WritePin(GPIOD, 0x2000, *(payload + processCount + 1)-48);	//D13
	HAL_GPIO_WritePin(GPIOB, 0x0010, *(payload + processCount + 2)-48);	//B4
}

void uart7_write_char(char c)
{
	while (!(UART7->ISR & USART_ISR_TXE)); // TX boş değilse bekle
	UART7->TDR = c;
}
uint8_t uart7_receive_bytes(uint8_t *buffer, uint32_t length)
{
    uint8_t timeout_occurred = 0;
    for (uint32_t i = 0; i < length; i++) {
        uint32_t start_time = HAL_GetTick();
        while (!(UART7->ISR & USART_ISR_RXNE)) {
            if ((HAL_GetTick() - start_time) > 100) {
                buffer[i] = 0;
                timeout_occurred = 1;
                break;
            }
        }
        if (!timeout_occurred)
            buffer[i] = UART7->RDR;
    }
    return !timeout_occurred; // true = başarılı, false = zaman aşımı oldu
}
