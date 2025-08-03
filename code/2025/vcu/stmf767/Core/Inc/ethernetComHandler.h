/*
 * ethernetComHandler.h
 *
 *  Created on: Apr 1, 2025
 *      Author: kadir
 */

#ifndef INC_ETHERNETCOMHANDLER_H_
#define INC_ETHERNETCOMHANDLER_H_

#include <stdint.h>

/*  NULL Pointer tanımı burada	*/
#include "stddef.h"
#include "common4defines.h"


#define MAX_PAYLOAD_SIZE 256
#define MAX_BIT_OPERATIONS 16


typedef enum
{
    TCP_IP,
    UDP_IP
} ProtocolType;


typedef struct {
    uint16_t timeStamp;
    uint16_t pID;
    uint8_t payload[MAX_PAYLOAD_SIZE];
} DataPacket;

extern uint8_t processCount;
extern uint8_t i_eth;
extern char uartTransmit[SCIENCE_COMMAND_SIZE];



/*	 Gelen Verinin Parse Hali	*/
//extern DataPacket myPacket;

/* Ethernet Üzerinden Gönderilecek Veri	*/
extern uint8_t ethBuf[MAX_PAYLOAD_SIZE];

/* SPI Üzerinden Gönderilecek Veri	*/
extern uint8_t txBuff[64];

extern uint8_t rxBuff[64];

extern uint8_t science_buffer[39];

typedef void (*BitOperationHandler)(uint8_t* payload, uint16_t length);
extern BitOperationHandler bitOperations[MAX_BIT_OPERATIONS];

void uart7_write_char(char c);
uint8_t uart7_receive_bytes(uint8_t *buffer, uint32_t length);

void parseData(const uint8_t *data, uint16_t length);
void comProcessHandler(uint16_t length);



/* TCP/IP veya UDP Komut fonksiyonları*/
void drivetrain_motorCommand(uint8_t* payload, uint16_t length);
void roboticarm_motorCommand(uint8_t* payload, uint16_t length);
void science_motorCommand(uint8_t* payload, uint16_t length);
void gnssData(uint8_t* payload, uint16_t length);
void ledState(uint8_t* payload, uint16_t length);
void sendFunc(void);




#endif /* INC_ETHERNETCOMHANDLER_H_ */
