/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpClientRAW.h
  Author:     ControllersTech.com
  Updated:    Jul 23, 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_UDPCLIENTRAW_H_
#define INC_UDPCLIENTRAW_H_

typedef struct {
    uint32_t timeStamp;
    uint8_t packageSize[16] ;
    uint8_t payload[256];
} UDP_DataPacket;

void udpClient_connect(uint8_t address);

void udpClient_send(void);


extern uint8_t ethBuf[256];

/* Kurulmuş bir bağlantı belirten FLAG
 */
extern uint8_t ethernetFlag;

//Eklemezsem sil
extern uint8_t IPadress;

#endif /* INC_UDPCLIENTRAW_H_ */
