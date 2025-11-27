/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  tcpServerRAW.h
  Author:     ControllersTech.com
  Updated:    26-Jul-2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_TCPSERVERRAW_H_
#define INC_TCPSERVERRAW_H_

#include "stdint.h"
#include "main.h"

void tcp_server_init(void);

typedef struct {
    uint16_t timeStamp;
    uint16_t pID;
    uint8_t payload[256];
} DataPacket;



// Declaring external variables
extern uint8_t ethBuf[64];
//extern Protocoltype protocol;



#endif /* INC_TCPSERVERRAW_H_ */
