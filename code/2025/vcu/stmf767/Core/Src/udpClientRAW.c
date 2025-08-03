/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpClientRAW.c
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


#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
#include "string.h"

#include "udpClientRAW.h"
#include "common4defines.h"


void udp_client_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_parse_message(struct pbuf *p);

struct udp_pcb *upcb;
char buffer[100];
int counter = 0;

void udpClient_connect(uint8_t address)
{
	err_t err;
	int port = 6102;

	/*  Eğer daha önce bir bağlantı varsa temizle	*/
	if (upcb != NULL) {
        udp_remove(upcb);
        upcb = NULL;
    }

	/* 1. Create a new UDP control block  */
	upcb = udp_new();

	if (upcb == NULL) {
        //Hata fonksiyonu eklenebilir belki
        return;
	}

	/* Bind the block to module's IP and port */
	ip_addr_t myIPaddr;
	IP_ADDR4(&myIPaddr, 192, 168, 1, 2);
	udp_bind(upcb, &myIPaddr, 6101);

	if (address == 105)
	{
		port = 8400;
	}
	else
	{
		port = 6102;
	}
	/* configure destination IP address and port */
	ip_addr_t DestIPaddr;
	IP_ADDR4(&DestIPaddr, 192, 168, 1, address);
	err= udp_connect(upcb, &DestIPaddr, port);

	if (err == ERR_OK)
	{
		/* 2. Send message to server */
		udpClient_send();

		/* 3. Set a receive callback for the upcb */
		udp_recv(upcb, udp_client_receive_callback, NULL);
	}
}


// Called in main.c
void udpClient_send(void)
{
  struct pbuf *txBuf = (struct pbuf * )0x20006000;
  char data[100];


  memcpy(data, ethBuf, 64);
  //int len = sprintf(data, "sending UDP client message %d", counter);

  /* allocate pbuf from pool*/
  //txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  txBuf = pbuf_alloc(PBUF_TRANSPORT, 64, PBUF_RAM);

  if (txBuf != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(txBuf, data, 64);

    /* send udp data */
    udp_send(upcb, txBuf);

    /* free pbuf */
    pbuf_free(txBuf);
  }
}


void udp_client_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	TIM7->CNT = 0;
	ethernetFlag = 1;
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	/* Copy the data from the pbuf */
	memcpy (buffer, (char *)p->payload, p->len);

	/*increment message count */
	counter++;
	udp_parse_message(p);
	/* Free receive pbuf */
	pbuf_free(p);
}

void udp_parse_message(struct pbuf *p){

	char buf[100];


	//memset (ethBuf, '\0', 1024);
	memset (buf, '\0', 100);
	memcpy(buf, (char *)p->payload, p->tot_len);
	parseData(buf,p->len);
	memcpy(&ethBuf, (char *)p->payload, timeStampSize + processIdSize);
	return;
}



