/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  udpServerRAW.c
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


#include <udpServerRAW.h>
#include <string.h>
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"


void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void udp_decode_message(char* buf);
static void process_led(char *buf);


/* IMPLEMENTATION FOR UDP Server :   source:https://www.geeksforgeeks.org/udp-server-client-implementation-c/

1. Create UDP socket.
2. Bind the socket to server address.
3. Wait until datagram packet arrives from client.
4. Process the datagram packet and send a reply to client.
5. Go back to Step 3.
*/

void udpServer_init(void)
{
	// UDP Control Block structure
   struct udp_pcb *upcb;
   err_t err;

   /* 1. Create a new UDP control block  */
   upcb = udp_new();

   /* 2. Bind the upcb to the local port */
   ip_addr_t myIPADDR;
   IP_ADDR4(&myIPADDR, 192, 168, 1, 2);

   err = udp_bind(upcb, &myIPADDR, 6100);  // 6100 is the server UDP port


   /* 3. Set a receive callback for the upcb */
   if(err == ERR_OK)
   {
	   udp_recv(upcb, udp_server_receive_callback, NULL);
   }
   else
   {
	   udp_remove(upcb);
   }
}

// udp_receive_callback will be called, when the client sends some data to the server
/* 4. Process the datagram packet and send a reply to client. */

void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	struct pbuf *txBuf = (struct pbuf * )0x20006000;

	/* Get the IP of the Client */
	//char *remoteIP = ipaddr_ntoa(addr);

	char buf[100];
	udp_decode_message((char*)p->payload);

	int len = sprintf (buf,"Hello %s From UDP SERVER\n", (char*)p->payload);

	/* allocate pbuf from RAM*/
	txBuf = pbuf_alloc(PBUF_TRANSPORT,len, PBUF_RAM);

	/* copy the data into the buffer  */
	pbuf_take(txBuf, buf, len);

	/* Connect to the remote client */
	udp_connect(upcb, addr, port);

	/* Send a Reply to the Client */
	udp_send(upcb, txBuf);

	/* free the UDP connection, so we can accept new clients */
	udp_disconnect(upcb);

	/* Free the p_tx(response) buffer */
	pbuf_free(txBuf);

	/* Free the p(incoming) buffer */
	pbuf_free(p);
}

static void udp_decode_message(char* buf)
{

	if(buf[0] == 'L')
	{
		process_led(buf);
	}
	else
	{

	}

}

static void process_led(char *buf)
{
	if(buf[1]=='T')
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	}
	else if(buf[1]=='S')
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
	}
	else if(buf[1]=='R')
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
	}
}
