/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "contiki-net.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/leds.h"

//#define DEBUG 1
#include "dev/uart.h"
#include "dev/uart1.h"

// Disable printf debug 
#if 1
  #define printf(...) printf("")
#endif

#define SERVER_PORT 80

static struct tcp_socket socket;

#define INPUTBUFSIZE 400
static uint8_t inputbuf[INPUTBUFSIZE];

#define OUTPUTBUFSIZE 400
static uint8_t outputbuf[OUTPUTBUFSIZE];

PROCESS(blink_process, "LED blink process");
PROCESS(tcp_server_process, "TCP echo process");

AUTOSTART_PROCESSES(&tcp_server_process, &blink_process);
static uint8_t get_received;
static int bytes_to_send;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
  static struct etimer timer;
  static uint8_t leds_state = 0;
  PROCESS_BEGIN();

  while (1)
  {
    // we set the timer from here every time
    etimer_set(&timer, CLOCK_CONF_SECOND / 4);

    // and wait until the vent we receive is the one we're waiting for
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

    // update the LEDs
    leds_off(0xFF);
    leds_on(leds_state);
    leds_state += 1;
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
input(struct tcp_socket *s, void *ptr,
      const uint8_t *inputptr, int inputdatalen)
{
  //printf("input %d bytes '%s'\r\n", inputdatalen, inputptr);
  if(!get_received) {

    printf("--> %c (%d)\n", (char *)&inputptr[5], atoi((char *)&inputptr[5]));

    /* See if we have a full GET request in the buffer. */
    /*
    if(strncmp((char *)inputptr, "GET /", 5) == 0) { //&&
       //atoi((char *)&inputptr[5]) != 0) {
      //bytes_to_send = atoi((char *)&inputptr[5]);
      bytes_to_send = 1;
      printf("bytes_to_send %d\r\n", bytes_to_send);
      return 0;
    }
    */
    printf("inputptr '%.*s'\r\n", inputdatalen, inputptr);
    /* Return the number of data bytes we received, to keep them all
       in the buffer. */

    // send TCP data to UART
    bytes_to_send = 0;
    uint8_t i;
    for(i=0; i<inputdatalen; ++i) {
      uart_write_byte((uint8_t)0, inputptr[i]);
      bytes_to_send++;
    }

    return inputdatalen;
  } else {
    /* Discard everything */
    return 0; /* all data consumed */
  }
}
/*---------------------------------------------------------------------------*/
static void
event(struct tcp_socket *s, void *ptr,
      tcp_socket_event_t ev)
{
  printf("event %d\r\n", ev);
  if(ev == TCP_SOCKET_CLOSED ||
    ev == TCP_SOCKET_TIMEDOUT ||
    ev == TCP_SOCKET_ABORTED) {
    printf("Closing Socket Connection...\r\n");
    tcp_socket_close(&s);
  }
}
/*---------------------------------------------------------------------------*/
static int 
uart_rx_callback(unsigned char c) 
{
  /*
  uint8_t u;
  printf("\nReceived %c",c);
  u = (uint8_t)c;
  printf("\nReceived %d",u);
  */
  
  // UART Loopback echo
  uart_write_byte((uint8_t)0, (uint8_t)c);

  outputbuf[0] = c;
  outputbuf[1] = '\0';
  tcp_socket_send_str(&socket, outputbuf);
 
  return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcp_server_process, ev, data)
{
  PROCESS_BEGIN();

  uart_init(BAUD2UBR(115200)); //set the baud rate as necessary
  uart_set_input((uint8_t)0, uart_rx_callback); //set the callback function  

  tcp_socket_register(&socket, NULL,
               inputbuf, sizeof(inputbuf),
               outputbuf, sizeof(outputbuf),
               input, event);
  tcp_socket_listen(&socket, SERVER_PORT);

  printf("Listening on %d\r\n", SERVER_PORT);
  while(1) {
    PROCESS_PAUSE();
/*
    if(bytes_to_send > 0) {
      // Send header
      printf("sending header\r\n");
      tcp_socket_send_str(&socket, "HTTP/1.0 200 ok\r\nServer: Contiki tcp-socket example\r\n\r\n");

      // Send data 
      printf("sending data\r\n");
      while(bytes_to_send > 0) {
        PROCESS_PAUSE();
        int len, tosend;
#define MIN(a,b) ((a)<(b)?(a):(b))
        tosend = MIN(bytes_to_send, sizeof(outputbuf));
        len = tcp_socket_send(&socket, (uint8_t *)"", tosend);
        bytes_to_send -= len;
      }

      tcp_socket_close(&socket);
    }
    */
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
