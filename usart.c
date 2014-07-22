/*
    This file is part of the rfm12 driver project.
    Copyright (C) 2013  Florian Menne (florianmenne@t-online.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see [http://www.gnu.org/licenses/].
*/

#include "config.h"
#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/setbaud.h>

#include <stdbool.h>

#define MSGBUF_SIZE 256


volatile char msgbuf[MSGBUF_SIZE];

volatile bool running;
volatile uint8_t pread;
volatile uint8_t pwrite;

void usart_init()
{
  uint16_t i;
  
  //init datastructure
  pread = 0;
  pwrite = 0;
  
  running = false;
  
  for(i=0; i < MSGBUF_SIZE; i++)
  {
    msgbuf[i] = 0;
  }
  
  //Setting Baudrate settings
  UBRR0 = UBRR_VALUE;
  
  //8n1
  UCSR0C |= ( 1 << UCSZ00 ) | ( 1 << UCSZ01 );
  
  //Enable TX
  UCSR0B |= ( 1 << TXEN0 );
  
  //Enable RX
  //UCSR0B |= ( 1 << RXEN0 );
  
  //enable TXC interrupt
  UCSR0B |= (1<<TXCIE0);
  
}

void usart_putc( char c )
{
  UDR0 = c;
    
  //Wating until transmission is completed
  while( ! ( UCSR0A & (  1 << UDRE0 ) ) );
}

void usart_puts( char *data )
{
  while( *data )
  {
    UDR0 = *data;
    
    //Wating until transmission is completed
    while( ! ( UCSR0A & (  1 << UDRE0 ) ) );
    
    //Inkrement data
    ++data;
  }
}

bool usart_puts_nonblock(char *data)
{
  while(*data)
  {
    if(!msgbuf[pwrite])
    {
      msgbuf[pwrite++] = *data++;
      //return false;
    }
  }
  
  if(!running)
  {
    running = true;
    UDR0 = msgbuf[pread];
    msgbuf[pread++] = 0;
  }
  
  return true;
}

bool usart_putc_nonblock(char data)
{
  msgbuf[pwrite++] = data;
  
  if(!running)
  {
    running = true;
    UDR0 = msgbuf[pread];
    msgbuf[pread++] = 0;
  }
  return true;
}

ISR(USART0_TX_vect)
{
  if(!running)
  {
    return;
  }
  
  if(msgbuf[pread])
  {
    UDR0 = msgbuf[pread];
    msgbuf[pread++] = 0;
  }
  
  else
  {
    running = false;
  }
}