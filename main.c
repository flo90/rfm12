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
#include "spi.h"
#include "rfm12.h"
#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdbool.h>

#include <stdio.h>

void delay_ms(uint16_t ms)
{
  while(ms--)
  {
    _delay_ms(1);
  }
}

void switchled(void)
{
  static bool led = false;
  if(led)
  {
    //PORTD = (1<<PD6);
    PORTD &= ~(1<<PD5);
    led = false;
  }
  else
  {
    PORTD = (1<<PD5);
    //PORTD &= ~(1<<PD6);
    led = true;
  }
}

int main(void)
{
  uint16_t byte;
  uint8_t i;
  unsigned char *bufptr;
  char string[] = {'L','E','D'};
  usart_init();
  spi_init(SPI_SPE | SPI_MSTR , SPI_CLK_8);
  
  DDRD |= (1<<PD6) | (1<<PD5);
  
  PORTD |= (1<<PD6);
  
  delay_ms(500);
  
  PORTD &= ~(1<<PD6);
  bufptr = rfm12_init();
  
  sei();
  
  
  
  usart_puts("Init Done\r\n");
  
  PORTA |= (1<<PA0);
  
  delay_ms(500);
  
  while(1)
  {
    if(!(PINA & (1<<PA0)))
    {
#ifdef DEBUG
      usart_puts("Sending...\r\n");
#endif
      if(!rfm12_tx(string, 3))
      {
#ifdef DEBUG
	usart_puts("Sent\r\n");
#endif
      }
      usart_puts("Sending...\r\n");
      delay_ms(500);
      
    }
    
    if(rfm12_haspacket())
    {
      byte = rfm12_getpacketptr();
      for(i = 0; i < 3; i++)
      {
	if(string[i] != bufptr[(byte+i+2)&0x1FF])
	{
	  break;
	}
	
	if(i == 2)
	{
	  switchled();
	}
      }
      rfm12_clearpacket();
    }
    
  }
  
  
  while(1);
  return 0;
}