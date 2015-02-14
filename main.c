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
#include "usart.h"
#include "rfm12.h"
#include "rfm12mac.h"


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>

#include <stdio.h>

static uint8_t txbuf[] = "Hallo";

static RFM12_MAC_Frame_t txframe;

void delay_ms(uint16_t ms)
{
  while(ms--)
  {
    _delay_ms(1);
  }
}

void switchled(void)
{
  PORTD ^= (1<<PD5);
}

void enablerfm12(void)
{
  PORTB &= ~(1<<PB4);
}

void disablerfm12(void)
{
  PORTB |= (1<<PB4);
}

void enableINT0(void)
{
  EIMSK = (1<<INT0);
}

void disableINT0(void)
{
  EIMSK = 0;
}

void (*rfm12_int_vect)(void) = NULL;

RFM12_PHY_FUNCPTR_t rfm12funcptr;
RFM12_MAC_Frame_t *pframe;

void rxtest(RFM12_MAC_Frame_t *pframe)
{
  uint8_t i;
  if(strcmp(pframe->data, txbuf) == 0)
  {
    switchled();
  }
}

int main(void)
{
  txframe.data = txbuf;
  txframe.header.dstAddr = 1;
  txframe.header.length = sizeof(txbuf);
  txframe.header.service = 0;
  
  //init ports
  DDRD |= (1<<PD6) | (1<<PD5);
  PORTD |= (1<<PD2);
  
  //config interrupts
  
  //EICRA |= (1<<ISC01);
  EIMSK |= (1<<INT0);
  
  usart_init();
  spi_init(SPI_SPE | SPI_MSTR , SPI_CLK_8);
  
  rfm12funcptr.exchangeWord = &spi_exchangeword;
  rfm12funcptr.SPISelect = &enablerfm12;
  rfm12funcptr.SPIDeselect = &disablerfm12;
  rfm12funcptr.enableINT = &enableINT0;
  rfm12funcptr.disableINT = &disableINT0;
  
  
  rfm12_init( rfm12funcptr, 1);
  rfm12_mac_addLLCService(0, &rxtest);
  
  PORTD |= (1<<PD6);
  
  delay_ms(500);
  
  PORTD &= ~(1<<PD6);
  
  
  //clear interrupt flags
  EIFR |= (1<<INTF0);
  
  sei();
  
  delay_ms(500);
  PORTD |= (1<<PD6);
  usart_puts_nonblock("Init!");
  
  while(1)
  {
    
#if 1
    delay_ms(500);
    rfm12_mac_startTransmission(&txframe);
#else
    rfm12_mac_LLCtaskHandler();
#endif
  }
  
  while(1);
  return 0;
}

ISR(INT0_vect)
{
  rfm12_phy_int_vect();
}
