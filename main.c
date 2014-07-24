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
#include "rfm12llc.h"


#include "rfm12phy.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdbool.h>

#include <stdio.h>

char txbuf[] = "Hallo";
char rxbuf[20];

char *prxbuf = rxbuf;
char *prxcompare = txbuf;
char volatile *ptxbuf = txbuf;


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


bool receive(uint8_t data, RFM12_Transfer_Status_t status)
{
  bool datatest = true;
  usart_putc_nonblock(data);
  if(status == RFM12_TRANSFER_STATUS_LASTBYTE)
  { 
    //switchled();
    prxbuf = rxbuf;
    prxbuf += 2;
    
    while(*prxcompare)
    {
      if(*prxcompare != *prxbuf)
      {
	datatest = false;
      }

      prxcompare++;
      prxbuf++;
    }

    if(datatest)
    {
      switchled();
    }
    goto reset;
  }
  
  else
  {
    *prxbuf++ = data;
  }
  
  return true;
  
  reset:
  prxbuf = rxbuf;
  prxcompare = txbuf;
  
  return false;
  
}

uint8_t transmit(void)
{
  uint8_t data = *ptxbuf++;
  return data;
}

RFM12_PHY_FUNCPTR_t rfm12funcptr;

int main(void)
{
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
  
  
  rfm12_init( rfm12funcptr, &transmit, 1);
  rfm12_llc_registerProto(0, &receive);
  
  //------------------PHY Layer TEST-----------------------
  /*
  rfm12_phy_init(&spi_exchangeword, &enablerfm12, &disablerfm12, &rfm12_int_vect);
  
  rfm12_phy_setConf( true, true, FREQBAND_433MHz, CAP12_5pf);
  
  rfm12_phy_setPowerManagement( true, true, true, false, false, false);
  
  rfm12_phy_setDataFilter(true, false, false, 4);
  
  rfm12_phy_setAFC(AUTOMODE_RECV, RANGELIMIT_3toMINUS4, false, true, true, true);
  
  rfm12_phy_modeRX();
  */
  
  PORTD |= (1<<PD6);
  
  delay_ms(500);
  
  PORTD &= ~(1<<PD6);
  
  
  //clear interrupt flags
  EIFR |= (1<<INTF0);
  
  sei();
  
  delay_ms(500);
  PORTD |= (1<<PD6);
  usart_puts_nonblock("Init!");
  while(0)
  {
    delay_ms(500);
    ptxbuf = txbuf;
    if(rfm12_llc_startTX(1, 0, sizeof(txbuf)))
    {
      usart_puts_nonblock("TX Data");
    }
  }
  
  while(1);
  return 0;
}

ISR(INT0_vect)
{
  rfm12_phy_int_vect();
}
