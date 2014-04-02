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

char rxbuf[10];
uint8_t txbuf[] = "Hallo";
uint8_t *txbufptr = txbuf;

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

void enablerfm12(void)
{
  PORTB &= ~(1<<PB4);
}

void disablerfm12(void)
{
  PORTB |= (1<<PB4);
}

void (*rfm12_int_vect)(void) = NULL;

bool receive(uint8_t data, bool lastbyte)
{
  if(!lastbyte)
  {
    
  }
  return true;
}

uint8_t transmit(void)
{
  return *txbufptr++;
}

int main(void)
{
  //init ports
  DDRD |= (1<<PD6) | (1<<PD5);
  PORTD |= (1<<PD2);
  
  //config interrupts
  
  //EICRA |= (1<<ISC01);
  EIMSK |= (1<<INT0);
  
  //while(1);
  
  usart_init();
  spi_init(SPI_SPE | SPI_MSTR , SPI_CLK_8);
  
  rfm12_init( &spi_exchangeword, &enablerfm12, &disablerfm12, &rfm12_int_vect, &receive, &transmit, 1);
  
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

  delay_ms(1000);
  //rfm12_llc_startTX(1, sizeof(txbuf));
  
  PORTD |= (1<<PD6);
  
  while(1);
  return 0;
}

ISR(INT0_vect)
{
  rfm12_int_vect();
  //PORTD |= (1<<PD6);
  //EIFR |= (1<<INTF0);
}
