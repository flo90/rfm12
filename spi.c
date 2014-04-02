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

#include <inttypes.h>
#include <avr/io.h>

void spi_init( uint8_t cfg, uint8_t spd )
{
  DDRB |= (1<<PB5) | (1<<PB7) | (1<<PB4);
  PORTB |= (1<<PB4);

  SPCR = cfg;
  SPCR |= 0x03 & spd;
  SPSR &= ~( 1 << 0 );
  SPSR |= ( spd >> 2 );
  
  /*
  SPCR = (1<<SPE) | (1<<SPR0) | (1<<MSTR);
  SPSR = (1<<SPI2X);
  */
}

uint8_t spi_exchangebyte( uint8_t data )
{
  SPDR = data;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  return SPDR;
}

uint16_t spi_exchangeword( uint16_t data )
{
  uint16_t temp;
  SPDR = (data>>8)&0xFF;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  
  temp = (SPDR<<8);
  
  SPDR = (data)&0xFF;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  
  temp |= SPDR;
  
  return temp;
}