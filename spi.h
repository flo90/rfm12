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

#ifndef _SPI_H_
#define _SPI_H_


#include <inttypes.h>

#define SPI_SPIE ( 1 << 7 )
#define SPI_SPE ( 1 << 6 )
#define SPI_DORD ( 1 << 5 )
#define SPI_MSTR ( 1 << 4 )
#define SPI_CPOL ( 1 << 3 )
#define SPI_CPHA ( 1 << 2 )

#define SPI_CLK_4 0x00
#define SPI_CLK_16 0x01
#define SPI_CLK_64 0x02
#define SPI_CLK_128 0x03

#define SPI_CLK_2 0x04
#define SPI_CLK_8 0x05
#define SPI_CLK_32 0x06

//#define SPI_CLK_64 0x07

#define DONT_CARE 0x00

void spi_init( uint8_t cfg, uint8_t spd );

unsigned char spi_exchangebyte( unsigned char data );



#endif
