#include "spi.h"

#include <inttypes.h>
#include <avr/io.h>

void spi_init( uint8_t cfg, uint8_t spd )
{
    SPCR = cfg;
    SPCR |= 0x03 & spd;
    SPSR &= ~( 1 << 0 );
    SPSR |= ( spd >> 2 );
}

unsigned char spi_exchangebyte( unsigned char data )
{
  SPDR = data;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  return SPDR;
}