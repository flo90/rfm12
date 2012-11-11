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
