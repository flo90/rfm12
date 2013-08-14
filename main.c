#include "config.h"
#include "spi.h"
#include "rfm12.h"
#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdbool.h>

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
  usart_puts("Init Done\r\n");
  
  sei();
  
  PORTA |= (1<<PA0);
  
  delay_ms(10);
  
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
      delay_ms(500);
      
    }
    delay_ms(2500);
    if(rfm12_haspacket())
    {
      byte = rfm12_getpacketptr();
      for(i = 0; i < 3; i++)
      {
	if(string[i] != bufptr[(byte+i+2)&0xFF])
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