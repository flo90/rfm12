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


char strbuf[100];

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
  
  delay_ms(3500);
  
  
  for(i=0; i < 8; i++)
  {
    sprintf(strbuf, "RAND %d\r\n", rfm12_getrandomnumber());
    usart_puts(strbuf);
  }
  
  
  while(1)
  {
    if(!(PINA & (1<<PA0)))
    {
      sprintf(strbuf, "RAND %d\r\n", rfm12_getrandomnumber());
      usart_puts(strbuf);
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