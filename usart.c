#include "config.h"

#include "usart.h"

#include <avr/io.h>
#include <util/setbaud.h>
void usart_init()
{
  
  //Setting Baudrate settings
  UBRR0 = UBRR_VALUE;
  
  //8n1
  UCSR0C |= ( 1 << UCSZ00 ) | ( 1 << UCSZ01 );
  
  //Enable TX
  UCSR0B |= ( 1 << TXEN0 );
  
  //Enable RX
  //UCSR0B |= ( 1 << RXEN0 );
}

void usart_putc( char c )
{
  UDR0 = c;
    
  //Wating until transmission is completed
  while( ! ( UCSR0A & (  1 << UDRE0 ) ) );
}

void usart_putc_nonblock( char c )
{
  UDR0 = c;
}

void usart_puts( char *data )
{
  while( *data )
  {
    UDR0 = *data;
    
    //Wating until transmission is completed
    while( ! ( UCSR0A & (  1 << UDRE0 ) ) );
    
    //Inkrement data
    ++data;
  }
}