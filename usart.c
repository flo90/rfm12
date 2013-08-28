#include "config.h"
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