#ifndef _USART_H_
#define _USART_H_

//Baudrate
/*
#define BAUD 115200UL
#define BAUD 19200UL 
 
#ifndef F_CPU
#define F_CPU 18432000UL
#endif
*/

void usart_init(void);

void usart_putc( char c );

void usart_putc_nonblock( char c );

void usart_puts( char *data );

#endif