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

#ifndef _USART_H_
#define _USART_H_

#include <stdbool.h>

void usart_init(void);

void usart_putc( char c );

bool usart_putc_nonblock( char c );

bool usart_puts_nonblock(char *data);

void usart_puts( char *data );

#endif