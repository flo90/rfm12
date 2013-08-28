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
#include "rfm12.h"
#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <inttypes.h>
#include <stdbool.h>

//#include <util/delay.h>

#ifdef DEBUG

#include <stdio.h>
char stringbuffer[100];

#endif

//This is a state machine
volatile enum RFM12_STATE state;

//this var only help to indicate if we have 3 bytes
volatile uint8_t bytecounter;

//pointer to the data we want to send
char *volatile ptxbuf;

//the lenght we want to send
volatile uint16_t txlength;

//Buffer
unsigned char buf[BUFFER_SIZE];

//Random buffer
uint8_t rbuf[RAND_BUFFER_SIZE];

//Buffer state
bufferstate volatile bufstate;

//Random number Buffer state
randombufferstate volatile randombufstate;

/****************************************/
/*	This are essential functions	*/
/****************************************/
static inline void select(void)
{
  //Do not forget to implement select!
  
  PORTB &= ~(1 << PB4);
}

static inline void deselect(void)
{
  //Do not forget to implement deselect!
  
  PORTB |= (1 << PB4);
}

static inline void disableint(void)
{
  //Do not forget to implement disable interrupt
  EIMSK &= ~(1<<INT0);
}

static inline void enableint(void)
{
  //Do not forget do implement enable interrupt
  EIMSK |= (1<<INT0);
}

static inline uint16_t rfm12_writeOp(uint16_t data)
{
  uint16_t temp = 0;
  select();
  
  SPDR = (data>>8);
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  temp = (SPDR<<8);
  
  SPDR = data;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  temp |= SPDR;
  
  deselect();
  
  return temp;
}

static inline void rfm12_writeData(uint8_t data)
{
  select();
  SPDR = ( RFM12_TXWR >> 8 );
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  SPDR = data;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  deselect();
}

static inline uint8_t rfm12_getData(void)
{
  select();
  SPDR = ( RFM12_RXFIFORD >> 8 );
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  SPDR = 0;
  while( ! (  SPSR & ( 1 << SPIF ) ) );
  deselect();
  return SPDR;
}

/****************************************/
/*	This are help functions		*/
/****************************************/

static inline void rfm12_freqset(double freq)
{ 
  rfm12_writeOp( RFM12_FREQSET | (uint16_t)(400*(freq-430)) );
}

/****************************************/
/*	This are high level functions	*/
/****************************************/

ISR(INT0_vect)
{
  uint8_t status = rfm12_writeOp(RFM12_STATUSRD) >> 8;
  static uint16_t length = 0;
  
  
  if( !( status & (RFM12_STATUSRD_RGIT_FFIT>>8) ) )
  {
    goto END;
  }
  
  static uint8_t length_high = 0; 
  static uint8_t length_low = 0;
  
  switch(state)
  {
    case IDLE:
      
      //get first byte - this is the length (high byte)
      length_high = rfm12_getData();
      
      //entering PRE_RX
      state = PRE_RX;
      
      //we have the first byte...
      bytecounter = 1;
      
      break;
      
    case PRE_RX:
      
      switch(bytecounter)
      {
	case 1:
	  
	  //get length (low byte)
	  length_low = rfm12_getData();
	  ++bytecounter;
	  break;
	  
	case 2:
	  //now the third byte arrive - this is the checksum
	  
	  length = length_high << 8;
	  length |= length_low;
	  
	  //get the checksum, calculate it and check if enough space is available in the buffer
	  if(rfm12_getData() == (uint8_t)(length_high + length_low) && ( (bufstate.clearedptr-bufstate.writeptr) & (BUFFER_SIZE-1) ) >= length)
	  {
#ifdef DEBUG
	    //usart_puts("Packet OK\r\n");
#endif
	    
	    buf[bufstate.writeptr++] = length_high;
	    bufstate.writeptr &= (BUFFER_SIZE-1);
	    
	    buf[bufstate.writeptr++] = length_low;
	    bufstate.writeptr &= (BUFFER_SIZE-1);
	    
	    state = RX;
	  }
	
	  else
	  {
	    //stop RX
#ifdef DEBUG
	    usart_puts("BIG\r\n");
#endif
	    //if an error occure restart the fifo trigger an enter state idle
	    rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_DR );
	    rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_FF | RFM12_FIFORSTMODE_DR );
	    
	    state = IDLE;
	  }
	  
	  break;
      }
      
      break;
      
    case RX:
      buf[bufstate.writeptr++] = rfm12_getData();
      bufstate.writeptr &= (BUFFER_SIZE-1);
      
      --length;
      
      if(!length)
      {
	++bufstate.packetcnt;
	//we are finished here - use the next byte to fill random buffer
	state = GET_RANDOM;
	
#ifdef DEBUG
	sprintf(stringbuffer, "nxtpkt: %d\r\n write: %d\r\n clear %d\r\n pkgcnt %d\r\n", bufstate.nextpacketptr, bufstate.writeptr, bufstate.clearedptr, bufstate.packetcnt);
	usart_puts(stringbuffer);
#endif      

      }
      
      break;
      
    case GET_RANDOM:
      
      if( ((randombufstate.writeptr+1) & (RAND_BUFFER_SIZE-1)) != randombufstate.nextbyte )
      {
	rbuf[randombufstate.writeptr++] = rfm12_getData();
	randombufstate.writeptr &= (RAND_BUFFER_SIZE-1);
      }
      
      state = IDLE;
      rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_DR );
      rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_FF | RFM12_FIFORSTMODE_DR );
	
      
      
      break;
      
    case PRE_TX:
      
      switch(bytecounter)
      {
	case 0:
	  //first sync pattern
	  rfm12_writeData(0x2D);
	  ++bytecounter;
	  break;
	case 1:
	  //second sync pattern
	  rfm12_writeData(0xD4);
	  ++bytecounter;
	  break;
	case 2:
	  //high byte length
	  rfm12_writeData(txlength>>8);
	  ++bytecounter;
	  break;
	case 3:
	  //low byte lenght
	  rfm12_writeData(txlength);
	  ++bytecounter;
	  break;
	case 4:
	  //checksum
	  rfm12_writeData( (uint8_t) ((txlength>>8) + (txlength&0xFF)) );
	  state = TX;
	  break;
      }
      break;
      
    case TX:
      rfm12_writeData(*ptxbuf);
      ++ptxbuf;
      --txlength;
      
      if(!txlength)
      {
	bytecounter = 0;
	state = POST_TX;
      }
      
      break;
      
    case POST_TX:
      
      switch(bytecounter)
      {
	case 0:
	  rfm12_writeData(0xAA);
	  ++bytecounter;
	  break;
	
	case 1:
	  //stop TX
	  rfm12_writeOp( RFM12_PWRMGM  | RFM12_PWRMGM_EBB | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC  | RFM12_PWRMGM_ES );
	  
	  state = IDLE;
	  
	  //start RX
	  rfm12_writeOp( RFM12_PWRMGM  | RFM12_PWRMGM_EBB | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC  | RFM12_PWRMGM_ES | RFM12_PWRMGM_ER );
	  
	  //enable FIFO
	  rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_FF | RFM12_FIFORSTMODE_DR );
	  
	  break;
      
      }
      
      break;
      
    
	  
  }
  
  END:
  return;
}

//init rfm12 (in this case a rfm12 with 433Mhz)
unsigned char *rfm12_init()
{ 
  //this is the interrupt pin - set up pullup
  PORTD |= ( 1 << PD2 );
  
  //start init
  rfm12_writeOp( RFM12_CFGSET | RFM12_CFGSET_EL | RFM12_CFGSET_EF  | RFM12_CFGSET_FREQ_433 | 0x08 );
  
  
  
  rfm12_writeOp( RFM12_PWRMGM | RFM12_PWRMGM_EBB | RFM12_PWRMGM_ES | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC );
  
  //115.2 kbps
  rfm12_writeOp( RFM12_DATARATE | 0x0002 );
  
  rfm12_writeOp( RFM12_RXCONTROL | RFM12_RXCONTROL_P20 | RFM12_RXCONTROL_BW_200 );
  
  rfm12_writeOp( RFM12_DATAFILTER | RFM12_DATAFILTER_AL | RFM12_DATAFILTER_ML | 0x0004 );
  
  rfm12_writeOp( RFM12_AFC | RFM12_AFC_INDEPENDENT | RFM12_AFC_DEVIATION_3TO4 | RFM12_AFC_FI | RFM12_AFC_OE | RFM12_AFC_EN);
  
  rfm12_writeOp( RFM12_TXCFG | 0x0070 );
  
  rfm12_writeOp( RFM12_WAKEUPTIMER );
  
  rfm12_writeOp( RFM12_LOWDUTY | 0x0000 );
  
  rfm12_writeOp( RFM12_BATCLOCK | 0x0000 );
  
  rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_DR );
  
  //setting frequency to 433.92 Mhz
  rfm12_writeOp( RFM12_FREQSET | 1568 );
  
  //Set state
  state = IDLE;

  bytecounter =  0;
  
  //enable receiver
  rfm12_writeOp( RFM12_PWRMGM  | RFM12_PWRMGM_EBB | RFM12_PWRMGM_ES | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC  | RFM12_PWRMGM_ER);
  
  //Data buffer init
  bufstate.nextpacketptr = 0;
  bufstate.writeptr = 0;
  bufstate.clearedptr = BUFFER_SIZE - 1;
  bufstate.packetcnt = 0;
  
  
  //Random number buffer init
  randombufstate.writeptr = 0;
  randombufstate.nextbyte = 0;
  
  //enable INT0 - falling edge
  EICRA |= ( 1 << ISC01 );
  
  //Just a dummy status read to clear a possible interrupt on the RFM12
  rfm12_writeOp(RFM12_STATUSRD);
  
  //clear interrupt
  EIFR = (1 << INTF0);
  
  //Enable interrupt
  enableint();
  
  
  //directly enable RX
  rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_FF | RFM12_FIFORSTMODE_DR );
  
  return buf;
}

uint8_t rfm12_tx(char *pbuf, uint16_t length)
{
  if(state != IDLE)
  {
    return 1;
  }
  //Stop the interrupt
  disableint();
  
  if(state != IDLE)
  {
    enableint();
    return 1;
  }
  
  
  
  //stop the fifo trigger
  rfm12_writeOp( RFM12_FIFORSTMODE | 0x0080 | RFM12_FIFORSTMODE_DR );
  
  
  
  //deactivate RX
  rfm12_writeOp( RFM12_PWRMGM  | RFM12_PWRMGM_EBB | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC  | RFM12_PWRMGM_ES );
  
  //write Preamble to buffer
  rfm12_writeData(0xAA);
  rfm12_writeData(0xAA);
  
  //entering PRE_TX
  state = PRE_TX;
  
  //set bytecounter to zero
  bytecounter = 0;
  
  //set pointer
  ptxbuf = pbuf;
  
  //set length
  txlength = length;
  
  //start the transmitter
  rfm12_writeOp( RFM12_PWRMGM  | RFM12_PWRMGM_EBB | RFM12_PWRMGM_ES | RFM12_PWRMGM_EX | RFM12_PWRMGM_DC  | RFM12_PWRMGM_ET);
  
  enableint();
  
  return 0;
}

bool rfm12_haspacket()
{
  return bufstate.packetcnt;
}

uint16_t rfm12_getpacketptr()
{
  return bufstate.nextpacketptr;
}

void rfm12_clearpacket()
{
  uint16_t length;
  if(bufstate.packetcnt)
  {
    length = (buf[bufstate.nextpacketptr] << 8) | (buf[(bufstate.nextpacketptr+1) & (BUFFER_SIZE-1)] & (BUFFER_SIZE-1));
    bufstate.clearedptr = (length+bufstate.nextpacketptr+2-1) & (BUFFER_SIZE-1);
    bufstate.nextpacketptr = (bufstate.clearedptr+1) & (BUFFER_SIZE-1);
    
    disableint();
    --bufstate.packetcnt;
    enableint();
/*
#ifdef DEBUG
    
	sprintf(stringbuffer, "nxtpkt: %d\r\n write: %d\r\n clear %d\r\n pkgcnt %d\r\n", bufstate.nextpacketptr, bufstate.writeptr, bufstate.clearedptr, bufstate.packetcnt);
	usart_puts(stringbuffer);
#endif
*/
  }
}

uint8_t rfm12_getrandomnumber()
{
  uint8_t temp;
  
  disableint();
  
  if(randombufstate.writeptr == randombufstate.nextbyte)
  {
    enableint();
    return 0;
  }
  temp = rbuf[randombufstate.nextbyte++];
  
  enableint();
  
  return temp;
}