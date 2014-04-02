/*
    This file is part of the rfm12 driver project.
    Copyright (C) 2014  Florian Menne (florianmenne@t-online.de)

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

#include "rfm12llc.h"

#include <stdbool.h>

#include "rfm12mac.h"

//--TEST--
#include <avr/io.h>

char buf[10];
char *bufptr = buf;
char compare[] = "Haalo";

RFM12_LLC_State_t volatile llcstate;
uint16_t ownAddr;
RFM12_LLC_Header_t volatile rxheader;
RFM12_LLC_Header_t volatile txheader;

uint8_t (*rfm12_llc_nextLayerTransmitCallback)(void);
bool (*rfm12_llc_nextLayerReceiveCallback)(uint8_t data, bool lastbyte);

void rfm12_llc_init(bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, bool lastbyte), uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void), uint16_t pownAddr )
{
  ownAddr = pownAddr;
  llcstate = RFM12_LLC_STATE_IDLE;
  
  rfm12_llc_nextLayerReceiveCallback = prfm12_llc_nextLayerReceiveCallback;
  rfm12_llc_nextLayerTransmitCallback = prfm12_llc_nextLayerTransmitCallback;
}

bool rfm12_llc_startTX(uint16_t dst, uint16_t length)
{
  txheader.dstAddr = dst;
  txheader.srcAddr = ownAddr;
  txheader.length = length;
  if(!rfm12_mac_startTransmission(txheader.length+4))
  {
    return false;
  }
  llcstate = RFM12_LLC_STATE_DST_HIGH;
  return true;
}

bool rfm12_llc_previousLayerReceiveCallback(uint8_t data, bool lastbyte)
{ 
  uint8_t i;
  if(lastbyte && (llcstate != RFM12_LLC_STATE_RXTX))
  {
    return false;
  }
  
  switch(llcstate)
  {
    case RFM12_LLC_STATE_IDLE:
      llcstate = RFM12_LLC_STATE_DST_HIGH;
      
    case RFM12_LLC_STATE_DST_HIGH:
      rxheader.dstAddr = (data<<8);
      llcstate = RFM12_LLC_STATE_DST_LOW;
      break;
      
    case RFM12_LLC_STATE_DST_LOW:
      rxheader.dstAddr |= data;
      llcstate = RFM12_LLC_STATE_SRC_HIGH;
	
      if(rxheader.dstAddr != ownAddr)
      {
	
	llcstate = RFM12_LLC_STATE_IDLE;
	return false;
      }
      break;
      
    case RFM12_LLC_STATE_SRC_HIGH:
      rxheader.srcAddr = (data<<8);
      llcstate = RFM12_LLC_STATE_SRC_LOW;
      break;
      
    case RFM12_LLC_STATE_SRC_LOW:
      rxheader.srcAddr = data;
      llcstate = RFM12_LLC_STATE_RXTX;
      break;
	
    case RFM12_LLC_STATE_RXTX:
      if(!lastbyte)
      {
	*bufptr++ = data;
	PORTD |= (1<<PD5);
      }
      else
      {
	//Call next layer
	*bufptr++ = data;
	for(i = 0; i < 5; i++)
	{
	  if(buf[i] != compare[i])
	  {
	    PORTD &= ~(1<<PD5);
	  }
	}
	//tell next layer that this is the last byte
	bufptr = buf;
	llcstate = RFM12_LLC_STATE_IDLE;
      }
      break;
  }
  
  return true;
}

uint8_t rfm12_llc_previousLayerTransmitCallback()
{
  
  switch(llcstate)
  {
    case RFM12_LLC_STATE_DST_HIGH:
      llcstate = RFM12_LLC_STATE_DST_LOW;
      return (txheader.dstAddr>>8)&0xFF;
      
    case RFM12_LLC_STATE_DST_LOW:
      llcstate = RFM12_LLC_STATE_SRC_HIGH;
      return txheader.dstAddr&0xFF;
      
    case RFM12_LLC_STATE_SRC_HIGH:
      llcstate = RFM12_LLC_STATE_SRC_LOW;
      return (txheader.srcAddr>>8)&0xFF;
      
    case RFM12_LLC_STATE_SRC_LOW:
      llcstate = RFM12_LLC_STATE_RXTX;
      return txheader.srcAddr&0xFF;
      
    case RFM12_LLC_STATE_RXTX:
      return rfm12_llc_nextLayerTransmitCallback();
      break;
  }
  return 0;
}

void rfm12_llc_resetStateMachine(RFM12_Transfer_Error_t err)
{
  llcstate = RFM12_LLC_STATE_IDLE;
}