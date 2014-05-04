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

#include <avr/io.h>

//---TEST---
#include "usart.h"

//state machine
RFM12_LLC_RX_State_t volatile rxllcstate;
RFM12_LLC_TX_State_t volatile txllcstate;

RFM12_LLC_Header_t volatile rxheader;
RFM12_LLC_Header_t volatile txheader;

//the addr of the System
uint16_t volatile ownAddr;

//interface to next layer
uint8_t (*rfm12_llc_nextLayerTransmitCallback)(void);
bool (*rfm12_llc_nextLayerReceiveCallback[10])(uint8_t data, RFM12_Transfer_Status_t status);

//init
void rfm12_llc_init(bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, RFM12_Transfer_Status_t status), uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void), uint16_t pownAddr )
{
  ownAddr = pownAddr;
  rxllcstate = RFM12_LLC_RX_STATE_IDLE;
  
  
  rfm12_llc_nextLayerTransmitCallback = prfm12_llc_nextLayerTransmitCallback;
}

//its possible to change the addr with this
void rfm12_llc_setAddr(uint16_t addr)
{
  ownAddr = addr;
}

bool rfm12_llc_startTX(uint16_t dst, uint8_t service, uint16_t length)
{
  txheader.dstAddr = dst;
  txheader.srcAddr = ownAddr;
  txheader.length = length;
  txheader.service = service;
  txllcstate = RFM12_LLC_TX_STATE_DST_HIGH;
  
  return rfm12_mac_startTransmission(txheader.length+5);
}

bool rfm12_llc_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status)
{
  switch(rxllcstate)
  {
    case RFM12_LLC_RX_STATE_IDLE:
      rxllcstate = RFM12_LLC_RX_STATE_DST_HIGH;
      
    case RFM12_LLC_RX_STATE_DST_HIGH:
      rxheader.dstAddr = (data<<8);
      rxllcstate = RFM12_LLC_RX_STATE_DST_LOW;
      break;
      
    case RFM12_LLC_RX_STATE_DST_LOW:
      rxheader.dstAddr |= data;
      rxllcstate = RFM12_LLC_RX_STATE_SRC_HIGH;
      
      if(rxheader.dstAddr != ownAddr)
      {
	rxllcstate = RFM12_LLC_RX_STATE_IDLE;
	goto reset;
      }
      break;
      
    case RFM12_LLC_RX_STATE_SRC_HIGH:
      rxheader.srcAddr = (data<<8);
      rxllcstate = RFM12_LLC_RX_STATE_SRC_LOW;
      break;
      
    case RFM12_LLC_RX_STATE_SRC_LOW:
      rxheader.srcAddr |= data;
      rxllcstate = RFM12_LLC_RX_STATE_SERVICE;
      break;
        
    case RFM12_LLC_RX_STATE_SERVICE:
      rxllcstate = RFM12_LLC_RX_STATE_PUT_SRC_ADDR;
      rxheader.service = data;
      break;
      
    case RFM12_LLC_RX_STATE_PUT_SRC_ADDR:
      rxllcstate = RFM12_LLC_RX_STATE_RX;
      if(!rfm12_llc_nextLayerReceiveCallback[0](rxheader.srcAddr>>8, status) || !rfm12_llc_nextLayerReceiveCallback[0](rxheader.srcAddr&0xFF, status))
      {
	usart_putc('E');
	goto reset;
      }
      
    case RFM12_LLC_RX_STATE_RX:
      if(status != RFM12_TRANSFER_STATUS_LASTBYTE)
      {
	if(!rfm12_llc_nextLayerReceiveCallback[0](data, status))
	{
	  usart_putc('F');
	  goto reset;
	}
      }
      
      else
      {
	//this was the last byte - clean up
	rfm12_llc_nextLayerReceiveCallback[0](data, status);
	goto reset;
      }
      break;
  }
  
  if(status != RFM12_TRANSFER_STATUS_CONTINUE)
  {
    goto reset;
  }
  
  return true;
  
  reset:
  rxllcstate = RFM12_LLC_RX_STATE_IDLE;
  return false;
}

uint8_t rfm12_llc_previousLayerTransmitCallback()
{
  uint8_t temp = 0;
  switch(txllcstate)
  {
    case RFM12_LLC_TX_STATE_DST_HIGH:
      txllcstate = RFM12_LLC_TX_STATE_DST_LOW;
      temp = (txheader.dstAddr>>8)&0xFF;
      break;
      
    case RFM12_LLC_TX_STATE_DST_LOW:
      txllcstate = RFM12_LLC_TX_STATE_SRC_HIGH;
      temp = txheader.dstAddr&0xFF;
      break;
      
    case RFM12_LLC_TX_STATE_SRC_HIGH:
      txllcstate = RFM12_LLC_TX_STATE_SRC_LOW;
      temp = (txheader.srcAddr>>8)&0xFF;
      break;
      
    case RFM12_LLC_TX_STATE_SRC_LOW:
      txllcstate = RFM12_LLC_TX_STATE_SERVICE;
      temp = txheader.srcAddr&0xFF;
      break;
      
    case RFM12_LLC_TX_STATE_SERVICE:
      txllcstate = RFM12_LLC_TX_STATE_TX;
      temp = txheader.service;
      break;
      
    case RFM12_LLC_TX_STATE_TX:
      temp = rfm12_llc_nextLayerTransmitCallback();
      break;
  }
  return temp;
}

void rfm12_llc_registerProto(uint8_t slot, bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, RFM12_Transfer_Status_t status))
{
  rfm12_llc_nextLayerReceiveCallback[slot] = prfm12_llc_nextLayerReceiveCallback;
}