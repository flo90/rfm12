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

#define LLC_BROADCAST_ADDR 0x7FFF
#define LLC_SERVICE_COUNT 10

//state machine
RFM12_LLC_RX_State_t volatile rxllcstate;
RFM12_LLC_TX_State_t volatile txllcstate;

RFM12_LLC_Header_t volatile rxheader;
RFM12_LLC_Header_t volatile txheader;

//interface to next layer
uint8_t (*rfm12_llc_nextLayerTransmitCallback)(void);
bool (*rfm12_llc_nextLayerReceiveCallback[LLC_SERVICE_COUNT])(uint8_t data, RFM12_Transfer_Status_t status);

//init
void rfm12_llc_init(uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void))
{
  rxllcstate = RFM12_LLC_RX_STATE_IDLE;
  
  for(uint16_t i = 0; i< LLC_SERVICE_COUNT; i++)
  {
    rfm12_llc_nextLayerReceiveCallback[i] = NULL;
  }
  
  rfm12_llc_nextLayerTransmitCallback = prfm12_llc_nextLayerTransmitCallback;
}

bool rfm12_llc_startTX(uint16_t dst, uint8_t service, uint16_t length)
{
  txheader.service = service;
  txheader.length = length;
  txllcstate = RFM12_LLC_TX_STATE_SERVICE;
  
  return rfm12_mac_startTransmission(dst, length+1);
}

bool rfm12_llc_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status)
{
  static uint8_t srcAddr[2];
  switch(rxllcstate)
  {
    case RFM12_LLC_RX_STATE_IDLE:
      rxllcstate = RFM12_LLC_RX_STATE_SRC_HIGH;
      
      
    case RFM12_LLC_RX_STATE_SRC_HIGH:
      rxllcstate = RFM12_LLC_RX_STATE_SRC_LOW;
      srcAddr[0] = data;
      break;
      
    case RFM12_LLC_RX_STATE_SRC_LOW:
      rxllcstate = RFM12_LLC_RX_STATE_SERVICE;
      srcAddr[1] = data;
      break;
    
    case RFM12_LLC_RX_STATE_SERVICE:
      rxllcstate = RFM12_LLC_RX_STATE_PUT_SRC_ADDR;
      rxheader.service = data;
      if(!rfm12_llc_nextLayerReceiveCallback[0] || (rxheader.service >= LLC_SERVICE_COUNT))
      {
	goto reset;
      }
      break;
      
    case RFM12_LLC_RX_STATE_PUT_SRC_ADDR:
      rxllcstate = RFM12_LLC_RX_STATE_RX;
      if(!rfm12_llc_nextLayerReceiveCallback[rxheader.service](srcAddr[0], status) || !rfm12_llc_nextLayerReceiveCallback[rxheader.service](srcAddr[1], status))
      {
	goto reset;
      }
      
    case RFM12_LLC_RX_STATE_RX:
      if(status != RFM12_TRANSFER_STATUS_LASTBYTE)
      {
	if(!rfm12_llc_nextLayerReceiveCallback[rxheader.service](data, status))
	{
	  goto reset;
	}
      }
      
      else
      {
	//this was the last byte - clean up
	rfm12_llc_nextLayerReceiveCallback[rxheader.service](data, status);
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