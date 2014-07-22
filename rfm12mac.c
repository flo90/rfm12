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

//#include "rfm12phy.h"
#include "rfm12mac.h"

#include <inttypes.h>
#include <stdbool.h>

#include "rfm12llc.h"
#include "rfm12channel.h"

//----TEST----
#include <avr/io.h>
#include "usart.h"

#define MAC_BROADCAST_ADDR 0x7FFF

const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};

RFM12_MAC_RX_State_t volatile rxmacstate;
RFM12_MAC_TX_State_t volatile txmacstate;
RFM12_MAC_Header_t rxheader;
RFM12_MAC_Header_t txheader;
RFM12_MAC_t macconfig;

volatile uint16_t rxlength;
volatile uint16_t txlength;

bool (*rfm12_mac_nextLayerReceiveCallback)(uint8_t data) = NULL;

void rfm12_mac_init()
{
  rxmacstate = RFM12_MAC_RX_STATE_IDLE;
}

void rfm12_mac_setChannel(uint8_t chan, RFM12_PHY_VDI_t vdi, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh, RFM12_PHY_OutPwr_t pwr)
{
  --chan;
  rfm12_phy_setBaudrate(channel[chan].baud);
  rfm12_phy_setRecvCtrl(false, vdi, channel[chan].bw, lna, rssiDTh);
  rfm12_phy_setTxConf(false, channel[chan].deviation, pwr);
  rfm12_phy_setFrequency(channel[chan].frequency);
}

void rfm12_mac_setAddr(uint16_t addr)
{
  macconfig.ownAddr = addr;
}

void rfm12_mac_setGroup(uint16_t grps)
{
  macconfig.grps = grps & ~RFM12_MAC_GROUP_SIGN;
}

bool rfm12_mac_startTransmission(uint16_t pdst, uint16_t plength)
{
  txlength = plength;
  txheader.dstAddr = pdst;
  txmacstate = RFM12_MAC_TX_STATE_PREAMBLE;
  return rfm12_phy_modeTX();
}

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status)
{ 
  //just a cheap checksum to ensure that the length is not corrupted 
  static uint8_t checksum = 0;
  
  switch(rxmacstate)
  {
    case RFM12_MAC_RX_STATE_IDLE:
      rxmacstate = RFM12_MAC_RX_STATE_LENGTH_HIGH;
    
    case RFM12_MAC_RX_STATE_LENGTH_HIGH:
      rxmacstate = RFM12_MAC_RX_STATE_LENGTH_LOW;
      rxlength = (data<<8);
      checksum = data;
      break;

    case RFM12_MAC_RX_STATE_LENGTH_LOW:
      rxmacstate = RFM12_MAC_RX_STATE_DST_HIGH;
      rxlength |= data&0xFF;
      
      if(rxlength > RFM12_MAX_FRAME_SIZE)
      {
	goto reset;
      }
      checksum += data;
      break;
	  
    case RFM12_MAC_RX_STATE_DST_HIGH:
      rxmacstate = RFM12_MAC_RX_STATE_DST_LOW;
      rxheader.dstAddr = (data<<8);
      checksum += data;
      break;
      
    case RFM12_MAC_RX_STATE_DST_LOW:
      rxmacstate = RFM12_MAC_RX_STATE_SRC_HIGH;
      rxheader.dstAddr |= data&0xFF;
      checksum += data;
      
      if(rxheader.dstAddr & RFM12_MAC_GROUP_SIGN)
      {
	//check if we are in any group
	if(!(rxheader.dstAddr & macconfig.grps))
	{
	  goto reset;
	}
      }
      
      else
      {
	//only check address
	if((rxheader.dstAddr != macconfig.ownAddr) && (rxheader.dstAddr != MAC_BROADCAST_ADDR))
	{
	  goto reset;
	}
      }
      break;
      
    case RFM12_MAC_RX_STATE_SRC_HIGH:
      rxmacstate = RFM12_MAC_RX_STATE_SRC_LOW;
      rxheader.srcAddr = (data<<8);
      checksum += data;
      break;
      
    case RFM12_MAC_RX_STATE_SRC_LOW:
      rxmacstate = RFM12_MAC_RX_STATE_CHECKSUM;
      rxheader.srcAddr |= data&0xFF;
      checksum += data;
      break;
      
    case RFM12_MAC_RX_STATE_CHECKSUM:
      rxmacstate = RFM12_MAC_RX_STATE_PUT_SRC_ADDR;
      if(checksum != data)
      {
	goto reset;
      }
      
     
    case RFM12_MAC_RX_STATE_PUT_SRC_ADDR:
      rxmacstate = RFM12_MAC_RX_STATE_RX;
      if(!rfm12_llc_previousLayerReceiveCallback(rxheader.srcAddr>>8, status) || !rfm12_llc_previousLayerReceiveCallback(rxheader.srcAddr&0xFF, status))
      {
	goto reset;
      }
      break;
      
    case RFM12_MAC_RX_STATE_RX:
      --rxlength;
      
      if(rxlength)
      {
	if(!rfm12_llc_previousLayerReceiveCallback(data, status))
	{
	  goto reset;
	}
      }
      
      else
      {
	rfm12_llc_previousLayerReceiveCallback(data, RFM12_TRANSFER_STATUS_LASTBYTE);
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
  rxmacstate = RFM12_MAC_RX_STATE_IDLE;
  return false;
}


uint8_t rfm12_mac_previousLayerTransmitCallback(void)
{
  uint8_t temp = 0x00;
  
  switch(txmacstate)
  {
    case RFM12_MAC_TX_STATE_PREAMBLE:
      txmacstate = RFM12_MAC_TX_STATE_SYNC0;
      temp = 0xAA;
      break;
    
    case RFM12_MAC_TX_STATE_SYNC0:
      txmacstate = RFM12_MAC_TX_STATE_SYNC1;
      temp = _rfm12_mac_sync[0];
      break;
      
    case RFM12_MAC_TX_STATE_SYNC1:
      txmacstate = RFM12_MAC_TX_STATE_LENGTH_HIGH;
      temp = _rfm12_mac_sync[1];
      break;
      
      
    case RFM12_MAC_TX_STATE_LENGTH_HIGH:
      txmacstate = RFM12_MAC_TX_STATE_LENGTH_LOW;
      temp = (txlength>>8)&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_LENGTH_LOW:
      txmacstate = RFM12_MAC_TX_STATE_DST_HIGH;
      temp = txlength&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_DST_HIGH:
      txmacstate = RFM12_MAC_TX_STATE_DST_LOW;
      temp = (txheader.dstAddr>>8)&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_DST_LOW:
      txmacstate = RFM12_MAC_TX_STATE_SRC_HIGH;
      temp = txheader.dstAddr&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_SRC_HIGH:
      txmacstate = RFM12_MAC_TX_STATE_SRC_LOW;
      temp = (macconfig.ownAddr>>8)&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_SRC_LOW:
      txmacstate = RFM12_MAC_TX_STATE_CHECKSUM;
      temp = macconfig.ownAddr&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_CHECKSUM:
      txmacstate = RFM12_MAC_TX_STATE_TX;
      temp = (txlength>>8)&0xFF;
      temp += txlength&0xFF;
      
      temp += (txheader.dstAddr>>8)&0xFF;
      temp += txheader.dstAddr&0xFF;
      
      temp += (macconfig.ownAddr>>8)&0xFF;
      temp += macconfig.ownAddr&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_TX:
      --txlength;
      
      if(txlength)
      {
	temp = rfm12_llc_previousLayerTransmitCallback();
      }
      
      else
      {
	txmacstate = RFM12_MAC_TX_STATE_END;
	//just dummy byte
	temp = 0xAA;
      }
      break;
      
    case RFM12_MAC_TX_STATE_END:
      //set to receive mode
      rfm12_phy_modeRX();
      temp = 0xAA;
      break;
  }
  return temp;
}