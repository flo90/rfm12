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
#include "rfm12macbuf.h"

//----TEST----
#include <avr/io.h>
#include "usart.h"
#include <stdio.h>

#define MAC_BROADCAST_ADDR 0x7FFF
static char string[20];
const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};

RFM12_MAC_RX_State_t volatile rxmacstate;
RFM12_MAC_TX_State_t volatile txmacstate;
RFM12_MAC_Header_t txheader;
RFM12_MAC_t macconfig;

#ifdef RFM12_MAC_USEBUFFER
static RFM12_MAC_TX_FRAME_t *ptxframe = NULL;
#endif

volatile uint16_t txlength;
volatile uint8_t *txdata;
bool (*rfm12_mac_nextLayerReceiveCallback)(uint8_t data) = NULL;

void rfm12_mac_init()
{
#ifdef RFM12_MAC_USEBUFFER
  rfm12_mac_buf_init();
#endif
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
#ifdef RFM12_MAC_USEBUFFER
bool rfm12_mac_startTransmission(RFM12_MAC_TX_FRAME_t *pframe)
{
  ptxframe = pframe;
  txlength = ptxframe->length;
  txdata = ptxframe->data;
  txmacstate = RFM12_MAC_TX_STATE_PREAMBLE;
  return rfm12_phy_modeTX();
}
#else
bool rfm12_mac_startTransmission(uint16_t pdst, uint16_t plength)
{
  txlength = plength;
  txheader.dstAddr = pdst;
  txmacstate = RFM12_MAC_TX_STATE_PREAMBLE;
  return rfm12_phy_modeTX();
}
#endif

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status)
{ 
  //just a cheap checksum to ensure that the length is not corrupted 
  static uint8_t checksum = 0;
  static RFM12_MAC_Frame_t volatile *pframe;
  static RFM12_MAC_Header_t rxheader;
  static volatile uint8_t *pdata;
  
  switch(rxmacstate)
  {
    case RFM12_MAC_RX_STATE_IDLE:
      rxmacstate = RFM12_MAC_RX_STATE_LENGTH_HIGH;
    
    case RFM12_MAC_RX_STATE_LENGTH_HIGH:
      rxmacstate = RFM12_MAC_RX_STATE_LENGTH_LOW;
      rxheader.length = (data<<8);
      checksum = data;
      break;

    case RFM12_MAC_RX_STATE_LENGTH_LOW:
      rxmacstate = RFM12_MAC_RX_STATE_DST_HIGH;
      rxheader.length |= data&0xFF;
      
      if(rxheader.length > RFM12_MAX_FRAME_SIZE)
      {
	goto reset;
      }
      /*
      snprintf(string, sizeof(string), "RX Size: %u", rxheader.length);
      usart_puts_nonblock(string);
      */
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
#ifdef RFM12_MAC_USEBUFFER
      if((pframe = rfm12_mac_buf_reqSpace(rxheader.length)) == NULL)
      {
	goto reset;
      }
      pframe->header.dstAddr = rxheader.dstAddr;
      pframe->header.srcAddr = rxheader.srcAddr;
      pframe->header.length = rxheader.length;
      pdata = pframe->data;
#else
      if(!rfm12_llc_previousLayerReceiveCallback(rxheader.srcAddr>>8, status) || !rfm12_llc_previousLayerReceiveCallback(rxheader.srcAddr&0xFF, status))
      {
	goto reset;
      }
#endif
      break;
    case RFM12_MAC_RX_STATE_RX:
      //usart_putc_nonblock(data);
      //--rxheader.length;
#ifdef RFM12_MAC_USEBUFFER
      *(pdata++) = data;
#endif    
     
      
      if(0 < --rxheader.length)
      {
#ifndef RFM12_MAC_USEBUFFER
	if(!rfm12_llc_previousLayerReceiveCallback(data, status))
	{
	  goto reset;
	}
#endif
      }
      
      else
      {
#ifdef RFM12_MAC_USEBUFFER
	pframe->finished = true;
#else
	rfm12_llc_previousLayerReceiveCallback(data, RFM12_TRANSFER_STATUS_LASTBYTE);
#endif
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
#ifdef RFM12_MAC_USEBUFFER
      temp = ((ptxframe->length)>>8)&0xFF;
#else
      temp = (txlength>>8)&0xFF;
#endif
      break;
      
    case RFM12_MAC_TX_STATE_LENGTH_LOW:
      txmacstate = RFM12_MAC_TX_STATE_DST_HIGH;
#ifdef RFM12_MAC_USEBUFFER
      temp = (ptxframe->length)&0xFF;
#else
      temp = txlength&0xFF;
#endif
      break;
      
    case RFM12_MAC_TX_STATE_DST_HIGH:
      txmacstate = RFM12_MAC_TX_STATE_DST_LOW;
#ifdef RFM12_MAC_USEBUFFER
      temp = ((ptxframe->dstAddr)>>8)&0xFF;
#else
      temp = (txheader.dstAddr>>8)&0xFF;
#endif
      break;
      
    case RFM12_MAC_TX_STATE_DST_LOW:
      txmacstate = RFM12_MAC_TX_STATE_SRC_HIGH;
#ifdef RFM12_MAC_USEBUFFER
      temp = (ptxframe->dstAddr)&0xFF;
#else
      temp = txheader.dstAddr&0xFF;
#endif
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
#ifdef RFM12_MAC_USEBUFFER
      temp = ((ptxframe->length)>>8)&0xFF;
      temp += (ptxframe->length)&0xFF;
      
      temp += ((ptxframe->dstAddr)>>8)&0xFF;
      temp += (ptxframe->dstAddr)&0xFF;
#else
      temp = (txlength>>8)&0xFF;
      temp += txlength&0xFF;
      
      temp += (txheader.dstAddr>>8)&0xFF;
      temp += txheader.dstAddr&0xFF;
#endif
      temp += (macconfig.ownAddr>>8)&0xFF;
      temp += macconfig.ownAddr&0xFF;
      
      //txlength++;
      break;
      
    case RFM12_MAC_TX_STATE_TX:
      
      if(0 < txlength--)
      {
#ifdef RFM12_MAC_USEBUFFER
	temp = *txdata++;
#else
	temp = rfm12_llc_previousLayerTransmitCallback();
#endif
      }
      
      else
      {
	txmacstate = RFM12_MAC_TX_STATE_END;
	//just dummy byte
	temp = 0xAA;
      }
      break;
      
    case RFM12_MAC_TX_STATE_END:
#ifdef RFM12_MAC_USEBUFFER
      ptxframe = NULL;
#endif
      //set to receive mode
      rfm12_phy_modeRX();
      
      //again just a dummy byte
      temp = 0xAA;
      break;
  }
  return temp;
}