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

const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};

RFM12_MAC_RX_State_t volatile rxmacstate;
RFM12_MAC_TX_State_t volatile txmacstate;
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

bool rfm12_mac_startTransmission(uint16_t plength)
{
  txlength = plength;
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
      rxlength = (data<<8);
      checksum = data;
      rxmacstate = RFM12_MAC_RX_STATE_LENGTH_LOW;
      break;

    case RFM12_MAC_RX_STATE_LENGTH_LOW:
      rxlength |= data&0xFF;
      
      if(rxlength > RFM12_MAX_FRAME_SIZE)
      {
	goto reset;
      }
      
      checksum += data;
      rxmacstate = RFM12_MAC_RX_STATE_CHECKSUM;
      break;
	  
    case RFM12_MAC_RX_STATE_CHECKSUM:
      if(checksum == data)
      {
	rxmacstate = RFM12_MAC_RX_STATE_RX;
      }
      
      else
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
      
      txmacstate = RFM12_MAC_TX_STATE_CHECKSUM;
      temp = txlength&0xFF;
      break;
      
    case RFM12_MAC_TX_STATE_CHECKSUM:
      txmacstate = RFM12_MAC_TX_STATE_TX;
      temp = (txlength>>8)&0xFF;
      temp += txlength&0xFF;
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
      rfm12_phy_modeRX();
      temp = 0xAA;
      break;
  }
  return temp;
}