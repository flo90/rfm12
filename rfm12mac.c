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

const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};

RFM12_MAC_State_t volatile macstate;
volatile uint16_t length;

bool (*rfm12_mac_nextLayerReceiveCallback)(uint8_t data) = NULL;

void rfm12_mac_init()
{
  macstate = RFM12_MAC_STATE_IDLE;
  rfm12_phy_modeRX();
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
  if(rfm12_phy_busy())
  {
    return false;
  }
  length = plength;
  macstate = RFM12_MAC_STATE_TX_PREAMBLE1;
  rfm12_phy_modeTX();
  return true;
}

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data)
{ 
  //just a cheap checksum to ensure that the length is not corrupted 
  static uint8_t checksum = 0;
  
  switch(macstate)
  {
    case RFM12_MAC_STATE_IDLE:
      macstate = RFM12_MAC_STATE_LENGTH_HIGH;
      
    case RFM12_MAC_STATE_LENGTH_HIGH:
      length = (data<<8);
      checksum = data;
      macstate = RFM12_MAC_STATE_LENGTH_LOW;
      break;

    case RFM12_MAC_STATE_LENGTH_LOW:
      length |= data;
      checksum += data;
      macstate = RFM12_MAC_STATE_CHECKSUM;
      break;
	  
    case RFM12_MAC_STATE_CHECKSUM:
      if(checksum == data)
      {
	macstate = RFM12_MAC_STATE_RXTX;
      }
      
      else
      {
	macstate = RFM12_MAC_STATE_IDLE;
	return false;
      }
      break;
      
    case RFM12_MAC_STATE_RXTX:
      --length;
      
      if(length)
      {
	if(!rfm12_llc_previousLayerReceiveCallback(data, false))
	{
	  macstate = RFM12_MAC_STATE_IDLE;
	  return false;
	}
      }
      
      else
      {
	rfm12_llc_previousLayerReceiveCallback(data, true);
	macstate = RFM12_MAC_STATE_IDLE;
	return false;
      }
      break;
  }
  return true;
}


uint8_t rfm12_mac_previousLayerTransmitCallback(void)
{
  uint8_t temp = 0;
  switch(macstate)
  {
    case RFM12_MAC_STATE_TX_PREAMBLE1:
      
      macstate = RFM12_MAC_STATE_TX_PREAMBLE2;
      temp = _rfm12_mac_sync[0];
      break;
      
    case RFM12_MAC_STATE_TX_PREAMBLE2:
      
      macstate = RFM12_MAC_STATE_LENGTH_HIGH;
      temp = _rfm12_mac_sync[1];
      break;
      
    case RFM12_MAC_STATE_LENGTH_HIGH:
      
      macstate = RFM12_MAC_STATE_LENGTH_LOW;
      temp = (length>>8)&0xFF;
      break;
      
    case RFM12_MAC_STATE_LENGTH_LOW:
      
      macstate = RFM12_MAC_STATE_CHECKSUM;
      temp = length&0xFF;
      break;
      
    case RFM12_MAC_STATE_CHECKSUM:
      macstate = RFM12_MAC_STATE_RXTX;
      temp = (uint8_t)(( (length>>8) & 0xFF ) + ( (length) & 0xFF));
      break;
      
    case RFM12_MAC_STATE_RXTX:
      --length;
      
      if(length)
      {
	
	temp = rfm12_llc_previousLayerTransmitCallback();
      }
      
      else
      {
	macstate = RFM12_MAC_STATE_TX_END;
	//just dummy byte
	temp = 0xAA;
      }
      break;
      
    case RFM12_MAC_STATE_TX_END:
      macstate = RFM12_MAC_STATE_IDLE;
      rfm12_phy_modeRX();
      temp = 0xAA;
      break;
  }
  
  return temp;
}

void rfm12_mac_resetStateMachine(RFM12_Transfer_Error_t err)
{
  macstate = RFM12_MAC_STATE_IDLE;
  rfm12_llc_resetStateMachine(err);
}