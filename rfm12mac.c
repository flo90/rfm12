//#include "rfm12phy.h"
#include "rfm12mac.h"

#include <inttypes.h>
#include <stdbool.h>

#include "rfm12llc.h"

const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};

RFM12_MAC_State_t macstate;
uint16_t length;

bool (*rfm12_mac_nextLayerReceiveCallback)(uint8_t data) = NULL;

void rfm12_mac_setChannel(RFM12_MAC_Channel_t channel, RFM12_PHY_VDI_t vdi, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh, RFM12_PHY_OutPwr_t pwr)
{
  rfm12_phy_setBaudrate(channel.baud);
  rfm12_phy_setRecvCtrl(false, vdi, channel.bw, lna, rssiDTh);
  rfm12_phy_setTxConf(false, channel.deviation, pwr);
}

void rfm12_mac_startTransmission(void)
{
  while(rfm12_phy_busy());
  rfm12_phy_modeTX();
}

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data)
{ 
  //just a cheap checksum to ensure that the length is not corrupted 
  static uint8_t checksum = 0;
  switch(macstate)
  {
    case RFM12_MAC_STATE_IDLE:
      macstate = RFM12_MAC_STATE_RX_LENGTH_HIGH;
      
    case RFM12_MAC_STATE_RX_LENGTH_HIGH:
      length = (data<<8);
      checksum = data;
      macstate = RFM12_MAC_STATE_RX_LENGTH_LOW;
      break;

    case RFM12_MAC_STATE_RX_LENGTH_LOW:
      length |= data;
      checksum += data;
      macstate = RFM12_MAC_STATE_CHECKSUM;
      break;
	  
    case RFM12_MAC_STATE_CHECKSUM:
      if(checksum == data)
      {
	macstate = RFM12_MAC_STATE_RX;
      }
      
      else
      {
	macstate = RFM12_MAC_STATE_IDLE;
	return false;
      }
      
    case RFM12_MAC_STATE_RX:
      --length;
      
      if(length)
      {
	//Call LLC
      }
      else
      {
	//tell LLC to stop
	macstate = RFM12_MAC_STATE_IDLE;
	return false;
      }
      break;
  }
  return true;
}


uint8_t rfm12_mac_previousLayerTransmitCallback(void)
{
  switch(macstate)
  {
    case RFM12_MAC_STATE_TX_PREAMBLE1:
      return _rfm12_mac_sync[0];
      
    case RFM12_MAC_STATE_TX_PREAMBLE2:
      return _rfm12_mac_sync[1];
      
    case RFM12_MAC_STATE_TX_LENGTH_HIGH:
      return length>>8;
      
    case RFM12_MAC_STATE_TX_LENGTH_LOW:
      return length&0xFF;
      
    case RFM12_MAC_STATE_TX:
      --length;
      if(length)
      {
	//TODO: return with llc function
	return 0x00;
      }
      else
      {
	macstate = RFM12_MAC_STATE_TX_END;
	//just dummy byte
	return 0xAA;
      }
    case RFM12_MAC_STATE_TX_END:
      rfm12_phy_modeRX();
      return 0xAA;
  }
}

void rfm12_mac_resetStateMachine(RFM12_Transfer_Error_t err)
{
  macstate = RFM12_MAC_STATE_IDLE;
  rfm12_llc_resetStateMachine(err);
}