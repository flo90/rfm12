#include "rfm12mac.h"

#include <inttypes.h>
#include <stdbool.h>

const uint8_t _rfm12_mac_sync[] = {0x2D, 0xD4};
RFM12_MAC_State_t macstate;

bool (*rfm12_mac_nextLayerReceiveCallback)(uint8_t data) = NULL;

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data)
{
  static uint16_t length = 0;
  
  switch(macstate)
  {
    case RFM12_MAC_STATE_IDLE:
      macstate = RFM12_MAC_STATE_RX_LENGTH_HIGH;
      
    case RFM12_MAC_STATE_RX_LENGTH_HIGH:
      length = (data<<8);
      macstate = RFM12_MAC_STATE_RX_LENGTH_LOW;
      break;
      
    case RFM12_MAC_STATE_RX_LENGTH_LOW:
      length |= data;
      macstate = RFM12_MAC_STATE_RX;
      break;
      
    case RFM12_MAC_STATE_RX:
      if(length)
      {
	bool *rfm12_mac_nextLayerReceiveCallback(data);
	--length;
      }
      
      break;
  }
}

