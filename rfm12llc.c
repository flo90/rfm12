#include "rfm12llc.h"

#include <stdbool.h>

RFM12_LLC_State_t llcstate;
uint16_t DstAddr;

bool rfm12_llc_previousLayerReceiveCallback(uint8_t data)
{
  static RFM12_LLC_Header_t headerRX;
  switch(llcstate)
  {
    case RFM12_LLC_STATE_IDLE:
      llcstate = RFM12_LLC_STATE_RX_LENGTH_HIGH;
      
    case RFM12_LLC_STATE_RX_LENGTH_HIGH:
      headerRX.length = (data<<8);
      llcstate = RFM12_LLC_STATE_RX_LENGTH_LOW;
      break;
      
    case RFM12_LLC_STATE_RX_LENGTH_LOW:
      headerRX.length |= data;
      llcstate = RFM12_LLC_STATE_DST_HIGH;
      break;
      
    case RFM12_LLC_STATE_DST_HIGH:
      headerRX.DstAddr = (data<<8);
      llcstate = RFM12_LLC_STATE_DST_LOW;
      break;
      
    case RFM12_LLC_STATE_DST_LOW:
      headerRX.DstAddr |= data;
      llcstate = RFM12_LLC_STATE_SRC_HIGH;
	
      if(headerRX.DstAddr != DstAddr)
      {
	llcstate = RFM12_LLC_STATE_IDLE;
	return false;
      }
      break;
      
    case RFM12_LLC_STATE_SRC_HIGH:
      headerRX.SrcAddr = (data<<8);
      llcstate = RFM12_LLC_STATE_SRC_LOW;
      break;
      
    case RFM12_LLC_STATE_SRC_LOW:
      headerRX.SrcAddr = data;
      llcstate = RFM12_LLC_STATE_RX;
      break;
	
    case RFM12_LLC_STATE_RX:
      //call next layer
      break;
  }
  
  return true;
}

void rfm12_llc_resetStateMachine(RFM12_Transfer_Error_t err)
{
  llcstate = RFM12_LLC_STATE_IDLE;
}