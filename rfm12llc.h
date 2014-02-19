#ifndef _RFM12LLC_H_
#define _RFM12LLC_H_

#include <inttypes.h>

#include "rfm12.h"

typedef struct RFM12_LLC_Header
{
  uint16_t length;
  uint16_t DstAddr;
  uint16_t SrcAddr;
}RFM12_LLC_Header_t;

typedef enum RFM12_LLC_State
{
  RFM12_LLC_STATE_IDLE,
  RFM12_LLC_STATE_RX_LENGTH_LOW,
  RFM12_LLC_STATE_RX_LENGTH_HIGH,
  RFM12_LLC_STATE_DST_LOW,
  RFM12_LLC_STATE_DST_HIGH,
  RFM12_LLC_STATE_SRC_LOW,
  RFM12_LLC_STATE_SRC_HIGH,
  RFM12_LLC_STATE_RX
} RFM12_LLC_State_t;


bool rfm12_llc_previousLayerReceiveCallback(uint8_t data);

void rfm12_llc_resetStateMachine(RFM12_Transfer_Error_t err);

#endif