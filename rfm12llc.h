#ifndef _RFM12LLC_H_
#define _RFM12LLC_H_

#include <inttypes.h>
#include <stdbool.h>

#include "rfm12.h"

typedef struct RFM12_LLC_Header
{
  uint16_t length;
  uint16_t dstAddr;
  uint16_t srcAddr;
}RFM12_LLC_Header_t;

typedef enum RFM12_LLC_State
{
  RFM12_LLC_STATE_IDLE,
  RFM12_LLC_STATE_DST_LOW,
  RFM12_LLC_STATE_DST_HIGH,
  RFM12_LLC_STATE_SRC_LOW,
  RFM12_LLC_STATE_SRC_HIGH,
  RFM12_LLC_STATE_RXTX
} RFM12_LLC_State_t;

void rfm12_llc_init(bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, bool lastbyte), uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void), uint16_t pownAddr );

bool rfm12_llc_startTX(uint16_t dst, uint16_t length);

bool rfm12_llc_previousLayerReceiveCallback(uint8_t data, bool lastbyte);

void rfm12_llc_resetStateMachine(RFM12_Transfer_Error_t err);

uint8_t rfm12_llc_previousLayerTransmitCallback(void);



#endif