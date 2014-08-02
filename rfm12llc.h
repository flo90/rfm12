#ifndef _RFM12LLC_H_
#define _RFM12LLC_H_

#include <inttypes.h>
#include <stdbool.h>

#include "rfm12.h"
#include "rfm12macbuf.h"

#define RFM12_LLC_GROUP_SIGN (1<<15)

typedef struct RFM12_LLC_Header
{
  RFM12_MAC_Frame_t *pframe;
  uint8_t service;
  uint16_t length;
  uint16_t dstAddr;
  uint16_t srcAddr;
}RFM12_LLC_Header_t;

typedef enum RFM12_LLC_RX_State
{
  RFM12_LLC_RX_STATE_IDLE,
  RFM12_LLC_RX_STATE_SRC_LOW,
  RFM12_LLC_RX_STATE_SRC_HIGH,
  RFM12_LLC_RX_STATE_SERVICE,
  RFM12_LLC_RX_STATE_PUT_SRC_ADDR,
  RFM12_LLC_RX_STATE_RX
} RFM12_LLC_RX_State_t;

typedef enum RFM12_LLC_TX_State
{
  RFM12_LLC_TX_STATE_SERVICE,
  RFM12_LLC_TX_STATE_TX
} RFM12_LLC_TX_State_t;

void rfm12_llc_init(uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void));

#ifdef RFM12_MAC_USEBUFFER
void rfm12_llc_taskHandler(void);
uint8_t *rfm12_llc_getSpace(uint16_t size);
#endif

#ifdef RFM12_MAC_USEBUFFER
bool rfm12_llc_startTX(uint16_t dstAddr, uint8_t service);
#else
bool rfm12_llc_startTX(uint16_t dst, uint8_t service, uint16_t length);
#endif
bool rfm12_llc_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status);

uint8_t rfm12_llc_previousLayerTransmitCallback(void);

#ifdef RFM12_MAC_USEBUFFER
void rfm12_llc_registerProto(uint8_t slot, void (*prfm12_llc_nextLayerReceiveCallback)(RFM12_MAC_Frame_t *pframe));
#else
void rfm12_llc_registerProto(uint8_t slot, bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, RFM12_Transfer_Status_t status));
#endif
#endif