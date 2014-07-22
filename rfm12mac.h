#ifndef _RFM12MAC_H_
#define _RFM12MAC_H_

#include "rfm12phy.h"

#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

#include "rfm12.h"

#define RFM12_MAC_GROUP_SIGN (1<<15)

typedef enum RFM12_MAC_RX_State
{
  RFM12_MAC_RX_STATE_IDLE,
  RFM12_MAC_RX_STATE_LENGTH_LOW,
  RFM12_MAC_RX_STATE_LENGTH_HIGH,
  RFM12_MAC_RX_STATE_DST_LOW,
  RFM12_MAC_RX_STATE_DST_HIGH,
  RFM12_MAC_RX_STATE_SRC_LOW,
  RFM12_MAC_RX_STATE_SRC_HIGH,
  RFM12_MAC_RX_STATE_CHECKSUM,
  RFM12_MAC_RX_STATE_PUT_SRC_ADDR,
  RFM12_MAC_RX_STATE_RX,
} RFM12_MAC_RX_State_t;

typedef enum RFM12_MAC_TX_State
{
  RFM12_MAC_TX_STATE_PREAMBLE,
  RFM12_MAC_TX_STATE_LENGTH_LOW,
  RFM12_MAC_TX_STATE_LENGTH_HIGH,
  RFM12_MAC_TX_STATE_DST_LOW,
  RFM12_MAC_TX_STATE_DST_HIGH,
  RFM12_MAC_TX_STATE_SRC_LOW,
  RFM12_MAC_TX_STATE_SRC_HIGH,
  RFM12_MAC_TX_STATE_CHECKSUM,
  RFM12_MAC_TX_STATE_TX,
  RFM12_MAC_TX_STATE_SYNC0,
  RFM12_MAC_TX_STATE_SYNC1,
  RFM12_MAC_TX_STATE_END
} RFM12_MAC_TX_State_t;

typedef struct RFM12_MAC_Channel
{
  RFM12_PHY_BAUDRATE_t baud;
  RFM12_PHY_BW_t bw;
  RFM12_PHY_FREQDEVIATION_t deviation;
  uint16_t frequency;
} RFM12_MAC_Channel_t;

typedef struct RFM12_MAC_Header
{
  uint16_t length;
  uint16_t dstAddr;
  uint16_t srcAddr;
}RFM12_MAC_Header_t;

typedef struct RFM12_MAC
{
  uint16_t ownAddr;
  uint16_t grps;
}RFM12_MAC_t;

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data, RFM12_Transfer_Status_t status);
uint8_t rfm12_mac_previousLayerTransmitCallback(void);

void rfm12_mac_init(void);

void rfm12_mac_setChannel(uint8_t chan, RFM12_PHY_VDI_t vdi, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh, RFM12_PHY_OutPwr_t pwr);

void rfm12_mac_setAddr(uint16_t addr);

void rfm12_mac_setGroup(uint16_t grps);

bool rfm12_mac_mediaBusy(void);

bool rfm12_mac_startTransmission(uint16_t pdst, uint16_t length);

void rfm12_mac_stopTransmission(void);

#endif