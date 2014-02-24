#ifndef _RFM12MAC_H_
#define _RFM12MAC_H_

#include "rfm12phy.h"

#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

#include "rfm12.h"

typedef enum RFM12_MAC_State
{
  RFM12_MAC_STATE_IDLE,
  RFM12_MAC_STATE_RX_LENGTH_LOW,
  RFM12_MAC_STATE_RX_LENGTH_HIGH,
  RFM12_MAC_STATE_CHECKSUM,
  RFM12_MAC_STATE_RX,
  RFM12_MAC_STATE_TX_PREAMBLE1,
  RFM12_MAC_STATE_TX_PREAMBLE2,
  RFM12_MAC_STATE_TX_LENGTH_LOW,
  RFM12_MAC_STATE_TX_LENGTH_HIGH,
  RFM12_MAC_STATE_TX,
  RFM12_MAC_STATE_TX_END
} RFM12_MAC_State_t;

typedef struct RFM12_MAC_Channel
{
  RFM12_PHY_BAUDRATE_t baud;
  RFM12_PHY_BW_t bw;
  RFM12_PHY_FREQDEVIATION_t deviation;
} RFM12_MAC_Channel_t;

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data);
uint8_t rfm12_mac_previousLayerTransmitCallback(void);

void rfm12_mac_resetStateMachine(RFM12_Transfer_Error_t err);

void rfm12_mac_setChannel(RFM12_MAC_Channel_t channel, RFM12_PHY_VDI_t vdi, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh, RFM12_PHY_OutPwr_t pwr);

bool rfm12_mac_mediaBusy(void);

void rfm12_mac_startTransmission(void);

void rfm12_mac_stopTransmission(void);

#endif