#ifndef _RFM12PHY_H_
#define _RFM12PHY_H_

#include <inttypes.h>
#include <stdbool.h>

typedef enum RFM12_MAC_State
{
  RFM12_MAC_STATE_IDLE;
  RFM12_MAC_STATE_RX_LENGTH_LOW;
  RFM12_MAC_STATE_RX_LENGTH_HIGH;
  RFM12_MAC_STATE_RX;
} RFM12_MAC_State_t;

bool rfm12_mac_previousLayerReceiveCallback(uint8_t data);
uint8_t rfm12_mac_previousLayerTransmitCallback(void);

void rfm12_mac_setChannel(uint8_t channel);

bool rfm12_mac_mediaBusy(void);

void rfm12_mac_startTransmission(void);

#endif