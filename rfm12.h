#ifndef _RFM12_H_
#define _RFM12_H_

#include <inttypes.h>
#include <stdbool.h>

//Configuration Setting Command
#define RFM12_CFGSET 0x8000

#define RFM12_CFGSET_EL (1<<7)
#define RFM12_CFGSET_EF (1<<6)
#define RFM12_CFGSET_FREQ_315 (0b00<<4)
#define RFM12_CFGSET_FREQ_433 (0b01<<4)
#define RFM12_CFGSET_FREQ_868 (0b10<<4)
#define RFM12_CFGSET_FREQ_915 (0b11<<4)

//Power Management Command
#define RFM12_PWRMGM 0x8200

#define RFM12_PWRMGM_ER (1<<7)
#define RFM12_PWRMGM_EBB (1<<6)
#define RFM12_PWRMGM_ET (1<<5)
#define RFM12_PWRMGM_ES (1<<4)
#define RFM12_PWRMGM_EX (1<<3)
#define RFM12_PWRMGM_EB (1<<2)
#define RFM12_PWRMGM_EW (1<<1)
#define RFM12_PWRMGM_DC (1<<0)

//Frequency Setting Command
#define RFM12_FREQSET 0xA000

//Data Rate Command
#define RFM12_DATARATE 0xC600

#define RFM12_DATARATE_CS (1<<7)

//Receiver Control Command
#define RFM12_RXCONTROL 0x9000

#define RFM12_RXCONTROL_P20 (1<<10)
#define RFM12_RXCONTROL_VDIRESPONSE_FAST (0b00<<8)
#define RFM12_RXCONTROL_VDIRESPONSE_MEDIUM (0b01<<8)
#define RFM12_RXCONTROL_VDIRESPONSE_SLOW (0b10<<8)
#define RFM12_RXCONTROL_VDIRESPONSE_ALWAYSON (0b11<<8)

#define RFM12_RXCONTROL_BW_400 (0b001<<5)
#define RFM12_RXCONTROL_BW_340 (0b010<<5)
#define RFM12_RXCONTROL_BW_270 (0b011<<5)
#define RFM12_RXCONTROL_BW_200 (0b100<<5)
#define RFM12_RXCONTROL_BW_134 (0b101<<5)
#define RFM12_RXCONTROL_BW_67 (0b110<<5)

#define RFM12_RXCONTROL_LNA_0 (0b00<<3)
#define RFM12_RXCONTROL_LNA_6 (0b01<<3)
#define RFM12_RXCONTROL_LNA_14 (0b10<<3)
#define RFM12_RXCONTROL_LNA_20 (0b11<<3)

#define RFM12_RXCONTROL_RSSI_103 (0b000<<0)
#define RFM12_RXCONTROL_RSSI_97 (0b001<<0)
#define RFM12_RXCONTROL_RSSI_91 (0b010<<0)
#define RFM12_RXCONTROL_RSSI_85 (0b011<<0)
#define RFM12_RXCONTROL_RSSI_79 (0b100<<0)
#define RFM12_RXCONTROL_RSSI_73 (0b101<<0)
#define RFM12_RXCONTROL_RSSI_67 (0b110<<0)
#define RFM12_RXCONTROL_RSSI_61 (0b111<<0)


//Data Filter Command
#define RFM12_DATAFILTER 0xC228

#define RFM12_DATAFILTER_AL (1<<7)
#define RFM12_DATAFILTER_ML (1<<6)
#define RFM12_DATAFILTER_S (1<<4)

//FIFO and Reset Mode Command
#define RFM12_FIFORSTMODE 0xCA00

#define RFM12_FIFORSTMODE_AL (1<<2)
#define RFM12_FIFORSTMODE_FF (1<<1)
#define RFM12_FIFORSTMODE_DR (1<<0)

//Receiver FIFO Read Command
#define RFM12_RXFIFORD 0xB000

//AFC Command
#define RFM12_AFC 0xC400

#define RFM12_AFC_AUTOMODEOFF (0b00<<6)
#define RFM12_AFC_ONCEPOWERUP (0b01<<6)
#define RFM12_AFC_DURINGRX (0b10<<6)
#define RFM12_AFC_INDEPENDENT (0b11<<6)

#define RFM12_AFC_DEVIATION_NORESTRICTION (0b00<<4)
#define RFM12_AFC_DEVIATION_15TO16 (0b01<<4)
#define RFM12_AFC_DEVIATION_7TO8 (0b10<<4)
#define RFM12_AFC_DEVIATION_3TO4 (0b11<<4)

#define RFM12_AFC_ST (1<<3)
#define RFM12_AFC_FI (1<<2)
#define RFM12_AFC_OE (1<<1)
#define RFM12_AFC_EN (1<<0)

//TX Configuration Control Command
#define RFM12_TXCFG 0x9800

#define RFM12_TXCFG_MP (1<<8)

#define RFM12_TXCFG_POWER_0 (0b000<<0)
#define RFM12_TXCFG_POWER_3 (0b001<<0)
#define RFM12_TXCFG_POWER_6 (0b010<<0)
#define RFM12_TXCFG_POWER_9 (0b011<<0)
#define RFM12_TXCFG_POWER_12 (0b100<<0)
#define RFM12_TXCFG_POWER_15 (0b101<<0)
#define RFM12_TXCFG_POWER_18 (0b110<<0)
#define RFM12_TXCFG_POWER_21 (0b111<<0)

//Transmitter Register Write Command
#define RFM12_TXWR 0xB800

//Wake-Up Timer Command
#define RFM12_WAKEUPTIMER 0xE000

//Low Duty-Cycle Command
#define RFM12_LOWDUTY 0xC800

#define RFM12_LOWDUTY_EN (1<<0)

//Low Battery Detector and Microcontroller Clock Divider Command
#define RFM12_BATCLOCK 0xC000

//Status Read Command
#define RFM12_STATUSRD 0x0000

//This defines are used to check the bits of the status
#define RFM12_STATUSRD_RGIT_FFIT (1<<15)
#define RFM12_STATUSRD_POR (1<<14)
#define RFM12_STATUSRD_RGUR_FFOV (1<<13)
#define RFM12_STATUSRD_WKUP (1<<12)
#define RFM12_STATUSRD_EXT (1<<11)
#define RFM12_STATUSRD_LBD (1<<10)
#define RFM12_STATUSRD_FFEM (1<<9)
#define RFM12_STATUSRD_ATS_RSSI (1<<8)
#define RFM12_STATUSRD_DQD (1<<7)
#define RFM12_STATUSRD_CRL (1<<6)
#define RFM12_STATUSRD_ATGL (1<<5)

#define BUFFER_SIZE 256

enum RFM12_STATE {IDLE, PRE_RX, RX, PRE_TX, TX, POST_TX};

typedef struct
{
  uint16_t nextpacketptr;
  uint16_t writeptr;
  uint16_t clearedptr;
  uint8_t packetcnt;
}bufferstate;

unsigned char *rfm12_init(void);

uint8_t rfm12_tx(char *buf, uint16_t length);
bool rfm12_haspacket(void);
uint16_t rfm12_getpacketptr(void);
void rfm12_clearpacket(void);

#endif
