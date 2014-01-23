#ifndef _RFM12PHY_H_
#define _RFM12PHY_H_

#include <stdbool.h>
#include <inttypes.h>

typedef enum RFM12_FREQBAND
{
  FREQBAND_315MHz,
  FREQBAND_433MHz,
  FREQBAND_868MHZ,
  FREQBAND_915MHz
} RFM12_FREQBAND_t;

typedef enum RFM12_CAP
{
  CAP8_5pf,
  CAP9_0pf,
  CAP9_5pf,
  CAP10_0pf,
  CAP10_5pf,
  CAP11_0pf,
  CAP11_5pf,
  CAP12_0pf,
  CAP12_5pf,
  CAP13_0pf,
  CAP13_5pf,
  CAP14_0pf,
  CAP14_5pf,
  CAP15_0pf,
  CAP15_5pf,
  CAP16_0pf
} RFM12_CAP_t;

typedef enum RFM12_VDI
{
  VDI_FAST,
  VDI_MEDIUM,
  VDI_SLOW,
  VDI_ALWAYSON
} RFM12_VDI_t;

typedef enum RFM12_BW
{
  BW400 = 1,
  BW340 = 2,
  BW270 = 3,
  BW200 = 4,
  BW134 = 5,
  BW67 = 6
} RFM12_BW_t;

typedef enum RFM12_LNAGAIN
{
  LNA_0,
  LNA_MINUS6,
  LNA_MINUS14,
  LNA_MINUS20
} RFM12_LNAGAIN_t;

typedef enum RFM12_RSSIDTH
{
  RSSI_MINUS103,
  RSSI_MINUS97,
  RSSI_MINUS91,
  RSSI_MINUS85,
  RSSI_MINUS79,
  RSSI_MINUS73,
  RSSI_MINUS67,
  RSSI_MINUS61
} RFM12_RSSIDTH_t;

typedef enum RFM12_AutoMode
{
  AUTOMODE_OFF,
  AUTOMODE_POWERUP,
  AUTOMODE_RECV,
  AUTOMODE_INDEPENDENT
} RFM12_AutoMode_t;

typedef enum RFM12_RangeLimit
{
  RANGELIMIT_NORESTRICT,
  RANGELIMIT_15toMINUS16,
  RANGELIMIT_7toMINUS8,
  RANGELIMIT_3toMINUS4
} RFM12_RangeLimit_t;

typedef enum RFM12_OutPwr
{
  OUTPWR_0,
  OUTPWR_MINUS3,
  OUTPWR_MINUS6,
  OUTPWR_MINUS9,
  OUTPWR_MINUS12,
  OUTPWR_MINUS15,
  OUTPWR_MINUS18,
  OUTPWR_MINUS21
} RFM12_OutPwr_t;

typedef enum RFM12_PHY_State
{
  RFM12_PHY_STATE_IDLE,
  RFM12_PHY_STATE_RECEIVE,
  RFM12_PHY_STATE_TRANSMIT
} RFM12_PHY_State_t;

typedef struct RFM12_PHY_RegStatus
{
  uint16_t PWRMGMT;
} RFM12_PHY_RegStatus_t;


void rfm12_phy_init(uint16_t (*pRFM12_phy_exchangeWord)(uint16_t word), void (*pRFM12_phy_SPISelect) (void), void (*pRFM12_phy_SPIDeselect)(void), bool (*RFM12_phy_nextLayerReceiveCallback)(uint8_t), uint8_t (*RFM12_phy_nextLayerTransmitCallback)(void));

uint16_t rfm12_phy_SPIWrite(uint16_t data);

void rfm12_phy_modeTX(void);
void rfm12_phy_modeRX(void);
bool rfm12_phy_busy(void);

void rfm12_phy_int_vect(void);

// OK
void rfm12_phy_setConf( bool enDataReg, bool enFIFO, RFM12_FREQBAND_t freqband, RFM12_CAP_t loadcap);

// OK
void rfm12_phy_setPowerManagement( bool enBB, bool enSynth, bool enOSC, bool enBat, bool enWkT, bool clkOff);

// OK
void rfm12_phy_setFrequency(uint16_t freq);

// OK
void rfm12_phy_setBaudrate(uint8_t baudrate);

// OK
void rfm12_phy_setRecvCtrl(bool p20, RFM12_VDI_t vdi, RFM12_BW_t bw, RFM12_LNAGAIN_t lna, RFM12_RSSIDTH_t rssiDTh);

// OK
void rfm12_phy_setDataFilter(bool autoLock, bool fastMode, bool analog, uint8_t dqdThres);

// OK
void rfm12_phy_setFIFORst(uint8_t bittrigger, bool alwaysfill, bool enFIFO, bool disableHighSensRst);

// OK
void rfm12_phy_setAFC(RFM12_AutoMode_t automode, RFM12_RangeLimit_t rangelimit, bool strobeEdge, bool fineMode, bool offsetRegister, bool enAFC);

//  OK
void rfm12_phy_setTxConf(uint8_t FSKModParm, RFM12_OutPwr_t pwr);

#endif