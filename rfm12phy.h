#ifndef _RFM12PHY_H_
#define _RFM12PHY_H_

#include <stdbool.h>
#include <inttypes.h>

typedef enum RFM12_PHY_FREQBAND
{
  FREQBAND_315MHz,
  FREQBAND_433MHz,
  FREQBAND_868MHZ,
  FREQBAND_915MHz
} RFM12_PHY_FREQBAND_t;

typedef enum RFM12_PHY_CAP
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
} RFM12_PHY_CAP_t;

typedef enum RFM12_PHY_VDI
{
  VDI_FAST = 0,
  VDI_MEDIUM = 1,
  VDI_SLOW = 2,
  VDI_ALWAYSON = 3,
} RFM12_PHY_VDI_t;

typedef enum RFM12_PHY_BW
{
  RFM12_PHY_BW400 = 1,
  RFM12_PHY_BW340 = 2,
  RFM12_PHY_BW270 = 3,
  RFM12_PHY_BW200 = 4,
  RFM12_PHY_BW134 = 5,
  RFM12_PHY_BW67 = 6,
} RFM12_PHY_BW_t;

typedef enum RFM12_PHY_LNAGAIN
{
  LNA_0 = 0,
  LNA_MINUS6 = 1,
  LNA_MINUS14 = 2,
  LNA_MINUS20 = 3,
} RFM12_PHY_LNAGAIN_t;

typedef enum RFM12_PHY_RSSIDTH
{
  RSSI_MINUS103 = 0,
  RSSI_MINUS97 = 1,
  RSSI_MINUS91 = 2,
  RSSI_MINUS85 = 3,
  RSSI_MINUS79 = 4,
  RSSI_MINUS73 = 5,
  RSSI_MINUS67 = 6,
  RSSI_MINUS61 = 7,
} RFM12_PHY_RSSIDTH_t;

typedef enum RFM12_PHY_AutoMode
{
  AUTOMODE_OFF = 0,
  AUTOMODE_POWERUP = 1,
  AUTOMODE_RECV = 2,
  AUTOMODE_INDEPENDENT = 3
} RFM12_PHY_AutoMode_t;

typedef enum RFM12_PHY_RangeLimit
{
  RANGELIMIT_NORESTRICT = 0,
  RANGELIMIT_15toMINUS16 = 1,
  RANGELIMIT_7toMINUS8 = 2,
  RANGELIMIT_3toMINUS4 = 3,
} RFM12_PHY_RangeLimit_t;

typedef enum RFM12_PHY_FREQDEVIATION
{
  RFM12_FREQDEVIATION_45KHz = 2,
  RFM12_FREQDEVIATION_90KHz = 5,
  RFM12_FREQDEVIATION_120KHz = 7,
} RFM12_PHY_FREQDEVIATION_t;

typedef enum RFM12_PHY_OutPwr
{
  OUTPWR_0 = 0,
  OUTPWR_MINUS3 = 1,
  OUTPWR_MINUS6 = 2,
  OUTPWR_MINUS9 = 3,
  OUTPWR_MINUS12 = 4,
  OUTPWR_MINUS15 = 5,
  OUTPWR_MINUS18 = 6,
  OUTPWR_MINUS21 = 7,
} RFM12_PHY_OutPwr_t;

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

typedef enum RFM12_PHY_BAUDRATE
{
  RFM12_PHY_BAUDRATE_9600 = 35,
  RFM12_PHY_BAUDRATE_19200 = 17,
  RFM12_PHY_BAUDRATE_115200 = 2
} RFM12_PHY_BAUDRATE_t;

void rfm12_phy_init(uint16_t (*pRFM12_phy_exchangeWord)(uint16_t word), void (*pRFM12_phy_SPISelect) (void), void (*pRFM12_phy_SPIDeselect)(void), void (**prfm12_phy_int_vect)(void));

uint16_t rfm12_phy_SPIWrite(uint16_t data);

void rfm12_phy_modeTX(void);
void rfm12_phy_modeRX(void);
bool rfm12_phy_busy(void);

void rfm12_phy_int_vect(void);

// OK
void rfm12_phy_setConf( bool enDataReg, bool enFIFO, RFM12_PHY_FREQBAND_t freqband, RFM12_PHY_CAP_t loadcap);

// OK
void rfm12_phy_setPowerManagement( bool enBB, bool enSynth, bool enOSC, bool enBat, bool enWkT, bool clkOff);

// OK
void rfm12_phy_setFrequency(uint16_t freq);

// OK
void rfm12_phy_setBaudrate(RFM12_PHY_BAUDRATE_t baudrate);

// OK
void rfm12_phy_setRecvCtrl(bool p20, RFM12_PHY_VDI_t vdi, RFM12_PHY_BW_t bw, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh);

// OK
void rfm12_phy_setDataFilter(bool autoLock, bool fastMode, bool analog, uint8_t dqdThres);

// OK
void rfm12_phy_setFIFORst(uint8_t bittrigger, bool alwaysfill, bool enFIFO, bool disableHighSensRst);

// OK
void rfm12_phy_setAFC(RFM12_PHY_AutoMode_t automode, RFM12_PHY_RangeLimit_t rangelimit, bool strobeEdge, bool fineMode, bool offsetRegister, bool enAFC);

//  OK
void rfm12_phy_setTxConf(bool mp, RFM12_PHY_FREQDEVIATION_t deviation, RFM12_PHY_OutPwr_t pwr);

#endif