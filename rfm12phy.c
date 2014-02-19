#include "rfm12.h"

#include <stdlib.h>
#include <stdbool.h>

#include "rfm12phy.h"
#include "rfm12mac.h"

uint16_t (*RFM12_phy_exchangeWord)(uint16_t word) = NULL;
void (*RFM12_phy_SPISelect) (void) = NULL;
void (*RFM12_phy_SPIDeselect)(void) = NULL;

uint8_t (*RFM12_phy_nextLayerTransmitCallback)(void) = NULL;

RFM12_PHY_State_t phystate;
RFM12_PHY_RegStatus_t regstatus;

static inline uint16_t __inline_rfm12_phy_getStatus(void)
{
  return rfm12_phy_SPIWrite(RFM12_STATUSRD);
}

static inline uint8_t __inline_rfm12_phy_getFIFOByte(void)
{
  return rfm12_phy_SPIWrite(RFM12_RXFIFORD) & 0xFF;
}

static inline void __inline_rfm12_phy_putFIFOByte(uint8_t data)
{
  rfm12_phy_SPIWrite(RFM12_TXWR | data);
}

static inline void __inline_rfm12_phy_startTX(void)
{
  regstatus.PWRMGMT |= (1<<5);
  rfm12_phy_SPIWrite(RFM12_PWRMGM | regstatus.PWRMGMT);
}

static inline void __inline_rfm12_phy_startRX(void)
{
  regstatus.PWRMGMT |= (1<<7);
  rfm12_phy_SPIWrite(RFM12_PWRMGM | regstatus.PWRMGMT);
}

static inline void __inline_rfm12_phy_disableRXTX(void)
{
  regstatus.PWRMGMT &= 0x5F;
  rfm12_phy_SPIWrite(RFM12_PWRMGM | regstatus.PWRMGMT);
}

void rfm12_phy_init(uint16_t (*pRFM12_phy_exchangeWord)(uint16_t word), void (*pRFM12_phy_SPISelect) (void), void (*pRFM12_phy_SPIDeselect)(void))
{
  RFM12_phy_exchangeWord = pRFM12_phy_exchangeWord;
  RFM12_phy_SPISelect = pRFM12_phy_SPISelect;
  RFM12_phy_SPIDeselect = pRFM12_phy_SPIDeselect;
}

uint16_t rfm12_phy_SPIWrite(uint16_t data)
{
  uint16_t datareturned;
  RFM12_phy_SPISelect();
  datareturned = RFM12_phy_exchangeWord(data);
  RFM12_phy_SPIDeselect();
  return datareturned;
}

void rfm12_phy_modeTX()
{
  //disable receive
  rfm12_phy_setFIFORst(8, false, false, false);
  __inline_rfm12_phy_disableRXTX();
  
  //write preamble
  __inline_rfm12_phy_putFIFOByte(0xAA);
  __inline_rfm12_phy_putFIFOByte(0xAA);
  
  phystate = RFM12_PHY_STATE_TRANSMIT;
  __inline_rfm12_phy_startTX();
}

void rfm12_phy_modeRX()
{
  __inline_rfm12_phy_disableRXTX();
  rfm12_phy_setFIFORst(8, false, true, false);
  __inline_rfm12_phy_startRX();
}

void rfm12_phy_setConf( bool enDataReg, bool enFIFO, RFM12_PHY_FREQBAND_t freqband, RFM12_PHY_CAP_t loadcap)
{
  rfm12_phy_SPIWrite(RFM12_CFGSET | ((enDataReg<<7) & 0x80) | ((enFIFO<<6) & 0x40) | ((freqband<<4)&0x30) | (loadcap&0xF));
}

void rfm12_phy_setPowerManagement(bool enBB, bool enSynth, bool enOSC, bool enBat, bool enWkT, bool clkOff)
{
  regstatus.PWRMGMT &= 0xA0;
  regstatus.PWRMGMT |= ((enBB<<6)&0x40) | ((enSynth<<4)&0x10) | ((enOSC<<3)&0x8) | ((enBat<<2)&0x4) | ((enWkT<<1)&0x2) | (clkOff&0x1);
  rfm12_phy_SPIWrite( RFM12_PWRMGM | regstatus.PWRMGMT);
}

void rfm12_phy_setFrequency(uint16_t freq)
{
  rfm12_phy_SPIWrite(RFM12_FREQSET | (freq & 0xFF));
}

void rfm12_phy_setBaudrate(RFM12_PHY_BAUDRATE_t baudrate)
{
  rfm12_phy_SPIWrite(RFM12_DATARATE | (baudrate & 0xFF));
}

void rfm12_phy_setRecvCtrl(bool p20, RFM12_PHY_VDI_t vdi, RFM12_PHY_BW_t bw, RFM12_PHY_LNAGAIN_t lna, RFM12_PHY_RSSIDTH_t rssiDTh)
{
  rfm12_phy_SPIWrite(RFM12_RXCONTROL | ((p20<<10)&0x400) | ((vdi<<8)&0x300) | ((bw<<5)&0xE00) | ((lna<<3)&0x18) | (rssiDTh&0x7));
}

void rfm12_phy_setDataFilter(bool autoLock, bool fastMode, bool analog, uint8_t dqdThres)
{
  rfm12_phy_SPIWrite(RFM12_DATAFILTER | ((autoLock<<7)&0x80) | ((fastMode<<6)&0x40) | ((analog<<4)&0x10) | (dqdThres&0x7));
}

void rfm12_phy_setFIFORst(uint8_t bittrigger, bool alwaysfill, bool enFIFO, bool disableHighSensRst)
{
  rfm12_phy_SPIWrite(RFM12_FIFORSTMODE | ((bittrigger<<4)&0xF0) | ((alwaysfill<<2) & 0x4) | ((enFIFO<<1) & 0x2) | (disableHighSensRst & 0x1));
}

void rfm12_phy_setAFC(RFM12_PHY_AutoMode_t automode, RFM12_PHY_RangeLimit_t rangelimit, bool strobeEdge, bool fineMode, bool offsetRegister, bool enAFC)
{
  rfm12_phy_SPIWrite(RFM12_AFC | ((automode<<6)&0xC0) | ((rangelimit<<4)&0x30) | ((strobeEdge<<3)&0x8) | ((fineMode<<2)&0x4) | ((offsetRegister<<1)&0x2) | (enAFC&0x1));
}

void rfm12_phy_setTxConf(bool mp, RFM12_PHY_FREQDEVIATION_t deviation, RFM12_PHY_OutPwr_t pwr)
{
  rfm12_phy_SPIWrite(RFM12_TXCFG | ((mp<<8) & 0x100) | ((deviation<<4)&0x1F0) | (pwr&0x7));
}

void rfm12_phy_int_vect()
{
  uint16_t status = __inline_rfm12_phy_getStatus();
  uint8_t buffer;
  
  if(status & RFM12_STATUSRD_RGIT_FFIT)
  {
    switch(phystate)
    {
      case RFM12_PHY_STATE_IDLE:
	
	phystate = RFM12_PHY_STATE_RECEIVE;
      
      case RFM12_PHY_STATE_RECEIVE:
	buffer = __inline_rfm12_phy_getFIFOByte();
	
	if(!rfm12_mac_previousLayerReceiveCallback(buffer))
	{
	  rfm12_phy_modeRX();
	  phystate = RFM12_PHY_STATE_IDLE;
	}

	break;
	
      case RFM12_PHY_STATE_TRANSMIT:
	__inline_rfm12_phy_putFIFOByte(RFM12_phy_nextLayerTransmitCallback());
	break;
    }
  }
}