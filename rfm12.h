#ifndef _RFM12_H_
#define _RFM12_H_

#include <inttypes.h>

//Configuration Setting Command
#define RFM12_CFGSET 0x8000




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

//Power Setting Command
#define RFM12_PWRSET 0x9000

//Data Filter Command
#define RFM12_DATAFILTER 0xC228

//FIFO and Reset Mode Command
#define RFM12_FIFORSTMODE 0xCA00

//Receiver FIFO Read Command
#define RFM12_RXFIFORD 0xB000

//AFC Command
#define RFM12_AFC 0xC400

//TX Configuration Control Command
#define RFM12_TXCFG 0x9800

//Transmitter Register Write Command
#define RFM12_TXWR 0xB800

//Wake-Up Timer Command
#define RFM12_WAKEUPTIMER 0xE000

//Low Duty-Cycle Command
#define RFM12_LOWDUTY 0xC800

//Low Battery Detector and Microcontroller Clock Divider Command
#define RFM12_BATCLOCK 0xC000

//Status Read Command
#define RFM12_STATUSRD 0x0000

void rfm12_init(uint8_t (*pexchangebyte)(uint8_t data));



#endif