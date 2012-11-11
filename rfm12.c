#include "rfm12.h"

#include <inttypes.h>


uint8_t (*exchangebyte)(uint8_t data);

static inline void select(void)
{
  //Do not forget to implement select!
}

static inline void deselect(void)
{
  //Do not forget to implement deselect
}

static inline void rfm12_writeOp(uint16_t data)
{
  select();
  
  exchangebyte(data>>8);
  exchangebyte(data);
  
  deselect();
}

static inline void rfm12_freqset(unsigned int freq)
{ 
  rfm12_writeOp( RFM12_FREQSET | (400*(freq-430)) );
}

//init rfm12 (in this case a rfm12 with 433Mhz)
void rfm12_init(uint8_t (*pexchangebyte)(uint8_t data))
{
  exchangebyte = pexchangebyte;
  
  //start init
  rfm12_writeOp(CFGSET | 0x00D8);
  
  //setting frequency to 433.92 Mhz
  rfm12_freqset(433.92);
  
  
  
  
}