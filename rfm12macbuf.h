#ifndef _RFM12MACBUF_H_
#define _RFM12MACBUF_H_

#include <stdbool.h>
#include "rfm12mac.h"


#define RFM12MACBUFSIZE 32
#define MAXFRAMES 20

void rfm12_mac_buf_init(void);
RFM12_MAC_Frame_t volatile *rfm12_mac_buf_reqSpace(uint16_t size);
RFM12_MAC_Frame_t *rfm12_mac_buf_nextPkt(void);
void rfm12_mac_buf_freeSpace(RFM12_MAC_Frame_t frame);
void rfm12_mac_buf_clearFrame(void);

#endif