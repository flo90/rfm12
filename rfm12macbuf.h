#ifndef _RFM12MACBUF_H_
#define _RFM12MACBUF_H_

#include "rfm12mac.h"

#define RFM12MACBUFSIZE 1024
#define MAXFRAMES 20

typedef struct RFM12_MAC_Frame
{
  RFM12_MAC_Header_t header;
  uint8_t *data;
}RFM12_MAC_Frame_t;


void rfm12_mac_buf_init(void);
RFM12_MAC_Frame_t *rfm12_mac_buf_reqSpace(uint16_t size);
RFM12_MAC_Frame_t *rfm12_mac_buf_nextPkt(void);
void rfm12_mac_buf_freeSpace(RFM12_MAC_Frame_t frame);
void rfm12_mac_buf_clearFrame(RFM12_MAC_Frame_t *pframe);


#endif