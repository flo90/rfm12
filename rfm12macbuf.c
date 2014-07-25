#include "rfm12macbuf.h"

static uint8_t buffer[RFM12MACBUFSIZE];
static RFM12_MAC_Frame_t frames[MAXFRAMES];
static uint8_t *pCurrentByte;
static RFM12_MAC_Frame_t *pCurrentFrame, *pLeastFrame, *pNextFrame;

void rfm12_mac_buf_init()
{
  uint8_t i;
  
  //Currentbyte at the beginning of the buffer
  pCurrentByte = buffer;
  
  //Current frame at the beginning
  pCurrentFrame = frames;
  pLeastFrame = frames;
  
  for(i=0; i < MAXFRAMES; i++)
  {
    //all frames has invalid data first
    frames[i].data = NULL;
  }
}

RFM12_MAC_Frame_t *rfm12_mac_buf_reqSpace(uint16_t size)
{
  RFM12_MAC_Frame_t *pframe;
  //check free size of buffer
  if((pCurrentByte + size) <= (buffer + RFM12MACBUFSIZE))
  {
    pCurrentFrame->data = pCurrentByte;
    pCurrentByte += size;
    pframe = pCurrentFrame;
    ++pCurrentFrame;
    if(pCurrentFrame > (frames + MAXFRAMES))
    {
      pCurrentFrame = frames;
    }
  }
  //try to find space from the beginning
  else if ((buffer + size) < pLeastFrame->data)
  {
    pCurrentFrame->data = buffer;
    pCurrentByte = buffer + size;
    pframe = pCurrentFrame;
    ++pCurrentFrame;
    
  }
  else
  {
    pframe = NULL;
  }
  return pframe;
}

RFM12_MAC_Frame_t *rfm12_mac_buf_nextPkt()
{
  RFM12_MAC_Frame_t *pframe;
  if(pNextFrame->data == NULL)
  {
    return NULL;
  }
  
  pframe = pNextFrame;
  ++pNextFrame;
  
  if(pNextFrame > (frames + MAXFRAMES))
  {
    pNextFrame = frames;
  }
  
  return pframe;
}

void rfm12_mac_buf_clearFrame(RFM12_MAC_Frame_t *pframe)
{
  if(pLeastFrame == pframe)
  {
    while((pLeastFrame->data == NULL) && (pLeastFrame != pCurrentFrame))
    {
      ++pLeastFrame;
      if(pLeastFrame > (frames + MAXFRAMES))
      {
	pLeastFrame = frames;
      }
    }
  }
  
  else
  {
    pframe->data = NULL;
  }
}