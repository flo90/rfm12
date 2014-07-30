#include "rfm12macbuf.h"

#include "usart.h"
static volatile uint8_t buffer[RFM12MACBUFSIZE];
static RFM12_MAC_Frame_t frames[MAXFRAMES];
static volatile uint8_t volatile *pCurrentByte;

static RFM12_MAC_Frame_t volatile *pCurrentFrame;
static RFM12_MAC_Frame_t volatile *pNextFrame;

void rfm12_mac_buf_init()
{
  uint8_t i;
  
  //Currentbyte at the beginning of the buffer
  pCurrentByte = buffer;
  
  //Current frame at the beginning
  pCurrentFrame = frames;
  //pLeastFrame = frames;
  pNextFrame = frames;
  
  for(i=0; i < MAXFRAMES; i++)
  {
    //all frames has invalid data first
    frames[i].data = NULL;
    frames[i].finished = false;
  }
}

RFM12_MAC_Frame_t volatile *rfm12_mac_buf_reqSpace(uint16_t size)
{
  RFM12_MAC_Frame_t volatile *pframe;
  
  //check if the next Frame is free
  if(pCurrentFrame->data != NULL)
  {
    usart_puts_nonblock("NoFrames");
    return NULL;
  }
  
  //check free size of buffer
  if((pCurrentByte + size) <= (buffer + RFM12MACBUFSIZE))
  {
    pCurrentFrame->data = pCurrentByte;
    pCurrentByte += size;
    pframe = pCurrentFrame;
    
    ++pCurrentFrame;
    if(pCurrentFrame >= (frames + MAXFRAMES))
    {
      pCurrentFrame = frames;
    }
  }
  //try to find space from the beginning
  else if ((pNextFrame->data == NULL) || ((buffer + size) < pNextFrame->data))
  {
    pCurrentFrame->data = buffer;
    pCurrentByte = buffer + size;
    pframe = pCurrentFrame;
    
    ++pCurrentFrame;
    if(pCurrentFrame >= (frames + MAXFRAMES))
    {
      pCurrentFrame = frames;
    }
  }
  
  else
  {
    pframe = NULL;
  }
  
  return pframe;
}

RFM12_MAC_Frame_t *rfm12_mac_buf_nextPkt()
{
  //cant get packet if data is NULL or the packet is not finished
  if(pNextFrame->data == NULL || !(pNextFrame->finished))
  {
    return NULL;
  }
  
  //just return next frame 
  return (RFM12_MAC_Frame_t *) pNextFrame;
}

//void rfm12_mac_buf_clearFrame(RFM12_MAC_Frame_t *pframe)
void rfm12_mac_buf_clearFrame()
{
  //cant clear frame if there is no data or the packet has not finished
  if(pNextFrame->data == NULL || !(pNextFrame->finished))
  {
    return;
  }
  
  //set data to invalid pointer
  pNextFrame->data = NULL;
  
  //the frame is not finished
  pNextFrame->finished = false;
  
  //increment frame
  ++pNextFrame;
  
  //check if the pointer goes out of bounds
  if(pNextFrame >= (frames + MAXFRAMES))
  {
    pNextFrame = frames;
  }
}