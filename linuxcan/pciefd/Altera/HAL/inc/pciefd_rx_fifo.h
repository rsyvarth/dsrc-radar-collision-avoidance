#ifndef __PCIEFD_RX_FIFO_H__
#define __PCIEFD_RX_FIFO_H__

#include "VCanOsIf.h"

unsigned int fifoIrqStatus(void * base);
void fifoIrqClear(void * base, unsigned int icl);
void fifoIrqInit(void * base);
void fifoIrqEnableReceivedDataAvailable(void * base);

void fifoIrqEnableIllegalAccess(void * base);
void fifoIrqEnableUnalignedRead(void * base);
void fifoIrqEnableMissingTag(void * base);
void fifoIrqEnableUnderflow(void * base);

int fifoPacketCountRx(void * base);
int fifoPacketCountRxMax(void * base);

int fifoDataAvailable(void * base);
int dmaIdle(void * base);

int fifoOffset(void * base);
int fifoChannelId(void * base);
int fifoDataAvailableWords(void * base);


int hwSupportDMA(void * base);

int isEOP(void * base);
int readFIFO(VCanCardData *vCard, pciefd_packet_t *packet);

void enableDMA(void *base);
void disableDMA(void *base);

void armDMA0(void * base);
void armDMA1(void * base);
void armDMA(void * base, int id);

#endif
