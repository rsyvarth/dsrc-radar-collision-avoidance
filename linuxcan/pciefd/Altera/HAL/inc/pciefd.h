#ifndef __PCIEFD_H__
#define __PCIEFD_H__

#include "inc/pciefd_regs.h"
#include "inc/pciefd_packet_defs.h"
#include "HAL/inc/pciefd_packet.h"
#include "VCanOsIf.h"

extern char *pciefd_ctrl_reg_names[];

int statResetRequest(void * base);
int statInResetMode(void * base);
int statTransmitterIdle(void * base);
int statAbortRequest(void * base);
int statPendingRequest(void *base);
int statIdle(void *base);
int statFlushRequest(void * base);

int istatTransmitterFlushDone(void * base);
void irqClearTransmitterFlushDone(void * base);

int fifoPacketCountTx(void * base);
int fifoPacketCountTxMax(void * base);

int hwSupportCanFD(void * base);

int readIrqReceiverOverflow(void * base);
unsigned int irqReceiverOverflow(unsigned int irq);

int readIrqReceiverUnaligned(void * base);
unsigned int irqReceiverUnaligned(unsigned int irq);

int readIrqTransmitterOverflow(void * base);
unsigned int irqTransmitterOverflow(unsigned int irq);
unsigned int irqTransmitterEmpty(unsigned int irq);
int irqDisableTransmitterEmpty(void * base);

int readIrqTransmitterUnderflow(void * base);
unsigned int irqTransmitterUnderflow(unsigned int irq);

int readIrqTransmitterUnaligned(void * base);
unsigned int irqTransmitterUnaligned(unsigned int irq);

void irqClearReceivedDataAvailable(void * base);
unsigned int irqReceivedDataAvailable(unsigned int irq);
void irqEnableReceivedDataAvailable(void * base);
void irqEnableTransmitterError(void * base);
void irqEnableTransmitFifoEmpty(void * base);
void irqClearTransmitFifoEmpty(void * base);

void irqClearReceivedUnaligned(void * base);

int irqEnabled(void * base);
unsigned int irqStatus(void * base);
void irqClear(void * base, unsigned int icl);
int istatCheck(void * base, uint32_t icl);
void irqInit(void * base);

void enableErrorPackets(void * base);
void disableErrorPackets(void * base);

void enableNonIsoMode(void * base);
void disableNonIsoMode(void * base);

void enableSlaveMode(void * base);
void disableSlaveMode(void * base);
unsigned int getSlaveMode(void * base);

void enableErrorPassiveMode(void * base);
void disableErrorPassiveMode(void * base);
unsigned int getErrorPassiveMode(void * base);

void enableClassicCanMode(void * base);
void disableClassicCanMode(void * base);
unsigned int getClassicCanMode(void * base);

void speedUp(int canid, int value);
void speedNominal(int canid);

void fpgaStatusRequest(void * base, int seq_no);
void fpgaFlushTransmit(void * base);
void fpgaFlushAll(void * base, int seq_no);
void resetErrorCount(void * base);

int writeFIFO(VCanChanData *vChd, pciefd_packet_t *packet);

void setLedStatus(void * base, int on);
void ledOn(void * base);
void ledOff(void * base);

#endif
