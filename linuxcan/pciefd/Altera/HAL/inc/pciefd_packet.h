#ifndef __PCIEFD_PACKET_H__
#define __PCIEFD_PACKET_H__

#include "VCanOsIf.h"
#include "inc/pciefd_packet_defs.h"

// Packet handling
#define USE_CAN_FD (1)

#if USE_CAN_FD
#define NR_OF_DATA_WORDS 16
#else
#define NR_OF_DATA_WORDS 2
#endif


typedef struct {
  uint32_t id;
  uint32_t control;
  uint32_t data[NR_OF_DATA_WORDS];
  uint64_t timestamp;
} pciefd_packet_t;


int isDataPacket(pciefd_packet_t *packet);
int isAckPacket(pciefd_packet_t *packet);
int isTxrqPacket(pciefd_packet_t *packet);
int isErrorPacket(pciefd_packet_t *packet);
int isEFlushAckPacket(pciefd_packet_t *packet);
int isEFrameAckPacket(pciefd_packet_t *packet);
int isOffsetPacket(pciefd_packet_t *packet);
int isBusLoadPacket(pciefd_packet_t *packet);
int isStatusPacket(pciefd_packet_t *packet);
int isDelayPacket(pciefd_packet_t *packet);

int packetChannelId(pciefd_packet_t *packet);

int statusInResetMode(pciefd_packet_t *packet);
int statusResetModeChangeDetected(pciefd_packet_t *packet);
int statusReset(pciefd_packet_t *packet);
int statusOverflow(pciefd_packet_t *packet);
int statusErrorWarning(pciefd_packet_t *packet);
int statusErrorPassive(pciefd_packet_t *packet);
int statusBusOff(pciefd_packet_t *packet);
int statusReceiveErrorCount(pciefd_packet_t *packet);
int statusTransmitErrorCount(pciefd_packet_t *packet);
int statusCmdSeqNo(pciefd_packet_t *packet);

int getChannelId(pciefd_packet_t *packet);

int dlcToBytes(int dlc);
int dlcToBytesFD(int dlc);
int bytesToDlc(int bytes);

#define PACKET_WORDS(length)  (2 + (3+length)/4)
#define FD_PACKET_WORDS  (2 + (3+packet_size[packet_size_index])/4)

int bytesToWordsCeil(int bytes);

// +----------------------------------------------------------------------------
// Received data packets
// +----------------------------------------------------------------------------
int getId(pciefd_packet_t *packet);
void setId(pciefd_packet_t *packet, int id);

int isErrorPassive(pciefd_packet_t *packet);
int receiverOverflow(pciefd_packet_t *packet);
int errorWarning(pciefd_packet_t *packet);

int isFlexibleDataRateFormat(pciefd_packet_t *packet);
int isAlternateBitRate(pciefd_packet_t *packet);
int errorStateIndicated(pciefd_packet_t *packet);
int getSRR(pciefd_packet_t *packet);
int isExtendedId(pciefd_packet_t *packet);
int isRemoteRequest(pciefd_packet_t *packet);
int getDLC(pciefd_packet_t *packet);
void setDLC(pciefd_packet_t *packet, int dlc);

int getSeqNo(pciefd_packet_t *packet);

int getOffsetQuantas(pciefd_packet_t *packet);
int getOffsetNbits(pciefd_packet_t *packet);

int getBusLoad(pciefd_packet_t *packet);

// +----------------------------------------------------------------------------
// | Received ack packets
// +----------------------------------------------------------------------------
int getAckSeqNo(pciefd_packet_t *packet);
int isFlushed(pciefd_packet_t *packet);
int isControlAck(pciefd_packet_t *packet);

// +----------------------------------------------------------------------------
// | Received txrq packets
// +----------------------------------------------------------------------------
int getTxrqSeqNo(pciefd_packet_t *packet);

// +----------------------------------------------------------------------------
// | Received error packets
// +----------------------------------------------------------------------------
int getTransmitErrorCount(pciefd_packet_t *packet);
int getReceiveErrorCount(pciefd_packet_t *packet);

etype_t getErrorType(pciefd_packet_t *packet);
int isTransmitError(pciefd_packet_t *packet);
int getErrorField(pciefd_packet_t *packet);
int getErrorFieldPos(pciefd_packet_t *packet);
void printErrorCode(pciefd_packet_t *packet);

// +----------------------------------------------------------------------------
// | Transmit packets
// +----------------------------------------------------------------------------
void setRemoteRequest(pciefd_packet_t *packet);
void setExtendedId(pciefd_packet_t *packet);
void setBitRateSwitch(pciefd_packet_t *packet);
void setAckRequest(pciefd_packet_t *packet);
void setTxRequest(pciefd_packet_t *packet);

int setupBaseFormat(pciefd_packet_t *packet, int id, int dlc, int seqno);
int setupExtendedFormat(pciefd_packet_t *packet, int id, int dlc, int seqno);
int setupFDBaseFormat(pciefd_packet_t *packet, int id, int dlc, int seqno);
int setupFDExtendedFormat(pciefd_packet_t *packet, int id, int dlc, int seqno);

#endif
