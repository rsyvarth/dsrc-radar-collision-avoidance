// Packet handling
#include "HAL/inc/pciefd_packet.h"

int isDataPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_DATA;
}

int isAckPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_ACK;
}

int isTxrqPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_TXRQ;
}

int isErrorPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_ERROR;
}

int isEFlushAckPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_EFLUSH_ACK;
}

int isEFrameAckPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_EFRAME_ACK;
}

int isOffsetPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_OFFSET;
}

int isBusLoadPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_BUS_LOAD;
}

int isStatusPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_STATUS;
}


int isDelayPacket(pciefd_packet_t *packet)
{
  return RPACKET_PTYPE_GET(packet->control) == RPACKET_PTYPE_DELAY;
}

int packetChannelId(pciefd_packet_t *packet)
{
  return RPACKET_CHID_GET(packet->control);
}

// +----------------------------------------------------------------------------
// | Status packet
// | * Can also be used for error packets as packet->id has the same fields
// +----------------------------------------------------------------------------
int statusInResetMode(pciefd_packet_t *packet)
{
  return SPACKET_IRM_GET(packet->id);
}

int statusResetModeChangeDetected(pciefd_packet_t *packet)
{
  return SPACKET_RMCD_GET(packet->id);
}

int statusReset(pciefd_packet_t *packet)
{
  return SPACKET_RESET_GET(packet->id);
}

int statusOverflow(pciefd_packet_t *packet)
{
  return SPACKET_ROVF_GET(packet->control);
}

int statusErrorWarning(pciefd_packet_t *packet)
{
  return SPACKET_EWLR_GET(packet->control);
}

int statusErrorPassive(pciefd_packet_t *packet)
{
  return SPACKET_EPLR_GET(packet->control);
}

int statusBusOff(pciefd_packet_t *packet)
{
  return SPACKET_BOFF_GET(packet->id);
}

int statusReceiveErrorCount(pciefd_packet_t *packet)
{
  return SPACKET_RXERR_GET(packet->id);
}

int statusTransmitErrorCount(pciefd_packet_t *packet)
{
  return SPACKET_TXERR_GET(packet->id);
}

int statusCmdSeqNo(pciefd_packet_t *packet)
{
  return SPACKET_CMD_SEQ_GET(packet->control);
}

// +----------------------------------------------------------------------------
// Received data packets
// +----------------------------------------------------------------------------
int getId(pciefd_packet_t *packet)
{
  return RTPACKET_ID_GET(packet->id);
}

void setId(pciefd_packet_t *packet, int id)
{
  id <<= RTPACKET_ID_LSHIFT;
  id &= RTPACKET_ID_MSK;

  packet->id &= ~RTPACKET_ID_MSK;
  packet->id |= id;
}

int isErrorPassive(pciefd_packet_t *packet)
{
  return RPACKET_EPLR_GET(packet->control);
}

int receiverOverflow(pciefd_packet_t *packet)
{
  return RPACKET_ROVF_GET(packet->control);
}

int errorWarning(pciefd_packet_t *packet)
{
  return RPACKET_EWLR_GET(packet->control);
}

int isFlexibleDataRateFormat(pciefd_packet_t *packet)
{
  return RTPACKET_FDF_GET(packet->control);
}

int isAlternateBitRate(pciefd_packet_t *packet)
{
  return RTPACKET_BRS_GET(packet->control);
}

int errorStateIndicated(pciefd_packet_t *packet)
{
  return RTPACKET_ESI_GET(packet->control);
}

int getSRR(pciefd_packet_t *packet)
{
  return RTPACKET_SRR_GET(packet->id);
}

int isExtendedId(pciefd_packet_t *packet)
{
  return RTPACKET_IDE_GET(packet->id);
}

int isRemoteRequest(pciefd_packet_t *packet)
{
  return RTPACKET_RTR_GET(packet->id);
}

int getDLC(pciefd_packet_t *packet)
{
  return RTPACKET_DLC_GET(packet->control);
}

void setDLC(pciefd_packet_t *packet, int dlc)
{
  packet->control &= ~RTPACKET_DLC_MSK;
  packet->control |= RTPACKET_DLC(dlc);
}

int signext(int value,int signpos)
{
  if ( value & (1<<signpos) ) {
    return value | ~((1UL << (signpos+1)) - 1);
  }

  return value;
}

// +----------------------------------------------------------------------------
// Received offset packets
// +----------------------------------------------------------------------------
int getOffsetQuantas(pciefd_packet_t *packet)
{
  int quantas = OPACKET_OFFSET_GET(packet->id); // 13-bits

  return signext(quantas,12);
}

int getOffsetNbits(pciefd_packet_t *packet)
{
  return OPACKET_NBITS_GET(packet->id);
}

int getBusLoad(pciefd_packet_t *packet)
{
  return BPACKET_BUS_LOAD_GET(packet->id);
}

int getSeqNo(pciefd_packet_t *packet)
{
  return RTPACKET_SEQ_GET(packet->control);
}

// +----------------------------------------------------------------------------
// | Received ack packets
// +----------------------------------------------------------------------------
int getAckSeqNo(pciefd_packet_t *packet)
{
  return APACKET_SEQ_GET(packet->id);
}

int isFlushed(pciefd_packet_t *packet)
{
  return APACKET_FLUSHED_GET(packet->id);
}

int isControlAck(pciefd_packet_t *packet)
{
  return APACKET_CONTROL_ACK_GET(packet->id);
}

// +----------------------------------------------------------------------------
// | Received txrq packets
// +----------------------------------------------------------------------------
int getTxrqSeqNo(pciefd_packet_t *packet)
{
  return APACKET_SEQ_GET(packet->id);
}

// +----------------------------------------------------------------------------
// | Received error packets
// +----------------------------------------------------------------------------
int getTransmitErrorCount(pciefd_packet_t *packet)
{
  return EPACKET_TXE_GET(packet->id);
}

int getReceiveErrorCount(pciefd_packet_t *packet)
{
  return EPACKET_RXE_GET(packet->id);
}

etype_t getErrorType(pciefd_packet_t *packet)
{
  return (etype_t)EPACKET_TYPE_GET(packet->control);
}

static eseg_t getErrorSegment(pciefd_packet_t *packet)
{
  return (eseg_t)EPACKET_SEG_GET(packet->control);
}

static eseg2_t getErrorSegment2(pciefd_packet_t *packet)
{
  return (eseg2_t)EPACKET_SEG2_GET(packet->control);
}

int isTransmitError(pciefd_packet_t *packet)
{
  return EPACKET_DIR_GET(packet->control);
}

int getErrorField(pciefd_packet_t *packet)
{
  if (getErrorSegment(packet) == ESEG_OTHER) {
    return getErrorSegment2(packet) + ESEG_OTHER;
  }

  return getErrorSegment(packet);
}

int getErrorFieldPos(pciefd_packet_t *packet)
{
  if (getErrorSegment(packet) == ESEG_OTHER) {
    return 1; // No field pos available, always report bit pos 1
  }

  return (int)getErrorSegment2(packet);
}

static char *etype_str[] =
  {
    "No",
    "Bit",
    "Bit (ssp)",
    "Stuff (arbitration)",
    "Stuff",
    "Form",
    "Ack",
    "Crc"
  };

static char *epos_str[] =
  {
    "eid",
    "crc",
    "bid",
    "dlc",
    "eof",
    "overload flag",
    "active error flag",
    "passive error flag",
    "error delim",
    "intermission",
    "sof",
    "srr/rtr",
    "ide",
    "rtr",
    "r0",
    "fd r0",
    "r1",
    "brs",
    "esi",
    "data",
    "crc delim",
    "ack slot",
    "ack delim",
    "BUG"
  };

void printErrorCode(pciefd_packet_t *packet)
{
  unsigned int len=0;
  char buf[512];

  if (isTransmitError(packet)) {
    len += sprintf(buf + len, "#                                        Transmit ");
  }
  else {
    len += sprintf(buf + len, "#                                        Receive ");
  }

  len += sprintf(buf + len, "%s Error ",etype_str[getErrorType(packet)]);
  len += sprintf(buf + len, "in field %s ",epos_str[getErrorField(packet)]);

  if (getErrorSegment(packet) != ESEG_OTHER) {
    len += sprintf(buf + len, "pos %u ",getErrorFieldPos(packet));
  }

  len += sprintf(buf + len, "\n");

#ifdef PCIEFD_DEBUG
  printk("<1>%s",buf);
#endif
}

int dlcToBytes(int dlc)
{
  dlc &= 0xf;

  if ( dlc > 8) return 8;

  return dlc;
}

int dlcToBytesFD(int dlc)
{
  dlc &= 0xf;

  switch(dlc) {
  case 9:   return 12;
  case 10:  return 16;
  case 11:  return 20;
  case 12:  return 24;
  case 13:  return 32;
  case 14:  return 48;
  case 15:  return 64;
  default:  return dlc;
  }
}

int bytesToDlc(int bytes)
{
  if (bytes > 64) {
    printk("<1>Invalid number of bytes, %u.",bytes);
  }

  if(bytes > 48)
    return DLC64;
  else if(bytes > 32)
    return DLC48;
  else if(bytes > 24)
    return DLC32;
  else if(bytes > 20)
    return DLC24;
  else if(bytes > 16)
    return DLC20;
  else if(bytes > 12)
    return DLC16;
  else if(bytes > 8)
    return DLC12;
  else
    return bytes;
}

int bytesToWordsCeil(int bytes)
{
  return (bytes+3)/4;
}

// +----------------------------------------------------------------------------
// | Transmit packets
// +----------------------------------------------------------------------------
void setRemoteRequest(pciefd_packet_t *packet)
{
  packet->id |= RTPACKET_RTR(1);
}

void setExtendedId(pciefd_packet_t *packet)
{
  packet->id |= RTPACKET_IDE(1) | RTPACKET_SRR(1);
}

void setBitRateSwitch(pciefd_packet_t *packet)
{
  packet->control |= RTPACKET_BRS(1);
}

void setAckRequest(pciefd_packet_t *packet)
{
  packet->control |= TPACKET_AREQ(1);
}

void setTxRequest(pciefd_packet_t *packet)
{
  packet->control |= TPACKET_TREQ(1);
}

int setupBaseFormat(pciefd_packet_t *packet, int id, int dlc, int seqno)
{
  packet->id = RTPACKET_RTR(0) | RTPACKET_IDE(0) | RTPACKET_SRR(0) | RTPACKET_ID(id);
  packet->control = RTPACKET_DLC(dlc) | RTPACKET_BRS(0) | RTPACKET_FDF(0) | RTPACKET_SEQ(seqno);

  return dlcToBytes(dlc);
}


int setupExtendedFormat(pciefd_packet_t *packet, int id, int dlc, int seqno)
{
  packet->id = RTPACKET_RTR(0) | RTPACKET_IDE(1) | RTPACKET_SRR(1) | RTPACKET_ID(id);
  packet->control = RTPACKET_DLC(dlc) | RTPACKET_BRS(0) | RTPACKET_FDF(0) | RTPACKET_SEQ(seqno);

  return dlcToBytes(dlc);
}

int setupFDBaseFormat(pciefd_packet_t *packet, int id, int dlc, int seqno)
{
  packet->id = RTPACKET_RTR(0) | RTPACKET_IDE(0) | RTPACKET_SRR(0) | RTPACKET_ID(id);
  packet->control = RTPACKET_DLC(dlc) | RTPACKET_BRS(0) | RTPACKET_FDF(1) | RTPACKET_SEQ(seqno);

  return dlcToBytesFD(dlc);
}

int setupFDExtendedFormat(pciefd_packet_t *packet, int id, int dlc, int seqno)
{
  packet->id = RTPACKET_RTR(0) | RTPACKET_IDE(1) | RTPACKET_SRR(1) | RTPACKET_ID(id);
  packet->control = RTPACKET_DLC(dlc) | RTPACKET_BRS(0) | RTPACKET_FDF(1) | RTPACKET_SEQ(seqno);

  return dlcToBytesFD(dlc);
}
