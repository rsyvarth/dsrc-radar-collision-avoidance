#include "HAL/inc/pciefd.h"
#include "pciefd_hwif.h"

char *pciefd_ctrl_reg_names[] = {
  "cmd",
  "ioc",
  "ien",
  "irq",
  "rx_words",
  "rx_packets",
  "tx_words",
  "tx_packets",
  "flags",
};

int statResetRequest(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return (tmp & PCIEFD_STAT_RM_MSK) != 0;
}

int statInResetMode(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return (tmp & PCIEFD_STAT_ISRM_MSK) != 0;
}

int statTransmitterIdle(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return (tmp & PCIEFD_STAT_TX_IDLE_MSK) != 0;
}

int statAbortRequest(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return (tmp & PCIEFD_STAT_ABORT_REQ_MSK) != 0;
}

int statPendingRequest(void *base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  if(PCIEFD_STAT_SRQ_ACTIVE_GET(tmp))
    {
      return PCIEFD_STAT_SRQ_ID_GET(tmp);
    }
  return -1;
}

int statIdle(void *base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return PCIEFD_STAT_IDLE_STATE_GET(tmp);
}

int statFlushRequest(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_STAT(base);

  return (tmp & PCIEFD_STAT_TX_FLUSH_REQ_MSK) != 0;
}

int istatTransmitterFlushDone(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_ISTAT(base);

  return (tmp & PCIEFD_IRQ_TFD_MSK) != 0;
}

void irqClearTransmitterFlushDone(void * base)
{
  IOWR_PCIEFD_IRQ(base, PCIEFD_IRQ_TFD_MSK);
}

int fifoPacketCountTx(void * base)
{
  return PCIEFD_TX_PACKET_COUNT_GET((int)IORD_PCIEFD_TX_PACKET_COUNT(base));
}

int fifoPacketCountTxMax(void * base)
{
  return PCIEFD_TX_PACKET_MAXCOUNT_GET((int)IORD_PCIEFD_TX_PACKET_COUNT(base)) - 1; // subtract one for allocated packet
}

int readIrqReceiverOverflow(void * base)
{
  uint32_t irq = IORD_PCIEFD_IRQ(base);

  if(irq & PCIEFD_IRQ_ROF_MSK)
    {
      IOWR_PCIEFD_IRQ(base, PCIEFD_IRQ_ROF_MSK);
      return 1;
    }

  return 0;
}

unsigned int irqReceiverOverflow(unsigned int irq)
{
  return irq & PCIEFD_IRQ_ROF_MSK;
}

int readIrqTransmitterOverflow(void * base)
{
  uint32_t irq = IORD_PCIEFD_IRQ(base);

  if(irq & PCIEFD_IRQ_TOF_MSK)
    {
      IOWR_PCIEFD_IRQ(base, PCIEFD_IRQ_TOF_MSK);
      return 1;
    }

  return 0;
}

unsigned int irqTransmitterOverflow(unsigned int irq)
{
  return irq & PCIEFD_IRQ_TOF_MSK;
}

unsigned int irqTransmitterEmpty(unsigned int irq)
{
  return irq & PCIEFD_IRQ_TE_MSK;
}

int irqDisableTransmitterEmpty(void * base)
{
  uint32_t tmp = IORD_PCIEFD_IEN(base);

  IOWR_PCIEFD_IEN(base, tmp & ~PCIEFD_IRQ_TE_MSK);

  return 0;
}

int readIrqTransmitterUnaligned(void * base)
{
  uint32_t irq = IORD_PCIEFD_IRQ(base);

  if(irq & PCIEFD_IRQ_TAL_MSK)
    {
      IOWR_PCIEFD_IRQ(base, PCIEFD_IRQ_TAL_MSK);
      return 1;
    }

  return 0;
}

unsigned int irqTransmitterUnaligned(unsigned int irq)
{
  return irq & PCIEFD_IRQ_TAL_MSK;
}

void irqEnableTransmitterError(void * base)
{
  uint32_t tmp;
  tmp = IORD_PCIEFD_IEN(base);

  IOWR_PCIEFD_IEN(base, tmp
                | PCIEFD_IRQ_TAE_MSK
                | PCIEFD_IRQ_TAR_MSK
                | PCIEFD_IRQ_BPP_MSK
                | PCIEFD_IRQ_FDIC_MSK
                | PCIEFD_IRQ_TOF_MSK
                | PCIEFD_IRQ_TAL_MSK);
}

void irqEnableTransmitFifoEmpty(void * base)
{
  uint32_t tmp;
  tmp = IORD_PCIEFD_IEN(base);

  IOWR_PCIEFD_IEN(base, tmp | PCIEFD_IRQ_TE_MSK);
}

void irqClearTransmitFifoEmpty(void * base)
{
  IOWR_PCIEFD_IRQ(base, PCIEFD_IRQ_TE_MSK);
}

int irqEnabled(void * base)
{
  return IORD_PCIEFD_IEN(base);
}

unsigned int irqStatus(void * base)
{
  return IORD_PCIEFD_IRQ(base);
}

void irqClear(void * base, unsigned int icl)
{
  IOWR_PCIEFD_IRQ(base, icl);
}

int istatCheck(void * base, uint32_t icl)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_ISTAT(base);

  return (tmp & icl) == icl;
}

void irqInit(void * base)
{
  IOWR_PCIEFD_IEN(base, 0);
  IOWR_PCIEFD_IRQ(base, -1);
}

// Register access
void nominalBitRate(void *base, int seg1, int seg2, int sjw, int brp)
{
  uint32_t tmp;

  tmp = PCIEFD_BTR_SEG2(seg2) | PCIEFD_BTR_SEG1(seg1) | PCIEFD_BTR_SJW(sjw) | PCIEFD_BTR_BRP(brp);

  IOWR_PCIEFD_BTRN(base, tmp);
}

void dataPhaseBitRate(void *base, int seg1, int seg2, int sjw, int brp)
{
  uint32_t tmp;

  tmp = PCIEFD_BTR_SEG2(seg2) | PCIEFD_BTR_SEG1(seg1) | PCIEFD_BTR_SJW(sjw) | PCIEFD_BTR_BRP(brp);

  IOWR_PCIEFD_BTRD(base, tmp);
}

void busLoadPrescaler(void *base, int prescaler, int interval)
{
  uint32_t tmp;

  tmp = PCIEFD_BLP_PRESC(prescaler) | PCIEFD_BLP_INTERV(interval);

  IOWR_PCIEFD_BLP(base, tmp);
}

void init(void *base, int rm, int lom, int een, int sso, int egen, int dwh)
{
  uint32_t tmp;

  tmp = PCIEFD_MOD_EWL(96)  |
        PCIEFD_MOD_LOM(lom) |
        PCIEFD_MOD_EEN(een) |
        PCIEFD_MOD_SSO(sso) |
        PCIEFD_MOD_DWH(dwh);

  IOWR_PCIEFD_MOD(base, tmp);
}


void enableErrorPackets(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp | PCIEFD_MOD_EPEN_MSK);
}

void disableErrorPackets(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp & ~PCIEFD_MOD_EPEN_MSK);
}

void enableNonIsoMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);
  
  IOWR_PCIEFD_MOD(base, tmp | PCIEFD_MOD_NIFDEN_MSK);
}

void disableNonIsoMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);
  
  IOWR_PCIEFD_MOD(base, tmp & ~PCIEFD_MOD_NIFDEN_MSK);
}

void enableSlaveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp | PCIEFD_MOD_S_MODE_MSK);
}

void disableSlaveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp & ~PCIEFD_MOD_S_MODE_MSK);
}

unsigned int getSlaveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  return (tmp & PCIEFD_MOD_S_MODE_MSK) != 0;
}

void enableErrorPassiveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp & ~PCIEFD_MOD_EEN_MSK);
}

void disableErrorPassiveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp | PCIEFD_MOD_EEN_MSK);
}

unsigned int getErrorPassiveMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  return (tmp & PCIEFD_MOD_EEN_MSK) == 0;
}

void enableClassicCanMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp & ~PCIEFD_MOD_CLASSIC_MSK);
}

void disableClassicCanMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  IOWR_PCIEFD_MOD(base, tmp | PCIEFD_MOD_CLASSIC_MSK);
}

unsigned int getClassicCanMode(void * base)
{
  uint32_t tmp = IORD_PCIEFD_MOD(base);

  return PCIEFD_MOD_CLASSIC_GET(tmp);
}

void fpgaStatusRequest(void * base, int seq_no)
{
  IOWR_PCIEFD_CCMD(base, PCIEFD_CCMD_SRQ(1) | PCIEFD_CCMD_SEQ_NO(seq_no));
}

void fpgaFlushTransmit(void * base)
{
  IOWR_PCIEFD_CCMD(base, PCIEFD_CCMD_FTX(1));
}

void fpgaFlushAll(void * base, int seq_no)
{
  IOWR_PCIEFD_CCMD(base, PCIEFD_CCMD_AT(1) | PCIEFD_CCMD_SEQ_NO(seq_no));
}

void resetErrorCount(void * base)
{
  IOWR_PCIEFD_CCMD(base, PCIEFD_CCMD_REC(1) | PCIEFD_CCMD_TEC(1) | PCIEFD_CCMD_EC_DATA(0));
}

int writeFIFO(VCanChanData *vChd, pciefd_packet_t *packet)
{
  PciCanChanData *hChd = vChd->hwChanData;
  int nbytes;
  int nwords;
  int dlc = getDLC(packet);
  void *address = hChd->canControllerBase;

  if (isFlexibleDataRateFormat(packet)) {
    nbytes = dlcToBytesFD(dlc);
  }
  else {
    nbytes = dlcToBytes(dlc);
  }

  nwords = bytesToWordsCeil(nbytes);

  if (isRemoteRequest(packet)) {
    nbytes = 0;
    nwords = 0;
  }

  IOWR_REP_PCIEFD_FIFO(address, &packet[0], nwords + 1);
  IOWR_PCIEFD_FIFO_LAST(address, ((uint32_t*)packet)[nwords + 1]);

  return 1;
}

int hwSupportCanFD(void * base)
{
  uint32_t data;

  data = IORD_PCIEFD_STAT(base);

  return PCIEFD_STAT_CAN_FD_GET(data);
}

void setLedStatus(void * base, int on)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_IOC(base);

  if(on)
    {
      IOWR_PCIEFD_IOC(base, tmp & ~PCIEFD_IOC_LED_MSK);
    }
  else
    {
      IOWR_PCIEFD_IOC(base, tmp | PCIEFD_IOC_LED_MSK);
    }
}

void ledOn(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_IOC(base);

  IOWR_PCIEFD_IOC(base, tmp & ~PCIEFD_IOC_LED_MSK);
}

void ledOff(void * base)
{
  uint32_t tmp;

  tmp = IORD_PCIEFD_IOC(base);

  IOWR_PCIEFD_IOC(base, tmp | PCIEFD_IOC_LED_MSK);
}
