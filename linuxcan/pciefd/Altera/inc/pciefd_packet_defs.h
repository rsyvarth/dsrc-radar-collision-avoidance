#ifndef __PCIEFD_PACKET_DEFS_H__
#define __PCIEFD_PACKET_DEFS_H__

#include "inc/bits.h"

// +----------------------------------------------------------------------------
// | Received and transmitted Packets (RTPACKET)
// | Received Packets (RPACKET)
// | Transmitted Packets (TPACKET)
// +----------------------------------------------------------------------------

// Id Field (W0)
#define RTPACKET_SRR_LSHIFT             31
#define RTPACKET_SRR_NBITS              1
#define RTPACKET_SRR_MSK                mask(RTPACKET_SRR)
#define RTPACKET_SRR_GET(value)         get(RTPACKET_SRR, value)
#define RTPACKET_SRR(value)             field(RTPACKET_SRR, value)

#define RTPACKET_IDE_LSHIFT             30
#define RTPACKET_IDE_NBITS              1
#define RTPACKET_IDE_MSK                mask(RTPACKET_IDE)
#define RTPACKET_IDE_GET(value)         get(RTPACKET_IDE, value)
#define RTPACKET_IDE(value)             field(RTPACKET_IDE, value)

#define RTPACKET_RTR_LSHIFT             29
#define RTPACKET_RTR_NBITS              1
#define RTPACKET_RTR_MSK                mask(RTPACKET_RTR)
#define RTPACKET_RTR_GET(value)         get(RTPACKET_RTR, value)
#define RTPACKET_RTR(value)             field(RTPACKET_RTR, value)

#define RTPACKET_ID_LSHIFT              0
#define RTPACKET_ID_NBITS               29
#define RTPACKET_ID_MSK                 mask(RTPACKET_ID)
#define RTPACKET_ID_GET(value)          get(RTPACKET_ID, value)
#define RTPACKET_ID(value)              field(RTPACKET_ID, value)

// Control field
// Control field (W1)
#define TPACKET_AREQ_LSHIFT             31
#define TPACKET_AREQ_NBITS              1
#define TPACKET_AREQ_MSK                mask(TPACKET_AREQ)
#define TPACKET_AREQ_GET(value)         get(TPACKET_AREQ, value)
#define TPACKET_AREQ(value)             field(TPACKET_AREQ, value)

#define TPACKET_TREQ_LSHIFT             30
#define TPACKET_TREQ_NBITS              1
#define TPACKET_TREQ_MSK                mask(TPACKET_TREQ)
#define TPACKET_TREQ_GET(value)         get(TPACKET_TREQ, value)
#define TPACKET_TREQ(value)             field(TPACKET_TREQ, value)

#define RPACKET_PTYPE_LSHIFT            28
#define RPACKET_PTYPE_NBITS             4
#define RPACKET_PTYPE_MSK               mask(RPACKET_PTYPE)
#define RPACKET_PTYPE_GET(value)        get(RPACKET_PTYPE, value)

#define RPACKET_PTYPE_DATA              0
#define RPACKET_PTYPE_ACK               1
#define RPACKET_PTYPE_TXRQ              2
#define RPACKET_PTYPE_ERROR             3
#define RPACKET_PTYPE_EFLUSH_ACK        4
#define RPACKET_PTYPE_EFRAME_ACK        5
#define RPACKET_PTYPE_STATUS            8
#define RPACKET_PTYPE_BUS_LOAD          9
#define RPACKET_PTYPE_OFFSET            10
#define RPACKET_PTYPE_GLITCH            11
#define RPACKET_PTYPE_DIAG              12
#define RPACKET_PTYPE_DELAY             13

#define RPACKET_CHID_LSHIFT             25
#define RPACKET_CHID_NBITS              3
#define RPACKET_CHID_MSK                mask(RPACKET_CHID)
#define RPACKET_CHID_GET(value)         get(RPACKET_CHID, value)

#define RPACKET_EPLR_LSHIFT             24
#define RPACKET_EPLR_NBITS              1
#define RPACKET_EPLR_MSK                mask(RPACKET_EPLR)
#define RPACKET_EPLR_GET(value)         get(RPACKET_EPLR, value)

#define RPACKET_EWLR_LSHIFT             23
#define RPACKET_EWLR_NBITS              1
#define RPACKET_EWLR_MSK                mask(RPACKET_EWLR)
#define RPACKET_EWLR_GET(value)         get(RPACKET_EWLR, value)

#define RPACKET_ROVF_LSHIFT             22
#define RPACKET_ROVF_NBITS              1
#define RPACKET_ROVF_MSK                mask(RPACKET_ROVF)
#define RPACKET_ROVF_GET(value)         get(RPACKET_ROVF, value)

#define RTPACKET_FDF_LSHIFT             15
#define RTPACKET_FDF_NBITS              1
#define RTPACKET_FDF_MSK                mask(RTPACKET_FDF)
#define RTPACKET_FDF_GET(value)         get(RTPACKET_FDF, value)
#define RTPACKET_FDF(value)             field(RTPACKET_FDF, value)

#define RTPACKET_BRS_LSHIFT             14
#define RTPACKET_BRS_NBITS              1
#define RTPACKET_BRS_MSK                mask(RTPACKET_BRS)
#define RTPACKET_BRS_GET(value)         get(RTPACKET_BRS, value)
#define RTPACKET_BRS(value)             field(RTPACKET_BRS, value)

#define RTPACKET_ESI_LSHIFT             13
#define RTPACKET_ESI_NBITS              1
#define RTPACKET_MSK_ESI                mask(RTPACKET_ESI)
#define RTPACKET_ESI_GET(value)         get(RTPACKET_ESI, value)

#define RTPACKET_EHSF_LSHIFT            12
#define RTPACKET_EHSF_NBITS             1
#define RTPACKET_EHSF_MSK               mask(RTPACKET_EHSF)
#define RTPACKET_EHSF_GET(value)        get(RTPACKET_EHSF, value)
#define RTPACKET_EHSF(value)            field(RTPACKET_EHSF, value)

#define RTPACKET_DLC_LSHIFT             8
#define RTPACKET_DLC_NBITS              4
#define RTPACKET_DLC_MSK                mask(RTPACKET_DLC)
#define RTPACKET_DLC_GET(value)         get(RTPACKET_DLC, value)
#define RTPACKET_DLC(value)             field(RTPACKET_DLC, value)

#define RTPACKET_SEQ_LSHIFT             0
#define RTPACKET_SEQ_NBITS              8
#define RTPACKET_SEQ_MSK                mask(RTPACKET_SEQ)
#define RTPACKET_SEQ_GET(value)         get(RTPACKET_SEQ, value)
#define RTPACKET_SEQ(value)             field(RTPACKET_SEQ, value)

// Offset
#define OPACKET_NBITS_LSHIFT            16
#define OPACKET_NBITS_NBITS             10
#define OPACKET_NBITS_MSK               mask(OPACKET_NBITS)
#define OPACKET_NBITS_GET(value)        get(OPACKET_NBITS, value)

#define OPACKET_OFFSET_LSHIFT           0
#define OPACKET_OFFSET_NBITS            13
#define OPACKET_OFFSET_MSK              mask(OPACKET_OFFSET)
#define OPACKET_OFFSET_GET(value)       get(OPACKET_OFFSET, value)

// Bus Load
#define BPACKET_BUS_LOAD_LSHIFT         0
#define BPACKET_BUS_LOAD_NBITS          20
#define BPACKET_BUS_LOAD_MSK            mask(BPACKET_BUS_LOAD)
#define BPACKET_BUS_LOAD_GET(value)     get(BPACKET_BUS_LOAD, value)

// TXACK/TXRQ
#define APACKET_SEQ_LSHIFT              0
#define APACKET_SEQ_NBITS               8
#define APACKET_SEQ_MSK                 mask(APACKET_SEQ)
#define APACKET_SEQ_GET(value)          get(APACKET_SEQ, value)

#define APACKET_FLUSHED_LSHIFT          8
#define APACKET_FLUSHED_NBITS           1
#define APACKET_FLUSHED_MSK             mask(APACKET_FLUSHED)
#define APACKET_FLUSHED_GET(value)      get(APACKET_FLUSHED, value)

#define APACKET_CONTROL_ACK_LSHIFT      9
#define APACKET_CONTROL_ACK_NBITS       1
#define APACKET_CONTROL_ACK_MSK         mask(APACKET_CONTROL_ACK)
#define APACKET_CONTROL_ACK_GET(value)  get(APACKET_CONTROL_ACK, value)

// Error packet
#define EPACKET_IRM_LSHIFT              21
#define EPACKET_IRM_NBITS               1
#define EPACKET_IRM_MSK                 mask(EPACKET_IRM)
#define EPACKET_IRM_GET(value)          get(EPACKET_IRM, value)

#define EPACKET_ROVF_LSHIFT             22
#define EPACKET_ROVF_NBITS              1
#define EPACKET_ROVF_MSK                mask(EPACKET_ROVF)
#define EPACKET_ROVF_GET(value)         get(EPACKET_ROVF, value)

#define EPACKET_EWLR_LSHIFT             23
#define EPACKET_EWLR_NBITS              1
#define EPACKET_EWLR_MSK                mask(EPACKET_EWLR)
#define EPACKET_EWLR_GET(value)         get(EPACKET_EWLR, value)

#define EPACKET_EPLR_LSHIFT             24
#define EPACKET_EPLR_NBITS              1
#define EPACKET_EPLR_MSK                mask(EPACKET_EPLR)
#define EPACKET_EPLR_GET(value)         get(EPACKET_EPLR, value)

#define EPACKET_BOFF_LSHIFT             16
#define EPACKET_BOFF_NBITS              1
#define EPACKET_BOFF_MSK                mask(EPACKET_BOFF)
#define EPACKET_BOFF_GET(value)         get(EPACKET_BOFF, value)

#define EPACKET_TXE_LSHIFT              0
#define EPACKET_TXE_NBITS               8
#define EPACKET_TXE_MSK                 mask(EPACKET_TXE)
#define EPACKET_TXE_GET(value)          get(EPACKET_TXE, value)

#define EPACKET_RXE_LSHIFT              8
#define EPACKET_RXE_NBITS               8
#define EPACKET_RXE_MSK                 mask(EPACKET_RXE)
#define EPACKET_RXE_GET(value)          get(EPACKET_RXE, value)

#define EPACKET_TYPE_LSHIFT             10
#define EPACKET_TYPE_NBITS              3
#define EPACKET_TYPE_MSK                mask(EPACKET_TYPE)
#define EPACKET_TYPE_GET(value)         get(EPACKET_TYPE, value)

#define EPACKET_SEG_LSHIFT              6
#define EPACKET_SEG_NBITS               4
#define EPACKET_SEG_MSK                 mask(EPACKET_SEG)
#define EPACKET_SEG_GET(value)          get(EPACKET_SEG, value)

#define EPACKET_SEG2_LSHIFT             1
#define EPACKET_SEG2_NBITS              5
#define EPACKET_SEG2_MSK                mask(EPACKET_SEG2)
#define EPACKET_SEG2_GET(value)         get(EPACKET_SEG2, value)

#define EPACKET_DIR_LSHIFT              0
#define EPACKET_DIR_NBITS               1
#define EPACKET_DIR_MSK                 mask(EPACKET_DIR)
#define EPACKET_DIR_GET(value)          get(EPACKET_DIR, value)

typedef enum {
  ESEG_EXT_ID,
  ESEG_CRC_SEQ,
  ESEG_BASE_ID,
  ESEG_DLC,
  ESEG_EOF,
  ESEG_OVERLOAD,
  ESEG_ACTIVE_ERROR,
  ESEG_PASSIVE_ERROR,
  ESEG_ERROR_DELIM,
  ESEG_IM,
  ESEG_STUFF_CNT,
  ESEG_OTHER = 15 // Not bit count field
} eseg_t;

typedef  enum {
  ESEG_SOF,
  ESEG_SRR_RTR,
  ESEG_IDE,
  ESEG_RTR,
  ESEG_R0,
  ESEG_FD_R0,
  ESEG_R1,
  ESEG_BRS,
  ESEG_ESI,
  ESEG_DATA,
  ESEG_CRC_DELIM,
  ESEG_ACK_SLOT,
  ESEG_ACK_DELIM,
  ESEG_BUG
} eseg2_t;

typedef enum {
  E_OK,
  E_BIT,
  E_BIT_SSP,
  E_STUFF_ARB,
  E_STUFF,
  E_FORM,
  E_ACK,
  E_CRC
} etype_t;

enum {
  DLC12   = 9,
  DLC16   = 10,
  DLC20   = 11,
  DLC24   = 12,
  DLC32   = 13,
  DLC48   = 14,
  DLC64   = 15
};

// Status packet
#define SPACKET_RMCD_LSHIFT             22
#define SPACKET_RMCD_NBITS              1
#define SPACKET_RMCD_MSK                mask(SPACKET_RMCD)
#define SPACKET_RMCD_GET(value)         get(SPACKET_RMCD, value)

#define SPACKET_IRM_LSHIFT              21
#define SPACKET_IRM_NBITS               1
#define SPACKET_IRM_MSK                 mask(SPACKET_IRM)
#define SPACKET_IRM_GET(value)          get(SPACKET_IRM, value)

#define SPACKET_RESET_LSHIFT            20
#define SPACKET_RESET_NBITS             1
#define SPACKET_RESET_MSK               mask(SPACKET_RESET)
#define SPACKET_RESET_GET(value)        get(SPACKET_RESET, value)

#define SPACKET_AUTO_LSHIFT             21
#define SPACKET_AUTO_NBITS              1
#define SPACKET_AUTO_MSK                mask(SPACKET_AUTO)
#define SPACKET_AUTO_GET(value)         get(SPACKET_AUTO, value)

#define SPACKET_ROVF_LSHIFT             22
#define SPACKET_ROVF_NBITS              1
#define SPACKET_ROVF_MSK                mask(SPACKET_ROVF)
#define SPACKET_ROVF_GET(value)         get(SPACKET_ROVF, value)

#define SPACKET_EWLR_LSHIFT             23
#define SPACKET_EWLR_NBITS              1
#define SPACKET_EWLR_MSK                mask(SPACKET_EWLR)
#define SPACKET_EWLR_GET(value)         get(SPACKET_EWLR, value)

#define SPACKET_EPLR_LSHIFT             24
#define SPACKET_EPLR_NBITS              1
#define SPACKET_EPLR_MSK                mask(SPACKET_EPLR)
#define SPACKET_EPLR_GET(value)         get(SPACKET_EPLR, value)

#define SPACKET_BOFF_LSHIFT             16
#define SPACKET_BOFF_NBITS              1
#define SPACKET_BOFF_MSK                mask(SPACKET_BOFF)
#define SPACKET_BOFF_GET(value)         get(SPACKET_BOFF, value)

#define SPACKET_RXERR_LSHIFT            8
#define SPACKET_RXERR_NBITS             8
#define SPACKET_RXERR_MSK               mask(SPACKET_RXERR)
#define SPACKET_RXERR_GET(value)        get(SPACKET_RXERR, value)

#define SPACKET_TXERR_LSHIFT            0
#define SPACKET_TXERR_NBITS             8
#define SPACKET_TXERR_MSK               mask(SPACKET_TXERR)
#define SPACKET_TXERR_GET(value)        get(SPACKET_TXERR, value)

#define SPACKET_CMD_SEQ_LSHIFT          0
#define SPACKET_CMD_SEQ_NBITS           8
#define SPACKET_CMD_SEQ_MSK             mask(SPACKET_CMD_SEQ)
#define SPACKET_CMD_SEQ_GET(value)      get(SPACKET_CMD_SEQ, value)

// +----------------------------------------------------------------------------
// | Transmitted Packets
// |
// +----------------------------------------------------------------------------
#define TPACKET_PTYPE_EGEN                0
#define TPACKET_PTYPE_TOG                 1
#define TPACKET_PTYPE_GGEN                2
#define TPACKET_PTYPE_OVERLOAD            3
#define TPACKET_PTYPE_END_FLUSH           4
#define TPACKET_PTYPE_ERROR_FRAME         5

#define TPACKET_CONTROL_PTYPE_LSHIFT      29
#define TPACKET_CONTROL_PTYPE_NBITS       3
#define TPACKET_CONTROL_PTYPE_MSK         mask(TPACKET_CONTROL_PTYPE)
#define TPACKET_CONTROL_PTYPE_GET(value)  get(TPACKET_CONTROL_PTYPE, value)
#define TPACKET_CONTROL_PTYPE(value)      field(TPACKET_CONTROL_PTYPE, value)

#define PCIEFD_CONTROL_END_FLUSH          TPACKET_CONTROL_PTYPE(TPACKET_PTYPE_END_FLUSH)
#define PCIEFD_CONTROL_ERROR_FRAME        TPACKET_CONTROL_PTYPE(TPACKET_PTYPE_ERROR_FRAME)

#define PCIEFD_DIAG_COUNT_LSHIFT          16
#define PCIEFD_DIAG_COUNT_NBITS           8
#define PCIEFD_DIAG_COUNT_MSK             mask(PCIEFD_DIAG_COUNT)
#define PCIEFD_DIAG_COUNT_GET(value)      get(PCIEFD_DIAG_COUNT, value)
#define PCIEFD_DIAG_COUNT(value)          field(PCIEFD_DIAG_COUNT, value)

#define PCIEFD_DIAG_EPL_LSHIFT            0
#define PCIEFD_DIAG_EPL_NBITS             1
#define PCIEFD_DIAG_EPL_MSK               mask(PCIEFD_DIAG_EPL)
#define PCIEFD_DIAG_EPL_GET(value)        get(PCIEFD_DIAG_EPL, value)
#define PCIEFD_DIAG_EPL(value)            field(PCIEFD_DIAG_EPL, value)

#define PCIEFD_DIAG_EWL_LSHIFT            31
#define PCIEFD_DIAG_EWL_NBITS             1
#define PCIEFD_DIAG_EWL_MSK               mask(PCIEFD_DIAG_EWL)
#define PCIEFD_DIAG_EWL_GET(value)        get(PCIEFD_DIAG_EWL, value)
#define PCIEFD_DIAG_EWL(value)            field(PCIEFD_DIAG_EWL, value)

#define PCIEFD_DIAG_RXE_LSHIFT            23
#define PCIEFD_DIAG_RXE_NBITS             8
#define PCIEFD_DIAG_RXE_MSK               mask(PCIEFD_DIAG_RXE)
#define PCIEFD_DIAG_RXE_GET(value)        get(PCIEFD_DIAG_RXE, value)
#define PCIEFD_DIAG_RXE(value)            field(PCIEFD_DIAG_RXE, value)

#define PCIEFD_DIAG_TXE_LSHIFT            15
#define PCIEFD_DIAG_TXE_NBITS             8
#define PCIEFD_DIAG_TXE_MSK               mask(PCIEFD_DIAG_TXE)
#define PCIEFD_DIAG_TXE_GET(value)        get(PCIEFD_DIAG_TXE, value)
#define PCIEFD_DIAG_TXE(value)            field(PCIEFD_DIAG_TXE, value)

#define PCIEFD_DIAG_TSEG1_LSHIFT          9
#define PCIEFD_DIAG_TSEG1_NBITS           6
#define PCIEFD_DIAG_TSEG1_MSK             mask(PCIEFD_DIAG_TSEG1)
#define PCIEFD_DIAG_TSEG1_GET(value)      get(PCIEFD_DIAG_TSEG1, value)
#define PCIEFD_DIAG_TSEG1(value)          field(PCIEFD_DIAG_TSEG1, value)

#define PCIEFD_DIAG_TSEG2_LSHIFT          4
#define PCIEFD_DIAG_TSEG2_NBITS           5
#define PCIEFD_DIAG_TSEG2_MSK             mask(PCIEFD_DIAG_TSEG2)
#define PCIEFD_DIAG_TSEG2_GET(value)      get(PCIEFD_DIAG_TSEG2, value)
#define PCIEFD_DIAG_TSEG2(value)          field(PCIEFD_DIAG_TSEG2, value)

#define PCIEFD_DIAG_SJW_LSHIFT            0
#define PCIEFD_DIAG_SJW_NBITS             4
#define PCIEFD_DIAG_SJW_MSK               mask(PCIEFD_DIAG_SJW)
#define PCIEFD_DIAG_SJW_GET(value)        get(PCIEFD_DIAG_SJW, value)
#define PCIEFD_DIAG_SJW(value)            field(PCIEFD_DIAG_SJW, value)

#define PCIEFD_DELAY_LSHIFT               0
#define PCIEFD_DELAY_NBITS                8
#define PCIEFD_DELAY_MSK                  mask(PCIEFD_DELAY)
#define PCIEFD_DELAY_GET(value)           get(PCIEFD_DELAY, value)
#define PCIEFD_DELAY(value)               field(PCIEFD_DELAY, value)

#define PCIEFD_DELAY_HS_START_LSHIFT      8
#define PCIEFD_DELAY_HS_START_NBITS       3
#define PCIEFD_DELAY_HS_START_MSK         mask(PCIEFD_DELAY_HS_START)
#define PCIEFD_DELAY_HS_START_GET(value)  get(PCIEFD_DELAY_HS_START, value)
#define PCIEFD_DELAY_HS_START(value)      field(PCIEFD_DELAY_HS_START, value)

#define PCIEFD_DELAY_HS_STOP_LSHIFT       16
#define PCIEFD_DELAY_HS_STOP_NBITS        3
#define PCIEFD_DELAY_HS_STOP_MSK          mask(PCIEFD_DELAY_HS_STOP)
#define PCIEFD_DELAY_HS_STOP_GET(value)   get(PCIEFD_DELAY_HS_STOP, value)
#define PCIEFD_DELAY_HS_STOP(value)       field(PCIEFD_DELAY_HS_STOP, value)

#define PCIEFD_DELAY_LSHIFT               0
#define PCIEFD_DELAY_NBITS                8
#define PCIEFD_DELAY_MSK                  mask(PCIEFD_DELAY)
#define PCIEFD_DELAY_GET(value)           get(PCIEFD_DELAY, value)
#define PCIEFD_DELAY(value)               field(PCIEFD_DELAY, value)

#endif
