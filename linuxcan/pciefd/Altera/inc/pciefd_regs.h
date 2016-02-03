#ifndef __PCIEFD_REGS_H__
#define __PCIEFD_REGS_H__

#include <inc/io.h>
#include "inc/bits.h"

// +----------------------------------------------------------------------------
// | Address map
// |
// +----------------------------------------------------------------------------

// Bit 0 to 3 is the data or header word offset
// Bit 4 is the header flag
// Bit 5 is the last word flag
// Bit 6 is the fifo mode flag
// Bit 7 is the control word flag
#define PCIEFD_ADDR_HEADER_FLAG           4
#define PCIEFD_ADDR_LAST_FLAG             5
#define PCIEFD_ADDR_FIFO_FLAG             6
#define PCIEFD_ADDR_CONTROL_FLAG          7

#define PCIEFD_HEADER_LAST_BASE           ((1<<PCIEFD_ADDR_HEADER_FLAG) | (1<<PCIEFD_ADDR_LAST_FLAG))
#define PCIEFD_CONTROL_BASE               ((1<<PCIEFD_ADDR_CONTROL_FLAG) | PCIEFD_HEADER_LAST_BASE)

#define PCIEFD_FIFO_BASE                  (1<<PCIEFD_ADDR_FIFO_FLAG)
#define PCIEFD_FIFO_LAST_BASE             ((1<<PCIEFD_ADDR_FIFO_FLAG) | (1<<PCIEFD_ADDR_LAST_FLAG))

// Word address offset for pciefd sub-modules
// (The pciefd address space is a representation of 3 Avalon slave interfaces)
#define PCIEFD_CONTROL_OFFSET             (0x0400/4)  // Convert byte address to 32-bit word address

// Control register (all offsets are in 32-bit word addresses steps)
#define PCIEFD_CCMD_REG                   (PCIEFD_CONTROL_OFFSET + 0)
#define PCIEFD_IOC_REG                    (PCIEFD_CONTROL_OFFSET + 1)
#define PCIEFD_IEN_REG                    (PCIEFD_CONTROL_OFFSET + 2)
#define PCIEFD_ISTAT_REG                  (PCIEFD_CONTROL_OFFSET + 3)
#define PCIEFD_IRQ_REG                    (PCIEFD_CONTROL_OFFSET + 4)
#define PCIEFD_TX_PACKET_COUNT_REG        (PCIEFD_CONTROL_OFFSET + 5)
#define PCIEFD_STAT_REG                   (PCIEFD_CONTROL_OFFSET + 6)
#define PCIEFD_MOD_REG                    (PCIEFD_CONTROL_OFFSET + 7)
#define PCIEFD_BTRN_REG                   (PCIEFD_CONTROL_OFFSET + 8)
#define PCIEFD_BLP_REG                    (PCIEFD_CONTROL_OFFSET + 9)
#define PCIEFD_BTRD_REG                   (PCIEFD_CONTROL_OFFSET + 10)
#define PCIEFD_TX_RATE_LIMIT_REG          (PCIEFD_CONTROL_OFFSET + 11)
#define PCIEFD_PWM_CTRL_REG               (PCIEFD_CONTROL_OFFSET + 12)

// Should read 0xdeadbeef if can controller is responding
#define PCIEFD_UNDEF_REG                  (PCIEFD_PWM_CTRL_REG + 1)

// +----------------------------------------------------------------------------
// | Mode register bits
// |
// +----------------------------------------------------------------------------
#define PCIEFD_MOD_EWL_LSHIFT             0
#define PCIEFD_MOD_EWL_NBITS              8
#define PCIEFD_MOD_EWL_MSK                mask(PCIEFD_MOD_EWL)
#define PCIEFD_MOD_EWL_GET(value)         get(PCIEFD_MOD_EWL, value)
#define PCIEFD_MOD_EWL(value)             field(PCIEFD_MOD_EWL, value)

#define PCIEFD_MOD_RM_LSHIFT              8
#define PCIEFD_MOD_RM_NBITS               1
#define PCIEFD_MOD_RM_MSK                 mask(PCIEFD_MOD_RM)
#define PCIEFD_MOD_RM_GET(value)          get(PCIEFD_MOD_RM, value)
#define PCIEFD_MOD_RM(value)              field(PCIEFD_MOD_RM, value)

#define PCIEFD_MOD_LOM_LSHIFT             9
#define PCIEFD_MOD_LOM_NBITS              1
#define PCIEFD_MOD_LOM_MSK                mask(PCIEFD_MOD_LOM)
#define PCIEFD_MOD_LOM_GET(value)         get(PCIEFD_MOD_LOM, value)
#define PCIEFD_MOD_LOM(value)             field(PCIEFD_MOD_LOM, value)

#define PCIEFD_MOD_DWH_LSHIFT            10
#define PCIEFD_MOD_DWH_NBITS             1
#define PCIEFD_MOD_DWH_MSK               mask(PCIEFD_MOD_DWH)
#define PCIEFD_MOD_DWH_GET(value)        get(PCIEFD_MOD_DWH, value)
#define PCIEFD_MOD_DWH(value)            field(PCIEFD_MOD_DWH, value)

#define PCIEFD_MOD_EPEN_LSHIFT            12
#define PCIEFD_MOD_EPEN_NBITS             1
#define PCIEFD_MOD_EPEN_MSK               mask(PCIEFD_MOD_EPEN)
#define PCIEFD_MOD_EPEN_GET(value)        get(PCIEFD_MOD_EPEN, value)
#define PCIEFD_MOD_EPEN(value)            field(PCIEFD_MOD_EPEN, value)

#define PCIEFD_MOD_S_MODE_LSHIFT          13
#define PCIEFD_MOD_S_MODE_NBITS           1
#define PCIEFD_MOD_S_MODE_MSK             mask(PCIEFD_MOD_S_MODE)
#define PCIEFD_MOD_S_MODE_GET(value)      get(PCIEFD_MOD_S_MODE, value)
#define PCIEFD_MOD_S_MODE(value)          field(PCIEFD_MOD_S_MODE, value)

#define PCIEFD_MOD_NIFDEN_LSHIFT          15
#define PCIEFD_MOD_NIFDEN_NBITS           1
#define PCIEFD_MOD_NIFDEN_MSK             mask(PCIEFD_MOD_NIFDEN)
#define PCIEFD_MOD_NIFDEN_GET(value)      get(PCIEFD_MOD_NIFDEN, value)
#define PCIEFD_MOD_NIFDEN(value)          field(PCIEFD_MOD_NIFDEN, value)

#define PCIEFD_MOD_CHANNEL_LSHIFT         16
#define PCIEFD_MOD_CHANNEL_NBITS          3
#define PCIEFD_MOD_CHANNEL_MSK            mask(PCIEFD_MOD_CHANNEL)
#define PCIEFD_MOD_CHANNEL_GET(value)     get(PCIEFD_MOD_CHANNEL, value)
#define PCIEFD_MOD_CHANNEL(value)         field(PCIEFD_MOD_CHANNEL, value)

#define PCIEFD_MOD_EEN_LSHIFT             23
#define PCIEFD_MOD_EEN_NBITS              1
#define PCIEFD_MOD_EEN_MSK                mask(PCIEFD_MOD_EEN)
#define PCIEFD_MOD_EEN_GET(value)         get(PCIEFD_MOD_EEN, value)
#define PCIEFD_MOD_EEN(value)             field(PCIEFD_MOD_EEN, value)

#define PCIEFD_MOD_SSO_LSHIFT             24
#define PCIEFD_MOD_SSO_NBITS              6
#define PCIEFD_MOD_SSO_MSK                mask(PCIEFD_MOD_SSO)
#define PCIEFD_MOD_SSO_GET(value)         get(PCIEFD_MOD_SSO, value)
#define PCIEFD_MOD_SSO(value)             field(PCIEFD_MOD_SSO, value)

#define PCIEFD_MOD_SWTX_LSHIFT            30
#define PCIEFD_MOD_SWTX_NBITS             1
#define PCIEFD_MOD_SWTX_MSK               mask(PCIEFD_MOD_SWTX)
#define PCIEFD_MOD_SWTX_GET(value)        get(PCIEFD_MOD_SWTX, value)
#define PCIEFD_MOD_SWTX(value)            field(PCIEFD_MOD_SWTX, value)

#define PCIEFD_MOD_CLASSIC_LSHIFT         31
#define PCIEFD_MOD_CLASSIC_NBITS          1
#define PCIEFD_MOD_CLASSIC_MSK            mask(PCIEFD_MOD_CLASSIC)
#define PCIEFD_MOD_CLASSIC_GET(value)     get(PCIEFD_MOD_CLASSIC, value)
#define PCIEFD_MOD_CLASSIC(value)         field(PCIEFD_MOD_CLASSIC, value)

// +----------------------------------------------------------------------------
// | Bit rate register bits
// | Nominal and Data Phase
// +----------------------------------------------------------------------------
#define PCIEFD_BTR_BRP_LSHIFT             0
#define PCIEFD_BTR_BRP_NBITS              13
#define PCIEFD_BTR_BRP_MSK                mask(PCIEFD_BTR_BRP)
#define PCIEFD_BTR_BRP_GET(value)         get(PCIEFD_BTR_BRP, value)
#define PCIEFD_BTR_BRP(value)             field(PCIEFD_BTR_BRP, value)

#define PCIEFD_BTR_SJW_LSHIFT             13
#define PCIEFD_BTR_SJW_NBITS              4
#define PCIEFD_BTR_SJW_MSK                mask(PCIEFD_BTR_SJW)
#define PCIEFD_BTR_SJW_GET(value)         get(PCIEFD_BTR_SJW, value)
#define PCIEFD_BTR_SJW(value)             field(PCIEFD_BTR_SJW, value)

#define PCIEFD_BTR_SEG1_LSHIFT            17
#define PCIEFD_BTR_SEG1_NBITS             9
#define PCIEFD_BTR_SEG1_MSK               mask(PCIEFD_BTR_SEG1)
#define PCIEFD_BTR_SEG1_GET(value)        get(PCIEFD_BTR_SEG1, value)
#define PCIEFD_BTR_SEG1(value)            field(PCIEFD_BTR_SEG1, value)

#define PCIEFD_BTR_SEG2_LSHIFT            26
#define PCIEFD_BTR_SEG2_NBITS             5
#define PCIEFD_BTR_SEG2_MSK               mask(PCIEFD_BTR_SEG2)
#define PCIEFD_BTR_SEG2_GET(value)        get(PCIEFD_BTR_SEG2, value)
#define PCIEFD_BTR_SEG2(value)            field(PCIEFD_BTR_SEG2, value)

// +----------------------------------------------------------------------------
// | Bus load prescaler
// |
// +----------------------------------------------------------------------------
#define PCIEFD_BLP_PRESC_LSHIFT           24
#define PCIEFD_BLP_PRESC_NBITS            8
#define PCIEFD_BLP_PRESC_MSK              mask(PCIEFD_BLP_PRESC)
#define PCIEFD_BLP_PRESC_GET(value)       get(PCIEFD_BLP_PRESC, value)
#define PCIEFD_BLP_PRESC(value)           field(PCIEFD_BLP_PRESC, value)

#define PCIEFD_BLP_INTERV_LSHIFT          0
#define PCIEFD_BLP_INTERV_NBITS           20
#define PCIEFD_BLP_INTERV_MSK             mask(PCIEFD_BLP_INTERV)
#define PCIEFD_BLP_INTERV_GET(value)      get(PCIEFD_BLP_INTERV, value)
#define PCIEFD_BLP_INTERV(value)          field(PCIEFD_BLP_INTERV, value)

// +----------------------------------------------------------------------------
// | Command register
// |
// +----------------------------------------------------------------------------
// SRQ - Request error status packet
#define PCIEFD_CCMD_SRQ_LSHIFT            0
#define PCIEFD_CCMD_SRQ_NBITS             1
#define PCIEFD_CCMD_SRQ_MSK               mask(PCIEFD_CCMD_SRQ)
#define PCIEFD_CCMD_SRQ(value)            field(PCIEFD_CCMD_SRQ, value)

// AT  - Abort and reset
#define PCIEFD_CCMD_AT_LSHIFT             1
#define PCIEFD_CCMD_AT_NBITS              1
#define PCIEFD_CCMD_AT_MSK                mask(PCIEFD_CCMD_AT)
#define PCIEFD_CCMD_AT(value)             field(PCIEFD_CCMD_AT, value)

// FTX - Flush transmit fifo
#define PCIEFD_CCMD_FTX_LSHIFT            2
#define PCIEFD_CCMD_FTX_NBITS             1
#define PCIEFD_CCMD_FTX_MSK               mask(PCIEFD_CCMD_FTX)
#define PCIEFD_CCMD_FTX(value)            field(PCIEFD_CCMD_FTX, value)

// Set TXERR and RXERR
#define PCIEFD_CCMD_REC_LSHIFT            3
#define PCIEFD_CCMD_REC_NBITS             1
#define PCIEFD_CCMD_REC_MSK               mask(PCIEFD_CCMD_REC)
#define PCIEFD_CCMD_REC(value)            field(PCIEFD_CCMD_REC, value)

#define PCIEFD_CCMD_TEC_LSHIFT            4
#define PCIEFD_CCMD_TEC_NBITS             1
#define PCIEFD_CCMD_TEC_MSK               mask(PCIEFD_CCMD_TEC)
#define PCIEFD_CCMD_TEC(value)            field(PCIEFD_CCMD_TEC, value)

#define PCIEFD_CCMD_SEQ_NO_LSHIFT         16
#define PCIEFD_CCMD_SEQ_NO_NBITS          8
#define PCIEFD_CCMD_SEQ_NO_MSK            mask(PCIEFD_CCMD_SEQ_NO)
#define PCIEFD_CCMD_SEQ_NO(value)         field(PCIEFD_CCMD_SEQ_NO, value)

#define PCIEFD_CCMD_EC_DATA_LSHIFT        16
#define PCIEFD_CCMD_EC_DATA_NBITS         9
#define PCIEFD_CCMD_EC_DATA_MSK           mask(PCIEFD_CCMD_EC_DATA)
#define PCIEFD_CCMD_EC_DATA(value)        field(PCIEFD_CCMD_EC_DATA, value)

// +----------------------------------------------------------------------------
// | IO Control register
// |
// +----------------------------------------------------------------------------
#define PCIEFD_IOC_LED_LSHIFT             0
#define PCIEFD_IOC_LED_NBITS              1
#define PCIEFD_IOC_LED_MSK                mask(PCIEFD_IOC_LED)
#define PCIEFD_IOC_LED_GET(value)         get(PCIEFD_IOC_LED, value)
#define PCIEFD_IOC_LED(value)             field(PCIEFD_IOC_LED, value)

// +----------------------------------------------------------------------------
// | Interrupt bit mask
// |
// +----------------------------------------------------------------------------
#define PCIEFD_IRQ_TAR_MSK                (1<<0)  // Tx FIFO Unaligned Read
#define PCIEFD_IRQ_TAE_MSK                (1<<1)  // Tx FIFO Unaligned End
#define PCIEFD_IRQ_BPP_MSK                (1<<2)  // Bus Param Protect Error
#define PCIEFD_IRQ_FDIC_MSK               (1<<3)  // FDF bit sent in classic mode
#define PCIEFD_IRQ_IDLE_MSK               (1<<4)  // Idle State
#define PCIEFD_IRQ_ROF_MSK                (1<<5)  // Rx Fifo Overflow
#define PCIEFD_IRQ_ABD_MSK                (1<<13) // Abort Done
#define PCIEFD_IRQ_TFD_MSK                (1<<14) // Tx Buffer Flush Sone
#define PCIEFD_IRQ_TOF_MSK                (1<<15) // Tx FIFO Overflow
#define PCIEFD_IRQ_TE_MSK                 (1<<16) // Tx FIFO Empty
#define PCIEFD_IRQ_TAL_MSK                (1<<17) // Tx Unaligned


// +----------------------------------------------------------------------------
// | Packet count
// |
// +----------------------------------------------------------------------------
#define PCIEFD_TX_PACKET_COUNT_LSHIFT          0
#define PCIEFD_TX_PACKET_COUNT_NBITS           8
#define PCIEFD_TX_PACKET_COUNT_MSK             mask(PCIEFD_TX_PACKET_COUNT)
#define PCIEFD_TX_PACKET_COUNT_GET(value)      get(PCIEFD_TX_PACKET_COUNT, value)
#define PCIEFD_TX_PACKET_COUNT(value)          field(PCIEFD_TX_PACKET_COUNT, value)

#define PCIEFD_TX_PACKET_MAXCOUNT_LSHIFT      16
#define PCIEFD_TX_PACKET_MAXCOUNT_NBITS       8
#define PCIEFD_TX_PACKET_MAXCOUNT_MSK         mask(PCIEFD_TX_PACKET_MAXCOUNT)
#define PCIEFD_TX_PACKET_MAXCOUNT_GET(value)  get(PCIEFD_TX_PACKET_MAXCOUNT, value)
#define PCIEFD_TX_PACKET_MAXCOUNT(value)      field(PCIEFD_TX_PACKET_MAXCOUNT, value)

// +----------------------------------------------------------------------------
// | Status
// |
// +----------------------------------------------------------------------------
#define PCIEFD_STAT_SOP_LSHIFT          6
#define PCIEFD_STAT_SOP_NBITS           1
#define PCIEFD_STAT_SOP_MSK             mask(PCIEFD_STAT_SOP)
#define PCIEFD_STAT_SOP_GET(value)      get(PCIEFD_STAT_SOP, value)
#define PCIEFD_STAT_SOP(value)          field(PCIEFD_STAT_SOP, value)

#define PCIEFD_STAT_EOP_LSHIFT          5
#define PCIEFD_STAT_EOP_NBITS           1
#define PCIEFD_STAT_EOP_MSK             mask(PCIEFD_STAT_EOP)
#define PCIEFD_STAT_EOP_GET(value)      get(PCIEFD_STAT_EOP, value)
#define PCIEFD_STAT_EOP(value)          field(PCIEFD_STAT_EOP, value)

#define PCIEFD_STAT_AVAIL_LSHIFT        4
#define PCIEFD_STAT_AVAIL_NBITS         1
#define PCIEFD_STAT_AVAIL_MSK           mask(PCIEFD_STAT_AVAIL)
#define PCIEFD_STAT_AVAIL_GET(value)    get(PCIEFD_STAT_AVAIL, value)
#define PCIEFD_STAT_AVAIL(value)        field(PCIEFD_STAT_AVAIL, value)

#define PCIEFD_STAT_CAN_FD_LSHIFT       19
#define PCIEFD_STAT_CAN_FD_NBITS        1
#define PCIEFD_STAT_CAN_FD_MSK          mask(PCIEFD_STAT_CAN_FD)
#define PCIEFD_STAT_CAN_FD_GET(value)   get(PCIEFD_STAT_CAN_FD, value)
#define PCIEFD_STAT_CAN_FD(value)       field(PCIEFD_STAT_CAN_FD, value)

#define PCIEFD_STAT_HS_SAMPLING_LSHIFT     16
#define PCIEFD_STAT_HS_SAMPLING_NBITS      1
#define PCIEFD_STAT_HS_SAMPLING_MSK        mask(PCIEFD_STAT_HS_SAMPLING)
#define PCIEFD_STAT_HS_SAMPLING_GET(value) get(PCIEFD_STAT_HS_SAMPLING, value)
#define PCIEFD_STAT_HS_SAMPLING(value)     field(PCIEFD_STAT_HS_SAMPLING, value)

#define PCIEFD_STAT_ISRM_MSK            (1<< 15)
#define PCIEFD_STAT_RM_MSK              (1<< 14)
#define PCIEFD_STAT_TX_IDLE_MSK         (1<< 13)
#define PCIEFD_STAT_TX_ENABLE_MSK       (1<< 12)
#define PCIEFD_STAT_BUS_OFF_MSK         (1<< 11)
#define PCIEFD_STAT_MODE_UPDATED_MSK    (1<< 10)
#define PCIEFD_STAT_IRQ_MSK             (1<< 9)
#define PCIEFD_STAT_TX_FLUSH_REQ_MSK    (1<< 8)
#define PCIEFD_STAT_ABORT_REQ_MSK       (1<< 7)

#define PCIEFD_STAT_IDLE_STATE_LSHIFT     10
#define PCIEFD_STAT_IDLE_STATE_NBITS      1
#define PCIEFD_STAT_IDLE_STATE_MSK        mask(PCIEFD_STAT_IDLE_STATE)
#define PCIEFD_STAT_IDLE_STATE_GET(value) get(PCIEFD_STAT_IDLE_STATE, value)
#define PCIEFD_STAT_IDLE_STATE(value)     field(PCIEFD_STAT_IDLE_STATE, value)

#define PCIEFD_STAT_SRQ_ACTIVE_LSHIFT     6
#define PCIEFD_STAT_SRQ_ACTIVE_NBITS      1
#define PCIEFD_STAT_SRQ_ACTIVE_MSK        mask(PCIEFD_STAT_SRQ_ACTIVE)
#define PCIEFD_STAT_SRQ_ACTIVE_GET(value) get(PCIEFD_STAT_SRQ_ACTIVE, value)
#define PCIEFD_STAT_SRQ_ACTIVE(value)     field(PCIEFD_STAT_SRQ_ACTIVE, value)

#define PCIEFD_STAT_SRQ_ID_LSHIFT     24
#define PCIEFD_STAT_SRQ_ID_NBITS      8
#define PCIEFD_STAT_SRQ_ID_MSK        mask(PCIEFD_STAT_SRQ_ID)
#define PCIEFD_STAT_SRQ_ID_GET(value) get(PCIEFD_STAT_SRQ_ID, value)
#define PCIEFD_STAT_SRQ_ID(value)     field(PCIEFD_STAT_SRQ_ID, value)

#define PCIEFD_PWM_CTRL_TRIGGER_LSHIFT     0
#define PCIEFD_PWM_CTRL_TRIGGER_NBITS      8
#define PCIEFD_PWM_CTRL_TRIGGER_MSK        mask(PCIEFD_PWM_CTRL_TRIGGER)
#define PCIEFD_PWM_CTRL_TRIGGER_GET(value) get(PCIEFD_PWM_CTRL_TRIGGER, value)
#define PCIEFD_PWM_CTRL_TRIGGER(value)     field(PCIEFD_PWM_CTRL_TRIGGER, value)

#define PCIEFD_PWM_CTRL_TOP_LSHIFT     16
#define PCIEFD_PWM_CTRL_TOP_NBITS      8
#define PCIEFD_PWM_CTRL_TOP_MSK        mask(PCIEFD_PWM_CTRL_TOP)
#define PCIEFD_PWM_CTRL_TOP_GET(value) get(PCIEFD_PWM_CTRL_TOP, value)
#define PCIEFD_PWM_CTRL_TOP(value)     field(PCIEFD_PWM_CTRL_TOP, value)

// +----------------------------------------------------------------------------
// | Register Access
// |
// +----------------------------------------------------------------------------
#define IOADDR_PCIEFD_FIFO(base)               __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_FIFO_BASE)
#define IOWR_PCIEFD_FIFO(base, data)           IOWR(base, PCIEFD_FIFO_BASE, data)
#define IOWR_REP_PCIEFD_FIFO(base, data, rep)  IOWR_REP(base, PCIEFD_FIFO_BASE, data, rep)

#define IOADDR_PCIEFD_FIFO_LAST(base)     __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_FIFO_LAST_BASE)
#define IOWR_PCIEFD_FIFO_LAST(base, data) IOWR(base, PCIEFD_FIFO_LAST_BASE, data)

#define IOADDR_PCIEFD_CONTROL(base)       __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_CONTROL_BASE)
#define IOWR_PCIEFD_CONTROL(base, data)   IOWR(base, PCIEFD_CONTROL_BASE, data)

#define IOADDR_PCIEFD_CCMD(base)          __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_CCMD_REG)
#define IORD_PCIEFD_CCMD(base)            IORD(base, PCIEFD_CCMD_REG)
#define IOWR_PCIEFD_CCMD(base, data)      IOWR(base, PCIEFD_CCMD_REG, data)

#define IOADDR_PCIEFD_IOC(base)           __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_IOC_REG)
#define IORD_PCIEFD_IOC(base)             IORD(base, PCIEFD_IOC_REG)
#define IOWR_PCIEFD_IOC(base, data)       IOWR(base, PCIEFD_IOC_REG, data)

#define IOADDR_PCIEFD_IEN(base)           __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_IEN_REG)
#define IORD_PCIEFD_IEN(base)             IORD(base, PCIEFD_IEN_REG)
#define IOWR_PCIEFD_IEN(base, data)       IOWR(base, PCIEFD_IEN_REG, data)

#define IOADDR_PCIEFD_ISTAT(base)         __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_ISTAT_REG)
#define IORD_PCIEFD_ISTAT(base)           IORD(base, PCIEFD_ISTAT_REG)
#define IOWR_PCIEFD_ISTAT(base, data)     IOWR(base, PCIEFD_ISTAT_REG, data)

#define IOADDR_PCIEFD_IRQ(base)           __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_IRQ_REG)
#define IORD_PCIEFD_IRQ(base)             IORD(base, PCIEFD_IRQ_REG)
#define IOWR_PCIEFD_IRQ(base, data)       IOWR(base, PCIEFD_IRQ_REG, data)

#define IOADDR_PCIEFD_TX_PACKET_COUNT(base)     __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_TX_PACKET_COUNT_REG)
#define IORD_PCIEFD_TX_PACKET_COUNT(base)       IORD(base, PCIEFD_TX_PACKET_COUNT_REG)
#define IOWR_PCIEFD_TX_PACKET_COUNT(base, data) IOWR(base, PCIEFD_TX_PACKET_COUNT_REG, data)

#define IOADDR_PCIEFD_STAT(base)          __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_STAT_REG)
#define IORD_PCIEFD_STAT(base)            IORD(base, PCIEFD_STAT_REG)
#define IOWR_PCIEFD_STAT(base, data)      IOWR(base, PCIEFD_STAT_REG, data)

#define IOADDR_PCIEFD_MOD(base)           __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_MOD_REG)
#define IORD_PCIEFD_MOD(base)             IORD(base, PCIEFD_MOD_REG)
#define IOWR_PCIEFD_MOD(base, data)       IOWR(base, PCIEFD_MOD_REG, data)

#define IOADDR_PCIEFD_BTRN(base)          __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_BTRN_REG)
#define IORD_PCIEFD_BTRN(base)            IORD(base, PCIEFD_BTRN_REG)
#define IOWR_PCIEFD_BTRN(base, data)      IOWR(base, PCIEFD_BTRN_REG, data)

#define IOADDR_PCIEFD_BLP(base)           __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_BLP_REG)
#define IORD_PCIEFD_BLP(base)             IORD(base, PCIEFD_BLP_REG)
#define IOWR_PCIEFD_BLP(base, data)       IOWR(base, PCIEFD_BLP_REG, data)

#define IOADDR_PCIEFD_BTRD(base)          __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_BTRD_REG)
#define IORD_PCIEFD_BTRD(base)            IORD(base, PCIEFD_BTRD_REG)
#define IOWR_PCIEFD_BTRD(base, data)      IOWR(base, PCIEFD_BTRD_REG, data)

#define IOADDR_PCIEFD_TX_RATE_LIMIT(base)      __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_TX_RATE_LIMIT_REG)
#define IORD_PCIEFD_TX_RATE_LIMIT(base)        IORD(base, PCIEFD_TX_RATE_LIMIT_REG)
#define IOWR_PCIEFD_TX_RATE_LIMIT(base, data)  IOWR(base, PCIEFD_TX_RATE_LIMIT_REG, data)

#define IOADDR_PCIEFD_PWM_CTRL(base)      __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_PWM_CTRL_REG)
#define IORD_PCIEFD_PWM_CTRL(base)        IORD(base, PCIEFD_PWM_CTRL_REG)
#define IOWR_PCIEFD_PWM_CTRL(base, data)  IOWR(base, PCIEFD_PWM_CTRL_REG, data)

#define IOADDR_PCIEFD_UNDEF(base)              __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_UNDEF_REG)
#define IORD_PCIEFD_UNDEF(base)                IORD(base, PCIEFD_UNDEF_REG)

#define IOADDR_FIFO_CONTROL(base)         __IO_CALC_ADDRESS_NATIVE(base, PCIEFD_FIFO_CONTROL_BASE)
#define IORD_FIFO_CONTROL(base)           IORD(base, PCIEFD_FIFO_CONTROL_BASE)
#define IOWR_FIFO_CONTROL(base, data)     IOWR(base, PCIEFD_FIFO_CONTROL_BASE, data)

#define PCIEFD_SPEEDUP_EN_LSHIFT          8
#define PCIEFD_SPEEDUP_EN_NBITS           1
#define PCIEFD_SPEEDUP_EN_MSK             mask(PCIEFD_SPEEDUP_EN)
#define PCIEFD_SPEEDUP_EN_GET(value)      get(PCIEFD_SPEEDUP_EN, value)
#define PCIEFD_SPEEDUP_EN(value)          field(PCIEFD_SPEEDUP_EN, value)

#define PCIEFD_SPEEDUP_DIR_LSHIFT         16
#define PCIEFD_SPEEDUP_DIR_NBITS          1
#define PCIEFD_SPEEDUP_DIR_MSK            mask(PCIEFD_SPEEDUP_DIR)
#define PCIEFD_SPEEDUP_DIR_GET(value)     get(PCIEFD_SPEEDUP_DIR, value)
#define PCIEFD_SPEEDUP_DIR(value)         field(PCIEFD_SPEEDUP_DIR, value)

#define PCIEFD_SPEEDUP_VAL_LSHIFT         0
#define PCIEFD_SPEEDUP_VAL_NBITS          6
#define PCIEFD_SPEEDUP_VAL_MSK            mask(PCIEFD_SPEEDUP_VAL)
#define PCIEFD_SPEEDUP_VAL_GET(value)     get(PCIEFD_SPEEDUP_VAL, value)
#define PCIEFD_SPEEDUP_VAL(value)         field(PCIEFD_SPEEDUP_VAL, value)

#endif /* __PCIEFD_REGS_H__ */
