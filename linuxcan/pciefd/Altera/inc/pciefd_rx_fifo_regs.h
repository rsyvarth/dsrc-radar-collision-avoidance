#ifndef __PCIEFD_RX_FIFO_REGS_H__
#define __PCIEFD_RX_FIFO_REGS_H__

#include <inc/io.h>
#include "inc/bits.h"

// Word address offset for receive buffer sub-modules
// (The receive buffer address space is a representation of 2 Avalon slave interfaces)
#define RXBUF_CONTROL_OFFSET            (0x200/4)

// Bit 0 to 3 is the data or header word offset
// Bit 4 is the header flag
// Bit 5 is the last word flag
// Bit 6 is the fifo mode flag
#define RXBUF_ADDR_HEADER_FLAG          4
#define RXBUF_ADDR_LAST_FLAG            5
#define RXBUF_ADDR_FIFO_FLAG            6

#define RXBUF_DATA_BASE                 0x000
#define RXBUF_HEADER_BASE               (1<<RXBUF_ADDR_HEADER_FLAG)
#define RXBUF_DATA_LAST_BASE            (1<<RXBUF_ADDR_LAST_FLAG)
#define RXBUF_HEADER_LAST_BASE          ((1<<RXBUF_ADDR_HEADER_FLAG) | (1<<RXBUF_ADDR_LAST_FLAG))

#define RXBUF_FIFO_BASE                 (1<<RXBUF_ADDR_FIFO_FLAG)
#define RXBUF_FIFO_LAST_BASE            ((1<<RXBUF_ADDR_FIFO_FLAG) | (1<<RXBUF_ADDR_LAST_FLAG))

#define RXBUF_CMD_REG                   (RXBUF_CONTROL_OFFSET + 0)
#define RXBUF_IEN_REG                   (RXBUF_CONTROL_OFFSET + 1)
#define RXBUF_ISTAT_REG                 (RXBUF_CONTROL_OFFSET + 2)
#define RXBUF_IRQ_REG                   (RXBUF_CONTROL_OFFSET + 3)
#define RXBUF_STAT_REG                  (RXBUF_CONTROL_OFFSET + 4)
#define RXBUF_NPACKETS_REG              (RXBUF_CONTROL_OFFSET + 5)
#define RXBUF_CTRL_REG                  (RXBUF_CONTROL_OFFSET + 6)

#define IORD_RXBUF_FIFO(base)           IORD(base, RXBUF_FIFO_BASE)
#define IORD_RXBUF_FIFO_LAST(base)      IORD(base, RXBUF_FIFO_LAST_BASE)

#define IOADDR_RXBUF_CMD(base)          __IO_CALC_ADDRESS_NATIVE(base, RXBUF_CMD_REG)
#define IORD_RXBUF_CMD(base)            IORD(base, RXBUF_CMD_REG)
#define IOWR_RXBUF_CMD(base, data)      IOWR(base, RXBUF_CMD_REG, data)

#define IOADDR_RXBUF_IEN(base)          __IO_CALC_ADDRESS_NATIVE(base, RXBUF_IEN_REG)
#define IORD_RXBUF_IEN(base)            IORD(base, RXBUF_IEN_REG)
#define IOWR_RXBUF_IEN(base, data)      IOWR(base, RXBUF_IEN_REG, data)

#define IOADDR_RXBUF_ISTAT(base)        __IO_CALC_ADDRESS_NATIVE(base, RXBUF_ISTAT_REG)
#define IORD_RXBUF_ISTAT(base)          IORD(base, RXBUF_ISTAT_REG)
#define IOWR_RXBUF_ISTAT(base, data)    IOWR(base, RXBUF_ISTAT_REG, data)

#define IOADDR_RXBUF_IRQ(base)          __IO_CALC_ADDRESS_NATIVE(base, RXBUF_IRQ_REG)
#define IORD_RXBUF_IRQ(base)            IORD(base, RXBUF_IRQ_REG)
#define IOWR_RXBUF_IRQ(base, data)      IOWR(base, RXBUF_IRQ_REG, data)

#define IOADDR_RXBUF_STAT(base)         __IO_CALC_ADDRESS_NATIVE(base, RXBUF_STAT_REG)
#define IORD_RXBUF_STAT(base)           IORD(base, RXBUF_STAT_REG)
#define IOWR_RXBUF_STAT(base, data)     IOWR(base, RXBUF_STAT_REG, data)

#define IOADDR_RXBUF_NPACKETS(base)     __IO_CALC_ADDRESS_NATIVE(base, RXBUF_NPACKETS_REG)
#define IORD_RXBUF_NPACKETS(base)       IORD(base, RXBUF_NPACKETS_REG)
#define IOWR_RXBUF_NPACKETS(base, data) IOWR(base, RXBUF_NPACKETS_REG, data)

#define IOADDR_RXBUF_CTRL(base)         __IO_CALC_ADDRESS_NATIVE(base, RXBUF_CTRL_REG)
#define IORD_RXBUF_CTRL(base)           IORD(base, RXBUF_CTRL_REG)
#define IOWR_RXBUF_CTRL(base, data)     IOWR(base, RXBUF_CTRL_REG, data)


#define RXBUF_STAT_CHID_LSHIFT          0
#define RXBUF_STAT_CHID_NBITS           2
#define RXBUF_STAT_CHID_MSK             mask(RXBUF_STAT_CHID)
#define RXBUF_STAT_CHID_GET(value)      get(RXBUF_STAT_CHID, value)
#define RXBUF_STAT_CHID(value)          field(RXBUF_STAT_CHID, value)

#define RXBUF_STAT_LAST_POS_LSHIFT      8
#define RXBUF_STAT_LAST_POS_NBITS       5
#define RXBUF_STAT_LAST_POS_MSK         mask(RXBUF_STAT_LAST_POS)
#define RXBUF_STAT_LAST_POS_GET(value)  get(RXBUF_STAT_LAST_POS, value)
#define RXBUF_STAT_LAST_POS(value)      field(RXBUF_STAT_LAST_POS, value)

#define RXBUF_STAT_AVAILABLE_LSHIFT     13
#define RXBUF_STAT_AVAILABLE_NBITS      1
#define RXBUF_STAT_AVAILABLE_MSK        mask(RXBUF_STAT_AVAILABLE)
#define RXBUF_STAT_AVAILABLE_GET(value) get(RXBUF_STAT_AVAILABLE, value)
#define RXBUF_STAT_AVAILABLE(value)     field(RXBUF_STAT_AVAILABLE, value)

#define RXBUF_STAT_EOP_LSHIFT           14
#define RXBUF_STAT_EOP_NBITS            1
#define RXBUF_STAT_EOP_MSK              mask(RXBUF_STAT_EOP)
#define RXBUF_STAT_EOP_GET(value)       get(RXBUF_STAT_EOP, value)
#define RXBUF_STAT_EOP(value)           field(RXBUF_STAT_EOP, value)

#define RXBUF_STAT_DMA_IDLE_LSHIFT      15
#define RXBUF_STAT_DMA_IDLE_NBITS       1
#define RXBUF_STAT_DMA_IDLE_MSK         mask(RXBUF_STAT_DMA_IDLE)
#define RXBUF_STAT_DMA_IDLE_GET(value)  get(RXBUF_STAT_DMA_IDLE, value)
#define RXBUF_STAT_DMA_IDLE(value)      field(RXBUF_STAT_DMA_IDLE, value)

#define RXBUF_STAT_FIFO_PTR_LSHIFT      16
#define RXBUF_STAT_FIFO_PTR_NBITS       5
#define RXBUF_STAT_FIFO_PTR_MSK         mask(RXBUF_STAT_FIFO_PTR)
#define RXBUF_STAT_FIFO_PTR_GET(value)  get(RXBUF_STAT_FIFO_PTR, value)
#define RXBUF_STAT_FIFO_PTR(value)      field(RXBUF_STAT_FIFO_PTR, value)

#define RXBUF_STAT_DMA_SUPPORT_LSHIFT     24
#define RXBUF_STAT_DMA_SUPPORT_NBITS      1
#define RXBUF_STAT_DMA_SUPPORT_MSK        mask(RXBUF_STAT_DMA_SUPPORT)
#define RXBUF_STAT_DMA_SUPPORT_GET(value) get(RXBUF_STAT_DMA_SUPPORT, value)
#define RXBUF_STAT_DMA_SUPPORT(value)     field(RXBUF_STAT_DMA_SUPPORT, value)


#define RXBUF_NPACKETS_LSHIFT           0
#define RXBUF_NPACKETS_NBITS            8
#define RXBUF_NPACKETS_MSK              mask(RXBUF_NPACKETS)
#define RXBUF_NPACKETS_GET(value)       get(RXBUF_NPACKETS, value)
#define RXBUF_NPACKETS(value)           field(RXBUF_NPACKETS, value)

#define RXBUF_NPACKETS_MAX_LSHIFT       16
#define RXBUF_NPACKETS_MAX_NBITS        8
#define RXBUF_NPACKETS_MAX_MSK          mask(RXBUF_NPACKETS_MAX)
#define RXBUF_NPACKETS_MAX_GET(value)   get(RXBUF_NPACKETS_MAX, value)
#define RXBUF_NPACKETS_MAX(value)       field(RXBUF_NPACKETS_MAX, value)


#define RXBUF_CTRL_DMAEN_LSHIFT         0
#define RXBUF_CTRL_DMAEN_NBITS          1
#define RXBUF_CTRL_DMAEN_MSK            mask(RXBUF_CTRL_DMAEN)
#define RXBUF_CTRL_DMAEN_GET(value)     get(RXBUF_CTRL_DMAEN, value)
#define RXBUF_CTRL_DMAEN(value)         field(RXBUF_CTRL_DMAEN, value)

#define RXBUF_CMD_RESET_LSHIFT          0
#define RXBUF_CMD_RESET_NBITS           1
#define RXBUF_CMD_RESET_MSK             mask(RXBUF_CMD_RESET)
#define RXBUF_CMD_RESET_GET(value)      get(RXBUF_CMD_RESET, value)
#define RXBUF_CMD_RESET(value)          field(RXBUF_CMD_RESET, value)

#define RXBUF_CMD_DMATRIG_LSHIFT        4
#define RXBUF_CMD_DMATRIG_NBITS         2
#define RXBUF_CMD_DMATRIG_MSK           mask(RXBUF_CMD_DMATRIG)
#define RXBUF_CMD_DMATRIG_GET(value)    get(RXBUF_CMD_DMATRIG, value)
#define RXBUF_CMD_DMATRIG(value)        field(RXBUF_CMD_DMATRIG, value)

#define RXBUF_CMD_DMATRIG0_LSHIFT        4
#define RXBUF_CMD_DMATRIG0_NBITS         1
#define RXBUF_CMD_DMATRIG0_MSK           mask(RXBUF_CMD_DMATRIG0)
#define RXBUF_CMD_DMATRIG0_GET(value)    get(RXBUF_CMD_DMATRIG0, value)
#define RXBUF_CMD_DMATRIG0(value)        field(RXBUF_CMD_DMATRIG0, value)

#define RXBUF_CMD_DMATRIG1_LSHIFT       5
#define RXBUF_CMD_DMATRIG1_NBITS        1
#define RXBUF_CMD_DMATRIG1_MSK          mask(RXBUF_CMD_DMATRIG1)
#define RXBUF_CMD_DMATRIG1_GET(value)   get(RXBUF_CMD_DMATRIG1, value)
#define RXBUF_CMD_DMATRIG1(value)       field(RXBUF_CMD_DMATRIG1, value)


#define RXBUF_IRQ_ILLEGAL_ACCESS_MSK    (1<<0)
#define RXBUF_IRQ_UNALIGNED_READ_MSK    (1<<1)
#define RXBUF_IRQ_MISSING_TAG_MSK       (1<<2)
#define RXBUF_IRQ_UNDERFLOW_MSK         (1<<3)
#define RXBUF_IRQ_RA_MSK                (1<<4)
#define RXBUF_IRQ_DMA_DONE0_MSK         (1<<8)
#define RXBUF_IRQ_DMA_DONE1_MSK         (1<<9)
#define RXBUF_IRQ_DMA_OVF0_MSK          (1<<10)
#define RXBUF_IRQ_DMA_OVF1_MSK          (1<<11)
#define RXBUF_IRQ_DMA_UNF0_MSK          (1<<12)
#define RXBUF_IRQ_DMA_UNF1_MSK          (1<<13)

#endif
