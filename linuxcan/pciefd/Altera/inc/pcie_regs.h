#ifndef __PCIE_REGS_H__
#define __PCIE_REGS_H__

#include <inc/io.h>
#include "inc/bits.h"

// Interrupt control registers in the Altera PCIe IP function

// The irq register contains the interrupt status and the ien register is the interrupt enable.
#define PCIE_IRQ_REG (0x0040/4)
#define PCIE_IEN_REG (0x0050/4)

#define IOADDR_PCIE_IRQ(base)                 __IO_CALC_ADDRESS_NATIVE(base, PCIE_IRQ_REG)
#define IORD_PCIE_IRQ(base)                   IORD(base, PCIE_IRQ_REG)
#define IOWR_PCIE_IRQ(base, data)             IOWR(base, PCIE_IRQ_REG, data)

#define IOADDR_PCIE_IEN(base)                 __IO_CALC_ADDRESS_NATIVE(base, PCIE_IEN_REG)
#define IORD_PCIE_IEN(base)                   IORD(base, PCIE_IEN_REG)
#define IOWR_PCIE_IEN(base, data)             IOWR(base, PCIE_IEN_REG, data)

#endif
