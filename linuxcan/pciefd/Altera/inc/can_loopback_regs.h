#ifndef __CAN_LOOPBACK_REGS_H__
#define __CAN_LOOPBACK_REGS_H__

#include "io.h"

// Baudrate
#define CAN_LOOPBACK_CTRL_REG                  0
#define IOADDR_CAN_LOOPBACK_CTRL(base)                    \
  __IO_CALC_ADDRESS_NATIVE(base, CAN_LOOPBACK_CTRL_REG)
#define IORD_CAN_LOOPBACK_CTRL(base)              \
  IORD(base, CAN_LOOPBACK_CTRL_REG)
#define IOWR_CAN_LOOPBACK_CTRL(base, data)        \
  IOWR(base, CAN_LOOPBACK_CTRL_REG, data)


#define CAN_LOOPBACK_CTRL_NO_LOOPBACK (0)
#define CAN_LOOPBACK_CTRL_LOOPBACK    (3)

#endif
