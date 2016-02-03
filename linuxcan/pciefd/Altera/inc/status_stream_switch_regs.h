#ifndef __STATUS_STREAM_SWITCH_REGS_H__
#define __STATUS_STREAM_SWITCH_REGS_H__

#include <inc/io.h>
#include "inc/bits.h"

#define SSWITCH_OUTPUT0_REG                  0
#define SSWITCH_OUTPUT1_REG                  1
#define SSWITCH_OUTPUT2_REG                  2
#define SSWITCH_OUTPUT3_REG                  3
#define SSWITCH_OUTPUT4_REG                  4
#define SSWITCH_OUTPUT5_REG                  5
#define SSWITCH_OUTPUT6_REG                  6
#define SSWITCH_OUTPUT7_REG                  7

#define IOADDR_SSWITCH_OUTPUT0(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT0_REG)
#define IORD_SSWITCH_OUTPUT0(base)           IORD(base, SSWITCH_OUTPUT0_REG)
#define IOWR_SSWITCH_OUTPUT0(base, data)     IOWR(base, SSWITCH_OUTPUT0_REG, data)

#define IOADDR_SSWITCH_OUTPUT1(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT1_REG)
#define IORD_SSWITCH_OUTPUT1(base)           IORD(base, SSWITCH_OUTPUT1_REG)
#define IOWR_SSWITCH_OUTPUT1(base, data)     IOWR(base, SSWITCH_OUTPUT1_REG, data)

#define IOADDR_SSWITCH_OUTPUT2(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT2_REG)
#define IORD_SSWITCH_OUTPUT2(base)           IORD(base, SSWITCH_OUTPUT2_REG)
#define IOWR_SSWITCH_OUTPUT2(base, data)     IOWR(base, SSWITCH_OUTPUT2_REG, data)

#define IOADDR_SSWITCH_OUTPUT3(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT3_REG)
#define IORD_SSWITCH_OUTPUT3(base)           IORD(base, SSWITCH_OUTPUT3_REG)
#define IOWR_SSWITCH_OUTPUT3(base, data)     IOWR(base, SSWITCH_OUTPUT3_REG, data)

#define IOADDR_SSWITCH_OUTPUT4(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT4_REG)
#define IORD_SSWITCH_OUTPUT4(base)           IORD(base, SSWITCH_OUTPUT4_REG)
#define IOWR_SSWITCH_OUTPUT4(base, data)     IOWR(base, SSWITCH_OUTPUT4_REG, data)

#define IOADDR_SSWITCH_OUTPUT5(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT5_REG)
#define IORD_SSWITCH_OUTPUT5(base)           IORD(base, SSWITCH_OUTPUT5_REG)
#define IOWR_SSWITCH_OUTPUT5(base, data)     IOWR(base, SSWITCH_OUTPUT5_REG, data)

#define IOADDR_SSWITCH_OUTPUT6(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT6_REG)
#define IORD_SSWITCH_OUTPUT6(base)           IORD(base, SSWITCH_OUTPUT6_REG)
#define IOWR_SSWITCH_OUTPUT6(base, data)     IOWR(base, SSWITCH_OUTPUT6_REG, data)

#define IOADDR_SSWITCH_OUTPUT7(base)         __IO_CALC_ADDRESS_NATIVE(base, SSWITCH_OUTPUT7_REG)
#define IORD_SSWITCH_OUTPUT7(base)           IORD(base, SSWITCH_OUTPUT7_REG)
#define IOWR_SSWITCH_OUTPUT7(base, data)     IOWR(base, SSWITCH_OUTPUT7_REG, data)

#endif
