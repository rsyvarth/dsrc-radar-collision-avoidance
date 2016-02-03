#ifndef __SYSID_H__
#define __SYSID_H__

#include "io.h"
#include "inc/bits.h"

// Baudrate
#define SYSID_DATE_REG                  0
#define SYSID_TIME_REG                  1
#define SYSID_REVISION_REG              2
#define SYSID_FREQUENCY_REG             3
#define SYSID_BUS_FREQUENCY_REG         4
#define SYSID_BUILD_L_REG               5
#define SYSID_BUILD_H_REG               6

#define IOADDR_SYSID_DATE(base)               __IO_CALC_ADDRESS_NATIVE(base, SYSID_DATE_REG)
#define IORD_SYSID_DATE(base)                 IORD(base, SYSID_DATE_REG)
#define IOWR_SYSID_DATE(base, data)           IOWR(base, SYSID_DATE_REG, data)

#define IOADDR_SYSID_TIME(base)               __IO_CALC_ADDRESS_NATIVE(base, SYSID_TIME_REG)
#define IORD_SYSID_TIME(base)                 IORD(base, SYSID_TIME_REG)
#define IOWR_SYSID_TIME(base, data)           IOWR(base, SYSID_TIME_REG, data)

#define IOADDR_SYSID_REVISION(base)           __IO_CALC_ADDRESS_NATIVE(base, SYSID_REVISION_REG)
#define IORD_SYSID_REVISION(base)             IORD(base, SYSID_REVISION_REG)
#define IOWR_SYSID_REVISION(base, data)       IOWR(base, SYSID_REVISION_REG, data)

#define IOADDR_SYSID_FREQUENCY(base)          __IO_CALC_ADDRESS_NATIVE(base, SYSID_FREQUENCY_REG)
#define IORD_SYSID_FREQUENCY(base)            IORD(base, SYSID_FREQUENCY_REG)
#define IOWR_SYSID_FREQUENCY(base, data)      IOWR(base, SYSID_FREQUENCY_REG, data)

#define IOADDR_SYSID_BUS_FREQUENCY(base)      __IO_CALC_ADDRESS_NATIVE(base, SYSID_BUS_FREQUENCY_REG)
#define IORD_SYSID_BUS_FREQUENCY(base)        IORD(base, SYSID_BUS_FREQUENCY_REG)
#define IOWR_SYSID_BUS_FREQUENCY(base, data)  IOWR(base, SYSID_BUS_FREQUENCY_REG, data)

#define IOADDR_SYSID_BUILD_L(base)            __IO_CALC_ADDRESS_NATIVE(base, SYSID_BUILD_L_REG)
#define IORD_SYSID_BUILD_L(base)              IORD(base, SYSID_BUILD_L_REG)
#define IOWR_SYSID_BUILD_L(base, data)        IOWR(base, SYSID_BUILD_L_REG, data)

#define IOADDR_SYSID_BUILD_H(base)            __IO_CALC_ADDRESS_NATIVE(base, SYSID_BUILD_H_REG)
#define IORD_SYSID_BUILD_H(base)              IORD(base, SYSID_BUILD_H_REG)
#define IOWR_SYSID_BUILD_H(base, data)        IOWR(base, SYSID_BUILD_H_REG, data)

#define SYSID_VERSION_NUM_CONT_LSHIFT       24
#define SYSID_VERSION_NUM_CONT_NBITS        8
#define SYSID_VERSION_NUM_CONT_MSK          mask(SYSID_VERSION_NUM_CONT)
#define SYSID_VERSION_NUM_CONT_GET(value)   get(SYSID_VERSION_NUM_CONT, value)
#define SYSID_VERSION_NUM_CONT(value)       field(SYSID_VERSION_NUM_CONT, value)

#define SYSID_VERSION_MAJOR_LSHIFT          16
#define SYSID_VERSION_MAJOR_NBITS           8
#define SYSID_VERSION_MAJOR_MSK             mask(SYSID_VERSION_MAJOR)
#define SYSID_VERSION_MAJOR_GET(value)      get(SYSID_VERSION_MAJOR, value)
#define SYSID_VERSION_MAJOR(value)          field(SYSID_VERSION_MAJOR, value)

#define SYSID_VERSION_MINOR_LSHIFT          0
#define SYSID_VERSION_MINOR_NBITS           8
#define SYSID_VERSION_MINOR_MSK             mask(SYSID_VERSION_MINOR)
#define SYSID_VERSION_MINOR_GET(value)      get(SYSID_VERSION_MINOR, value)
#define SYSID_VERSION_MINOR(value)          field(SYSID_VERSION_MINOR, value)

#define SYSID_BUILD_UC_LSHIFT               0
#define SYSID_BUILD_UC_NBITS                1
#define SYSID_BUILD_UC_MSK                  mask(SYSID_BUILD_UC)
#define SYSID_BUILD_UC_GET(value)           get(SYSID_BUILD_UC, value)
#define SYSID_BUILD_UC(value)               field(SYSID_BUILD_UC, value)

#define SYSID_BUILD_SEQNO_LSHIFT            1
#define SYSID_BUILD_SEQNO_NBITS             15
#define SYSID_BUILD_SEQNO_MSK               mask(SYSID_BUILD_SEQNO)
#define SYSID_BUILD_SEQNO_GET(value)        get(SYSID_BUILD_SEQNO, value)
#define SYSID_BUILD_SEQNO(value)            field(SYSID_BUILD_SEQNO, value)

#define SYSID_BUILD_L_HG_ID_LSHIFT          16
#define SYSID_BUILD_L_HG_ID_NBITS           16
#define SYSID_BUILD_L_HG_ID_MSK             mask(SYSID_BUILD_L_HG_ID)
#define SYSID_BUILD_L_HG_ID_GET(value)      get(SYSID_BUILD_L_HG_ID, value)
#define SYSID_BUILD_L_HG_ID(value)          field(SYSID_BUILD_L_HG_ID, value)

#endif
