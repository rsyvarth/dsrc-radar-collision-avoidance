#ifndef __PLL_H__
#define __PLL_H__

#include "io.h"
#include "inc/bits.h"

// Baudrate
#define PLL_DELAY_REG                   0
#define PLL_STEP_SZ_REG                 1
#define PLL_ENABLE_REG                  2
#define PLL_MIN_PHASE_SHIFT_REG         3
#define PLL_MAX_PHASE_SHIFT_REG         4
#define PLL_MAX_OFFSET_REG              5
#define PLL_SCANCLK_FREQUENCY_REG       6
#define PLL_C0_C1_DIV_REG               7

#define IOADDR_PLL_DELAY(base)                 __IO_CALC_ADDRESS_NATIVE(base, PLL_DELAY_REG)
#define IORD_PLL_DELAY(base)                   IORD(base, PLL_DELAY_REG)
#define IOWR_PLL_DELAY(base, data)             IOWR(base, PLL_DELAY_REG, data)

#define IOADDR_PLL_STEP_SZ(base)               __IO_CALC_ADDRESS_NATIVE(base, PLL_STEP_SZ_REG)
#define IORD_PLL_STEP_SZ(base)                 IORD(base, PLL_STEP_SZ_REG)
#define IOWR_PLL_STEP_SZ(base, data)           IOWR(base, PLL_STEP_SZ_REG, data)

#define IOADDR_PLL_ENABLE(base)                __IO_CALC_ADDRESS_NATIVE(base, PLL_ENABLE_REG)
#define IORD_PLL_ENABLE(base)                  IORD(base, PLL_ENABLE_REG)
#define IOWR_PLL_ENABLE(base, data)            IOWR(base, PLL_ENABLE_REG, data)

#define IOADDR_PLL_MIN_PHASE_SHIFT(base)       __IO_CALC_ADDRESS_NATIVE(base, PLL_MIN_PHASE_SHIFT_REG)
#define IORD_PLL_MIN_PHASE_SHIFT(base)         IORD(base, PLL_MIN_PHASE_SHIFT_REG)
#define IOWR_PLL_MIN_PHASE_SHIFT(base, data)   IOWR(base, PLL_MIN_PHASE_SHIFT_REG, data)

#define IOADDR_PLL_MAX_PHASE_SHIFT(base)       __IO_CALC_ADDRESS_NATIVE(base, PLL_MAX_PHASE_SHIFT_REG)
#define IORD_PLL_MAX_PHASE_SHIFT(base)         IORD(base, PLL_MAX_PHASE_SHIFT_REG)
#define IOWR_PLL_MAX_PHASE_SHIFT(base, data)   IOWR(base, PLL_MAX_PHASE_SHIFT_REG, data)

#define IOADDR_PLL_MAX_OFFSET(base)            __IO_CALC_ADDRESS_NATIVE(base, PLL_MAX_OFFSET_REG)
#define IORD_PLL_MAX_OFFSET(base)              IORD(base, PLL_MAX_OFFSET_REG)
#define IOWR_PLL_MAX_OFFSET(base, data)        IOWR(base, PLL_MAX_OFFSET_REG, data)

#define IOADDR_PLL_SCANCLK_FREQUENCY(base)     __IO_CALC_ADDRESS_NATIVE(base, PLL_SCANCLK_FREQUENCY_REG)
#define IORD_PLL_SCANCLK_FREQUENCY(base)       IORD(base, PLL_SCANCLK_FREQUENCY_REG)
#define IOWR_PLL_SCANCLK_FREQUENCY(base, data) IOWR(base, PLL_SCANCLK_FREQUENCY_REG, data)

#define IOADDR_PLL_C0_C1_DIV(base)             __IO_CALC_ADDRESS_NATIVE(base, PLL_C0_C1_DIV_REG)
#define IORD_PLL_C0_C1_DIV(base)               IORD(base, PLL_C0_C1_DIV_REG)
#define IOWR_PLL_C0_C1_DIV(base, data)         IOWR(base, PLL_C0_C1_DIV_REG, data)


#define PCIEFD_PLL_IDLE_LSHIFT            1
#define PCIEFD_PLL_IDLE_NBITS             1
#define PCIEFD_PLL_IDLE_MSK               mask(PCIEFD_PLL_IDLE)
#define PCIEFD_PLL_IDLE_GET(value)        get(PCIEFD_PLL_IDLE, value)
#define PCIEFD_PLL_IDLE(value)            field(PCIEFD_PLL_IDLE, value)

#define PCIEFD_PLL_ENABLE_LSHIFT          0
#define PCIEFD_PLL_ENABLE_NBITS           1
#define PCIEFD_PLL_ENABLE_MSK             mask(PCIEFD_PLL_ENABLE)
#define PCIEFD_PLL_ENABLE_GET(value)      get(PCIEFD_PLL_ENABLE, value)
#define PCIEFD_PLL_ENABLE(value)          field(PCIEFD_PLL_ENABLE, value)

#endif
