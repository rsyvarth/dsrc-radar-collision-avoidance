#ifndef __ALTERA_H__
#define __ALTERA_H__

#include "io.h"

#include "../../pciefd_config.h"
#include "HAL/inc/pwm_util.h"
#include "inc/pcie_regs.h"

#include "HAL/inc/pciefd_packet.h"
#include "inc/pciefd_rx_fifo_regs.h"
#include "HAL/inc/pciefd_rx_fifo.h"



#include "inc/status_stream_switch_regs.h"

#include "HAL/inc/pciefd.h"

#if LOOPBACK_VERSION == 2
#include "can_loopback_regs.h"
#elif LOOPBACK_VERSION == 4
#include "can_loopback_v4_regs.h"
#else
#error "LOOPBACK_VERSION not defined"
#endif

#include "sysid.h"

#include "pll_regs.h"
#include "HAL/inc/pll.h"

#include "HAL/inc/altera_avalon_epcs_flash_controller.h"

#endif
