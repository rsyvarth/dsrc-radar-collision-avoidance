/*
**                Copyright 2012 by Kvaser AB, Mölndal, Sweden
**                        http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ===============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ===============================================================================
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** ---------------------------------------------------------------------------
**/

/* Kvaser CAN driver PCIcan hardware specific parts
** PCIcan definitions
*/

#ifndef _PCIEFD_HWIF_H_
#define _PCIEFD_HWIF_H_

#include "VCanOsIf.h"
#include "osif_kernel.h"

// Common includes for Altera design
#include "inc/altera.h"
#include "pciefd_config.h"

/*****************************************************************************/
/* defines */
/*****************************************************************************/
// Use this to set alternate implementation.

#define DEVICE_NAME_STRING "pciefd"
#define MAX_CHANNELS  4
#define KVASER_VENDOR 0x1a07 // KVASER
#define PCIEFD_FPGA_MAJOR_VER 2

#define MAX_ERROR_COUNT       64 //128
#define ERROR_DISABLE_TIME_MS 200

#define PCIEFD_SRQ_RESP_WAIT_TIME 100

#define BLP_INTERVAL 400000 // About 5Hz
#define BLP_PRESC_MAX 255
#define BLP_PRESC_MIN 1
#define BLP_DIVISOR (BLP_INTERVAL/10000) // To get 0.00-100.00%

// Most of the functionality is placed behind a bus bridge.
// The bridges adds an offset to all addresses on the bridged segment of the bus.
//
#define MM_BRIDGE_OFFSET      (0x00010000) // Bytes address

// Memory map (All base addresses are byte adresses)
#define PCIE_HARD_IP_BASE     0x0000
#define CAN_LOOPBACK_BASE     (0xf000 + MM_BRIDGE_OFFSET)
#define SYSID_BASE            (0xf020 + MM_BRIDGE_OFFSET)
#if USE_ADJUSTABLE_PLL
#define PLL_BASE              (0xf040 + MM_BRIDGE_OFFSET)
#endif
#define TIMESTAMP_BASE        (0xf040 + MM_BRIDGE_OFFSET)
#define BUS_ANALYZER_BASE     (0xf080 + MM_BRIDGE_OFFSET)
#define PATTERN_GEN_BASE      (0xf100 + MM_BRIDGE_OFFSET)
#define STATUS_SWITCH_BASE    (0xf180 + MM_BRIDGE_OFFSET)
#define RECEIVE_BUFFER_BASE   (0xf200 + MM_BRIDGE_OFFSET)
#define SERIALFLASH_BASE      (0xf800 + MM_BRIDGE_OFFSET)

#define CAN_CONTROLLER_0_BASE (0x0000 + MM_BRIDGE_OFFSET)
#define CAN_CONTROLLER_1_BASE (0x1000 + MM_BRIDGE_OFFSET)
#define CAN_CONTROLLER_2_BASE (0x2000 + MM_BRIDGE_OFFSET)
#define CAN_CONTROLLER_3_BASE (0x3000 + MM_BRIDGE_OFFSET)

// Interrupt map
#define CAN_CONTROLLER_0_IRQ  (1<<0)
#define CAN_CONTROLLER_1_IRQ  (1<<1)
#define CAN_CONTROLLER_2_IRQ  (1<<2)
#define CAN_CONTROLLER_3_IRQ  (1<<3)
#define RECEIVE_BUFFER_IRQ    (1<<4)
#define SERIALFLASH_IRQ       (1<<5)
#define BUS_ANALYZER_IRQ      (1<<6)

// Alias for status stream switch control
#define IOWR_SSWITCH_BUS_ANALYZER IOWR_SSWITCH_OUTPUT0
#define IORD_SSWITCH_BUS_ANALYZER IORD_SSWITCH_OUTPUT0
#define IOWR_SSWITCH_PATTERN_GEN  IOWR_SSWITCH_OUTPUT1
#define IORD_SSWITCH_PATTERN_GEN  IORD_SSWITCH_OUTPUT1

// Loopback
#define LOOPBACK_N_CHANNELS_INT (4)
#define LOOPBACK_N_CHANNELS_EXT (2)
#define LOOPBACK_N_CHANNELS     (LOOPBACK_N_CHANNELS_INT+LOOPBACK_N_CHANNELS_EXT)
#define LOOPBACK_BUS_ANALYZER    2
#define LOOPBACK_PATTERN_GEN     3

#define CAN_CONTROLLER_SPAN (CAN_CONTROLLER_1_BASE-CAN_CONTROLLER_0_BASE)

#define OFFSET_FROM_BASE(base,offset) ((void __iomem*)((unsigned char*)(base) + (offset)))

#define ALL_INTERRUPT_SOURCES_MSK ( RECEIVE_BUFFER_IRQ          \
				    | CAN_CONTROLLER_0_IRQ      \
                                    | CAN_CONTROLLER_1_IRQ      \
                                    | CAN_CONTROLLER_2_IRQ      \
                                    | CAN_CONTROLLER_3_IRQ )

// The actual max value supported by HW can be read using fifoPacketCountTxMax (Altera/HAL/src/pciefd.c)
#define MAX_OUTSTANDING_TX 17

#if USE_DMA

#define DMA_BUFFER_SZ 4096

typedef struct {
  uint32_t *data;
  dma_addr_t address;
} dmaCtxBuffer_t;

typedef struct {
  dmaCtxBuffer_t bufferCtx[2];
  int active;
  unsigned int position;
  unsigned int psize;
  unsigned int enabled;
} dmaCtx_t;


#endif

// Avalon Address Translation Table Address Space Bit Encodings
enum {
  AV_ATT_ASBE_MS32 = 0, // 32-bit address (bits 63:32 are ignored)
  AV_ATT_ASBE_MS64 = 1  // 64 bit address
};

/* Channel specific data */
typedef struct PciCanChanData
{
  CAN_MSG current_tx_message[MAX_OUTSTANDING_TX];

  atomic_t outstanding_tx;
  unsigned int nebits;

  /* Ports and addresses */
  void __iomem       *canControllerBase;

  unsigned long tx_irq_msk;
  OS_IF_SEMAPHORE busOnMutex; // Used to make sure that multiple bus on commands in a row is not executed.

  OS_IF_LOCK lock;
#if !defined(TRY_RT_QUEUE)
  OS_IF_TASK_QUEUE_HANDLE txTaskQ;
#else
  OS_IF_WQUEUE *txTaskQ;
  OS_IF_TASK_QUEUE_HANDLE txWork;
#endif

  pciefd_packet_t packet;

  // Flags set if an overrun has been detected
  int overrun_occured;

  // Bus load
  int load;

  VCanChanData *vChan;

  // Statistics and error checks
  struct {
    unsigned char last_seq_no;
    int got_seq_no;

    int seq_no_mismatch;

    uint64_t max_ts_diff;
    uint64_t avg_ts_diff;
    int avg_ts_diff_cnt;
    int ts_error_cnt;
    int ts_wrong_order_cnt;

    int packet_count;
    int ack_packet_count;
    int trq_packet_count;
    int err_packet_count;
    int tx_packet_count;

    int requested_status;
    int received_status;

    uint32_t max_level; // Maximum receive buffer level
    uint32_t avg_level;
    uint32_t avg_level_cnt;
    uint32_t fifo_full_cnt;
    uint32_t max_tx_level;

    int unaligned_read;

    int unaligned_transmit;
    int transmit_overflow;
    int transmit_underflow;

    int bus_load;
  } debug;

  // Debug ack packet receiver
  int compareTransId;
  int lastTransId;
  uint64_t last_ts;

  unsigned int delay_vec[60*8];

#ifdef PCIEFD_DEBUG
  struct dentry *debugfsdir;
#endif

  struct timer_list errorDisableTimer;

  OS_IF_WAITQUEUE_HEAD hwFlushWaitQ;

  struct {
    unsigned valid;
    unsigned nbits;
    unsigned tbit;
    unsigned tsyn;
    unsigned tcan;
    unsigned tsp;
  } ef_params;

  unsigned long bus_load_prescaler;

  int auto_sso;
  unsigned int sso;

} PciCanChanData;

/*  Cards specific data */
typedef struct PciCanCardData {
  void __iomem       *pcieBar0Base;

  /* Ports and addresses */
  void __iomem       *canControllerBase;
  void __iomem       *timestampBase;
  void __iomem       *serialFlashBase;
  void __iomem       *canLoopbackBase;
  void __iomem       *sysidBase;
#if USE_ADJUSTABLE_PLL
  void __iomem       *pllBase;
#endif
  void __iomem       *pcie;
  void __iomem       *canRxBuffer;
  void __iomem       *busAnalyzerBase;
  void __iomem       *patternGeneratorBase;
  void __iomem       *statusStreamSwitchBase;

  unsigned int       frequency;
  unsigned int       freqToTicksDivider;
  int                irq;
  alt_flash_epcs_dev epcsFlashDev;
  struct list_head   replyWaitList;
  rwlock_t           replyWaitListLock;
  OS_IF_LOCK         lock;
  uint64_t           last_ts;

  struct timer_list  interruptDisableTimer;

  struct pci_dev *dev;

#if USE_DMA
  dmaCtx_t dmaCtx;


#endif

  pciefd_packet_t packet;

#if USE_DMA
  int useDmaAddr64;
  int useDma;
#endif

#ifdef PCIEFD_DEBUG
  struct dentry *debugfsdir;
#endif

  atomic_t status_seq_no;


} PciCanCardData;


#endif
