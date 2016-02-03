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

//--------------------------------------------------
// NOTE! module_versioning HAS to be included first
#include "module_versioning.h"
//--------------------------------------------------

//
// Kvaser CAN driver pciefd hardware specific parts
//

#include <linux/version.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <linux/seq_file.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#   include <asm/system.h>
#endif

#include <asm/bitops.h>
#include <asm/uaccess.h>

#include <asm/delay.h>

// For 64-bit divisions in kernel
#include <asm/div64.h>

// Kvaser definitions
#include "VCanOsIf.h"
#include "pciefd_hwif.h"
#include "osif_kernel.h"
#include "osif_functions_kernel.h"
#include "queue.h"
#include "debug.h"
#include "util.h"
#include "vcan_ioctl.h"

#include "hwnames.h"


#ifdef PCIEFD_DEBUG
//
// If you do not define PCIEFD_DEBUG at all, all the debug code will be
// left out. If you compile with PCIEFD_DEBUG=0, the debug code will
// be present but disabled -- but it can then be enabled for specific
// modules at load time with a 'debug_level=#' option to insmod.
// i.e. >insmod kvpcican debug_level=#
//
int debug_level = PCIEFD_DEBUG;
MODULE_PARM_DESC(debug_level, "PCIe CAN debug level");
module_param(debug_level, int, 0644);
#define DEBUGPRINT(n, arg...)     if (debug_level >= (n)) { DEBUGOUT(n, (arg)); }

#else

#define DEBUGPRINT(n, arg...)     if ((n) == 1) { DEBUGOUT(n, (arg)); }

#endif

#define INITPRINT(arg...) printk(arg)

// For measuring delays during debugging
#include <linux/time.h>

#if USE_DMA
#include <linux/dma-mapping.h>
#endif

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("KVASER");
MODULE_DESCRIPTION("PCIe CAN module.");

#define PCIEFD_VENDOR (0x1a07)
#define PCIEFD_ID     (0x000d)

#define MAX_NB_PARAMETERS 256
#define PARAM_MAGIC       0xCAFEF00D

#define PARAM_SYSTEM_VERSION 1

// Disable loopback by default
int loopback = 0;

MODULE_PARM_DESC(loopback, "PCIe CAN Loopback");
module_param(loopback, int, 0644);

//======================================================================
// HW function pointers
//======================================================================

static int INIT pciCanInitAllDevices(void);
static int pciCanSetBusParams (VCanChanData *vChd, VCanBusParams *par);
static int pciCanGetBusParams (VCanChanData *vChd, VCanBusParams *par);
static int pciCanSetOutputMode (VCanChanData *vChd, int silent);
static int pciCanSetTranceiverMode (VCanChanData *vChd, int linemode, int resnet);
static int pciCanBusOn (VCanChanData *vChd);
static int pciCanBusOff(VCanChanData *vChd);
static int pciCanGetTxErr(VCanChanData *vChd);
static int pciCanGetRxErr(VCanChanData *vChd);
static int pciCanInSync (VCanChanData *vChd);
static int pciCanTxAvailable (VCanChanData *vChd);
static int EXIT pciCanCloseAllDevices(void);
static int pciCanTime(VCanCardData *vCard, unsigned long *time);
static int pciCanFlushSendBuffer (VCanChanData *vChan);
static int pciCanProcRead (struct seq_file* m, void* v);

static int pciCanRequestChipState (VCanChanData *vChd);
static unsigned long pciCanRxQLen(VCanChanData *vChd);
static unsigned long pciCanTxQLen(VCanChanData *vChd);
static void pciCanRequestSend (VCanCardData *vCard, VCanChanData *vChan);

static VCanDriverData driverData;

static VCanHWInterface hwIf = {
  .initAllDevices     = pciCanInitAllDevices,
  .setBusParams       = pciCanSetBusParams,
  .getBusParams       = pciCanGetBusParams,
  .setOutputMode      = pciCanSetOutputMode,
  .setTranceiverMode  = pciCanSetTranceiverMode,
  .busOn              = pciCanBusOn,
  .busOff             = pciCanBusOff,
  .txAvailable        = pciCanInSync,
  .procRead           = pciCanProcRead,
  .closeAllDevices    = pciCanCloseAllDevices,
  .getTime            = pciCanTime,
  .flushSendBuffer    = pciCanFlushSendBuffer,
  .getTxErr           = pciCanGetTxErr,
  .getRxErr           = pciCanGetRxErr,
  .rxQLen             = pciCanRxQLen,
  .txQLen             = pciCanTxQLen,
  .requestChipState   = pciCanRequestChipState,
  .requestSend        = pciCanRequestSend,
};


static int DEVINIT pciCanInitOne(struct pci_dev *dev, const struct pci_device_id *id);
static void DEVEXIT pciCanRemoveOne(struct pci_dev *dev);

static struct pci_device_id id_table[] DEVINITDATA = {
  {
    .vendor    = PCIEFD_VENDOR,
    .device    = PCIEFD_ID,
    .subvendor = PCI_ANY_ID,
    .subdevice = PCI_ANY_ID
  },
  {0,},
};

static struct pci_driver pcican_tbl = {
  .name     = "kvpciefd",
  .id_table = id_table,
  .probe    = pciCanInitOne,
  .remove   = DEVEXITP(pciCanRemoveOne),
};


typedef struct {
  uint32_t   magic;  //PARAM_MAGIC indicate used
  uint32_t   param_number;
  uint32_t   param_len;
  uint8_t    data[24];
} kcc_param_t;

typedef struct {
  uint32_t version;
  uint32_t magic;
  uint32_t crc;
  kcc_param_t param[MAX_NB_PARAMETERS];
} kcc_param_image_t;

//======================================================================
// DMA
//======================================================================
#if USE_DMA

//======================================================================
//
//======================================================================
static int dmaInit(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  struct device *dev = &hCard->dev->dev;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
  if (!dma_set_mask(dev, DMA_BIT_MASK(64))) {
    hCard->useDmaAddr64 = 1;
    DEBUGPRINT(3,"Use 64-bit dma address mask\n");
  } else
#endif
  if (!dma_set_mask(dev, DMA_BIT_MASK(32))) {
    hCard->useDmaAddr64 = 0;
    DEBUGPRINT(3,"Use 32-bit dma address mask\n");
  } else {
    DEBUGPRINT(1, "No suitable DMA available.\n");
    return VCAN_STAT_NO_RESOURCES;
  }

  return VCAN_STAT_OK;
}


//======================================================================
//
//======================================================================
static int mapOneBuffer(VCanCardData * vCard, dmaCtxBuffer_t * bufferCtx)
{
  PciCanCardData *hCard = vCard->hwCardData;
  struct device *dev = &hCard->dev->dev;

  void *buffer;
  dma_addr_t dma_handle;

  buffer = kmalloc(DMA_BUFFER_SZ, GFP_KERNEL);

  if(!buffer){
    DEBUGPRINT(1,"Failed DMA buffer setup\n");
    return VCAN_STAT_NO_MEMORY;
  }

  bufferCtx->data = buffer;

  dma_handle = dma_map_single(dev, buffer, DMA_BUFFER_SZ, DMA_FROM_DEVICE);

  if (dma_mapping_error(dev,dma_handle)) {
    /*
     * reduce current DMA mapping usage,
     * delay and try again later or
     * reset driver.
     */
    DEBUGPRINT(1,"Failed DMA mapping\n");
    return VCAN_STAT_FAIL;
  }

  bufferCtx->address = dma_handle;

  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
static int setupBusMasterTranslationTable(VCanCardData * vCard, int pos, dmaCtxBuffer_t * bufferCtx)
{
  PciCanCardData *hCard = vCard->hwCardData;

  // Every table entry is 64 bits, two table entries are used for each channel.
  unsigned int tabEntryOffset = 0x1000+pos*8;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
  DEBUGPRINT(3,"CONFIG_ARCH64\n");

  if(hCard->useDmaAddr64)
    {
      DEBUGPRINT(3,"useDmaAddr64\n");

      iowrite32( (uint32_t)(bufferCtx->address | AV_ATT_ASBE_MS64),
                 OFFSET_FROM_BASE(hCard->pcieBar0Base, tabEntryOffset) );

      iowrite32( (uint32_t)(bufferCtx->address >> 32),
                 OFFSET_FROM_BASE(hCard->pcieBar0Base, tabEntryOffset+4) );
    }
  else
    {
#endif
      iowrite32( (uint32_t)(bufferCtx->address | AV_ATT_ASBE_MS32),
                 OFFSET_FROM_BASE(hCard->pcieBar0Base, tabEntryOffset) );
      iowrite32(0, OFFSET_FROM_BASE(hCard->pcieBar0Base, tabEntryOffset+4) );

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    }
#endif

  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
static int setupDmaMappings(VCanCardData * vCard )
{
  PciCanCardData *hCard = vCard->hwCardData;
  int i;

  for(i=0; i<2; i++)
  {
    int retval;

    retval = mapOneBuffer(vCard, &(hCard->dmaCtx.bufferCtx[i]));

    if( retval != VCAN_STAT_OK )
      return retval;

    DEBUGPRINT(3,"CAN Receiver DMA Buffer %u Addr:%p : Handle:%x\n",i,
               hCard->dmaCtx.bufferCtx[i].data,
               (unsigned int)hCard->dmaCtx.bufferCtx[i].address);

    setupBusMasterTranslationTable(vCard, i, &(hCard->dmaCtx.bufferCtx[i]));
  }


  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
static int removeOneBuffer(VCanCardData * vCard, dmaCtxBuffer_t * bufferCtx)
{
  PciCanCardData *hCard = vCard->hwCardData;
  struct device *dev = &hCard->dev->dev;

  if(bufferCtx->address)
    {
      dma_unmap_single(dev, bufferCtx->address, DMA_BUFFER_SZ, DMA_FROM_DEVICE);
      bufferCtx->address = 0;
    }
  else
    {
      DEBUGPRINT(3, "dmaCtx.address was not initialized\n");
    }

  if(bufferCtx->data)
    {
      kfree(bufferCtx->data);
      bufferCtx->data = 0;
    }
  else
    {
      DEBUGPRINT(3, "dmaCtx.buffer was not initialized\n");
    }

  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
static int removeDmaMappings(VCanCardData * vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  int i;

  for(i=0; i<2; i++)
    {
      DEBUGPRINT(3,"removeDmaMappings buffer:%u\n",i);

      removeOneBuffer(vCard, &(hCard->dmaCtx.bufferCtx[i]));
    }


  return 0;
}

//======================================================================
//
//======================================================================
static void aquireDmaBuffer(VCanCardData *vCard, int id)
{
  PciCanCardData *hCard = vCard->hwCardData;
  struct device *dev = &hCard->dev->dev;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;

  if(dmaCtx->active >= 0)
    {
      DEBUGPRINT(3,"DMA handling error\n");
    }

  dma_sync_single_for_cpu(dev, dmaCtx->bufferCtx[id].address, DMA_BUFFER_SZ, DMA_FROM_DEVICE);

  dmaCtx->active = id;
  dmaCtx->position = 0;
}

//======================================================================
//
//======================================================================
static void releaseDmaBuffer(VCanCardData *vCard, int id)
{
  PciCanCardData *hCard = vCard->hwCardData;
  struct device *dev = &hCard->dev->dev;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;

  dma_sync_single_for_device(dev, dmaCtx->bufferCtx[id].address, DMA_BUFFER_SZ, DMA_FROM_DEVICE);
}

//======================================================================
//
//======================================================================
static int startDMA(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  uint32_t tmp;
  uint32_t dmaInterrupts = RXBUF_IRQ_DMA_DONE0_MSK | RXBUF_IRQ_DMA_DONE1_MSK
    | RXBUF_IRQ_DMA_OVF0_MSK | RXBUF_IRQ_DMA_OVF1_MSK;

  DEBUGPRINT(3,"startDMA\n");

  IOWR_RXBUF_IRQ( hCard->canRxBuffer, dmaInterrupts);

  tmp = IORD_RXBUF_IEN(hCard->canRxBuffer);
  IOWR_RXBUF_IEN( hCard->canRxBuffer, tmp | dmaInterrupts);

  hCard->dmaCtx.active = -1;

  armDMA0(hCard->canRxBuffer);
  armDMA1(hCard->canRxBuffer);

  if(!dmaIdle(hCard->canRxBuffer)) {
    DEBUGPRINT(3,"DMA is not idle before start\n");
  }

  enableDMA(hCard->canRxBuffer);

  hCard->dmaCtx.enabled=1;

  return 0;
}

//======================================================================
//
//======================================================================
static int stopDMA(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  uint32_t tmp;
  uint32_t dmaInterrupts = RXBUF_IRQ_DMA_DONE0_MSK | RXBUF_IRQ_DMA_DONE1_MSK
    | RXBUF_IRQ_DMA_OVF0_MSK | RXBUF_IRQ_DMA_OVF1_MSK;

  DEBUGPRINT(3,"stopDMA\n");

  disableDMA(hCard->canRxBuffer);
  if(!dmaIdle(hCard->canRxBuffer)) {
    DEBUGPRINT(3,"Warning: Has not yet disabled DMA\n");
  }

  tmp = IORD_RXBUF_IEN(hCard->canRxBuffer);
  // Disable interrupts
  IOWR_RXBUF_IEN(hCard->canRxBuffer, tmp & ~dmaInterrupts);
  // Clear pending interrupts
  IOWR_RXBUF_IRQ(hCard->canRxBuffer, dmaInterrupts);

  hCard->dmaCtx.active = -1;
  hCard->dmaCtx.enabled=0;

  return 0;
}

//======================================================================
//
//======================================================================
static int getDmaWord(VCanCardData *vCard, uint32_t *data)
{
  PciCanCardData *hCard = vCard->hwCardData;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;
  uint32_t *buffer;

  // Sanity check used during development
  if( (dmaCtx->active < 0) || (dmaCtx->active > 1) )
    {
      DEBUGPRINT(1,"Error: Invalid buffer\n");
      return VCAN_STAT_BAD_PARAMETER;
    }

  buffer = dmaCtx->bufferCtx[dmaCtx->active].data;

  if(!buffer)
    {
      DEBUGPRINT(1,"Error: DMA buffer uninitialized\n");
      return VCAN_STAT_NO_DEVICE;
    }

  if(dmaCtx->position >= DMA_BUFFER_SZ/4)
    {
      DEBUGPRINT(1,"Error: DMA buffer pointer wraparound\n");
      return VCAN_STAT_BAD_PARAMETER;
    }

  --dmaCtx->psize; // Decrement before assigning data (data could be a pointer to psize!)

  *data = buffer[dmaCtx->position];

  ++dmaCtx->position;

  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
static int getDmaPacketSize(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;
  int status;
  uint32_t tmp;

  status = getDmaWord(vCard, &tmp);

  if( status ) {
    DEBUGPRINT(1, "Error: Failed to get packet size\n");
    dmaCtx->active = -1;
    return status;
  }

  dmaCtx->psize = (unsigned int)tmp;

  if(dmaCtx->psize > 21) {
    DEBUGPRINT(1,"Error: Buffer corruption detected psize:%u pos:%u\n",
               dmaCtx->psize,dmaCtx->position);

    dmaCtx->active = -1;

    return VCAN_STAT_FAIL;
  }

  if( !dmaCtx->psize )
    {
      // End of buffer reached
      dmaCtx->active = -1;
    }
  else
    {
      --dmaCtx->psize;
    }

  return dmaCtx->psize;
}

//======================================================================
//
//======================================================================
static int getDmaPacketTS(VCanCardData *vCard, pciefd_packet_t *packet)
{
  PciCanCardData *hCard = vCard->hwCardData;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;
  int status;
  uint32_t *timestamp = (uint32_t*)&packet->timestamp;

  status = getDmaWord(vCard, &timestamp[0]);

  if( status ) return status;

  if( !dmaCtx->psize ) DEBUGPRINT(1,"Error: Packet unaligned\n");

  status = getDmaWord(vCard, &timestamp[1]);

  if( status ) return status;

  return VCAN_STAT_OK;
}

//======================================================================
//
//======================================================================
int readDMA(VCanCardData * vCard, pciefd_packet_t *packet)
{
  PciCanCardData *hCard = vCard->hwCardData;
  dmaCtx_t *dmaCtx = &hCard->dmaCtx;
  int status;
  int channel;

  if(dmaCtx->active < 0)
    {
      DEBUGPRINT(1,"Error: Invalid buffer\n");
      return VCAN_STAT_BAD_PARAMETER;
    }

  if(!dmaCtx->bufferCtx[dmaCtx->active].address)
    {
      DEBUGPRINT(1,"Warning: DMA Buffer %u Not Setup\n",dmaCtx->active);
      return VCAN_STAT_BAD_PARAMETER;
    }

  if(!dmaCtx->bufferCtx[dmaCtx->active].data)
    {
      DEBUGPRINT(1,"dmaCtx.buffer uninitialized\n");
      return VCAN_STAT_BAD_PARAMETER;
    }

  status = getDmaPacketSize(vCard);
  if( status < 0 ) goto leave;
  if( status == 0 ) {
    status = VCAN_STAT_FAIL;
    goto leave;
  }

  // Read first two words to determine packet type
  status = getDmaWord(vCard, &packet->id);

  if(status) goto leave;

  status = getDmaWord(vCard, &packet->control);

  if(status) goto leave;

  channel = packetChannelId(packet);

  if( isDataPacket(packet) )
    {
      int i;
      int dlc;
      int nbytes;
      int nwords;

      DEBUGPRINT(3, "Data packet CH:%u.%u\n",channel, dmaCtx->active);

      if( !dmaCtx->psize ) goto unaligned;

      dlc = getDLC(packet);

      if((dlc > 0) && !dmaCtx->psize) goto unaligned;

      if(isFlexibleDataRateFormat(packet))
        {
          nbytes = dlcToBytesFD(dlc);
        }
      else
        {
          nbytes = dlcToBytes(dlc);
        }

      if ( isRemoteRequest(packet) ) {
        nbytes = 0;
      }

      nwords = bytesToWordsCeil(nbytes);

      status = getDmaPacketTS(vCard, packet);
      if(status) goto leave;

      for(i=0;i<nwords;i++)
        {
          if( !dmaCtx->psize ) goto unaligned;
          status = getDmaWord(vCard, &packet->data[i]);
          if(status) goto leave;
        }

      if( dmaCtx->psize ) goto unaligned;

      goto leave;
    }
  else if( isAckPacket(packet)       ||
           isTxrqPacket(packet)      ||
           isEFlushAckPacket(packet) ||
           isEFrameAckPacket(packet))
    {
      DEBUGPRINT(3,"ack packet CH:%u.%u\n",channel, dmaCtx->active);
      status = getDmaPacketTS(vCard, packet);
      if( dmaCtx->psize ) goto unaligned;
      goto leave;
    }
  else if( isErrorPacket(packet) )
    {
      DEBUGPRINT(3,"error packet CH:%u.%u\n",channel, dmaCtx->active);

      if( !dmaCtx->psize ) goto unaligned;

      status = getDmaPacketTS(vCard, packet);
      if( dmaCtx->psize ) goto unaligned;
    }
  else if( isOffsetPacket(packet) )
    {
      DEBUGPRINT(3,"offset CH:%u.%u\n",channel, dmaCtx->active);
      if( dmaCtx->psize ) goto unaligned;
    }
  else if( isBusLoadPacket(packet) )
    {
      DEBUGPRINT(3,"bus load CH:%u.%u\n",channel, dmaCtx->active);

      if( !dmaCtx->psize ) goto unaligned;

      status = getDmaPacketTS(vCard, packet);

      if( dmaCtx->psize ) goto unaligned;
    }
  else if( isStatusPacket(packet) )
    {
      DEBUGPRINT(3,"status CH:%u.%u\n",channel, dmaCtx->active);

      if( !dmaCtx->psize ) goto unaligned;

      status = getDmaPacketTS(vCard, packet);

      if( dmaCtx->psize ) goto unaligned;
    }
  else if( isDelayPacket(packet) )
    {
      DEBUGPRINT(3,"delay CH:%u.%u\n",channel, dmaCtx->active);
      if( dmaCtx->psize ) goto unaligned;
    }
  else{
    DEBUGPRINT(1,"Undefined packet CH:%u.%u. id = 0x%08x, ctrl = 0x%08x\n",
               channel, dmaCtx->active, packet->id, packet->control);
    goto deactivate;
  }

  goto leave;

 unaligned:
  DEBUGPRINT(1,"Packet unaligned\n");

 deactivate:
  status = VCAN_STAT_FAIL;
  dmaCtx->active = -1;

 leave:
  return status;
}
#endif // USE_DMA



//======================================================================
// Convert 64-bit, 12.5 ns resolution time to 32-bit tick count (10us)
//======================================================================
static unsigned long timestamp_to_ticks(VCanCardData *vCard, unsigned long long ts)
{
  PciCanCardData *hCard = vCard->hwCardData;

  do_div(ts, hCard->freqToTicksDivider);

  DEBUGPRINT(4,"Freq:%u US:%u TS:%llu\n", hCard->frequency, vCard->usPerTick, ts);

  return ts;
}


//======================================================================
// Current Time read from HW
// Must be called with card locked (to avoid access interference).
//
// Important: The LSB part must be read first. The MSB of the timestamp
// is registered when the LSB is read.
//======================================================================
static unsigned long long pciCanHwTime(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned long irqFlags;
  uint64_t sw_ts;

  os_if_spin_lock_irqsave(&hCard->lock, &irqFlags);

  sw_ts = IORD(hCard->timestampBase, 0);
  sw_ts |= (uint64_t)(IORD(hCard->timestampBase,1)) << 32;

  os_if_spin_unlock_irqrestore(&hCard->lock, irqFlags);

  return sw_ts;
}


//======================================================================
// Current Timestamp read from HW
//======================================================================
static int pciCanTime(VCanCardData *vCard, unsigned long *time)
{
  uint64_t sw_ts;

  sw_ts = pciCanHwTime(vCard);

  *time = timestamp_to_ticks(vCard, sw_ts);

  return VCAN_STAT_OK;
}


//======================================================================
// /proc read function
//======================================================================
static int pciCanProcRead (struct seq_file* m, void* v)
{
  seq_printf(m, "\ntotal channels %d\n", driverData.noOfDevices);

  return 0;
}


//======================================================================
//  All acks received?
//======================================================================
static int pciCanInSync (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;

  DEBUGPRINT(4,"pciCanInSync:%u\n", atomic_read(&hChd->outstanding_tx));

  return (atomic_read(&hChd->outstanding_tx) == 0);
} // pciCanInSync


//======================================================================
//  Can we send now?
//======================================================================
static int pciCanTxAvailable (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;

  DEBUGPRINT(4, "pciCanTxAvailable (outstanding_tx:%u)\n",atomic_read(&hChd->outstanding_tx));

  return (atomic_read(&hChd->outstanding_tx) < MAX_OUTSTANDING_TX);

} // pciCanTxAvailable


//======================================================================
//  Calculate busload prescaler
//======================================================================
static uint32_t calcBusloadPrescaler (unsigned int quantaPerCycle, unsigned int brp)
{
  unsigned int blp = (quantaPerCycle * brp) / 2;
  if (blp < BLP_PRESC_MIN) {
    blp = BLP_PRESC_MIN;
  }
  if (blp > BLP_PRESC_MAX) {
    blp = BLP_PRESC_MAX;
  }
  return (PCIEFD_BLP_PRESC (blp) | PCIEFD_BLP_INTERV (BLP_INTERVAL));
}

//======================================================================
//  Print card releated info
//======================================================================
static void printCardInfo(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  uint32_t date;
  uint32_t time;
  uint32_t rev,freq;
  uint32_t build_l,build_h;
  uint64_t hg_id;

  INITPRINT("----------------------------------------------------------------\n");
  INITPRINT("                        Card info\n");
  INITPRINT("----------------------------------------------------------------\n");
  INITPRINT("    * Build time      : %s\n", __TIMESTAMP__);

  date = IORD_SYSID_DATE(hCard->sysidBase);
  time = IORD_SYSID_TIME(hCard->sysidBase);

  rev = IORD_SYSID_REVISION(hCard->sysidBase);

  build_l = IORD_SYSID_BUILD_L(hCard->sysidBase);
  build_h = IORD_SYSID_BUILD_H(hCard->sysidBase);

  hg_id = build_h;
  hg_id <<= 16;
  hg_id |= SYSID_BUILD_L_HG_ID_GET(build_l);

  INITPRINT("    * FPGA Version    : %u.%u.%u\n",
            (uint32_t)SYSID_VERSION_MAJOR_GET(rev),
            (uint32_t)SYSID_VERSION_MINOR_GET(rev),
            (uint32_t)SYSID_BUILD_SEQNO_GET(build_l));
  INITPRINT("    * FPGA Build time : %u %u\n", date, time);
  INITPRINT("    * FPGA Hg id      : %08llx (%s)\n",
            hg_id,
            SYSID_BUILD_UC_GET(build_l) ? "has uncommitted changes" : "clean");

  if (vCard->firmwareVersionMajor != PCIEFD_FPGA_MAJOR_VER) {
    vCard->card_flags |= DEVHND_CARD_REFUSE_TO_USE_CAN;
    INITPRINT("\n    *** ERROR: Unsupported firmware version %d.%d.%d. Please upgrade to at least %d.0.1\n\n",
	      vCard->firmwareVersionMajor,
	      vCard->firmwareVersionMinor,
	      vCard->firmwareVersionBuild,
	      PCIEFD_FPGA_MAJOR_VER);
  }

  freq = IORD_SYSID_BUS_FREQUENCY(hCard->sysidBase);
  INITPRINT("    * Frequency(pci bus)  : %u.%u MHz\n", freq/1000000, freq/100000 - 10*(freq/1000000));

  freq = IORD_SYSID_FREQUENCY(hCard->sysidBase);
  INITPRINT("    * Frequency(can ctrl) : %u.%u MHz\n", freq/1000000, freq/100000 - 10*(freq/1000000));
}

//======================================================================
//  Verify parameter area
//======================================================================
#define KCC_PARAM_IMAGE_SIZE  64*1024
#define KCC_PARAM_START_ADDR     (31*65536L)

static int verifyParamImage(unsigned char * param_image)
{
  kcc_param_image_t *p;

  p= (kcc_param_image_t *)param_image;
  if ((p->version != PARAM_SYSTEM_VERSION) || (p->magic != PARAM_MAGIC))
  {
    DEBUGPRINT(1,"ERROR: verifyParamImage version or magic error\n");
    DEBUGPRINT(1,"       version: got %d, expected %d\n", p->version, PARAM_SYSTEM_VERSION);
    DEBUGPRINT(1,"       magic:   got %x, expected %x\n", p->magic, PARAM_MAGIC);
    return -6;
  }

  if (p->crc != calculateCRC32(p->param,sizeof(p->param)))
  {
    DEBUGPRINT(1,"ERROR: verifyParamImage incorrect crc should be 0x%08x, is 0x%08x\n", p->crc, calculateCRC32(p->param,sizeof(p->param)));
    return -6;
  }
  return 0;
}


//======================================================================
//  Read and verify parameter area
//======================================================================
static unsigned char * readParamImage(void *serialflash_base)
{
  unsigned char * param_image;
  int ret;

  alt_flash_epcs_dev epcsFlashDev;

  param_image = os_if_kernel_malloc(KCC_PARAM_IMAGE_SIZE);

  if(param_image == NULL)
  {
    DEBUGPRINT(1,"ERROR: readParamImage() os_if_kernel_malloc failed\n");
    return NULL;
  }

  alt_epcs_flash_init(&epcsFlashDev, serialflash_base);
  if(alt_epcs_flash_read(&epcsFlashDev.dev, KCC_PARAM_START_ADDR, param_image, KCC_PARAM_IMAGE_SIZE))
  {
      DEBUGPRINT(1,"ERROR: readParamImage() read failed\n");
      os_if_kernel_free(param_image);
      return NULL;
  }

  ret = verifyParamImage(param_image);
  if(ret)
  {
    DEBUGPRINT(1,"ERROR: readParamImage() version or magic error\n");
    os_if_kernel_free(param_image);
    return NULL;
  }

  return param_image;
}

//======================================================================
//  Read a parameter
//======================================================================
int readParameter (unsigned char * param_image,
                   unsigned int    paramNo,
                   void          * data,
                   unsigned int    paramLen)
{
  kcc_param_image_t *p;

  if(param_image == NULL)
  {
   return -1;
  }

  if(paramNo < MAX_NB_PARAMETERS)
  {
    p= (kcc_param_image_t *)param_image;
    if(p->param[paramNo].magic == PARAM_MAGIC)
    {
      if (paramLen < p->param[paramNo].param_len)
      {
        DEBUGPRINT(1,"Error: readParameter paramNo=%d paramLen=%d, actual_paramlen=%d\n", paramNo, paramLen, p->param[paramNo].param_len);
        return -1;
      }
      else
      {
        memcpy(data,p->param[paramNo].data, paramLen);
       return 0;
      }
    }
  }

  return -1;
}


#define PARAM_SERIAL_NUMBER                   124
#define PARAM_MANUFACTURING_DATE              125
#define PARAM_HW_REVISION                     127
#define PARAM_HW_TYPE                         128
#define PARAM_EAN_NUMBER                      129
#define PARAM_NUMBER_CHANNELS                 130
#define PARAM_CHANNEL_A_TRANS_TYPE            132
#define PARAM_CHANNEL_B_TRANS_TYPE            133
#define PARAM_HW_DESCRIPTION_0                134
#define PARAM_HW_DESCRIPTION_1                135
#define PARAM_HW_DESCRIPTION_2                136
#define PARAM_CHANNEL_C_TRANS_TYPE            137
#define PARAM_CHANNEL_D_TRANS_TYPE            138
#define PARAM_EAN_NUMBER_PRODUCT              143
#define PARAM_MAX_BITRATE                     148


// Parameter length, in bytes.
#define PARAM_SERIAL_NUMBER_LEN               4
#define PARAM_MANUFACTURING_DATE_LEN          4
#define PARAM_HW_REVISION_LEN                 1
#define PARAM_HW_TYPE_LEN                     1
#define PARAM_EAN_NUMBER_LEN                  8
#define PARAM_NUMBER_CHANNELS_LEN             1
#define PARAM_CHANNEL_TRANS_TYPE_LEN          1
#define PARAM_HW_DESCRIPTION_PART_LEN         8
#define PARAM_EAN_NUMBER_PRODUCT_LEN          8
#define PARAM_MAX_BITRATE_LEN                 4


//======================================================================
// Get H/W info from EPCS Serial Flash device
// This is only called before a card is initialized, so no one can
// interfere with the accesses (lock not even initialized at this point).
//======================================================================
static int DEVINIT getCardInfo(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;

  alt_flash_dev *epcsFlashDev;
  unsigned char * param_image;

  epcsFlashDev = &(hCard->epcsFlashDev.dev);

  alt_epcs_flash_init(&(hCard->epcsFlashDev), hCard->serialFlashBase);

  if ( hCard->epcsFlashDev.silicon_id == 0x14 ) /* EPCS16 */
    {
      param_image = readParamImage(hCard->serialFlashBase);
      if(param_image)
      {
        INITPRINT("----------------------------------------------------------------\n");
        INITPRINT("                        Parameters\n");
        INITPRINT("----------------------------------------------------------------\n");

        readParameter(param_image, PARAM_EAN_NUMBER, vCard->ean, PARAM_EAN_NUMBER_LEN);
        INITPRINT("    * Ean: %x-%05x-%05x-%x\n",
                  ((*(uint32_t*)&(vCard->ean[4])) >> 12),
                  (((*(uint32_t*)&(vCard->ean[4])) & 0xfff) << 8) | (((*(uint32_t*)&(vCard->ean[0])) >> 24) & 0xff),
                  ((*(uint32_t*)&(vCard->ean[0])) >> 4) & 0xfffff,
                  ((*(uint32_t*)&(vCard->ean[4])) & 0x0f));

        readParameter(param_image, PARAM_HW_REVISION, &vCard->hwRevisionMajor, PARAM_HW_REVISION_LEN);
        INITPRINT("    * HW Revision Major: %u\n",vCard->hwRevisionMajor);

        readParameter(param_image, PARAM_SERIAL_NUMBER, &vCard->serialNumber, PARAM_SERIAL_NUMBER_LEN);
        INITPRINT("    * Serial: %u\n",vCard->serialNumber);

        readParameter(param_image, PARAM_HW_TYPE, &vCard->hw_type, PARAM_HW_TYPE_LEN);
        INITPRINT("    * HW Type: %u\n",vCard->hw_type);

        readParameter(param_image, PARAM_NUMBER_CHANNELS, &vCard->nrChannels, PARAM_NUMBER_CHANNELS_LEN);
        INITPRINT("    * Number of channels: %u\n",vCard->nrChannels);

        if(vCard->nrChannels > 0) {
          readParameter(param_image, PARAM_CHANNEL_A_TRANS_TYPE, &vCard->chanData[0]->transType, PARAM_CHANNEL_TRANS_TYPE_LEN);
          INITPRINT("    * Transceiver type ch A: %u\n",vCard->chanData[0]->transType);
        }

        if(vCard->nrChannels > 1) {
          readParameter(param_image, PARAM_CHANNEL_B_TRANS_TYPE, &vCard->chanData[1]->transType, PARAM_CHANNEL_TRANS_TYPE_LEN);
          INITPRINT("    * Transceiver type ch B: %u\n",vCard->chanData[1]->transType);
        }

        if(vCard->nrChannels > 2) {
          readParameter(param_image, PARAM_CHANNEL_C_TRANS_TYPE, &vCard->chanData[2]->transType, PARAM_CHANNEL_TRANS_TYPE_LEN);
          INITPRINT("    * Transceiver type ch C: %u\n",vCard->chanData[2]->transType);
        }

        if(vCard->nrChannels > 3) {
          readParameter(param_image, PARAM_CHANNEL_D_TRANS_TYPE, &vCard->chanData[3]->transType, PARAM_CHANNEL_TRANS_TYPE_LEN);
          INITPRINT("    * Transceiver type ch D: %u\n",vCard->chanData[3]->transType);
        }

        os_if_kernel_free(param_image);

	return VCAN_STAT_OK;
      }
    }

  return VCAN_STAT_NO_DEVICE;
}


//======================================================================
// Find out some info about the H/W
// (*cd) must have pciIf, xilinx and sjaBase initialized
// This is only called before a card is initialized, so no one can
// interfere with the accesses (lock not even initialized at this point).
//======================================================================
static int DEVINIT pciCanProbe (VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  int i;
  int status;
  uint32_t rev;

  if (hCard->pcieBar0Base == 0) {
    DEBUGPRINT(1, "pcicanProbe card_present = 0\n");
    vCard->cardPresent = 0;
    return VCAN_STAT_NO_DEVICE;
  }


  status = getCardInfo(vCard);

  if (status != VCAN_STAT_OK) {
    return VCAN_STAT_NO_DEVICE;
  }

  rev = IORD_SYSID_REVISION(hCard->sysidBase); // FPGA Design FW rev.

  vCard->firmwareVersionMajor = SYSID_VERSION_MAJOR_GET(rev);
  vCard->firmwareVersionMinor = SYSID_VERSION_MINOR_GET(rev);
  vCard->firmwareVersionBuild = SYSID_BUILD_SEQNO_GET(IORD_SYSID_BUILD_L(hCard->sysidBase));

  vCard->hwRevisionMinor = 0;

  vCard->capabilities =
    VCAN_CHANNEL_CAP_SEND_ERROR_FRAMES    |
    VCAN_CHANNEL_CAP_RECEIVE_ERROR_FRAMES |
    VCAN_CHANNEL_CAP_TIMEBASE_ON_CARD     |
    VCAN_CHANNEL_CAP_BUSLOAD_CALCULATION  |
    VCAN_CHANNEL_CAP_ERROR_COUNTERS       |
    VCAN_CHANNEL_CAP_EXTENDED_CAN         |
    VCAN_CHANNEL_CAP_TXREQUEST            |
    VCAN_CHANNEL_CAP_TXACKNOWLEDGE        |
    VCAN_CHANNEL_CAP_CANFD                |
    VCAN_CHANNEL_CAP_CANFD_NONISO;

  if(vCard->nrChannels > MAX_CHANNELS) {
    vCard->nrChannels = MAX_CHANNELS;
  }

  printCardInfo(vCard);

#if USE_DMA
  hCard->useDma = 1;
#endif
#if USE_ADJUSTABLE_PLL
  INITPRINT("Min Phase Shift %u ps\n", IORD_PLL_MIN_PHASE_SHIFT(hCard->pllBase));
  INITPRINT("Max Phase Shift %u ps\n", IORD_PLL_MAX_PHASE_SHIFT(hCard->pllBase));
  INITPRINT("Max total Offset %u ppm\n", IORD_PLL_MAX_OFFSET(hCard->pllBase));
#endif

  INITPRINT( "----------------------------------------------------------------\n");
  INITPRINT( "                        Capabilites\n");
  INITPRINT( "----------------------------------------------------------------\n");

#if USE_DMA
  if (hwSupportDMA(hCard->canRxBuffer)) {
    INITPRINT( "    * Receiver DMA supported\n");
  }
  else {
    INITPRINT( "    * Receiver DMA Not supported\n");
    hCard->useDma = 0;
  }
#endif

  if (loopback) {
    INITPRINT("    * Loopback Enabled\n");
  } else {
    INITPRINT("    * Loopback Disabled\n");
  }

  INITPRINT( "    * Max read buffer support %u\n", fifoPacketCountRxMax(hCard->canRxBuffer));

  for (i = 0; i < vCard->nrChannels; i++) {
    VCanChanData   *vChd = vCard->chanData[i];

    if (vChd != NULL) {
      PciCanChanData *hChd = vChd->hwChanData;

      // Multiple instances of a device are placed in consecutive memory space with _SPAN bytes separation.
      // This is hardcoded in the Altera bus system.
      hChd->canControllerBase = OFFSET_FROM_BASE(hCard->canControllerBase, (i * CAN_CONTROLLER_SPAN));

      // PCIe interface interrupt mask for this channel
      hChd->tx_irq_msk = (CAN_CONTROLLER_0_IRQ << i);

      vChd->channel  = i;

      // Test if a CAN controller is present at given address
      if (IORD_PCIEFD_UNDEF(hChd->canControllerBase) == 0xdeadbeef) {
        INITPRINT( "    -----------------------------------------------------------\n");
        INITPRINT( "    Channel %u\n", i);
        INITPRINT( "    -----------------------------------------------------------\n");

        if (hwSupportCanFD(hChd->canControllerBase)) {
          INITPRINT( "        * CAN FD supported\n");
        }
        INITPRINT( "        * Max write buffer support %u\n",fifoPacketCountTxMax(hChd->canControllerBase));

        if (fifoPacketCountTxMax(hChd->canControllerBase) < MAX_OUTSTANDING_TX) {
          DEBUGPRINT(1, "        * Error, MAX_OUTSTANDING_TX=%u larger than buffer\n", MAX_OUTSTANDING_TX);
          return VCAN_STAT_NO_MEMORY;
        }
      }
      else {
        vCard->nrChannels = i;
      }
    }
    else {
      vCard->nrChannels = i;
    }
  }

  if (vCard->nrChannels == 0) {
    // No channels found
    vCard->cardPresent = 0;
    return VCAN_STAT_NO_DEVICE;
  }

  vCard->cardPresent = 1;

  hCard->frequency = IORD_SYSID_FREQUENCY(hCard->sysidBase);
  hCard->freqToTicksDivider = hCard->frequency / 100000;
  if (hCard->freqToTicksDivider == 0) hCard->freqToTicksDivider = 1;

  INITPRINT( "----------------------------------------------------------------\n");


  return VCAN_STAT_OK;
} // pciCanProbe

//======================================================================
// Perform transceiver-specific setup
// Important: Is it too late to start transceivers at bus on?
// Wait for stable power before leaving reset mode?
//======================================================================
static void pciCanActivateTransceiver (VCanChanData *vChd, int linemode)
{
  PciCanChanData *hChd = vChd->hwChanData;
  VCanCardData *vCard = vChd->vCard;
  PciCanCardData *hCard = vCard->hwCardData;
  int freq = IORD_SYSID_BUS_FREQUENCY(hCard->sysidBase);

  pwmInit(freq);

  // Start with 95% power
  pwmSetFrequency(hChd->canControllerBase,500000);
  pwmSetDutyCycle(hChd->canControllerBase,95);
}

//======================================================================
// Enable bus error interrupts, and reset the
// counters which keep track of the error rate
// Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanResetErrorCounter (VCanChanData *vChd)
{
  vChd->errorCount = 0;
} // pciCanResetErrorCounter


//======================================================================
//  Set bit timing
//======================================================================
static int pciCanSetBusParams (VCanChanData *vChd, VCanBusParams *par)
{
  PciCanChanData *hChd  = vChd->hwChanData;
  VCanCardData   *vCard = vChd->vCard;
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned int    quantaPerCycle;
  unsigned long   brp;
  unsigned long   mode;
  unsigned long   freq;
  unsigned int    tseg1;
  unsigned int    tseg2;
  unsigned int    sjw;
  unsigned long   irqFlags;
  unsigned long   btrn, btrd=0;
  uint32_t bus_load_prescaler;
  uint64_t        tmp;

  freq  = par->freq;
  sjw   = par->sjw;
  tseg1 = par->tseg1;
  tseg2 = par->tseg2;

  quantaPerCycle = tseg1 + tseg2 + 1;

  if (quantaPerCycle == 0 || freq == 0) {
    DEBUGPRINT(1,"Error: Bad parameter\n");
    return VCAN_STAT_BAD_PARAMETER;
  }

  brp = (hCard->frequency * 8) / (freq * quantaPerCycle);

  if ((brp & 0x7) != 0) {
    DEBUGPRINT(1,"Error: brp residual f:%lu s1:%u s2:%u\n",freq,tseg1,tseg2);
    return VCAN_STAT_BAD_PARAMETER;
  }

  brp = brp >> 3;

  btrn = PCIEFD_BTR_SEG2(tseg2-1) | PCIEFD_BTR_SEG1(tseg1-1) | PCIEFD_BTR_SJW(sjw-1) | PCIEFD_BTR_BRP(brp-1);

  bus_load_prescaler = calcBusloadPrescaler(quantaPerCycle, brp);

  if(hwSupportCanFD(hChd->canControllerBase))
    {
      freq  = par->freq_brs;
      sjw   = par->sjw_brs;
      tseg1 = par->tseg1_brs;
      tseg2 = par->tseg2_brs;

      quantaPerCycle = tseg1 + tseg2 + 1;

      if (quantaPerCycle == 0 || freq == 0) {
        DEBUGPRINT(1,"Error: Bad parameter\n");
        return VCAN_STAT_BAD_PARAMETER;
      }

      // Perform calculations on 64-bit value
      tmp = (uint64_t)hCard->frequency * 8192ULL;
      do_div(tmp,(freq*quantaPerCycle));

      brp = (unsigned long)tmp;

      brp = brp >> 13;

      if (brp   < 1 || brp   > 8192 || // 13-bits 1 + 0..8191
          sjw   < 1 || sjw   >   16 || // 4-bits  1 + 0..15
          tseg1 < 1 || tseg1 >  512 || // 9-bits  1 + 0..511
          tseg2 < 1 || tseg2 >   32) { // 5-bits  1 + 0..31
        DEBUGPRINT(1,"Error: Other checks can fd data phase brp:%lu sjw:%u tseg1:%u tseg2:%u\n",brp,sjw,tseg1,tseg2);
        return VCAN_STAT_BAD_PARAMETER;
      }

      btrd = PCIEFD_BTR_SEG2(tseg2-1) | PCIEFD_BTR_SEG1(tseg1-1) | PCIEFD_BTR_SJW(sjw-1) | PCIEFD_BTR_BRP(brp-1);

      if (OPEN_AS_CAN != vChd->canFdMode) {
        bus_load_prescaler = calcBusloadPrescaler(quantaPerCycle, brp);
      }
    }

  // Use mutex to allow this process to sleep
  os_if_down_sema(&hChd->busOnMutex);

  if( vChd->isOnBus ) {
    DEBUGPRINT(4, "Error: Trying to set bus parameter when on bus CH:%u\n",vChd->channel);

    os_if_up_sema(&hChd->busOnMutex);

    pciCanRequestChipState(vChd);

    return VCAN_STAT_OK;//FAIL; // Should be fail //FIXME QQQ
  }

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  IOWR_PCIEFD_BLP(hChd->canControllerBase, bus_load_prescaler);
  hChd->bus_load_prescaler = bus_load_prescaler;

  mode = IORD_PCIEFD_MOD(hChd->canControllerBase);

  /* Put the circuit in Reset Mode */
  IOWR_PCIEFD_MOD(hChd->canControllerBase, mode | PCIEFD_MOD_RM_MSK);

  IOWR_PCIEFD_BTRN(hChd->canControllerBase, btrn);

  if(hwSupportCanFD(hChd->canControllerBase))
    IOWR_PCIEFD_BTRD(hChd->canControllerBase, btrd);

  mode &= ~PCIEFD_MOD_SSO_MSK;
  mode |= PCIEFD_MOD_SSO(hChd->sso);

  IOWR_PCIEFD_MOD(hChd->canControllerBase, mode); // Restore previous reset mode status

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  os_if_up_sema(&hChd->busOnMutex);

  return VCAN_STAT_OK;
} // pciCanSetBusParams


//======================================================================
//  Get bit timing
//======================================================================
static int pciCanGetBusParams (VCanChanData *vChd, VCanBusParams *par)
{
  PciCanChanData *hChd = vChd->hwChanData;
  VCanCardData *vCard = vChd->vCard;
  PciCanCardData *hCard = vCard->hwCardData;

  unsigned int  quantaPerCycle;
  unsigned long brp;
  unsigned long irqFlags;
  unsigned long btrn,btrd=0;

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  btrn = IORD_PCIEFD_BTRN(hChd->canControllerBase);

  if(hwSupportCanFD(hChd->canControllerBase))
    {
      btrd = IORD_PCIEFD_BTRD(hChd->canControllerBase);
    }
  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  par->sjw   = 1 + PCIEFD_BTR_SJW_GET(btrn);
  par->tseg1 = 1 + PCIEFD_BTR_SEG1_GET(btrn);
  par->tseg2 = 1 + PCIEFD_BTR_SEG2_GET(btrn);
  par->samp3 = 1;

  quantaPerCycle = par->tseg1 + par->tseg2 + 1;
  brp = 1 + PCIEFD_BTR_BRP_GET(btrn);

  par->freq = hCard->frequency / (quantaPerCycle * brp);

  if(hwSupportCanFD(hChd->canControllerBase))
    {
      par->sjw_brs   = 1 + PCIEFD_BTR_SJW_GET(btrd);
      par->tseg1_brs = 1 + PCIEFD_BTR_SEG1_GET(btrd);
      par->tseg2_brs = 1 + PCIEFD_BTR_SEG2_GET(btrd);
      par->samp3 = 1;

      quantaPerCycle = par->tseg1_brs + par->tseg2_brs + 1;
      brp = 1 + PCIEFD_BTR_BRP_GET(btrd);

      par->freq_brs = hCard->frequency / (quantaPerCycle * brp);
    }

  return VCAN_STAT_OK;
} // pciCanGetBusParams


//======================================================================
//  Set silent or normal mode
//======================================================================
static int pciCanSetOutputMode (VCanChanData *vChd, int silent)
{
  PciCanChanData *hChd = vChd->hwChanData;
  unsigned long irqFlags;
  uint32_t mode;

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  // Save control register
  mode = IORD_PCIEFD_MOD(hChd->canControllerBase);
  // Put the circuit in Reset Mode
  IOWR_PCIEFD_MOD(hChd->canControllerBase, mode | PCIEFD_MOD_RM_MSK);

  if (silent) {
    mode |= PCIEFD_MOD_LOM_MSK;
  }
  else {
    mode &= ~PCIEFD_MOD_LOM_MSK;
  }

  // Restore control register
  IOWR_PCIEFD_MOD(hChd->canControllerBase, mode);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  return VCAN_STAT_OK;
} // pciCanSetOutputMode


//======================================================================
//  Line mode
//======================================================================
static int pciCanSetTranceiverMode (VCanChanData *vChd, int linemode, int resnet)
{
  vChd->lineMode = linemode;
  return VCAN_STAT_OK;
} // pciCanSetTranceiverMode


//======================================================================
//  Timeout handler for the waitResponse below
//======================================================================
static void responseTimeout (unsigned long voidWaitNode)
{
  WaitNode *waitNode = (WaitNode *)voidWaitNode;

  waitNode->timedOut = 1;
  os_if_up_sema(&waitNode->waitSemaphore);

  return;
}


//======================================================================
// Query chip status (internal)
// Should not be called with channel locked
//======================================================================
static int pciCanRequestChipState (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;
  VCanCardData *vCard = vChd->vCard;
  PciCanCardData *hCard = vCard->hwCardData;
  WaitNode waitNode;
  unsigned long irqFlags = 0;
  struct timer_list waitTimer;
  int current_id = atomic_inc_return(&hCard->status_seq_no);
  int active_id;

  DEBUGPRINT(4,"pciCanRequestChipState\n");

  if ( current_id > 255 ) {
    // Only one unit is allowed to wrap around counter
    os_if_spin_lock_irqsave(&hCard->lock, &irqFlags);

    if ( (atomic_read(&hCard->status_seq_no) > 255) )
      {
        atomic_set(&hCard->status_seq_no, 1);
        DEBUGPRINT(4,"Wraparound status request id\n");
      }

    os_if_spin_unlock_irqrestore(&hCard->lock, irqFlags);

    current_id = atomic_inc_return(&hCard->status_seq_no);
  }

  // Sleep until status request packet is returned or timeout.
  // Must not be called from interrupt context.
  os_if_init_sema(&waitNode.waitSemaphore);

  waitNode.replyPtr = NULL;
  waitNode.cmdNr    = vChd->channel; // Channel ID
  waitNode.transId  = current_id;
  waitNode.timedOut = 0;

  // Add to card's list of expected responses
  os_if_write_lock_irqsave(&hCard->replyWaitListLock, &irqFlags);

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  active_id = statPendingRequest(hChd->canControllerBase);

  if ( active_id >= 0 ) {
    DEBUGPRINT(4,"Found active %d\n",active_id);
    waitNode.transId  = active_id;
    current_id = active_id;
  } else {
    DEBUGPRINT(4,"Status request:%u\n",current_id);
    // Request chip state from CAN Controller
    fpgaStatusRequest(hChd->canControllerBase, current_id);

    hChd->debug.requested_status++;
  }

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  list_add(&waitNode.list, &hCard->replyWaitList);
  os_if_write_unlock_irqrestore(&hCard->replyWaitListLock, irqFlags);

  init_timer(&waitTimer);
  waitTimer.function = responseTimeout;
  waitTimer.data = (unsigned long)&waitNode;
  waitTimer.expires = jiffies + msecs_to_jiffies(PCIEFD_SRQ_RESP_WAIT_TIME);
  add_timer(&waitTimer);

  DEBUGPRINT(6,"Waiting for stat ch:%u\n",waitNode.cmdNr);

  os_if_down_sema(&waitNode.waitSemaphore);

  // Now we either got a response or a timeout
  os_if_write_lock_irqsave(&hCard->replyWaitListLock, &irqFlags);
  list_del(&waitNode.list);
  os_if_write_unlock_irqrestore(&hCard->replyWaitListLock, irqFlags);
  del_timer_sync(&waitTimer);

  if (waitNode.timedOut) {
    DEBUGPRINT(1, "pciefdWaitResponse: return VCAN_STAT_TIMEOUT ch:%u id:%u\n",waitNode.cmdNr,current_id);
    return VCAN_STAT_TIMEOUT;
  } else {
    DEBUGPRINT(4,"Status ack on ch:%u, id:%u\n",waitNode.cmdNr, current_id);
  }

  return VCAN_STAT_OK;
} // pciCanRequestChipState


//======================================================================
//  Wait functions for Bus On/Off
//======================================================================

static int waitForIdleMode(int loopmax, VCanChanData *vChd,
                           unsigned long irqFlags)
/* Make sure that unit is in reset mode before starting flush.
 *
 * Waits a maximum of approximately loopmax ms for unit to reach idle state.
 * Returns number of turns in waiting loop if less than loopmax, otherwise
 * returns -1 to indicate timeout.
 */
{
  PciCanChanData *hChd = vChd->hwChanData;
  int loopcount = loopmax;

  while ( loopcount && !statIdle(hChd->canControllerBase) )
  {
    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    os_if_set_task_uninterruptible();
    os_if_wait_for_event_timeout_simple(1); // Wait 1 ms

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    --loopcount;
  }

  if ( loopcount == 0 ) {
    return -1;
  } else {
    return loopmax - loopcount;
  }
}


static int waitForAbort(int loopmax, VCanChanData *vChd,
                        unsigned long irqFlags)
/* Wait for abort to complete before continuing.
 *
 * Waits a maximum of approximately loopmax ms for abort command to finish.
 * Returns number of turns in waiting loop if less than loopmax, otherwise
 * returns -1 to indicate timeout.
 */
{
  PciCanChanData *hChd = vChd->hwChanData;
  int loopcount = loopmax;

  while (loopcount)
  {
    if ( istatCheck(hChd->canControllerBase, PCIEFD_IRQ_ABD_MSK) ) {
      DEBUGPRINT(4,"Abort Done CH:%u\n",vChd->channel);
      break;
    } else {
      DEBUGPRINT(4,"Abort Busy CH:%u\n",vChd->channel);

      os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

      os_if_set_task_uninterruptible();
      os_if_wait_for_event_timeout_simple(1); // Wait 1 ms

      os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);
    }

    if ( statFlushRequest(hChd->canControllerBase) ) {
      DEBUGPRINT(3, "Warning: tx flush req is asserted CH:%u\n",
                 vChd->channel);
    }

    --loopcount;
  }

  if ( loopcount < loopmax - 10 ) {
    DEBUGPRINT(3, "Waiting for Abort Done more than 10ms (%u)\n",
               loopmax - loopcount);
  }

  if ( !istatCheck(hChd->canControllerBase, PCIEFD_IRQ_ABD_MSK) )
  {
    DEBUGPRINT(3, "Abort Still Busy, probably failed initiating CH:%u\n",
               vChd->channel);
  }

  if ( loopcount == 0 ) {
    return -1;
  } else {
    return loopmax - loopcount;
  }
}


static int waitForFlush(int loopmax, VCanChanData *vChd,
                        unsigned long irqFlags)
/* Wait for flush to complete before continuing.
 *
 * Waits a maximum of approximately loopmax ms for packet flush to finish.
 * Returns number of turns in waiting loop if less than loopmax, otherwise
 * returns -1 to indicate timeout.
 */
{
  PciCanChanData *hChd = vChd->hwChanData;
  int loopcount = loopmax;

  while (loopcount)
  {
    uint32_t level;

    level = fifoPacketCountTx(hChd->canControllerBase);

    if (level == 0) {
      break;
    }

    DEBUGPRINT(3, "TX FIFO Should have been empty P:%u CH:%u STAT:%x\n",
               level, vChd->channel,
               IORD_PCIEFD_STAT(hChd->canControllerBase));

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    os_if_set_task_uninterruptible();
    os_if_wait_for_event_timeout_simple(1); // Wait 1 ms

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    --loopcount;
  }

  if ( loopcount == 0 ) {
    DEBUGPRINT(1, "Warning: Transmit buffer is not empty:\n");
    DEBUGPRINT(1, " - Probably a failed flush operation [BUSON]\n");
    return -1;
  } else {
    return loopmax - loopcount;
  }
}


static int waitForBusOn(int loopmax, VCanChanData *vChd,
                        unsigned long irqFlags)
/* Wait for bus on
 *
 * Waits a maximum of approximately loopmax ms for card to go bus on.
 * Returns number of turns in waiting loop if less than loopmax, otherwise
 * returns -1 to indicate timeout.
 */
{
  int loopcount = loopmax;

  // Wait until bus on is entered.
  while (loopcount)
  {
    if ( pciCanRequestChipState(vChd) == VCAN_STAT_OK ) {
      if( (vChd->chipState.state & CHIPSTAT_BUSOFF) != CHIPSTAT_BUSOFF ) {
        break;
      }
    } else {
      DEBUGPRINT(1, "Stat Failed\n");
    }

    // Wait 1 ms for transceiver to reach bus on
    os_if_set_task_uninterruptible();
    os_if_wait_for_event_timeout_simple(1);

    --loopcount;
  }

  if ( loopcount == 0 ) {
    DEBUGPRINT(1, "Bus on failed CH:%u\n", vChd->channel);
    return -1;
  } else {
    DEBUGPRINT(4, "Bus on success CH:%u\n", vChd->channel);
    return loopmax - loopcount;
  }
}


//======================================================================
//  Go bus on
//======================================================================

static int pciCanBusOn (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;
  unsigned long tmp;
  unsigned long irqFlags;
  int stat, loopmax;

  // Use mutex to allow this process to sleep
  os_if_down_sema(&hChd->busOnMutex);

  if ( vChd->isOnBus ) {
    DEBUGPRINT(4, "Warning: Already on bus CH:%u\n",vChd->channel);

    os_if_up_sema(&hChd->busOnMutex);

    pciCanRequestChipState(vChd);

    return VCAN_STAT_OK;
  }
  DEBUGPRINT(4,"BUS ON CH:%u\n",vChd->channel);

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  // Disables all interrupts
  irqInit(hChd->canControllerBase);

  // Make sure that unit is in reset mode
  tmp = IORD_PCIEFD_MOD(hChd->canControllerBase);

  if ( !PCIEFD_MOD_RM_GET(tmp) ) {
    DEBUGPRINT(4, "Warning: Reset mode not commanded before bus on\n");

    IOWR_PCIEFD_MOD(hChd->canControllerBase, tmp | PCIEFD_MOD_RM_MSK);
  }

  atomic_set(&hChd->outstanding_tx, 0);

  hChd->last_ts = 0;
  hChd->compareTransId = 1;
  hChd->lastTransId = 0;

  memset(hChd->current_tx_message, 0, sizeof(hChd->current_tx_message));

  vChd->isOnBus = 1;
  vChd->overrun = 0;
  pciCanResetErrorCounter(vChd);

  // Make sure that unit is in reset mode before starting flush
  loopmax = 100; // Wait a maximum of 100 ms
  stat = waitForIdleMode(loopmax, vChd, irqFlags);

  if ( stat < 0 ) {
    tmp = IORD_PCIEFD_STAT(hChd->canControllerBase);

    DEBUGPRINT(3, "Timeout when waiting for idle state:\n");
    DEBUGPRINT(3, " - reset mode commanded(%c) tx idle(%c) in reset mode(%c) [BUS ON]\n",
               statResetRequest(hChd->canControllerBase)    ? 'x' : ' ',
               statTransmitterIdle(hChd->canControllerBase) ? 'x' : ' ',
               statInResetMode(hChd->canControllerBase)     ? 'x' : ' '
               );
  }

  // Clear abort done irq flag
  IOWR_PCIEFD_IRQ(hChd->canControllerBase, PCIEFD_IRQ_ABD_MSK);

  // Clear RXERR, TXERR and Flush buffers
  // * This command will reset the CAN controller. When the reset is done a
  //   status packet will be placed in the receiver buffer. The status packet
  //   will have the associated command sequence number and the init detected
  //   bit set.
  fpgaFlushAll(hChd->canControllerBase, 0);

  // Wait for abort done signal
  loopmax = 100; // Wait a maximum of 100 ms
  stat = waitForAbort(loopmax, vChd, irqFlags);

  // Wait for flush to finish
  loopmax = 10; // Wait a maximum of 10 ms
  stat = waitForFlush(loopmax, vChd, irqFlags);

  // Write a flush recovery control word
  IOWR_PCIEFD_CONTROL(hChd->canControllerBase, PCIEFD_CONTROL_END_FLUSH);

  // Enable bus load packets
  IOWR_PCIEFD_BLP(hChd->canControllerBase, hChd->bus_load_prescaler);

  // qqqcd: There are several writes to mode register here.
  //        Move them all into one wrapper function?
  if (OPEN_AS_CANFD_NONISO == vChd->canFdMode) {
    enableNonIsoMode(hChd->canControllerBase);
  }
  else if (OPEN_AS_CANFD_ISO == vChd->canFdMode) {
    disableNonIsoMode(hChd->canControllerBase);
  }

  tmp = IORD_PCIEFD_MOD(hChd->canControllerBase);

  if (OPEN_AS_CAN == vChd->canFdMode) {
    tmp |= PCIEFD_MOD_CLASSIC(1);
  } else {
    tmp &= ~PCIEFD_MOD_CLASSIC_MSK;
  }

  IOWR_PCIEFD_MOD(hChd->canControllerBase, tmp & ~PCIEFD_MOD_RM_MSK );


  irqEnableTransmitterError(hChd->canControllerBase);

  vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  // Wait for bus on
  loopmax = 1000; // Wait a maximum of 1000 ms
  stat = waitForBusOn(loopmax, vChd, irqFlags);

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  ledOn(hChd->canControllerBase);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  os_if_up_sema(&hChd->busOnMutex);

  return VCAN_STAT_OK;
} // pciCanBusOn


//======================================================================
//  Go bus off
//======================================================================
static int pciCanBusOff (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;
  unsigned long tmp;
  unsigned long irqFlags;
  int stat, loopmax;

  os_if_down_sema(&hChd->busOnMutex);

  DEBUGPRINT(4, "BUS OFF CH:%u\n", vChd->channel);

  // Since we are bus off we have no messages *on the way*
  atomic_set(&hChd->outstanding_tx, 0);

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  vChd->isOnBus = 0;
  vChd->overrun = 0;

  memset(hChd->current_tx_message, 0, sizeof(hChd->current_tx_message));

  tmp = IORD_PCIEFD_MOD(hChd->canControllerBase);

  IOWR_PCIEFD_MOD(hChd->canControllerBase, tmp | PCIEFD_MOD_RM_MSK);

  // Make sure that unit is in reset mode before starting flush
  loopmax = 100;
  stat = waitForIdleMode(loopmax, vChd, irqFlags);

  if ( stat < 0 ) {
    tmp = IORD_PCIEFD_STAT(hChd->canControllerBase);

    DEBUGPRINT(3, "Timeout when waiting for idle state:\n");
    DEBUGPRINT(3, " - reset mode commanded(%c) tx idle(%c) in reset mode(%c) [BUS OFF]\n",
               statResetRequest(hChd->canControllerBase)    ? 'x' : ' ',
               statTransmitterIdle(hChd->canControllerBase) ? 'x' : ' ',
               statInResetMode(hChd->canControllerBase)     ? 'x' : ' '
               );
  }

  // Disable bus load packets
  IOWR_PCIEFD_BLP(hChd->canControllerBase, 0);

  ledOff(hChd->canControllerBase);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  os_if_up_sema(&hChd->busOnMutex);

  pciCanRequestChipState(vChd);

  return VCAN_STAT_OK;
} // pciCanBusOff


//======================================================================
//  Enable/disable interrupts on card
//======================================================================
static void pciCanInterrupts (VCanCardData *vCard, int enable)
{
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned long irqFlags;

  // The card must be present!
  if (!vCard->cardPresent) {
    return;
  }

  os_if_spin_lock_irqsave(&hCard->lock, &irqFlags);

  // Disable/enable interrupts from card
  if (enable) {
    DEBUGPRINT(3,"Enable interrupts:%08x\n",ALL_INTERRUPT_SOURCES_MSK);
    IOWR_PCIE_IEN(hCard->pcie, ALL_INTERRUPT_SOURCES_MSK);
  }else{
    DEBUGPRINT(3,"Disable interrupts\n");
    IOWR_PCIE_IEN(hCard->pcie, 0);
  }

  os_if_spin_unlock_irqrestore(&hCard->lock, irqFlags);
}


//======================================================================
// Process status packet information
//======================================================================
static void updateChipState(VCanChanData *vChd)
{
  PciCanChanData *hChd  = vChd->hwChanData;
  VCanCardData *vCard = vChd->vCard;
  PciCanCardData *hCard = vCard->hwCardData;
  pciefd_packet_t *packet = &hCard->packet;

  vChd->chipState.txerr = getTransmitErrorCount(packet);
  vChd->chipState.rxerr = getReceiveErrorCount(packet);

  if ( statusBusOff(packet) && statusErrorPassive(packet) ) {
      DEBUGPRINT(5,"Is bus off due tx error == 256 (BOFF=1), and Error Passive\n");
      vChd->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE | CHIPSTAT_ERROR_WARNING;
  }
  else if ( statusBusOff(packet) ) {
    DEBUGPRINT(5,"Is bus off due to tx error == 256 (BOFF=1)\n");
    vChd->chipState.state = CHIPSTAT_BUSOFF;
  }
  else if ( statusInResetMode(packet) ) {
    DEBUGPRINT(5,"Is bus off in reset mode (BOFF=0)\n");
    vChd->chipState.state = CHIPSTAT_BUSOFF;
  }
  else if ( statusErrorPassive(packet) ) {
    DEBUGPRINT(5,"Error Passive Limit Reached\n");
    vChd->chipState.state = CHIPSTAT_ERROR_PASSIVE | CHIPSTAT_ERROR_WARNING;
  }
  // Warning level is disabled. See FB 16545 for info.
  /* else if ( statusErrorWarning(packet) ) { */
  /*   DEBUGPRINT(5,"Error Warning Limit Reached\n"); */
  /*   vChd->chipState.state = CHIPSTAT_ERROR_WARNING; */
  /* } */
  else {
    DEBUGPRINT(5,"Bus on\n");
    vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
  }

  if ( statusOverflow(packet) ) {
    DEBUGPRINT(5,"Receive Buffer Overrun Error\n");
    hChd->overrun_occured = 1;
  }
}

//======================================================================
//  Timeout handler for the waitResponse below
//======================================================================
static void errorRestart (unsigned long data)
{
  PciCanChanData *hChd = (PciCanChanData *)data;
  unsigned long irqFlags;

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  enableErrorPackets(hChd->canControllerBase);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  hChd->vChan->errorCount = 0;

  DEBUGPRINT(4,"Reenable error packets CH:%u\n", hChd->vChan->channel);
}

//======================================================================
//
//======================================================================
static void receivedPacketHandler (VCanCardData *vCard)
{
  int r;
  unsigned long irqFlags;
  VCAN_EVENT e;
  PciCanCardData *hCard = vCard->hwCardData;
  pciefd_packet_t *packet = &hCard->packet;
  int chid = packetChannelId(packet);
  VCanChanData *vChd;
  PciCanChanData *hChd;

  // Get time stamp when message is processed
  uint64_t sw_ts = pciCanHwTime(vCard);

  if(chid >= vCard->nrChannels) {
    DEBUGPRINT(1,"Invalid channel id %u\n",chid);
    return;
  }

  vChd = vCard->chanData[chid];
  hChd = vChd->hwChanData;

  if ( packet->timestamp )
    {
      if ( sw_ts < packet->timestamp )
        {
          DEBUGPRINT(1,"ts error, sw:%llu packet:%llu\n", sw_ts, packet->timestamp);
          hChd->debug.ts_error_cnt++;
        }
      else
        {
          uint64_t ts_diff;

          ts_diff = sw_ts - packet->timestamp;

          hChd->debug.avg_ts_diff += ts_diff;
          ++hChd->debug.avg_ts_diff_cnt;

          if( ts_diff > hChd->debug.max_ts_diff ){
            hChd->debug.max_ts_diff = ts_diff;
          }
        }

      if ( hCard->last_ts >= packet->timestamp )
        {
          uint64_t diff;

          diff = (hCard->last_ts - packet->timestamp) * 1000000000ULL;
          do_div(diff, hCard->frequency);

          DEBUGPRINT(5,"ts wrong order, last:%llu packet:%llu\n", hCard->last_ts, packet->timestamp);

          hChd->debug.ts_wrong_order_cnt++;
        }

      if ( hChd->last_ts >= packet->timestamp )
        {
          uint64_t diff;

          diff = (hChd->last_ts - packet->timestamp) * 1000000000ULL;
          do_div(diff, hCard->frequency);

          DEBUGPRINT(1,"ts wrong order, last:%llu packet:%llu\n", hChd->last_ts, packet->timestamp);

          hChd->debug.ts_wrong_order_cnt++;
        }

      hCard->last_ts = packet->timestamp;
    }

  // +----------------------------------------------------------------------
  // | Delay packet
  // +----------------------------------------------------------------------
  if( isDelayPacket(packet) )
    {
      int delay;
      DEBUGPRINT(3,"Delay packet\n");
      return;

      delay = PCIEFD_DELAY_GET(packet->id);

      if( (delay >= 0) && (delay < 60) )
        {
          int suboffset=PCIEFD_DELAY_HS_STOP_GET(packet->id)-PCIEFD_DELAY_HS_START_GET(packet->id);
          int index = delay*8+suboffset;

          if((index >= 0) && (index < 60*8))
            hChd->delay_vec[index] += 1;
        }
    }
  // +----------------------------------------------------------------------
  // | Status packet
  // +----------------------------------------------------------------------
  else if( isStatusPacket(packet) )
    {
      // Copy command and wakeup those who are waiting for this reply.
      struct list_head *currHead, *tmpHead;
      WaitNode *currNode;
      unsigned long irqFlags;

      DEBUGPRINT(4,"CH:%u Status (%x)\n",chid, packet->id);

      updateChipState(vChd);

      hChd->debug.received_status++;

      e.tag                              = V_CHIP_STATE;
      e.timeStamp                        = timestamp_to_ticks(vCard, packet->timestamp);
      e.transId                          = 0;
      e.tagData.chipState.busStatus      = (unsigned char)vChd->chipState.state;
      e.tagData.chipState.txErrorCounter = (unsigned char)vChd->chipState.txerr;
      e.tagData.chipState.rxErrorCounter = (unsigned char)vChd->chipState.rxerr;
      vCanDispatchEvent(vChd, &e);

      os_if_read_lock_irqsave(&hCard->replyWaitListLock, &irqFlags);

      list_for_each_safe(currHead, tmpHead, &hCard->replyWaitList) {
        currNode = list_entry(currHead, WaitNode, list);
        if (currNode->cmdNr == chid) {
          int diff;

          diff = statusCmdSeqNo(packet) - currNode->transId;

          if (statusCmdSeqNo(packet) == currNode->transId) {
            os_if_up_sema(&currNode->waitSemaphore);
            DEBUGPRINT(4,"Matching TID diff:%d\n",diff);
          } else {
            DEBUGPRINT(4,"Not Matching TID: got %u, expected %u diff:%d\n",
                       statusCmdSeqNo(packet), currNode->transId, diff);
          }
        }
      }

      os_if_read_unlock_irqrestore(&hCard->replyWaitListLock, irqFlags);
    }
  // +----------------------------------------------------------------------
  // | Error packet
  // +----------------------------------------------------------------------
  else if ( isErrorPacket(packet) )
    {
      VCAN_EVENT e;
      int error_code = packet->control & (EPACKET_TYPE_MSK | EPACKET_SEG_MSK | EPACKET_SEG2_MSK | EPACKET_DIR_MSK);

      ++hChd->debug.err_packet_count;
      updateChipState(vChd);

      vChd->errorCount++;

      DEBUGPRINT(4,"CH:%u Error status: Txerr(%u) Rxerr(%u)\n",
                 chid,
                 vChd->chipState.txerr,
                 vChd->chipState.rxerr);
      DEBUGPRINT(4, "CH:%u ErrorCount(%u)\n", chid, vChd->errorCount);

      if (vChd->errorCount == MAX_ERROR_COUNT) {

        os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);
        disableErrorPackets(hChd->canControllerBase);
        os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

        DEBUGPRINT(4,"Disable error packets CH:%u\n", chid);

        // Set a timer to restart interrupts
        init_timer(&hChd->errorDisableTimer);
        hChd->errorDisableTimer.function = errorRestart;
        hChd->errorDisableTimer.data = (unsigned long)hChd;
        hChd->errorDisableTimer.expires = jiffies + msecs_to_jiffies(ERROR_DISABLE_TIME_MS);
        add_timer(&hChd->errorDisableTimer);
      }
      else if (vChd->errorCount < MAX_ERROR_COUNT) {
        DEBUGPRINT(5, "Bus error code: 0x%02x\n isErrorPassive:%u\n",
                   error_code, (vChd->chipState.state & CHIPSTAT_ERROR_PASSIVE) != 0);

        e.tag                = V_RECEIVE_MSG;
        e.timeStamp          = timestamp_to_ticks(vCard, packet->timestamp);
        e.transId            = 0;
        e.tagData.msg.id     = 0;
        e.tagData.msg.flags  = VCAN_MSG_FLAG_ERROR_FRAME;
        e.tagData.msg.dlc    = 0;
        vCanDispatchEvent(vChd, &e);
      }
      else {
        DEBUGPRINT(5,"Error status blocked\n");
      }
    }
  // +----------------------------------------------------------------------
  // | TXACK | Error frame ack
  // +----------------------------------------------------------------------
  else if ( isAckPacket(packet) || isEFrameAckPacket(packet))
    {
      unsigned int transId = getAckSeqNo(packet);

      if (isAckPacket(packet))
        {
          DEBUGPRINT(4,"CH:%u Ack Packet, ID %u, Outstanding %d\n",
                     chid, transId, atomic_read(&hChd->outstanding_tx));
        }
      else
        {
          DEBUGPRINT(4,"CH:%u Error frame ack Packet, ID %u, Outstanding %d\n",
                     chid, transId, atomic_read(&hChd->outstanding_tx));
        }


      if(isFlushed(packet)) // Got flushed packet, do nothing.
        {
          DEBUGPRINT(4,"Ack was flushed, ID %u\n",transId);
        }
      else if ((transId == 0) || (transId > MAX_OUTSTANDING_TX))
        {
          DEBUGPRINT(3, "CMD_TX_ACKNOWLEDGE chan %d ERROR transid %d\n", chid, transId);
        }
      else
        {
          VCAN_EVENT *e = (VCAN_EVENT *)&hChd->current_tx_message[transId - 1];
          e->tag       = V_RECEIVE_MSG;
          e->timeStamp = timestamp_to_ticks(vCard, packet->timestamp);
          e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXRQ;
          e->tagData.msg.flags |= VCAN_MSG_FLAG_TXACK;
          vCanDispatchEvent(vChd, e);


        hChd->current_tx_message[transId - 1].user_data = 0;

        // Wake up those who are waiting for all sending to finish
        if (atomic_add_unless(&hChd->outstanding_tx, -1, 0)) {
          // Is anyone waiting for this ack?
          if ( (atomic_read(&hChd->outstanding_tx) == 0)
               && queue_empty(&vChd->txChanQueue)
               && test_and_clear_bit(0, &vChd->waitEmpty) )
            {
              os_if_wake_up_interruptible(&vChd->flushQ);
            }
          if (!queue_empty(&vChd->txChanQueue))
            {
              pciCanRequestSend(vCard, vChd);
            } else {
            DEBUGPRINT(4, "Queue empty\n");
          }
        }
        else
          {
            DEBUGPRINT(4, "TX ACK when not waiting for one\n");
          }
        ++hChd->debug.ack_packet_count;
      }
    }
  // +----------------------------------------------------------------------
  // | TXRQ Packet
  // +----------------------------------------------------------------------
  else if ( isTxrqPacket(packet) )
    {
      unsigned int transId = getTxrqSeqNo(packet);

      DEBUGPRINT(3,"### Txrq packet : chan(%u) seq no(%u)\n", chid, transId);

      // A TxReq. Take the current tx message, modify it to a
      // receive message and send it back.

      if ((transId == 0) || (transId > MAX_OUTSTANDING_TX)) {
        DEBUGPRINT(3, "CMD_TX_REQUEST chan %d ERROR transid to high %d\n",
                   chid, transId);
      }else{
        if (hChd->current_tx_message[transId - 1].flags & VCAN_MSG_FLAG_TXRQ)
          {
            VCAN_EVENT *e         = (VCAN_EVENT *)&hChd->current_tx_message[transId - 1];
            e->tag                = V_RECEIVE_MSG;
            e->timeStamp          = timestamp_to_ticks(vCard, packet->timestamp);
            e->tagData.msg.flags &=  ~VCAN_MSG_FLAG_TXACK;
            vCanDispatchEvent(vChd, e);
          }

        ++hChd->debug.trq_packet_count;
      }
    }
  // +----------------------------------------------------------------------
  // | Data packet
  // |
  // +----------------------------------------------------------------------
  else if ( isDataPacket(packet) )
    {
      unsigned short int  flags;
      unsigned int nbytes;

      int id = getId(packet);

      DEBUGPRINT(4,"### Data packet : chan(%u) dlc(%u) ext(%u) rem(%u) seq no(%u)\n",
                 chid,
                 getDLC(packet),
                 isExtendedId(packet),
                 isRemoteRequest(packet),
                 getSeqNo(packet) );

      if ( errorWarning(packet) ) {
        DEBUGPRINT(4,"### Error Warning Limit Reached\n");
      }

      if ( isErrorPassive(packet) ) {
        DEBUGPRINT(4,"### Error Passive Limit Reached\n");
      }

      if ( receiverOverflow(packet) ) {
        DEBUGPRINT(3,"### Receive Buffer Overrun Error\n");
        hChd->overrun_occured = 1;
      }

      if ( isExtendedId(packet) ) {
        id |= VCAN_EXT_MSG_ID;
      }

      flags = (unsigned char)(isRemoteRequest(packet) ? VCAN_MSG_FLAG_REMOTE_FRAME : 0);

      if (hChd->overrun_occured) {
        flags |= VCAN_MSG_FLAG_OVERRUN;
        hChd->overrun_occured = 0;
      }

      if( isFlexibleDataRateFormat(packet) )
        {
          flags |= VCAN_MSG_FLAG_FDF;
          DEBUGPRINT(4,"Received CAN FD frame\n");

          if( isAlternateBitRate(packet) ){
            flags |= VCAN_MSG_FLAG_BRS;
          }
          if( errorStateIndicated(packet) ){
            flags |= VCAN_MSG_FLAG_ESI;
          }

          nbytes = dlcToBytesFD(getDLC(packet));

          e.tagData.msg.dlc   = getDLC(packet) & 0x0f;
        }
      else
        {
          nbytes = dlcToBytes(getDLC(packet));
          e.tagData.msg.dlc = getDLC(packet) & 0x0f;
        }

      e.tag               = V_RECEIVE_MSG;
      e.timeStamp         = timestamp_to_ticks(vCard, packet->timestamp);
      e.transId           = 0;
      e.tagData.msg.id    = id;
      e.tagData.msg.flags = flags;

      memcpy(e.tagData.msg.data, packet->data, nbytes);

      r = vCanDispatchEvent(vChd, &e);

      if ( hChd->debug.got_seq_no ) {
        unsigned char next_seq_no = hChd->debug.last_seq_no + 1;

        if ( getSeqNo(packet) != next_seq_no ) {
          ++hChd->debug.seq_no_mismatch;
        }
      }else{
        hChd->debug.got_seq_no = 1;
      }
      hChd->debug.last_seq_no = getSeqNo(packet);

      ++hChd->debug.packet_count;

      if (vChd->errorCount > 0) {
        os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

        pciCanResetErrorCounter(vChd);

        os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);
      }
    }
  // +----------------------------------------------------------------------
  // | Bus Load packet
  // +----------------------------------------------------------------------
  else if( isBusLoadPacket(packet) )
    {
      DEBUGPRINT(4,"Bus Load\n");

      hChd->load = getBusLoad(packet) / BLP_DIVISOR;
      hChd->debug.bus_load = hChd->load;
    }
  // +----------------------------------------------------------------------
  // | End of flush ack packet
  // +----------------------------------------------------------------------
  else if( isEFlushAckPacket(packet) )
    {
      // Do nothing
      DEBUGPRINT(4,"CH:%u End of flush ack\n", chid);

      if(!isFlushed(packet))
        {
          DEBUGPRINT(3,"End of flush ack when device was not in flush mode\n");
        }
    }
  else
    {
      DEBUGPRINT(3,"Unknown packet type\n");
    }
}

//======================================================================
//  Timeout handler for the waitResponse below
//======================================================================
static void interruptRestart (unsigned long data)
{
  VCanCardData *vCard = (VCanCardData *)data;
  int i;
  for(i=0;i<(unsigned)vCard->nrChannels;i++)
    {
      VCanChanData   *vChd = vCard->chanData[i];

      DEBUGPRINT(3, "Reset error count %u txerr %u rxerr %u\n",vChd->errorCount,
                 (unsigned char)vChd->chipState.txerr,
                 (unsigned char)vChd->chipState.rxerr
                 );
      vChd->errorCount = 0;
    }

  DEBUGPRINT(3,"Reenable interrupt\n");
  pciCanInterrupts(vCard,1);
}

//======================================================================
//  Interrupt handling functions
//  Must be called with channel locked (to avoid access interference).
//  * Will be called without lock as all the receive FIFO accesses are card
//    specific and are accessed only from the interrupt context.
//======================================================================
static void pciCanReceiveIsr (VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  // Limit absolute number of loops
  unsigned int loopMax  = 10000;
  unsigned long irqFlags;
  unsigned int dataValid = 0;
  unsigned int irqHandled = 0;
  unsigned int irq;

  while(1)
    {
      dataValid = 0;
      // A loop counter as a safety measure.
      // NOTE: Returning from within the pciCanReceiveIsr main loop will result in interrupts being lost.
      if (--loopMax == 0) {
        int i;
        DEBUGPRINT(3, "pciCanReceiverIsr: Loop counter as a safety measure!!!\n");
        DEBUGPRINT(3, "During test, shutting down card!\n");
        for(i=0;i<(unsigned)vCard->nrChannels;i++)
          {
            VCanChanData   *vChd = vCard->chanData[i];
            PciCanChanData *hChd = vChd->hwChanData;

            DEBUGPRINT(3, "Errorcount %u\n",vChd->errorCount);

            DEBUGPRINT(3,"ch%u IEN:%08x IRQ:%08x\n",
                       i,
                       IORD_PCIEFD_IEN(hChd->canControllerBase),
                       IORD_PCIEFD_IRQ(hChd->canControllerBase)
                       );
          }
        pciCanInterrupts(vCard,0);

        // Set a timer to restart interrupts
        init_timer(&hCard->interruptDisableTimer);
        hCard->interruptDisableTimer.function = interruptRestart;
        hCard->interruptDisableTimer.data = (unsigned long)vCard;
        hCard->interruptDisableTimer.expires = jiffies + msecs_to_jiffies(100);
        add_timer(&hCard->interruptDisableTimer);

        break;//return;
      }

      os_if_spin_lock_irqsave(&hCard->lock, &irqFlags);

      irq = fifoIrqStatus(hCard->canRxBuffer);

      if(irq & RXBUF_IRQ_RA_MSK)
        {
#if USE_DMA
          if(hCard->useDma && hCard->dmaCtx.enabled) {
            DEBUGPRINT(3,"Warning:FIFO direct access\n");
          }
#endif
          // +----------------------------------------------------------------------
          // | Read from FIFO
          // | The lock could probably be removed as the FIFO is only accessed from
          // | this point.
          // +----------------------------------------------------------------------
          // 2. Store one packet for each channel with data at beginning of interrupt
          if(readFIFO(vCard, &hCard->packet) < 0)
            {
              uint32_t tmp = IORD_RXBUF_CTRL(hCard->canRxBuffer);
              DEBUGPRINT(3,"readFIFO error\n");

#if USE_DMA
              if(hCard->useDma)
                DEBUGPRINT(3,"IOC:%x EN:%u\n", tmp, hCard->dmaCtx.enabled);
#endif
              if(tmp & RXBUF_CTRL_DMAEN_MSK)
                {
                  DEBUGPRINT(3,"DMAEN\n");
                  tmp = IORD_RXBUF_IEN(hCard->canRxBuffer);
                  IOWR_RXBUF_IEN(hCard->canRxBuffer, tmp & ~RXBUF_IRQ_RA_MSK);
                }
            }
          else
            {
              dataValid = 1;
            }

          // Try to clear receieved interrupt (will not have any effect if more packets are available)
          irqHandled = RXBUF_IRQ_RA_MSK;
        }
#if USE_DMA
      // DMA Interrupts
      else if( irq & RXBUF_IRQ_DMA_DONE0_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_DONE0_MSK;

          aquireDmaBuffer(vCard,0);
        }
      else if( irq & RXBUF_IRQ_DMA_DONE1_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_DONE1_MSK;

          aquireDmaBuffer(vCard,1);
        }

      if(hCard->useDma && (hCard->dmaCtx.active >= 0))
        {
          int bufid = hCard->dmaCtx.active;
          int status = readDMA(vCard, &hCard->packet);

          if(status == VCAN_STAT_OK)
            { // A packet was read from theDMA buffer
              if(hCard->dmaCtx.active < 0)
                DEBUGPRINT(3,"DMA: Unexpected deactivation\n");

              dataValid = 1;
            }
          else
            { // Buffer is empty, release to device and rearm DMA
              hCard->dmaCtx.active = -1;

              releaseDmaBuffer(vCard, bufid);
              // No more data in current buffer
              if(bufid == 0)
                armDMA0(hCard->canRxBuffer);
              else if(bufid == 1)
                armDMA1(hCard->canRxBuffer);
            }
        }
#endif

      os_if_spin_unlock_irqrestore(&hCard->lock, irqFlags);

#if USE_DMA
      if( irq & RXBUF_IRQ_DMA_OVF0_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_OVF0_MSK;

          DEBUGPRINT(3,"DMA OVF 0\n");

          if(hCard->dmaCtx.active == 0)
            {
              DEBUGPRINT(3,"DMA restart\n");
              hCard->dmaCtx.active = -1;
            }
        }

      if( irq & RXBUF_IRQ_DMA_OVF1_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_OVF1_MSK;

          DEBUGPRINT(3,"DMA OVF 1\n");

          if(hCard->dmaCtx.active == 1)
            {
              DEBUGPRINT(3,"DMA restart\n");
              hCard->dmaCtx.active = -1;
            }
        }

      if( irq & RXBUF_IRQ_DMA_UNF0_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_UNF0_MSK;

          DEBUGPRINT(3,"DMA UNF 0\n");
        }

      if( irq & RXBUF_IRQ_DMA_UNF1_MSK )
        {
          irqHandled |= RXBUF_IRQ_DMA_UNF1_MSK;

          DEBUGPRINT(3,"DMA UNF 1\n");
        }

      if ((irq & RXBUF_IRQ_DMA_DONE0_MSK ) && (irq & RXBUF_IRQ_DMA_DONE1_MSK))
        {
          DEBUGPRINT(3,"Warning: Both buffer signals ready status\n");
        }

#endif

      if(irq & RXBUF_IRQ_UNDERFLOW_MSK)
        {
          DEBUGPRINT(3,"RXBUF_IRQ_UNDERFLOW\n");
          irqHandled |= RXBUF_IRQ_UNDERFLOW_MSK;
        }

      if(irq & RXBUF_IRQ_UNALIGNED_READ_MSK)
        {
          DEBUGPRINT(3,"RXBUF_IRQ_UNALIGNED_READ\n");
          irqHandled |= RXBUF_IRQ_UNALIGNED_READ_MSK;
        }

      if(irq & RXBUF_IRQ_MISSING_TAG_MSK)
        {
          DEBUGPRINT(3,"RXBUF_IRQ_MISSING_TAG\n");
          irqHandled |= RXBUF_IRQ_MISSING_TAG_MSK;
        }

      fifoIrqClear(hCard->canRxBuffer ,irqHandled);

      if( (irq & ~irqHandled) != 0 )
        {
          DEBUGPRINT(3,"Unhandled interrupt %x\n",irq & ~irqHandled);
          fifoIrqClear(hCard->canRxBuffer, irq & ~irqHandled);
        }

      if(dataValid)
        receivedPacketHandler(vCard);
      else
        break;//return;
    }
} // pciCanReceiveIsr


//======================================================================
//  Transmit interrupt handler
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanTransmitIsr (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;
  unsigned int irq;
  unsigned int irqHandled = 0;

  irq = irqStatus(hChd->canControllerBase);

  if(irq & PCIEFD_IRQ_TAR_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_TAR_MSK\n");
      irqHandled |= PCIEFD_IRQ_TAR_MSK;
    }

  if(irq & PCIEFD_IRQ_TAE_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_TAE_MSK\n");
      irqHandled |= PCIEFD_IRQ_TAE_MSK;
    }

  if(irq & PCIEFD_IRQ_BPP_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_BPP_MSK\n");
      irqHandled |= PCIEFD_IRQ_BPP_MSK;
    }

  if(irq & PCIEFD_IRQ_FDIC_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_FDIC_MSK\n");
      irqHandled |= PCIEFD_IRQ_FDIC_MSK;
    }


  if(irq & PCIEFD_IRQ_TOF_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_TOF_MSK\n");
      irqHandled |= PCIEFD_IRQ_TOF_MSK;
    }

  if(irq & PCIEFD_IRQ_TAL_MSK)
    {
      DEBUGPRINT(3,"PCIEFD_IRQ_TAL_MSK\n");
      irqHandled |= PCIEFD_IRQ_TAL_MSK;
    }


  irqClear(hChd->canControllerBase ,irqHandled);
} // pciCanTransmitIsr


//======================================================================
//  Main ISR
//======================================================================
// Interrupt handler prototype changed in 2.6.19.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
OS_IF_INTR_HANDLER pciCanInterrupt (int irq, void *dev_id, struct pt_regs *regs)
#else
  OS_IF_INTR_HANDLER pciCanInterrupt (int irq, void *dev_id)
#endif
{
  VCanCardData    *vCard   = (VCanCardData *)dev_id;
  PciCanCardData  *hCard;
  unsigned int    loopmax  = 10000;
  int             handled = 0;
  unsigned long   pcie_irq;

  if(!vCard)
    {
      DEBUGPRINT(1, "Unassigned IRQ\n");
      return IRQ_NONE;
    }

  hCard = vCard->hwCardData;

  // Read interrupt status from Altera PCIe HW
  while ( (pcie_irq = (IORD_PCIE_IRQ(hCard->pcie) & ALL_INTERRUPT_SOURCES_MSK)) )
    {
      int i;

      if (--loopmax == 0) {
        // Kill the card.
        DEBUGPRINT(1, "PCIcan runaway, shutting down card!! (pcie_irq:0x%x)\n",(unsigned int)pcie_irq);

        pciCanInterrupts(vCard,0);
        return IRQ_HANDLED;
      }

      handled = 1;

      // Handle shared receive buffer interrupt first
      if ( pcie_irq & RECEIVE_BUFFER_IRQ ) {
        pciCanReceiveIsr(vCard);
      }

      // Handle interrupts for each can controllerCFLAGS
      for (i = 0; i < vCard->nrChannels; i++) {
        VCanChanData    *vChd;
        PciCanChanData  *hChd;

        vChd = vCard->chanData[i];
        hChd = vChd->hwChanData;

        if ( pcie_irq & hChd->tx_irq_msk ) {
          pciCanTransmitIsr(vChd);
        }
      }
    }

  return IRQ_RETVAL(handled);
} // pciCanInterrupt


//======================================================================
//  pcicanTransmit
//======================================================================
static int pciCanTransmitMessage (VCanChanData *vChd, CAN_MSG *m)
{
  PciCanChanData     *hChd  = vChd->hwChanData;
  char*               msg   = (char*)m->data;
  signed long         ident = m->id;
  unsigned short int  flags = m->flags;
  unsigned long       irqFlags;
  int                 transId;
  pciefd_packet_t     packet;
  unsigned int        nbytes;

  if (!atomic_add_unless(&hChd->outstanding_tx, 1, MAX_OUTSTANDING_TX)) {
    DEBUGPRINT(1, "Trying to increase outstanding_tx above max\n");
    return VCAN_STAT_NO_RESOURCES;
  }

  transId = atomic_read(&vChd->transId);

  if (transId == 0) {
    DEBUGPRINT(1,"--- transId must not be zero\n");
    return VCAN_STAT_NO_RESOURCES;
  }

  if (hChd->current_tx_message[transId - 1].user_data) {
    DEBUGPRINT(1, "Transid is already in use: %x %d   %x %d CH:%u\n\n",
               hChd->current_tx_message[transId - 1].id,
               transId,
               m->id,
               atomic_read(&hChd->outstanding_tx),
               vChd->channel);
    return VCAN_STAT_NO_RESOURCES;
  }

  hChd->current_tx_message[transId - 1] = *m;

  if (flags & VCAN_MSG_FLAG_ERROR_FRAME) { // Error frame
    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    DEBUGPRINT(4,"CH:%u Generate Error Frame %u\n",vChd->channel,transId);

    // Write an error frame control word
    IOWR_PCIEFD_CONTROL(hChd->canControllerBase, PCIEFD_CONTROL_ERROR_FRAME | transId);
  }
  else {
    if (flags & VCAN_MSG_FLAG_FDF) { // CAN FD Format (Extended Data Length)

        DEBUGPRINT(4,"Send CAN FD Frame\n");

        nbytes = setupFDBaseFormat(&packet, ident & ~VCAN_EXT_MSG_ID, m->length & 0x0f, transId);

        if (flags & VCAN_MSG_FLAG_BRS) {
          setBitRateSwitch(&packet);
        }
    }
    else {
      nbytes = setupBaseFormat(&packet, ident & ~VCAN_EXT_MSG_ID, m->length & 0x0f, transId);
      if (flags & VCAN_MSG_FLAG_REMOTE_FRAME) {
        setRemoteRequest(&packet);
      }
    }

    setAckRequest(&packet);

    if (flags & VCAN_MSG_FLAG_TXRQ) {
      setTxRequest(&packet);
    }

    if (ident & VCAN_EXT_MSG_ID) { /* Extended CAN */
      setExtendedId(&packet);
    }

    memcpy(packet.data, msg, nbytes);

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    if (vChd->errorCount > 0) {
      pciCanResetErrorCounter(vChd);
    }

    // Write packet to transmit buffer
    // This region must be done locked
    writeFIFO(vChd, &packet);

    ++hChd->debug.tx_packet_count;
  }

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  // Update transId after we know QCmd() was succesful!
  if (transId + 1 > MAX_OUTSTANDING_TX) {
    DEBUGPRINT(4, "transId = %d (%p), rewind\n", transId, &vChd->transId);
    atomic_set(&vChd->transId, 1);
  }
  else {
    DEBUGPRINT(4, "transId = %d (%p), add\n", transId, &vChd->transId);
    atomic_add(1, &vChd->transId);
  }

  return VCAN_STAT_OK;
} // pciCanTransmitMessage


//======================================================================
//  Read transmit error counter
//======================================================================
static int pciCanGetTxErr (VCanChanData *vChd)
{
  pciCanRequestChipState(vChd);
  return vChd->chipState.txerr;
}


//======================================================================
//  Read transmit error counter
//======================================================================
static int pciCanGetRxErr (VCanChanData *vChd)
{
  pciCanRequestChipState(vChd);
  return vChd->chipState.rxerr;
}


//======================================================================
//  Read receive queue length in hardware/firmware
//======================================================================
static unsigned long pciCanRxQLen (VCanChanData *vChd)
{
  return queue_length(&vChd->txChanQueue);
}


//======================================================================
//  Read transmit queue length in hardware/firmware
//======================================================================
static unsigned long pciCanTxQLen (VCanChanData *vChd)
{
  PciCanChanData *hChd  = vChd->hwChanData;

  return atomic_read(&hChd->outstanding_tx);
}


//======================================================================
//  Clear send buffer on card
//======================================================================
static int pciCanFlushSendBuffer (VCanChanData *vChan)
{
  PciCanChanData *hChd  = vChan->hwChanData;
  unsigned long   irqFlags;
  unsigned int tmp;
  int loopmax = 100;

  os_if_down_sema(&hChd->busOnMutex);

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  atomic_set(&hChd->outstanding_tx, 0);

  memset(hChd->current_tx_message, 0, sizeof(hChd->current_tx_message));

  DEBUGPRINT(4,"Note: Flush Transmit fifo CH:%u\n",vChan->channel);

  // Clear tx flush done irq flag
  irqClear(hChd->canControllerBase, PCIEFD_IRQ_TFD_MSK);

  // Flush Transmit HW Buffer
  fpgaFlushTransmit(hChd->canControllerBase);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  while (loopmax)
    {
      if(istatCheck(hChd->canControllerBase, PCIEFD_IRQ_TFD_MSK))
        {
          DEBUGPRINT(4,"Tx Flush Done CH:%u\n",vChan->channel);
          break;
        }

      DEBUGPRINT(4,"Tx Flush Busy CH:%u\n",vChan->channel);

      os_if_set_task_uninterruptible();
      os_if_wait_for_event_timeout_simple(1); // Wait 1 ms for transceiver to reach bus on

      if (statAbortRequest(hChd->canControllerBase))
        {
          DEBUGPRINT(3,"Warning: Abort req is asserted CH:%u\n",vChan->channel);
        }

      --loopmax;
    }

  if(!loopmax)
    {
      DEBUGPRINT(1,"Tx Flush Still Busy? Loopmax reached CH:%u\n",vChan->channel);
    }

  tmp = IORD_PCIEFD_ISTAT(hChd->canControllerBase);

  if(!istatCheck(hChd->canControllerBase, PCIEFD_IRQ_TFD_MSK))
    {
      DEBUGPRINT(1,"Tx Flush Still Busy, probably failed initiating CH:%u\n",vChan->channel);
    }

  DEBUGPRINT(4,"Note: Flush Transmit fifo start CH:%u\n",vChan->channel);

  if (statAbortRequest(hChd->canControllerBase))
    {
      DEBUGPRINT(1,"Flush: Abort_Req  CH:%u\n",vChan->channel);
    }
  else if (statFlushRequest(hChd->canControllerBase))
    {
      DEBUGPRINT(1,"Flush: Tx_Flush_Req CH:%u\n",vChan->channel);
    }

  loopmax = 10;
  while(loopmax)
    {
      uint32_t level;

      level = fifoPacketCountTx(hChd->canControllerBase);

      if(level == 0) {
        break;
      }

      os_if_set_task_uninterruptible();
      os_if_wait_for_event_timeout_simple(1); // Wait 1 ms

      --loopmax;
    }

  if(loopmax == 0)
    {
      DEBUGPRINT(1,"Warning: Transmit buffer is not empty. Probably a failed flush operation [FLUSH]\n");
    }


  // Write a flush recovery control word
  IOWR_PCIEFD_CONTROL(hChd->canControllerBase, PCIEFD_CONTROL_END_FLUSH);

  queue_reinit(&vChan->txChanQueue);

  os_if_up_sema(&hChd->busOnMutex);

  return VCAN_STAT_OK;
}


//======================================================================
//  Initialize H/W
//  This is only called before a card is initialized, so no one can
//  interfere with the accesses.
//======================================================================
static int DEVINIT pciCanInitHW (VCanCardData *vCard)
{
  int             chNr;
  PciCanCardData  *hCard = vCard->hwCardData;
  unsigned long   irqFlags;

  // The card must be present!
  if (!vCard->cardPresent) {
    DEBUGPRINT(1, "Error: The card must be present!\n");
    return VCAN_STAT_NO_DEVICE;
  }

  if ( !hCard->pcieBar0Base ) {
    DEBUGPRINT(1, "Error: In address!\n");
    vCard->cardPresent = 0;
    return VCAN_STAT_FAIL;
  }


#if LOOPBACK_VERSION == 4
  {
    int int_ch;

    if (loopback)
      {
        for(int_ch=0;int_ch<LOOPBACK_N_CHANNELS;int_ch++)
          {
            DEBUGPRINT(3,"Loopback: Connect ch:%u to internal bus 0\n",int_ch);
            // Connect all buses to internal bus 0
            IOWR_LOOPBACK_CTRL(hCard->canLoopbackBase, int_ch,
                               LOOPBACK_CTRL_TX_ENABLE(0) | LOOPBACK_CTRL_RX_ENABLE(0));
          }
      }
    else
      {
        for(int_ch=0;int_ch<LOOPBACK_N_CHANNELS_INT;int_ch++)
          {
            DEBUGPRINT(3,"Connect internal ch:%u to internal bus %u (value:%x)\n",int_ch, int_ch,
                       LOOPBACK_CTRL_TX_ENABLE((1<<int_ch)) | LOOPBACK_CTRL_RX_ENABLE((1<<int_ch)));

            // Connect internal signals
            IOWR_LOOPBACK_CTRL(hCard->canLoopbackBase, int_ch,
                               LOOPBACK_CTRL_TX_ENABLE((1<<int_ch)) | LOOPBACK_CTRL_RX_ENABLE((1<<int_ch)));
          }

        for(int_ch=0;int_ch<LOOPBACK_N_CHANNELS_EXT;int_ch++)
          {
            DEBUGPRINT(3,"Connect external ch:%u to internal bus %u (value:%x)\n",int_ch, int_ch,
                       LOOPBACK_CTRL_TX_ENABLE((1<<int_ch)) | LOOPBACK_CTRL_RX_ENABLE((1<<int_ch)));

            // Connect external signals
            IOWR_LOOPBACK_CTRL(hCard->canLoopbackBase, int_ch+LOOPBACK_N_CHANNELS_INT,
                               LOOPBACK_CTRL_TX_ENABLE((1<<int_ch)) | LOOPBACK_CTRL_RX_ENABLE((1<<int_ch)));
          }
      }
  }
#elseif LOOPBACK_VERSION == 2
  // Controls loopback during test
  if (loopback){
    IOWR_CAN_LOOPBACK_CTRL(hCard->canLoopbackBase, 3);
  }else{
    IOWR_CAN_LOOPBACK_CTRL(hCard->canLoopbackBase, CAN_LOOPBACK_CTRL_NO_LOOPBACK);
  }
#endif

  // Controls loopback during test

  // Controls loopback during test

  for (chNr = 0; chNr < vCard->nrChannels; chNr++){
    VCanChanData   *vChd = vCard->chanData[chNr];
    PciCanChanData *hChd = vChd->hwChanData;
    void __iomem *addr = hChd->canControllerBase;
    uint32_t mode;

    if (!addr) {
      return VCAN_STAT_FAIL;
    }

    // Is this lock neccessary? See comment in function header.
    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    // Default 125 kbit/s, 75%
    IOWR_PCIEFD_BTRN(addr, PCIEFD_BTR_SEG2(4-1)
                     | PCIEFD_BTR_SEG1(11-1)
                     | PCIEFD_BTR_SJW(2)
                     | PCIEFD_BTR_BRP(32-1));

    if(hwSupportCanFD(hChd->canControllerBase))
      {
        // CAN FD
        IOWR_PCIEFD_BTRD(addr, PCIEFD_BTR_SEG2(4-1)
                         | PCIEFD_BTR_SEG1(11-1)
                         | PCIEFD_BTR_SJW(2)
                         | PCIEFD_BTR_BRP(32-1));
      }

    // Disable bus load calculation
    IOWR_PCIEFD_BLP(addr, 0);

    mode = PCIEFD_MOD_EWL(96) // Set the EWL value
      | PCIEFD_MOD_CHANNEL(chNr)
      | PCIEFD_MOD_EEN(1)
      | PCIEFD_MOD_EPEN(1) // Error packets are received
      | PCIEFD_MOD_RM_MSK    // Reset the circuit.
      | PCIEFD_MOD_DWH(1);

    IOWR_PCIEFD_MOD(addr, mode);

    // Flush and reset
    fpgaFlushAll(hChd->canControllerBase, 0);

    // Disable receiver interrupt
    irqInit(hChd->canControllerBase);

    ledOff(hChd->canControllerBase);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);
  }

  {
    uint32_t level;

    level = fifoPacketCountRx(hCard->canRxBuffer);
    if(level) {
      int to;

      DEBUGPRINT(3,"RX FIFO Has %u old packets\n", level);

      to = (level < 200) ? level : 200;

      while(to && level)
        {
          pciefd_packet_t packet;
          // Reset FIFO
          IOWR_RXBUF_CMD(hCard->canRxBuffer, RXBUF_CMD_RESET_MSK);

          readFIFO(vCard, &packet);

          level = fifoPacketCountRx(hCard->canRxBuffer);
          --to;
        }
    }
  }

  fifoIrqInit(hCard->canRxBuffer);
  fifoIrqEnableUnderflow(hCard->canRxBuffer);

  // Enable interrupts from card
  // Use Altera PCIe Enpoint Interrupt Handling
  pciCanInterrupts(vCard, 1);

  DEBUGPRINT(3, "pciefd: hw initialized. \n");

  return VCAN_STAT_OK;
}


//======================================================================
//  Find out addresses for one card
//======================================================================
static int DEVINIT readPCIAddresses (struct pci_dev *dev, VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  int i;

  if (pci_request_regions(dev, "Kvaser PCIe CAN")) {
    DEBUGPRINT(1, "request regions failed\n");
    return VCAN_STAT_FAIL;
  }

  if (pci_enable_device(dev)){
    DEBUGPRINT(1, "enable device failed\n");
    pci_release_regions(dev);
    return VCAN_STAT_NO_DEVICE;
  }

  for (i = 0; i < 2; i++) {
    unsigned long bar_start = pci_resource_start(dev, i);
    if (bar_start) {
      unsigned long bar_end = pci_resource_end(dev, i);
      unsigned long bar_flags = pci_resource_flags(dev, i);
      DEBUGPRINT(3, "BAR%d 0x%08lx-0x%08lx flags 0x%08lx\n",
                 i, bar_start, bar_end, bar_flags);
      if (bar_flags & IORESOURCE_IO) DEBUGPRINT(3,"IO resource flag\n");
      if (bar_flags & IORESOURCE_MEM) DEBUGPRINT(3,"Memory resource flag\n");
      if (bar_flags & IORESOURCE_PREFETCH) DEBUGPRINT(3,"Prefetch flag\n");
      if (bar_flags & IORESOURCE_READONLY) DEBUGPRINT(3,"Readonly flag\n");
    }
  }

  hCard->irq          = dev->irq;
  hCard->pcieBar0Base = pci_iomap(dev, 0, 0);

  if ( !hCard->pcieBar0Base ) {
    DEBUGPRINT(1, "pci_iomap failed\n");
    pci_disable_device(dev);
    pci_release_regions(dev);
    return VCAN_STAT_FAIL;
  }

  // Init FPGA bus system base addresses
  hCard->canControllerBase         = OFFSET_FROM_BASE(hCard->pcieBar0Base, CAN_CONTROLLER_0_BASE);

  hCard->timestampBase             = OFFSET_FROM_BASE(hCard->pcieBar0Base, TIMESTAMP_BASE);
  hCard->serialFlashBase           = OFFSET_FROM_BASE(hCard->pcieBar0Base, SERIALFLASH_BASE);
  hCard->canLoopbackBase           = OFFSET_FROM_BASE(hCard->pcieBar0Base, CAN_LOOPBACK_BASE);
  hCard->sysidBase                 = OFFSET_FROM_BASE(hCard->pcieBar0Base, SYSID_BASE);
#if USE_ADJUSTABLE_PLL
  hCard->pllBase                   = OFFSET_FROM_BASE(hCard->pcieBar0Base, PLL_BASE);
#endif
  hCard->pcie                      = OFFSET_FROM_BASE(hCard->pcieBar0Base, PCIE_HARD_IP_BASE);

  hCard->canRxBuffer               = OFFSET_FROM_BASE(hCard->pcieBar0Base, RECEIVE_BUFFER_BASE);
  hCard->busAnalyzerBase           = OFFSET_FROM_BASE(hCard->pcieBar0Base, BUS_ANALYZER_BASE);

  hCard->patternGeneratorBase      = OFFSET_FROM_BASE(hCard->pcieBar0Base, PATTERN_GEN_BASE);
  hCard->statusStreamSwitchBase    = OFFSET_FROM_BASE(hCard->pcieBar0Base, STATUS_SWITCH_BASE);

  DEBUGPRINT(2, "irq:%d pcieBar0Base:%lx\n", hCard->irq, (long)hCard->pcieBar0Base);

  return VCAN_STAT_OK;
}


//======================================================================
// Request send
//======================================================================
void pciCanRequestSend (VCanCardData *vCard, VCanChanData *vChan)
{
  PciCanChanData *hChan = vChan->hwChanData;

  if (pciCanTxAvailable(vChan)){
# if !defined(TRY_RT_QUEUE)
    os_if_queue_task(&hChan->txTaskQ);
# else
    os_if_queue_task_not_default_queue(hChan->txTaskQ, &hChan->txWork);
# endif
  }
#if DEBUG
  else {
    DEBUGPRINT(4, "SEND FAILED \n");
  }
#endif
}


//======================================================================
//  Process send Q - This function is called from the immediate queue
//  From pcicanII
//======================================================================
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void pciCanSend (void *void_chanData)
#else
  static void pciCanSend (OS_IF_TASK_QUEUE_HANDLE *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
  VCanChanData *chd = (VCanChanData *)void_chanData;
#else
# if !defined(TRY_RT_QUEUE)
  PciCanChanData *devChan = container_of(work, PciCanChanData, txTaskQ);
# else
  PciCanChanData *devChan = container_of(work, PciCanChanData, txWork);
# endif
  VCanChanData   *chd = devChan->vChan;
#endif
  int queuePos;

  os_if_down_sema(&devChan->busOnMutex);

  if (!chd->isOnBus) {
    DEBUGPRINT(4, "Attempt to send when not on bus\n");
    os_if_up_sema(&devChan->busOnMutex);

    return;
  }

  if (chd->minorNr < 0) {  // Channel not initialized?
    DEBUGPRINT(1, "Attempt to send on unitialized channel\n");
    os_if_up_sema(&devChan->busOnMutex);
    return;
  }

  if (!pciCanTxAvailable(chd)) {
    DEBUGPRINT(4, "Maximum number of messages outstanding reached\n");
    os_if_up_sema(&devChan->busOnMutex);
    return;
  }

  // Send Messages
  queuePos = queue_front(&chd->txChanQueue);
  if (queuePos >= 0) {
    if (pciCanTransmitMessage(chd, &chd->txChanBuffer[queuePos]) ==
        VCAN_STAT_OK) {
      queue_pop(&chd->txChanQueue);
      queue_wakeup_on_space(&chd->txChanQueue);

    } else {
      DEBUGPRINT(1, "Message send failed\n");
      queue_release(&chd->txChanQueue);

      // Need to retry work!
#if !defined(TRY_RT_QUEUE)
      os_if_queue_task(&devChan->txTaskQ);
#else
      os_if_queue_task_not_default_queue(devChan->txTaskQ, &devChan->txWork);
#endif
    }
  } else {
    queue_release(&chd->txChanQueue);
  }

  os_if_up_sema(&devChan->busOnMutex);

  return;
}

//======================================================================
//  Initialize H/W specific data
//======================================================================
static void DEVINIT pciCanInitData (VCanCardData *vCard)
{
  int chNr;
  PciCanCardData *hCard = vCard->hwCardData;

  vCard->driverData = &driverData;
  vCanInitData(vCard);

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    VCanChanData *vChd = vCard->chanData[chNr];
    queue_irq_lock(&vChd->txChanQueue);
  }

  INIT_LIST_HEAD(&hCard->replyWaitList);

  os_if_rwlock_init(&hCard->replyWaitListLock);
  os_if_spin_lock_init(&hCard->lock);

  hCard->last_ts = 0;
  atomic_set(&hCard->status_seq_no, 0);

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    VCanChanData *vChd   = vCard->chanData[chNr];
    PciCanChanData *hChd = vCard->chanData[chNr]->hwChanData;
#if !defined(TRY_RT_QUEUE)
    os_if_spin_lock_init(&hChd->lock);
    hChd->vChan = vChd;
    os_if_init_task(&hChd->txTaskQ, pciCanSend, vChd);
#else
    char name[] = "pcican_txX";
    name[9]     = '0' + chNr;   // Replace the X with channel number
    os_if_spin_lock_init(&hChd->lock);
    hChd->vChan = vChd;
    os_if_init_task(&hChd->txWork, pciCanSend, vChd);
    // Note that this will not create an RT task if the kernel
    // does not actually support it (only 2.6.28+ do).
    // In that case, you must (for now) do it manually using chrt.
    hChd->txTaskQ = os_if_declare_rt_task(name, &hChd->txWork);
#endif

    memset(hChd->current_tx_message, 0, sizeof(hChd->current_tx_message));
    atomic_set(&hChd->outstanding_tx, 0);
    atomic_set(&vChd->transId, 1);

    vChd->overrun    = 0;
    vChd->errorCount = 0;
    hChd->overrun_occured = 0;

    hChd->auto_sso = 1;
    hChd->sso = 0;

    os_if_init_sema(&hChd->busOnMutex);
    os_if_up_sema(&hChd->busOnMutex);

    os_if_init_waitqueue_head(&hChd->hwFlushWaitQ);
  }
}


//======================================================================
// Initialize the HW for one card
//======================================================================
static int DEVINIT pciCanInitOne (struct pci_dev *dev, const struct pci_device_id *id)
{
  // Helper struct for allocation
  typedef struct {
    VCanChanData *dataPtrArray[MAX_CHANNELS];
    VCanChanData vChd[MAX_CHANNELS];
    PciCanChanData hChd[MAX_CHANNELS];
  } ChanHelperStruct;

  ChanHelperStruct *chs;
  PciCanCardData *hCard;

  int chNr;
  VCanCardData *vCard;

  DEBUGPRINT(3, "pciCanInitOne - max channels:%u\n", MAX_CHANNELS);

  // Allocate data area for this card
  vCard = os_if_kernel_malloc(sizeof(VCanCardData) + sizeof(PciCanCardData));
  if (!vCard) {
    goto card_alloc_err;
  }
  memset(vCard, 0, sizeof(VCanCardData) + sizeof(PciCanCardData));

  // hwCardData is directly after VCanCardData
  vCard->hwCardData = vCard + 1;
  hCard = vCard->hwCardData;

  hCard->dev = dev; // Needed for DMA mapping

  // Allocate memory for n channels
  chs = os_if_kernel_malloc(sizeof(ChanHelperStruct));
  if (!chs) {
    goto chan_alloc_err;
  }
  memset(chs, 0, sizeof(ChanHelperStruct));

  // Init array and hwChanData
  for (chNr = 0; chNr < MAX_CHANNELS; chNr++){
    chs->dataPtrArray[chNr]    = &chs->vChd[chNr];
    chs->vChd[chNr].hwChanData = &chs->hChd[chNr];
    chs->vChd[chNr].minorNr    = -1;   // No preset minor number
    chs->vChd[chNr].currentTxMsg = NULL;
  }
  vCard->chanData = chs->dataPtrArray;

  // Get PCI controller and FPGA bus system addresses
  if (readPCIAddresses(dev, vCard) != VCAN_STAT_OK) {
    DEBUGPRINT(1, "readPCIAddresses failed\n");
    goto pci_err;
  }

  // Find out type of card i.e. N/O channels etc
  if (pciCanProbe(vCard) != VCAN_STAT_OK) {
    DEBUGPRINT(1, "pciCanProbe failed\n");
    goto probe_err;
  }

  // Init channels
  pciCanInitData(vCard);

  pci_set_drvdata(dev, vCard);

  if (request_irq(hCard->irq, pciCanInterrupt,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
                  SA_SHIRQ,
#else
                  IRQF_SHARED,
#endif
                  "Kvaser PCIe CAN", vCard)) {
    DEBUGPRINT(1, "request_irq failed\n");
    goto irq_err;
  }

#if USE_DMA
  // Setup DMA
  if(hCard->useDma && dmaInit(vCard))
    {
      DEBUGPRINT(3, "dmaInit failed\n");
      hCard->useDma = 0;
    }
#endif

  // Init h/w & enable interrupts in PCI Interface
  if (pciCanInitHW(vCard) != VCAN_STAT_OK) {
    DEBUGPRINT(1, "pciCanInitHW failed\n");
    goto inithw_err;
  }

  // Activate all channels
  for (chNr = 0; chNr < vCard->nrChannels; chNr++){
    VCanChanData *vChd = vCard->chanData[chNr];
    pciCanActivateTransceiver(vChd, vChd->lineMode);
  }

#if USE_DMA
  if( hCard->useDma && setupDmaMappings(vCard) )
    {
      DEBUGPRINT(3,"setupDmaMappings failed\n");

      hCard->useDma = 0;
    }

  if(hCard->useDma)
    {
      startDMA(vCard);
    }

  if(!hCard->useDma)
#endif
  {
    fifoIrqEnableReceivedDataAvailable(hCard->canRxBuffer);
    fifoIrqEnableMissingTag(hCard->canRxBuffer);
  }

  os_if_set_task_uninterruptible();
  os_if_wait_for_event_timeout_simple(5); // Wait 5 ms for transceiver to power up

  // Insert into list of cards
  os_if_spin_lock(&driverData.canCardsLock);
  vCard->next = driverData.canCards;
  driverData.canCards = vCard;
  os_if_spin_unlock(&driverData.canCardsLock);


#if USE_DMA
  if(hCard->useDma)
    pci_set_master(dev);
#endif

  return VCAN_STAT_OK;

 inithw_err:
  DEBUGPRINT(1,"inithw_err\n");
  free_irq(hCard->irq, vCard);
 irq_err:
  DEBUGPRINT(1,"irq_err\n");
  pci_set_drvdata(dev, NULL);
  DEBUGPRINT(3,"before loop\n");
  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    PciCanChanData *hChd = vCard->chanData[chNr]->hwChanData;
#if defined(TRY_RT_QUEUE)
    os_if_destroy_task(hChd->txTaskQ);
#endif
    os_if_spin_lock_remove(&hChd->lock);
    os_if_spin_lock_remove(&vCard->chanData[chNr]->openLock);
    os_if_delete_sema(&hChd->busOnMutex);
  }
  DEBUGPRINT(3,"after loop\n");

  os_if_rwlock_remove(&hCard->replyWaitListLock);
  os_if_spin_lock_remove(&hCard->lock);

 probe_err:
  DEBUGPRINT(1,"probe_err\n");
  pci_iounmap(dev, hCard->pcieBar0Base);

  DEBUGPRINT(1,"disble dev\n");
  pci_disable_device(dev);
  DEBUGPRINT(1,"release regions\n");
  pci_release_regions(dev);
 pci_err:
  DEBUGPRINT(1,"free chandata\n");
  os_if_kernel_free(vCard->chanData);
 chan_alloc_err:
  DEBUGPRINT(1,"free card\n");
  os_if_kernel_free(vCard);
 card_alloc_err:
  DEBUGPRINT(1,"done\n");

  return VCAN_STAT_FAIL;
} // pciCanInitOne


//======================================================================
// Shut down the HW for one card
//======================================================================
static void DEVEXIT pciCanRemoveOne (struct pci_dev *dev)
{
  VCanCardData *vCard, *lastCard;
  PciCanCardData *hCard;
  int chNr;

  DEBUGPRINT(3, "pciCanRemoveOne (0/8)\n");

  vCard = pci_get_drvdata(dev);

  DEBUGPRINT(3, "pciCanRemoveOne (1/8)\n");

  hCard   = vCard->hwCardData;

  pciCanInterrupts(vCard,0);

#if USE_DMA
  if(hCard->useDma) {
    stopDMA(vCard);

    pci_clear_master(dev);

    removeDmaMappings(vCard);
  }
#endif

  pci_set_drvdata(dev, NULL);

  free_irq(hCard->irq, vCard);

  DEBUGPRINT(3, "pciCanRemoveOne (2/8)\n");

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    VCanChanData *vChan = vCard->chanData[chNr];
    PciCanChanData *hChd = vChan->hwChanData;

    DEBUGPRINT(3, "Waiting for all closed on minor %d\n", vChan->minorNr);
    while (atomic_read(&vChan->fileOpenCount) > 0) {
      os_if_set_task_uninterruptible();
      os_if_wait_for_event_timeout_simple(10);
    }

    // Stop pwm
    pwmSetDutyCycle(hChd->canControllerBase, 0);
  }

  DEBUGPRINT(3, "pciCanRemoveOne - wait done (3/8)\n");


  DEBUGPRINT(3, "pciCanRemoveOne (4/8)\n");

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    PciCanChanData *hChd = vCard->chanData[chNr]->hwChanData;
#if defined(TRY_RT_QUEUE)
    os_if_destroy_task(hChd->txTaskQ);
#endif
    os_if_spin_lock_remove(&hChd->lock);
    os_if_delete_sema(&hChd->busOnMutex);

    os_if_spin_lock_remove(&vCard->chanData[chNr]->openLock);

  }

  DEBUGPRINT(3, "pciCanRemoveOne (5/8)\n");

  os_if_rwlock_remove(&hCard->replyWaitListLock);
  os_if_spin_lock_remove(&hCard->lock);

  pci_iounmap(dev, hCard->pcieBar0Base);

  pci_disable_device(dev);
  pci_release_regions(dev);

  DEBUGPRINT(3, "pciCanRemoveOne (6/8)\n");

  // Remove from canCards list
  os_if_spin_lock(&driverData.canCardsLock);
  lastCard = driverData.canCards;
  if (lastCard == vCard) {
    driverData.canCards = vCard->next;
  } else {
    while (lastCard && (lastCard->next != vCard)) {
      lastCard = lastCard->next;
    }
    if (!lastCard) {
      DEBUGPRINT(1, "Card not in list!\n");
    } else {
      lastCard->next = vCard->next;
    }
  }
  os_if_spin_unlock(&driverData.canCardsLock);


  DEBUGPRINT(3, "pciCanRemoveOne (7/8)\n");

  os_if_kernel_free(vCard->chanData);
  os_if_kernel_free(vCard);

  DEBUGPRINT(3, "pciCanRemoveOne (8/8)\n");
}


//======================================================================
// Find and initialize all cards
//======================================================================
static int INIT pciCanInitAllDevices (void)
{
  int found;

  driverData.deviceName = DEVICE_NAME_STRING;

  found = pci_register_driver(&pcican_tbl);
  DEBUGPRINT(3, "pciCanInitAllDevices %d\n", found);

  // We need to find at least one
  return (found < 0) ? found : 0;
} // pciCanInitAllDevices


//======================================================================
// Shut down and free resources before unloading driver
//======================================================================
static int EXIT pciCanCloseAllDevices (void)
{
  DEBUGPRINT(3, "pciCanCloseAllDevices\n");
  pci_unregister_driver(&pcican_tbl);

  return 0;
} // pciCanCloseAllDevices



INIT int init_module (void)
{
  driverData.hwIf = &hwIf;
  return vCanInit (&driverData, MAX_CHANNELS);
}

EXIT void cleanup_module (void)
{
  vCanCleanup (&driverData);
}
