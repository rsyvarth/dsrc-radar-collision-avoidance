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

//
// Linux USBcanII driver
//

#  include <linux/version.h>
#  include <linux/usb.h>
#  include <linux/types.h>
#  include <linux/seq_file.h>

// Non system headers
#include "osif_kernel.h"
#include "osif_functions_kernel.h"
#include "VCanOsIf.h"
#include "usbcanHWIf.h"
#include "helios_cmds.h"
#include "queue.h"
#include "debug.h"
#include "hwnames.h"
#include "vcan_ioctl.h"

// Get a minor range for your devices from the usb maintainer
// Use a unique set for each driver
#define USB_USBCAN_MINOR_BASE   96


    MODULE_LICENSE("Dual BSD/GPL");
    MODULE_AUTHOR("KVASER");
    MODULE_DESCRIPTION("USBcanII CAN module.");


//----------------------------------------------------------------------------
// If you do not define USBCAN_DEBUG at all, all the debug code will be
// left out.  If you compile with USBCAN_DEBUG=0, the debug code will
// be present but disabled -- but it can then be enabled for specific
// modules at load time with a 'pc_debug=#' option to insmod.
//
#   ifdef USBCAN_DEBUG
        static int pc_debug = USBCAN_DEBUG;
        MODULE_PARM_DESC(pc_debug, "USBCanII debug level");
        module_param(pc_debug, int, 0644);
#       define DEBUGPRINT(n, arg)     if (pc_debug >= (n)) { DEBUGOUT(n, arg); }
#   else
#       define DEBUGPRINT(n, arg)     if ((n) == 1) { DEBUGOUT(n, arg); }
#   endif
//----------------------------------------------------------------------------



//======================================================================
// HW function pointers
//======================================================================

static int INIT usbcan_init_driver(void);
static int usbcan_set_busparams(VCanChanData *vChd, VCanBusParams *par);
static int usbcan_get_busparams(VCanChanData *vChd, VCanBusParams *par);
static int usbcan_set_silent(VCanChanData *vChd, int silent);
static int usbcan_set_trans_type(VCanChanData *vChd, int linemode, int resnet);
static int usbcan_bus_on(VCanChanData *vChd);
static int usbcan_bus_off(VCanChanData *vChd);
static int usbcan_get_tx_err(VCanChanData *vChd);
static int usbcan_get_rx_err(VCanChanData *vChd);
static int usbcan_outstanding_sync(VCanChanData *vChan);
static int EXIT usbcan_close_all(void);
static int usbcan_proc_read(struct seq_file* m, void* v);
static int usbcan_get_chipstate(VCanChanData *vChd);
static int usbcan_get_time(VCanCardData *vCard, unsigned long *time);
static int usbcan_flush_tx_buffer(VCanChanData *vChan);
static void usbcan_schedule_send(VCanCardData *vCard, VCanChanData *vChan);
static unsigned long usbcan_get_hw_rx_q_len(VCanChanData *vChan);
static unsigned long usbcan_get_hw_tx_q_len(VCanChanData *vChan);
static int usbcan_objbuf_exists(VCanChanData *chd, int bufType, int bufNo);
static int usbcan_objbuf_free(VCanChanData *chd, int bufType, int bufNo);
static int usbcan_objbuf_alloc(VCanChanData *chd, int bufType, int *bufNo);
static int usbcan_objbuf_write(VCanChanData *chd, int bufType, int bufNo,
                               int id, int flags, int dlc, unsigned char *data);
static int usbcan_objbuf_enable(VCanChanData *chd, int bufType, int bufNo,
                                int enable);
static int usbcan_objbuf_set_filter(VCanChanData *chd, int bufType, int bufNo,
                                    int code, int mask);
static int usbcan_objbuf_set_flags(VCanChanData *chd, int bufType, int bufNo,
                                   int flags);
static int usbcan_objbuf_set_period(VCanChanData *chd, int bufType, int bufNo,
                                    int period);

static VCanDriverData driverData;

static VCanHWInterface hwIf = {
  .initAllDevices    = usbcan_init_driver,
  .setBusParams      = usbcan_set_busparams,
  .getBusParams      = usbcan_get_busparams,
  .setOutputMode     = usbcan_set_silent,
  .setTranceiverMode = usbcan_set_trans_type,
  .busOn             = usbcan_bus_on,
  .busOff            = usbcan_bus_off,
  .txAvailable       = usbcan_outstanding_sync,            // This isn't really a function thats checks if tx is available!
  .procRead          = usbcan_proc_read,
  .closeAllDevices   = usbcan_close_all,
  .getTime           = usbcan_get_time,
  .flushSendBuffer   = usbcan_flush_tx_buffer,
  .getRxErr          = usbcan_get_rx_err,
  .getTxErr          = usbcan_get_tx_err,
  .rxQLen            = usbcan_get_hw_rx_q_len,
  .txQLen            = usbcan_get_hw_tx_q_len,
  .requestChipState  = usbcan_get_chipstate,
  .requestSend       = usbcan_schedule_send,
  .objbufExists      = usbcan_objbuf_exists,
  .objbufFree        = usbcan_objbuf_free,
  .objbufAlloc       = usbcan_objbuf_alloc,
  .objbufWrite       = usbcan_objbuf_write,
  .objbufEnable      = usbcan_objbuf_enable,
  .objbufSetFilter   = usbcan_objbuf_set_filter,
  .objbufSetFlags    = usbcan_objbuf_set_flags,
  .objbufSetPeriod   = usbcan_objbuf_set_period,
};



//======================================================================
// Static declarations


// USB packet size
#define MAX_PACKET_OUT      3072        // To device
#define MAX_PACKET_IN       3072        // From device



//======================================================================
// Prototypes
static int    DEVINIT usbcan_plugin(struct usb_interface *interface,
                                    const struct usb_device_id *id);
static void   DEVEXIT usbcan_remove(struct usb_interface *interface);

// Interrupt handler prototype changed in 2.6.19.
 #if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
static void   usbcan_write_bulk_callback(struct urb *urb, struct pt_regs *regs);
 #else
static void   usbcan_write_bulk_callback(struct urb *urb);
 #endif


static int    usbcan_allocate(VCanCardData **vCard);
static void   usbcan_deallocate(VCanCardData *vCard);

static void   usbcan_start(VCanCardData *vCard);

static int    usbcan_tx_available(VCanChanData *vChan);
static int    usbcan_transmit(VCanCardData *vCard);
static int    usbcan_send_and_wait_reply(VCanCardData *vCard, heliosCmd *cmd,
                                         heliosCmd *replyPtr,
                                         unsigned char cmdNr,
                                         unsigned char transId);
static int    usbcan_queue_cmd(VCanCardData *vCard, heliosCmd *cmd,
                               unsigned int timeout);

static void   usbcan_handle_command(heliosCmd *cmd, VCanCardData *vCard);
static int    usbcan_get_trans_id(heliosCmd *cmd);

static int    usbcan_fill_usb_buffer(VCanCardData *vCard,
                                     unsigned char *buffer, int maxlen);
static void   usbcan_translate_can_msg(VCanChanData *vChan,
                                       heliosCmd *helios_msg, CAN_MSG *can_msg);

static void   usbcan_get_card_info(VCanCardData *vCard);
//----------------------------------------------------------------------



//----------------------------------------------------------------------------
// Supported KVASER hardware
#define KVASER_VENDOR_ID            0x0bfd
#define USB_USBCAN2_PRODUCT_ID      0x0004
#define USB_USBCAN_REVB_PRODUCT_ID  0x0002
#define USB_MEMORATOR_PRODUCT_ID    0x0005
#define USB_VCI2_PRODUCT_ID         0x0003

// Table of devices that work with this driver
static struct usb_device_id usbcan_table[] = {
  { USB_DEVICE(KVASER_VENDOR_ID, USB_USBCAN2_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_USBCAN_REVB_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_MEMORATOR_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_VCI2_PRODUCT_ID) },
  { }  // Terminating entry
};

MODULE_DEVICE_TABLE(usb, usbcan_table);

//
// USB class driver info in order to get a minor number from the usb core,
// and to have the device registered with devfs and the driver core
//
/*
static struct usb_class_driver usbcan_class = {
  // There will be a special file in /dev/usb called the below
  .name =         "usb/usbcanII%d",
  .fops =         &fops,
  // .mode removed somewhere between 2.6.8 and 2.6.15
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15))
  .mode =         S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif
  .minor_base =   USB_USBCAN_MINOR_BASE,
};
*/

// USB specific object needed to register this driver with the usb subsystem
static struct usb_driver usbcan_driver = {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15))
  .owner      =    THIS_MODULE,
#endif
  .name       =    "usbcanII",
  .probe      =    usbcan_plugin,
  .disconnect =    usbcan_remove,
  .id_table   =    usbcan_table,
};

//============================================================================





//------------------------------------------------------
//
//    ---- CALLBACKS ----
//
//------------------------------------------------------

//============================================================================
//  usbcan_write_bulk_callback
//
// Interrupt handler prototype changed in 2.6.19.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
static void usbcan_write_bulk_callback (struct urb *urb, struct pt_regs *regs)
#else
static void usbcan_write_bulk_callback (struct urb *urb)
#endif
{
  VCanCardData *vCard = (VCanCardData *)urb->context;
  UsbcanCardData *dev = (UsbcanCardData *)vCard->hwCardData;

  // sync/async unlink faults aren't errors
  if (urb->status && !(urb->status == -ENOENT ||
                       urb->status == -ECONNRESET ||
                       urb->status == -ESHUTDOWN)) {
    DEBUGPRINT(2, (TXT("%s - nonzero write bulk status received: %d\n"),
                   __FUNCTION__, urb->status));
  }

  // Notify anyone waiting that the write has finished
  os_if_up_sema(&dev->write_finished);
}




//----------------------------------------------------------------------------
//
//    ---- THREADS ----
//
//----------------------------------------------------------------------------


//============================================================================
//
// usbcan_rx_thread
//
static int usbcan_rx_thread (void *context)
{
  VCanCardData *vCard   = (VCanCardData *)context;
  UsbcanCardData *dev   = (UsbcanCardData *)vCard->hwCardData;
  int          result  = 0;
  int          usbErrorCounter;
  int          ret;
  int          len;

  if (!try_module_get(THIS_MODULE)) {
    return -ENODEV;
  }

  DEBUGPRINT(3, (TXT("rx thread started\n")));

  dev->read_urb = 0;

  usbErrorCounter = 0;

  while (dev->present) {

    // Verify that the device wasn't unplugged
    if (!dev->present){
      DEBUGPRINT(3, (TXT("rx thread Ended - device removed\n")));
      result = -ENODEV;
      break;
    }

    len = 0;
    // Do a blocking bulk read to get data from the device
    ret = usb_bulk_msg(dev->udev,
                       usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
                       dev->bulk_in_buffer, dev->bulk_in_size, &len,
// Timeout changed from jiffies to milliseconds in 2.6.12.
# if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 12))
                       HZ * 30);
# else
                       30000);
# endif

    if (ret) {
      if (ret != -ETIMEDOUT) {
        // Save if runaway
        if (ret == -EILSEQ || ret == -ESHUTDOWN || ret == -EINVAL) {
          DEBUGPRINT(2, (TXT ("usb_bulk_msg error (%d) - Device probably ")
                         TXT2("removed, closing down\n"), ret));
          dev->present = 0;
          result = -ENODEV;
          break;
        }

#if DEBUG
        if (usbErrorCounter++ % 10 == 0)
          DEBUGPRINT(2, (TXT("usb_bulk_msg error (%d) %dth time\n"),
                         ret, usbErrorCounter));
#endif
        if (usbErrorCounter > 100) {
          DEBUGPRINT(2, (TXT("rx thread Ended - error (%d)\n"), ret));

          // Since this has failed so many times, stop transfers to device
          dev->present = 0;
          result = -ENODEV;
          break;
        }
      }
    }
    else {
      //
      // We got a bunch of bytes. Now interpret them.
      //
      unsigned char  *buffer     = (unsigned char *)dev->bulk_in_buffer;
      heliosCmd      *cmd;
      int            loopCounter = 1000;
      unsigned int   count       = 0;


      while (count < len) {
        // A loop counter as a safety measure.
        if (--loopCounter == 0) {
          DEBUGPRINT(2, (TXT("ERROR usbcan_rx_thread() LOOPMAX. \n")));
          break;
        }

        // A command will never straddle a bulk_in_MaxPacketSize byte boundary.
        // The firmware will place a zero in the buffer to indicate that
        // the next command will follow after the next
        // bulk_in_MaxPacketSize bytes boundary.

        cmd = (heliosCmd *)&buffer[count];
        if (cmd->head.cmdLen == 0) {
          count += dev->bulk_in_MaxPacketSize;
          count &= -(dev->bulk_in_MaxPacketSize);
          continue;
        }
        else {
          count += cmd->head.cmdLen;
        }

        usbcan_handle_command(cmd, vCard);
        usbErrorCounter = 0;
      }
    }
  } // while (dev->present)



  DEBUGPRINT(3, (TXT("rx thread Ended - finalised\n")));

  os_if_exit_thread(THIS_MODULE, result);

  return result;
} // _rx_thread


//======================================================================
// Returns whether the transmit queue on a specific channel is empty
// (This is really the same as in VCanOSIf.c, but it may not be
//  intended that this file should call there.)
//======================================================================

static int txQEmpty (VCanChanData *chd)
{
  return queue_empty(&chd->txChanQueue);
}


//============================================================================
//
// usbcan_handle_command
// Handle a received heliosCmd.
//
static void usbcan_handle_command (heliosCmd *cmd, VCanCardData *vCard)
{
  UsbcanCardData   *dev = (UsbcanCardData *)vCard->hwCardData;
  struct list_head *currHead;
  struct list_head *tmpHead;
  WaitNode         *currNode;
  VCAN_EVENT       e;
  unsigned long    irqFlags;

  switch (cmd->head.cmdNo) {

    case CMD_RX_STD_MESSAGE:
    case CMD_RX_EXT_MESSAGE:
    {
      char          dlc;
      unsigned char flags;
      unsigned int  chan = cmd->rxCanMessage.channel;

      DEBUGPRINT(4, (TXT("CMD_RX_XXX_MESSAGE\n")));

      if (chan < (unsigned)vCard->nrChannels) {
        VCanChanData *vChan = vCard->chanData[cmd->rxCanMessage.channel];

        e.tag               = V_RECEIVE_MSG;

        e.transId           = 0;
        os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
        e.timeStamp         = (cmd->rxCanMessage.time + vCard->timeHi) /
                              USBCANII_TICKS_PER_10US;
        os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);

        e.tagData.msg.id     = cmd->rxCanMessage.rawMessage[0] & 0x1F;
        e.tagData.msg.id   <<= 6;
        e.tagData.msg.id    += cmd->rxCanMessage.rawMessage[1] & 0x3F;
        if (cmd->head.cmdNo == CMD_RX_EXT_MESSAGE) {
          e.tagData.msg.id <<= 4;
          e.tagData.msg.id  += cmd->rxCanMessage.rawMessage[2] & 0x0F;
          e.tagData.msg.id <<= 8;
          e.tagData.msg.id  += cmd->rxCanMessage.rawMessage[3] & 0xFF;
          e.tagData.msg.id <<= 6;
          e.tagData.msg.id  += cmd->rxCanMessage.rawMessage[4] & 0x3F;
          e.tagData.msg.id  += EXT_MSG;
        }

        flags = cmd->rxCanMessage.flags;
        e.tagData.msg.flags = 0;

        if (flags & MSGFLAG_OVERRUN)
          e.tagData.msg.flags |= VCAN_MSG_FLAG_OVERRUN;
        if (flags & MSGFLAG_REMOTE_FRAME)
          e.tagData.msg.flags |= VCAN_MSG_FLAG_REMOTE_FRAME;
        if (flags & MSGFLAG_ERROR_FRAME)
          e.tagData.msg.flags |= VCAN_MSG_FLAG_ERROR_FRAME;
        if (flags & MSGFLAG_TX)
          e.tagData.msg.flags |= VCAN_MSG_FLAG_TXACK;
        if (flags & MSGFLAG_TXRQ)
          e.tagData.msg.flags |= VCAN_MSG_FLAG_TXRQ;

        dlc = cmd->rxCanMessage.rawMessage[5] & 0x0F;
        e.tagData.msg.dlc = dlc;

        memcpy(e.tagData.msg.data, &cmd->rxCanMessage.rawMessage[6], 8);

        DEBUGPRINT(6, (TXT(" - vCanDispatchEvent id: %d (ch:%d), time %d\n"),
                       e.tagData.msg.id, vChan->channel, e.timeStamp));
        vCanDispatchEvent(vChan, &e);
      }
      break;
    }

    case CMD_GET_BUSPARAMS_RESP:
      // qqq Should this only be implemented in leaf?
      DEBUGPRINT(4, (TXT("CMD_GET_BUSPARAMS_RESP - Ignored\n")));
      break;

    case CMD_CHIP_STATE_EVENT:
    {
      unsigned int chan = cmd->chipStateEvent.channel;
      VCanChanData *vChd = vCard->chanData[chan];

      DEBUGPRINT(4, (TXT("CMD_CHIP_STATE_EVENT\n")));

      if (chan < (unsigned)vCard->nrChannels) {
        vChd->chipState.txerr = cmd->chipStateEvent.txErrorCounter;
        vChd->chipState.rxerr = cmd->chipStateEvent.rxErrorCounter;
        if (cmd->chipStateEvent.txErrorCounter ||
            cmd->chipStateEvent.rxErrorCounter) {
          DEBUGPRINT(6, (TXT("CMD_CHIP_STATE_EVENT, chan %d - "), chan));
          DEBUGPRINT(6, (TXT("txErr = %d/rxErr = %d\n"),
                         cmd->chipStateEvent.txErrorCounter,
                         cmd->chipStateEvent.rxErrorCounter));
        }

        // ".busStatus" is the contents of the CnSTRH register.
        switch (cmd->chipStateEvent.busStatus &
                (M16C_BUS_PASSIVE | M16C_BUS_OFF)) {
          case 0:
            vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
            break;

          case M16C_BUS_PASSIVE:
            vChd->chipState.state = CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            break;

          case M16C_BUS_OFF:
            vChd->chipState.state = CHIPSTAT_BUSOFF;
            break;

          case (M16C_BUS_PASSIVE | M16C_BUS_OFF):
            vChd->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            break;
        }

        // Reset is treated like bus-off
        if (cmd->chipStateEvent.busStatus & M16C_BUS_RESET) {
          vChd->chipState.state = CHIPSTAT_BUSOFF;
          vChd->chipState.txerr = 0;
          vChd->chipState.rxerr = 0;
        }

        //if (hCd->waitForChipState)
        //  wake_up(&hCd->waitResponse);

        e.tag       = V_CHIP_STATE;
        os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
        e.timeStamp = (cmd->chipStateEvent.time + vCard->timeHi) /
                      USBCANII_TICKS_PER_10US;
        os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);
        e.transId   = 0;
        e.tagData.chipState.busStatus      = (unsigned char)vChd->chipState.state;
        e.tagData.chipState.txErrorCounter = (unsigned char)vChd->chipState.txerr;
        e.tagData.chipState.rxErrorCounter = (unsigned char)vChd->chipState.rxerr;

        vCanDispatchEvent(vChd, &e);
      }
      break;
    }

    case CMD_GET_DRIVERMODE_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_DRIVERMODE_RESP - Ignored\n")));
      break;

    case CMD_START_CHIP_RESP:
      DEBUGPRINT(4, (TXT("CMD_START_CHIP_RESP - Ignored\n")));
      break;

    case CMD_STOP_CHIP_RESP:
      DEBUGPRINT(4, (TXT("CMD_STOP_CHIP_RESP - Ignored\n")));
      break;

    case CMD_READ_CLOCK_RESP:
      DEBUGPRINT(4, (TXT("CMD_READ_CLOCK_RESP\n")));
      os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
      vCard->timeHi = cmd->readClockResp.time[1] << 16;
      os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);
      DEBUGPRINT(6, (TXT("C %x\n"), vCard->timeHi));
      break;

    case CMD_CLOCK_OVERFLOW_EVENT:
      DEBUGPRINT(4, (TXT("CMD_CLOCK_OVERFLOW_EVENT\n")));
      os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
      vCard->timeHi = cmd->clockOverflowEvent.currentTime & 0xFFFF0000;
      os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);
      DEBUGPRINT(6, (TXT("O %x\n"), vCard->timeHi));
      break;

    case CMD_GET_CARD_INFO_RESP:
      // qqq Should this only be implemented in leaf?
      DEBUGPRINT(4, (TXT("CMD_GET_CARD_INFO_RESP - Ignored\n")));
      break;

    case CMD_GET_INTERFACE_INFO_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_INTERFACE_INFO_RESP - Ignored\n")));
      break;

    case CMD_GET_SOFTWARE_INFO_RESP:
      // qqq Should this only be implemented in leaf?
      DEBUGPRINT(4, (TXT("CMD_GET_SOFTWARE_INFO_RESP - Ignored\n")));
      break;

    case CMD_GET_BUSLOAD_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_BUSLOAD_RESP - Ignored\n")));
      break;

    case CMD_RESET_STATISTICS:
      DEBUGPRINT(4, (TXT("CMD_RESET_STATISTICS - Ignored\n")));
      break;

    case CMD_ERROR_EVENT:
      DEBUGPRINT(4, (TXT("CMD_ERROR_EVENT - Ignored\n")));
      break;

    case CMD_RESET_ERROR_COUNTER:
      DEBUGPRINT(4, (TXT("CMD_RESET_ERROR_COUNTER - Ignored\n")));
      break;

      // Send a TxRequest back to the application. This is a
      // little-used message that means that the firmware has _started_
      // sending the message (it is submitted to the CAN controller)
    case CMD_TX_REQUEST:
    {
      unsigned int  transId;
      unsigned int  chan   = cmd->txRequest.channel;
      VCanChanData  *vChan = vCard->chanData[cmd->txRequest.channel];

      DEBUGPRINT(4, (TXT("CMD_TX_REQUEST\n")));

      if (chan < (unsigned)vCard->nrChannels) {
        // A TxReq. Take the current tx message, modify it to a
        // receive message and send it back.
        transId = cmd->txRequest.transId;
        if ((transId == 0) || (transId > dev->max_outstanding_tx)) {
          DEBUGPRINT(6, (TXT ("CMD_TX_REQUEST chan %d ")
                         TXT2("ERROR transid too high %d\n"), chan, transId));
          break;
        }

        if (((UsbcanChanData *)vChan->hwChanData)->current_tx_message[transId - 1].flags & VCAN_MSG_FLAG_TXRQ) {
          VCAN_EVENT *e = (VCAN_EVENT *)&((UsbcanChanData *)vChan->hwChanData)->current_tx_message[transId - 1];
          e->tag = V_RECEIVE_MSG;
          os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
          e->timeStamp = (cmd->txRequest.time + vCard->timeHi) /
                         USBCANII_TICKS_PER_10US;
          os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);
          e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXACK;  // qqq TXACK/RQ???
          vCanDispatchEvent(vChan, e);
        }
      }
      //else {
        // qqq Can this happen? what if it does?
      //}
      break;
    }

    case CMD_TX_ACKNOWLEDGE:
    {
      unsigned int    transId;

      VCanChanData    *vChan   = vCard->chanData[cmd->txAck.channel];
      UsbcanChanData  *usbChan = vChan->hwChanData;
      UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

      //DEBUGPRINT(4, (TXT("CMD_TX_ACKNOWLEDGE\n")));

      if (cmd->txAck.channel < (unsigned)vCard->nrChannels) {
        DEBUGPRINT(4, (TXT ("CMD_TX_ACKNOWLEDGE on ch %d ")
                       TXT2("(outstanding tx = %d)\n"),
                       cmd->txAck.channel, usbChan->outstanding_tx));
        transId = cmd->txAck.transId;
        if ((transId == 0) || (transId > dev->max_outstanding_tx)) {
          DEBUGPRINT(2, (TXT("CMD_TX_ACKNOWLEDGE chan %d ERROR transid %d\n"),
                         cmd->txAck.channel, transId));
          break;
        }

        if (usbChan->current_tx_message[transId - 1].flags & VCAN_MSG_FLAG_TXACK) {
          VCAN_EVENT *e = (VCAN_EVENT *)&usbChan->current_tx_message[transId - 1];
          e->tag = V_RECEIVE_MSG;
          os_if_spin_lock_irqsave(&dev->timeHi_lock, &irqFlags);
          e->timeStamp = (cmd->txAck.time + vCard->timeHi) / USBCANII_TICKS_PER_10US;
          os_if_spin_unlock_irqrestore(&dev->timeHi_lock, irqFlags);
          e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXRQ; // qqq TXRQ???

          // qqq Are the lowspeed transceiver NERR things related to leaf only?
          vCanDispatchEvent(vChan, e);
        }

        usbChan->current_tx_message[transId - 1].user_data = 0;

        os_if_spin_lock(&usbChan->outTxLock);
        if (usbChan->outstanding_tx) {
          usbChan->outstanding_tx--;
        }

        // Outstanding is changing from *full* to at least one open slot?
        if (usbChan->outstanding_tx >= (dev->max_outstanding_tx - 1)) {
          os_if_spin_unlock(&usbChan->outTxLock);
          DEBUGPRINT(6, (TXT("Buffer in chan %d not full (%d) anymore\n"),
                         cmd->txAck.channel, usbChan->outstanding_tx));
          os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);
        }

        // Check if we should *wake* canwritesync
        else if ((usbChan->outstanding_tx == 0) && txQEmpty(vChan) &&
                 test_and_clear_bit(0, &vChan->waitEmpty)) {
          os_if_spin_unlock(&usbChan->outTxLock);
          os_if_wake_up_interruptible(&vChan->flushQ);
          DEBUGPRINT(6, (TXT("W%d\n"), cmd->txAck.channel));
        }
        else {
#if DEBUG
          if (usbChan->outstanding_tx < 4)
            DEBUGPRINT(6, (TXT("o%d ql%d we%d s%d\n"),
                           usbChan->outstanding_tx,
                           queue_length(&vChan->txChanQueue),
                           constant_test_bit(0, &vChan->waitEmpty),
                           dev->max_outstanding_tx));
#endif
          os_if_spin_unlock(&usbChan->outTxLock);
        }

        DEBUGPRINT(6, (TXT("X%d\n"), cmd->txAck.channel));
      }
      break;
    }

    case CMD_CAN_ERROR_EVENT:
    {
      int errorCounterChanged;

      // <windows> Known problem: if the error counters of both channels
      // are max then there is no way of knowing which channel got an errorframe
      // </windows>
      VCanChanData *vChd = vCard->chanData[0]; // qqq chan??

      DEBUGPRINT(4, (TXT("CMD_CAN_ERROR_EVENT\n")));

      // It's an error frame if any of our error counters has
      // increased..
      errorCounterChanged  = (cmd->canErrorEvent.txErrorCounterCh0 >
                              vChd->chipState.txerr);
      errorCounterChanged |= (cmd->canErrorEvent.rxErrorCounterCh0 >
                              vChd->chipState.rxerr);
      // It's also an error frame if we have seen a bus error while
      // the other channel hasn't seen any bus errors at all.
      errorCounterChanged |= ( (cmd->canErrorEvent.busStatusCh0 & M16C_BUS_ERROR) &&
                              !(cmd->canErrorEvent.busStatusCh1 & M16C_BUS_ERROR));

      vChd->chipState.txerr = cmd->canErrorEvent.txErrorCounterCh0;
      vChd->chipState.rxerr = cmd->canErrorEvent.rxErrorCounterCh0;


      switch (cmd->canErrorEvent.busStatusCh0 & (M16C_BUS_PASSIVE | M16C_BUS_OFF)) {
        case 0:
          vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
          break;

        case M16C_BUS_PASSIVE:
          vChd->chipState.state = CHIPSTAT_ERROR_PASSIVE |
                                  CHIPSTAT_ERROR_WARNING;
          break;

        case M16C_BUS_OFF:
          vChd->chipState.state = CHIPSTAT_BUSOFF;
          errorCounterChanged = 0;
          break;

        case (M16C_BUS_PASSIVE | M16C_BUS_OFF):
          vChd->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE |
                                  CHIPSTAT_ERROR_WARNING;
          errorCounterChanged = 0;
          break;

        default:
          break;
      }

      // Reset is treated like bus-off
      if (cmd->canErrorEvent.busStatusCh0 & M16C_BUS_RESET) {
        vChd->chipState.state = CHIPSTAT_BUSOFF;
        vChd->chipState.txerr = 0;
        vChd->chipState.rxerr = 0;
        errorCounterChanged = 0;
      }

      e.tag                              = V_CHIP_STATE;
      e.timeStamp                        = (cmd->canErrorEvent.time +
                                            vCard->timeHi) /
                                           USBCANII_TICKS_PER_10US;
      e.transId                          = 0;
      e.tagData.chipState.busStatus      = vChd->chipState.state;
      e.tagData.chipState.txErrorCounter = vChd->chipState.txerr;
      e.tagData.chipState.rxErrorCounter = vChd->chipState.rxerr;
      vCanDispatchEvent(vChd, &e);

      if (errorCounterChanged) {
        e.tag               = V_RECEIVE_MSG;
        e.transId           = 0;
        e.timeStamp         = (cmd->canErrorEvent.time + vCard->timeHi) /
                              USBCANII_TICKS_PER_10US;
        e.tagData.msg.id    = 0;
        e.tagData.msg.flags = VCAN_MSG_FLAG_ERROR_FRAME;
        e.tagData.msg.dlc   = 0;
        vCanDispatchEvent(vChd, &e);
      }

      // Next channel
      if ((unsigned)vCard->nrChannels > 0) {

        VCanChanData *vChd = vCard->chanData[1];

        // It's an error frame if any of our error counters has increased..
        errorCounterChanged  = (cmd->canErrorEvent.txErrorCounterCh1 >
                                vChd->chipState.txerr);
        errorCounterChanged |= (cmd->canErrorEvent.rxErrorCounterCh1 >
                                vChd->chipState.rxerr);

        // It's also an error frame if we have seen a bus error while
        // the other channel hasn't seen any bus errors at all.
        errorCounterChanged |= ( (cmd->canErrorEvent.busStatusCh1 & M16C_BUS_ERROR) &&
                                !(cmd->canErrorEvent.busStatusCh0 & M16C_BUS_ERROR));

        vChd->chipState.txerr = cmd->canErrorEvent.txErrorCounterCh1;
        vChd->chipState.rxerr = cmd->canErrorEvent.rxErrorCounterCh1;

        switch (cmd->canErrorEvent.busStatusCh1 & (M16C_BUS_PASSIVE | M16C_BUS_OFF)) {
          case 0:
            vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
            break;

          case M16C_BUS_PASSIVE:
            vChd->chipState.state = CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            break;

          case M16C_BUS_OFF:
            vChd->chipState.state = CHIPSTAT_BUSOFF;
            errorCounterChanged = 0;
            break;

          case (M16C_BUS_PASSIVE|M16C_BUS_OFF):
            vChd->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            errorCounterChanged = 0;
            break;

          default:
            break;
        }

        // Reset is treated like bus-off
        if (cmd->canErrorEvent.busStatusCh1 & M16C_BUS_RESET) {
          vChd->chipState.state = CHIPSTAT_BUSOFF;
          vChd->chipState.txerr = 0;
          vChd->chipState.rxerr = 0;
          errorCounterChanged = 0;
        }

        // Dispatch can event

        e.tag                              = V_CHIP_STATE;
        e.timeStamp                        = (cmd->canErrorEvent.time +
                                              vCard->timeHi) /
                                             USBCANII_TICKS_PER_10US;
        e.transId                          = 0;
        e.tagData.chipState.busStatus      = vChd->chipState.state;
        e.tagData.chipState.txErrorCounter = vChd->chipState.txerr;
        e.tagData.chipState.rxErrorCounter = vChd->chipState.rxerr;
        vCanDispatchEvent(vChd, &e);

        if (errorCounterChanged) {
          e.tag               = V_RECEIVE_MSG;
          e.transId           = 0;
          e.timeStamp         = (cmd->canErrorEvent.time + vCard->timeHi) /
                                USBCANII_TICKS_PER_10US;
          e.tagData.msg.id    = 0;
          e.tagData.msg.flags = VCAN_MSG_FLAG_ERROR_FRAME;
          e.tagData.msg.dlc   = 0;
          vCanDispatchEvent(vChd, &e);
        }
      }
      break;
    }

    default:
      DEBUGPRINT(2, (TXT("UNKNOWN COMMAND - %d\n"), cmd->head.cmdNo));
  }

  //
  // Wake up those who are waiting for a resp
  //

  os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
  list_for_each_safe(currHead, tmpHead, &dev->replyWaitList)
  {
    currNode = list_entry(currHead, WaitNode, list);

    if (currNode->cmdNr == cmd->head.cmdNo &&
        usbcan_get_trans_id(cmd) == currNode->transId) {
      memcpy(currNode->replyPtr, cmd, cmd->head.cmdLen);
      DEBUGPRINT(4, (TXT ("Match: cN->cmdNr(%d) == cmd->cmdNo(%d) && ")
                     TXT2("_get_trans_id(%d) == cN->transId(%d)\n"),
                     currNode->cmdNr, cmd->head.cmdNo,
                     usbcan_get_trans_id(cmd), currNode->transId));
      os_if_up_sema(&currNode->waitSemaphore);
    }
#if DEBUG
    else {
      DEBUGPRINT(4, (TXT ("cN->cmdNr(%d) == cmd->cmdNo(%d) && ")
                     TXT2("_get_trans_id(%d) == cN->transId(%d)\n"),
                     currNode->cmdNr, cmd->head.cmdNo,
                     usbcan_get_trans_id(cmd), currNode->transId));
    }
#endif
  }
  os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);
} // _handle_command


//============================================================================
// _get_trans_id
//
static int usbcan_get_trans_id (heliosCmd *cmd)
{
  // qqq Why not check if (cmd->head.cmdNo == CMD_GET_BUSPARAMS_REQ) ?
  //     for example: cmdAutoTxBufferReq does not have a transId
  if (cmd->head.cmdNo > CMD_TX_EXT_MESSAGE) {

    // Any of the commands
    return cmd->getBusparamsReq.transId;
  }
  else {
    DEBUGPRINT(2, (TXT("WARNING: won't give a correct transid\n")));
    return 0;
  }
} // _get_trans_id


//============================================================================
// _send
//
# if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
#  define USE_CONTEXT 1
# else
#  define USE_CONTEXT 0
# endif

#if USE_CONTEXT
static void usbcan_send (void *context)
#else
static void usbcan_send (OS_IF_TASK_QUEUE_HANDLE *work)
#endif
{
  unsigned int     i;
#if USE_CONTEXT
  VCanCardData     *vCard     = (VCanCardData *)context;
  UsbcanCardData   *dev       = (UsbcanCardData *)vCard->hwCardData;
#else
  UsbcanCardData   *dev       = container_of(work, UsbcanCardData, txWork);
  VCanCardData     *vCard     = dev->vCard;
#endif
  VCanChanData     *vChan     = NULL;
  int              tx_needed  = 0;

  if (!dev->present) {
    // The device was unplugged before the file was released
    // We cannot deallocate here, it is too early and handled elsewhere
    return;
  }

  // Wait for a previous write to finish up; we don't use a timeout
  // and so a nonresponsive device can delay us indefinitely. qqq
  os_if_down_sema(&dev->write_finished);

  if (!dev->present) {
    // The device was unplugged before the file was released
    // We cannot deallocate here it is to early and handled elsewhere
    os_if_up_sema(&dev->write_finished);
    return;
  }

  // Do we have any cmd to send
  DEBUGPRINT(5, (TXT("cmd queue length: %d)\n"), queue_length(&dev->txCmdQueue)));

  if (!queue_empty(&dev->txCmdQueue)) {
    tx_needed = 1;
  } else {
    // Process the channel queues (send can-messages)
    for (i = 0; i < vCard->nrChannels; i++) {
      int qLen;

      // Alternate between channels
      vChan = vCard->chanData[i];

      if (vChan->minorNr < 0) {  // Channel not initialized?
        continue;
      }

      // Test if queue is empty or Leaf has sent "queue high"qqq?
      // (TX_CHAN_BUF_SIZE is from vcanosif)
      qLen = queue_length(&vChan->txChanQueue);

      DEBUGPRINT(5, (TXT("Transmit queue%d length: %d\n"), i, qLen));
      if (qLen != 0) {
        tx_needed = 1;
        break;
      }
    }
  }

  if (tx_needed) {
    int result;

    if ((result = usbcan_transmit(vCard)) <= 0) {
      // The transmission failed - mark write as finished
      os_if_up_sema(&dev->write_finished);
    }

    // Wake up those who are waiting to send a cmd or msg
    // It seems rather likely that we emptied all our queues, and if not,
    // the awoken threads will go back to sleep again, anyway.
    // A better solution would be to do this inside usbcan_fill_usb_buffer,
    // where it is actually known what queues were touched.
    queue_wakeup_on_space(&dev->txCmdQueue);
    for (i = 0; i < vCard->nrChannels; i++) {
      vChan = vCard->chanData[i];
      if (vChan->minorNr < 0) {  // Channel not initialized?
        continue;
      }
      queue_wakeup_on_space(&vChan->txChanQueue);
    }
    if (result) {
      // Give ourselves a little extra work in case all the queues could not
      // be emptied this time.
      os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);
    }
  }
  else {
    os_if_up_sema(&dev->write_finished);
  }

  return;
} // _send



//============================================================================
//
// _translate_can_msg
// translate from CAN_MSG to heliosCmd
//
static void usbcan_translate_can_msg (VCanChanData *vChan,
                                      heliosCmd *helios_msg,
                                      CAN_MSG *can_msg)
{
  int transId;

  // Save a copy of the message.
  transId = atomic_read(&vChan->transId);
  if (((UsbcanChanData *)vChan->hwChanData)->current_tx_message[transId - 1].user_data) {
    DEBUGPRINT(1, (TXT("In use: %x %d   %x %d\n"), 
                   ((UsbcanChanData *)vChan->hwChanData)->current_tx_message[transId - 1].id, transId,
                   can_msg->id, ((UsbcanChanData *)vChan->hwChanData)->outstanding_tx));
  }
  ((UsbcanChanData *)vChan->hwChanData)->current_tx_message[transId - 1] = *can_msg;

  helios_msg->txCanMessage.cmdLen  = sizeof(cmdTxCanMessage);
  helios_msg->txCanMessage.channel = (unsigned char)vChan->channel;
  helios_msg->txCanMessage.transId = (unsigned char)transId;

  DEBUGPRINT(5, (TXT("can mesg channel:%d transid %d\n"),
                 helios_msg->txCanMessage.channel,
                 helios_msg->txCanMessage.transId));

  if (can_msg->id & VCAN_EXT_MSG_ID) {
    // Extended CAN
    helios_msg->txCanMessage.cmdNo         = CMD_TX_EXT_MESSAGE;
    helios_msg->txCanMessage.rawMessage[0] = (unsigned char)((can_msg->id >> 24) & 0x1F);
    helios_msg->txCanMessage.rawMessage[1] = (unsigned char)((can_msg->id >> 18) & 0x3F);
    helios_msg->txCanMessage.rawMessage[2] = (unsigned char)((can_msg->id >> 14) & 0x0F);
    helios_msg->txCanMessage.rawMessage[3] = (unsigned char)((can_msg->id >>  6) & 0xFF);
    helios_msg->txCanMessage.rawMessage[4] = (unsigned char)((can_msg->id      ) & 0x3F);
  }
  else {
    // Standard CAN
    helios_msg->txCanMessage.cmdNo         = CMD_TX_STD_MESSAGE;
    helios_msg->txCanMessage.rawMessage[0] = (unsigned char)((can_msg->id >>  6) & 0x1F);
    helios_msg->txCanMessage.rawMessage[1] = (unsigned char)((can_msg->id      ) & 0x3F);
  }
  helios_msg->txCanMessage.rawMessage[5]   = can_msg->length & 0x0F;
  memcpy(&helios_msg->txCanMessage.rawMessage[6], can_msg->data, 8);

  //usbChan->outstanding_tx++; // Removed because calling fkt sometimes breaks b4 actually queueing
  DEBUGPRINT(5, (TXT("outstanding(%d)++ id: %d\n"),
                 ((UsbcanChanData *)vChan->hwChanData)->outstanding_tx,
                 can_msg->id));
  DEBUGPRINT(5, (TXT("Trans %d, jif %ld\n"),
                 helios_msg->txCanMessage.transId, jiffies));

  helios_msg->txCanMessage.flags = can_msg->flags & (VCAN_MSG_FLAG_TX_NOTIFY   |
                                                     VCAN_MSG_FLAG_TX_START    |
                                                     VCAN_MSG_FLAG_ERROR_FRAME |
                                                     VCAN_MSG_FLAG_REMOTE_FRAME);
  /* Windows driver has a VCAN_MSG_FLAG_WAKEUP betwen the last two!! */
} // _translate_can_msg



//============================================================================
// Fill the buffer with commands from the sw-command-q (for transfer to USB)
// The firmware requires that no command straddle a
// bulk_out_MaxPacketSize byte boundary.
// This is because the bulk transfer sends bulk_out_MaxPacketSIze bytes per
// stage.
//
static int usbcan_fill_usb_buffer (VCanCardData *vCard, unsigned char *buffer,
                                   int maxlen)
{
  int             cmd_bwp = 0;
  int             msg_bwp = 0;
  unsigned int    j;
  int             more_messages_to_send;
  heliosCmd       command;
  UsbcanCardData  *dev = (UsbcanCardData *)vCard->hwCardData;
  VCanChanData    *vChan;
  int             len;
  int             queuePos;

  // Fill buffer with commands
  while (!queue_empty(&dev->txCmdQueue)) {
    heliosCmd   *commandPtr;
    int         len;

    queuePos = queue_front(&dev->txCmdQueue);
    if (queuePos < 0) {   // Did we actually get anything from queue?
      queue_release(&dev->txCmdQueue);
      break;
    }
    commandPtr = &dev->txCmdBuffer[queuePos];
    len = commandPtr->head.cmdLen;

    DEBUGPRINT(5, (TXT("fill buf with cmd nr %d\n"), commandPtr->head.cmdNo));


    // Any space left in the usb buffer?
    if (len > (maxlen - cmd_bwp)) {
      queue_release(&dev->txCmdQueue);
      break;
    }

    // Will this command straddle a bulk_out_MaxPacketSize bytes boundry?
    if ((cmd_bwp & -(dev->bulk_out_MaxPacketSize)) !=
        ((cmd_bwp + len) & -(dev->bulk_out_MaxPacketSize))) {
      // Yes. write a zero here and move the pointer to the next
      // bulk_out_MaxPacketSize bytes boundry
      buffer[cmd_bwp] = 0;
      cmd_bwp = (cmd_bwp + (dev->bulk_out_MaxPacketSize)) &
                -(dev->bulk_out_MaxPacketSize);
      queue_release(&dev->txCmdQueue);
      continue;
    }

    memcpy(&buffer[cmd_bwp], commandPtr, len);
    cmd_bwp += len;


    queue_pop(&dev->txCmdQueue);
  } // end while

  msg_bwp = cmd_bwp;

  DEBUGPRINT(5, (TXT("bwp: (%d)\n"), msg_bwp));

  // Add the messages

  for (j = 0; j < vCard->nrChannels; j++) {

    UsbcanChanData *usbChan;
    vChan   = (VCanChanData *)vCard->chanData[j];
    usbChan = vChan->hwChanData;

    if (vChan->minorNr < 0) {  // Channel not initialized?
      continue;
    }

    more_messages_to_send = 1;

    while (more_messages_to_send) {
      more_messages_to_send = !queue_empty(&vChan->txChanQueue);

      // Make sure we don't write more messages than
      // we are allowed to the usbcan
      if (!usbcan_tx_available(vChan)) {
        DEBUGPRINT(3, (TXT("Too many outstanding packets\n")));
        return msg_bwp;
      }

      if (more_messages_to_send == 0) {
        break;
      }

      // Get and translate message
      queuePos = queue_front(&vChan->txChanQueue);
      if (queuePos < 0) {   // Did we actually get anything from queue?
        queue_release(&vChan->txChanQueue);
        break;
      }
      usbcan_translate_can_msg(vChan, &command, &vChan->txChanBuffer[queuePos]);

      len = command.head.cmdLen;

      // Any space left in the usb buffer?
      if (len > (maxlen - msg_bwp)) {
        usbChan->current_tx_message[atomic_read(&vChan->transId) - 1].user_data = 0;
        queue_release(&vChan->txChanQueue);
        break;
      }

      // Will this command straddle a bulk_out_MaxPacketSize bytes boundry?
      if ((msg_bwp & -(dev->bulk_out_MaxPacketSize)) !=
          ((msg_bwp + len) & -(dev->bulk_out_MaxPacketSize))) {
        // Yes. write a zero here and move the pointer to the next
        // bulk_out_MaxPacketSize bytes boundry

        buffer[msg_bwp] = 0;
        msg_bwp = (msg_bwp + (dev->bulk_out_MaxPacketSize)) &
                  -(dev->bulk_out_MaxPacketSize);
        usbChan->current_tx_message[atomic_read(&vChan->transId) - 1].user_data = 0;
        queue_release(&vChan->txChanQueue);
        continue;
      }


      memcpy(&buffer[msg_bwp], &command, len);
      msg_bwp += len;
      DEBUGPRINT(5, (TXT("memcpy cmdno %d, len %d (%d)\n"),
                     command.head.cmdNo, len, msg_bwp));
      DEBUGPRINT(5, (TXT("x\n")));

      /// qqq, not really atomic (but that should not matter)
      if ((atomic_read(&vChan->transId) + 1u) > dev->max_outstanding_tx) {
        atomic_set(&vChan->transId, 1);
      }
      else {
        atomic_inc(&vChan->transId);
      }

      // Have to be here (after all the breaks and continues)
      os_if_spin_lock(&usbChan->outTxLock);
      usbChan->outstanding_tx++;
      os_if_spin_unlock(&usbChan->outTxLock);

      DEBUGPRINT(5, (TXT("t usbcan, chan %d, out %d\n"),
                     j, usbChan->outstanding_tx));

      queue_pop(&vChan->txChanQueue);
    } // while (more_messages_to_send)
  }

  return msg_bwp;
} // _fill_usb_buffer



//============================================================================
// The actual sending
//
static int usbcan_transmit (VCanCardData *vCard /*, void *cmd*/)
{
  UsbcanCardData   *dev     = (UsbcanCardData *)vCard->hwCardData;
  int              retval   = 0;
  int              fill     = 0;

  fill = usbcan_fill_usb_buffer(vCard, dev->write_urb->transfer_buffer,
                                MAX_PACKET_OUT);

  if (fill == 0) {
    // No data to send...
    DEBUGPRINT(5, (TXT("Couldn't send any messages\n")));
    return 0;
  }

  dev->write_urb->transfer_buffer_length = fill;

  if (!dev->present) {
    // The device was unplugged before the file was released
    // we cannot deallocate here it shouldn't be done from here
    return VCAN_STAT_NO_DEVICE;
  }

  retval = usb_submit_urb(dev->write_urb, GFP_KERNEL);
  if (retval) {
    DEBUGPRINT(1, (TXT("%s - failed submitting write urb, error %d"),
                   __FUNCTION__, retval));
    retval = -1; // qqq, VCAN_STAT_FAIL ???
  }
  else {
    // The semaphore is released on successful transmission
    retval = sizeof(heliosCmd);
  }

  return retval;
} // _transmit



//============================================================================
// _get_card_info
//
static void DEVINIT usbcan_get_card_info (VCanCardData* vCard)
{
  UsbcanCardData        *dev = (UsbcanCardData *)vCard->hwCardData;
  cmdGetCardInfoReq     card_cmd;
  heliosCmd             reply;
  cmdGetSoftwareInfoReq cmd;
  cmdAutoTxBufferReq    auto_cmd;

  cmd.cmdLen  = sizeof(cmdGetSoftwareInfoReq);
  cmd.cmdNo   = CMD_GET_SOFTWARE_INFO_REQ;
  cmd.transId = CMD_GET_SOFTWARE_INFO_REQ;

  // qqq Should this perhaps do the same as leaf?
  usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
                             CMD_GET_SOFTWARE_INFO_RESP, cmd.transId);
  dev->max_outstanding_tx     =  reply.getSoftwareInfoResp.maxOutstandingTx;
  vCard->firmwareVersionMajor = (unsigned char)(reply.getSoftwareInfoResp.applicationVersion >> 24) & 0xff;
  vCard->firmwareVersionMinor = (unsigned char)(reply.getSoftwareInfoResp.applicationVersion >> 16) & 0xff;
  vCard->firmwareVersionBuild = (unsigned short)(reply.getSoftwareInfoResp.applicationVersion) & 0xffff;

  DEBUGPRINT(2, (TXT("Using fw version: %d.%d.%d\n"), vCard->firmwareVersionMajor,
                 vCard->firmwareVersionMinor, vCard->firmwareVersionBuild));

  if (reply.getSoftwareInfoResp.swOptions & SWOPTION_BETA) {
    DEBUGPRINT(6, (TXT("Beta\n")));
    vCard->card_flags |= DEVHND_CARD_FIRMWARE_BETA;
  }
  if (reply.getSoftwareInfoResp.swOptions & SWOPTION_RC) {
    DEBUGPRINT(6, (TXT("Release Candidate\n")));
    vCard->card_flags |= DEVHND_CARD_FIRMWARE_RC;
  }
  if (reply.getSoftwareInfoResp.swOptions & SWOPTION_AUTO_TX_BUFFER) {
    DEBUGPRINT(6, (TXT("Auto tx buffer\n")));
    vCard->card_flags |= DEVHND_CARD_AUTO_TX_OBJBUFS;
  }

  card_cmd.cmdLen  = sizeof(cmdGetCardInfoReq);
  card_cmd.cmdNo   = CMD_GET_CARD_INFO_REQ;
  card_cmd.transId = CMD_GET_CARD_INFO_REQ;

  usbcan_send_and_wait_reply(vCard, (heliosCmd *)&card_cmd, &reply,
                             CMD_GET_CARD_INFO_RESP, card_cmd.transId);
  DEBUGPRINT(2, (TXT("channels: %d, s/n: %d, hwrev: %u\n"),
                 reply.getCardInfoResp.channelCount,
                 (int)reply.getCardInfoResp.serialNumberLow,
                 (unsigned int)reply.getCardInfoResp.hwRevision));
  vCard->nrChannels   = reply.getCardInfoResp.channelCount;
  vCard->serialNumber = reply.getCardInfoResp.serialNumberLow;
  memcpy(vCard->ean, &reply.getCardInfoResp.EAN[0], 8);
  vCard->hwRevisionMajor = (reply.getCardInfoResp.hwRevision >> 4);
  vCard->hwRevisionMinor = (reply.getCardInfoResp.hwRevision & 0xf);
  // qqq This should be per channel!
  vCard->capabilities = VCAN_CHANNEL_CAP_SEND_ERROR_FRAMES    |
                        VCAN_CHANNEL_CAP_RECEIVE_ERROR_FRAMES |
                        VCAN_CHANNEL_CAP_TIMEBASE_ON_CARD     |
                        VCAN_CHANNEL_CAP_BUSLOAD_CALCULATION  |
                        VCAN_CHANNEL_CAP_ERROR_COUNTERS       |
                        VCAN_CHANNEL_CAP_EXTENDED_CAN         |
                        VCAN_CHANNEL_CAP_TXREQUEST            |
                        VCAN_CHANNEL_CAP_TXACKNOWLEDGE;

  vCard->hw_type      = HWTYPE_DEMETER;

  if (vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
    auto_cmd.cmdLen      = sizeof(cmdAutoTxBufferReq);
    auto_cmd.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
    auto_cmd.requestType = AUTOTXBUFFER_CMD_GET_INFO;
    usbcan_send_and_wait_reply(vCard, (heliosCmd *)&auto_cmd, &reply,
                               CMD_AUTO_TX_BUFFER_RESP, auto_cmd.requestType);
    dev->autoTxBufferCount      = reply.autoTxBufferResp.bufferCount;
    dev->autoTxBufferResolution = reply.autoTxBufferResp.timerResolution;
    DEBUGPRINT(2, (TXT("objbufs supported, count=%d resolution=%d\n"),
                   dev->autoTxBufferCount, dev->autoTxBufferResolution));
  }
} // _get_card_info



//============================================================================
//  response_timeout
//  Used in usbcan_send_and_wait_reply
static void usbcan_response_timer (unsigned long voidWaitNode)
{
  WaitNode *waitNode = (WaitNode *)voidWaitNode;
  waitNode->timedOut = 1;
  os_if_up_sema(&waitNode->waitSemaphore);
} // response_timeout



//============================================================================
//  usbcan_send_and_wait_reply
//  Send a heliosCmd and wait for the usbcan to answer.
//
static int usbcan_send_and_wait_reply (VCanCardData *vCard, heliosCmd *cmd,
                                       heliosCmd *replyPtr,
                                       unsigned char cmdNr, unsigned char transId)
{
  UsbcanCardData     *dev = vCard->hwCardData;
  struct timer_list  waitTimer;
  WaitNode           waitNode;
  int                ret;
  unsigned long      irqFlags;

  // Maybe return something different...
  if (vCard == NULL) {
    return VCAN_STAT_NO_DEVICE;
  }

  // See if dev is present
  if (!dev->present) {
    return VCAN_STAT_NO_DEVICE;
  }

  os_if_init_sema(&waitNode.waitSemaphore);
  waitNode.replyPtr  = replyPtr;
  waitNode.cmdNr     = cmdNr;
  waitNode.transId   = transId;

  waitNode.timedOut  = 0;

  // Add to card's list of expected responses
  os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
  list_add(&waitNode.list, &dev->replyWaitList);
  os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);

  ret = usbcan_queue_cmd(vCard, cmd, USBCAN_Q_CMD_WAIT_TIME);
  if (ret) {
    // qqq Write lock?
    os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
    list_del(&waitNode.list);
    os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);
    return ret;
  }

  DEBUGPRINT(5, (TXT("b4 init timer\n")));
  init_timer(&waitTimer);
  waitTimer.function  = usbcan_response_timer;
  waitTimer.data      = (unsigned long)&waitNode;
  waitTimer.expires   = jiffies + msecs_to_jiffies(USBCAN_CMD_RESP_WAIT_TIME);
  add_timer(&waitTimer);

  os_if_down_sema(&waitNode.waitSemaphore);
  // Now we either got a response or a timeout
  os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
  list_del(&waitNode.list);
  os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);
  del_timer_sync(&waitTimer);

  DEBUGPRINT(5, (TXT("after del timer\n")));
  if (waitNode.timedOut) {
    DEBUGPRINT(2, (TXT("WARNING: waiting for response(%d) timed out! \n"),
                   waitNode.cmdNr));
    return VCAN_STAT_TIMEOUT;
  }

  return VCAN_STAT_OK;
} // _send_and_wait_reply



//============================================================================
//  usbcan_queue_cmd
//  Put the command in the command queue
//
// The unrolled sleep is used to catch a missing position in the queue
// qqq Protect the filling of the buffer with a semaphore
static int usbcan_queue_cmd (VCanCardData *vCard, heliosCmd *cmd,
                             unsigned int timeout)
{
  heliosCmd *bufCmdPtr = NULL;
  UsbcanCardData *dev  = (UsbcanCardData *)vCard->hwCardData;
  int queuePos;
  // Using an unrolled sleep
  OS_IF_WAITQUEUE wait;
  os_if_init_waitqueue_entry(&wait);
  queue_add_wait_for_space(&dev->txCmdQueue, &wait);

  // Sleep when buffer is full and timeout > 0
  while(1) {
    // We are indicated as interruptible
    DEBUGPRINT(5, (TXT("queue cmd len: %d\n"), queue_length(&dev->txCmdQueue)));

    os_if_set_task_interruptible();

    queuePos = queue_back(&dev->txCmdQueue);
    if (queuePos >= 0) {   // Did we actually find space in the queue?
      break;
    }
    queue_release(&dev->txCmdQueue);

    // Do we want a timeout?
    if (timeout == 0) {
      // We shouldn't wait and thus we must be active
      os_if_set_task_running();
      queue_remove_wait_for_space(&dev->txCmdQueue, &wait);
      DEBUGPRINT(2, (TXT("ERROR 1 NO_RESOURCES\n")));
      return VCAN_STAT_NO_RESOURCES;
    } else {
      if (os_if_wait_for_event_timeout(timeout, &wait) == 0) {
        // Sleep was interrupted by timer
        queue_remove_wait_for_space(&dev->txCmdQueue, &wait);
        DEBUGPRINT(2, (TXT("ERROR 2 NO_RESOURCES\n")));
        return VCAN_STAT_NO_RESOURCES;
      }
    }

    // Are we interrupted by a signal?
    if (os_if_signal_pending()) {
      queue_remove_wait_for_space(&dev->txCmdQueue, &wait);
      DEBUGPRINT(2, (TXT("ERROR 3 SIGNALED\n")));
      return VCAN_STAT_SIGNALED;
    }
  }

  os_if_set_task_running();
  queue_remove_wait_for_space(&dev->txCmdQueue, &wait);

  // Get a pointer to the right bufferspace
  bufCmdPtr = (heliosCmd *)&dev->txCmdBuffer[queuePos];
  memcpy(bufCmdPtr, cmd, cmd->head.cmdLen);
  queue_push(&dev->txCmdQueue);

  // Wake up the tx-thread
  os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);

  return VCAN_STAT_OK;
} // _queue_cmd


//============================================================================
//  usbcan_plugin
//
//  Called by the usb core when a new device is connected that it thinks
//  this driver might be interested in.
//  Also allocates card info struct mem space and starts workqueues
//
static int DEVINIT usbcan_plugin (struct usb_interface *interface,
                                  const struct usb_device_id *id)
{
  struct usb_device               *udev = interface_to_usbdev(interface);
  struct usb_host_interface       *iface_desc;
  struct usb_endpoint_descriptor  *endpoint;
  size_t                          buffer_size;
  unsigned int                    i;
  int                             retval = -ENOMEM;
  VCanCardData                    *vCard;
  UsbcanCardData                  *dev;

  DEBUGPRINT(3, (TXT("usbcan: _plugin\n")));

  // See if the device offered us matches what we can accept
  // Add here for more devices
  if ((udev->descriptor.idVendor != KVASER_VENDOR_ID) ||
      (
       (udev->descriptor.idProduct != USB_USBCAN2_PRODUCT_ID)     &&
       (udev->descriptor.idProduct != USB_USBCAN_REVB_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_MEMORATOR_PRODUCT_ID)
      )
     )
  {
    DEBUGPRINT(2, (TXT("==================\n")));
    DEBUGPRINT(2, (TXT("Vendor:  %d\n"),  udev->descriptor.idVendor));
    DEBUGPRINT(2, (TXT("Product:  %d\n"), udev->descriptor.idProduct));
    DEBUGPRINT(2, (TXT("==================\n")));
    return -ENODEV;
  }

#if DEBUG
  DEBUGPRINT(2, (TXT("\nKVASER ")));
  switch (udev->descriptor.idProduct) {
    case USB_USBCAN2_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("USBcanII plugged in\n")));
      break;

    case USB_USBCAN_REVB_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("USBcan rev B plugged in\n")));
      break;

    case USB_MEMORATOR_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Memorator plugged in\n")));
      break;

    default:
      DEBUGPRINT(2, (TXT("UNKNOWN product plugged in\n")));
      break;
  }
#endif

  // Allocate datastructures for the card
  if (usbcan_allocate(&vCard) != VCAN_STAT_OK) {
    // Allocation failed
    return -ENOMEM;
  }

  dev = vCard->hwCardData;
  os_if_init_sema(&((UsbcanCardData *)vCard->hwCardData)->sem);
  dev->udev = udev;
  dev->interface = interface;

  // Set up the endpoint information
  // Check out the endpoints
  // Use only the first bulk-in and bulk-out endpoints
  iface_desc = &interface->altsetting[0];
  for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
    endpoint = &iface_desc->endpoint[i].desc;

    if (!dev->bulk_in_endpointAddr &&
        (endpoint->bEndpointAddress & USB_DIR_IN) &&
        ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
         USB_ENDPOINT_XFER_BULK)) {
      // We found a bulk in endpoint
      buffer_size                = MAX_PACKET_IN;
      dev->bulk_in_size          = buffer_size;
      dev->bulk_in_endpointAddr  = endpoint->bEndpointAddress;
      dev->bulk_in_buffer        = os_if_kernel_malloc(buffer_size);
      dev->bulk_in_MaxPacketSize = le16_to_cpu(endpoint->wMaxPacketSize);
      DEBUGPRINT(2, (TXT("MaxPacketSize in = %d\n"),
                     dev->bulk_in_MaxPacketSize));

      DEBUGPRINT(2, (TXT("MALLOC bulk_in_buffer\n")));
      if (!dev->bulk_in_buffer) {
        DEBUGPRINT(1, (TXT("Couldn't allocate bulk_in_buffer\n")));
        goto error;
      }
      memset(dev->bulk_in_buffer, 0, buffer_size);
    }

    if (!dev->bulk_out_endpointAddr &&
        !(endpoint->bEndpointAddress & USB_DIR_IN) &&
        ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
         USB_ENDPOINT_XFER_BULK)) {
      // We found a bulk out endpoint
      // A probe() may sleep and has no restrictions on memory allocations
      dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
      if (!dev->write_urb) {
        DEBUGPRINT(1, (TXT("No free urbs available\n")));
        goto error;
      }
      dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;

      // On some platforms using this kind of buffer alloc
      // call eliminates a dma "bounce buffer".
      //
      // NOTE: you'd normally want i/o buffers that hold
      // more than one packet, so that i/o delays between
      // packets don't hurt throughput.
      //
      buffer_size                    = MAX_PACKET_OUT;
      dev->bulk_out_size             = buffer_size;
      dev->bulk_out_MaxPacketSize    = le16_to_cpu(endpoint->wMaxPacketSize);
      DEBUGPRINT(2, (TXT("MaxPacketSize out = %d\n"),
                     dev->bulk_out_MaxPacketSize));
      dev->write_urb->transfer_flags = (URB_NO_TRANSFER_DMA_MAP);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35))
      dev->bulk_out_buffer = usb_buffer_alloc(dev->udev,
                                              buffer_size, GFP_KERNEL,
                                              &dev->write_urb->transfer_dma);
#else
      dev->bulk_out_buffer = usb_alloc_coherent(dev->udev,
                                                buffer_size, GFP_KERNEL,
                                                &dev->write_urb->transfer_dma);
#endif
      if (!dev->bulk_out_buffer) {
        DEBUGPRINT(1, (TXT("Couldn't allocate bulk_out_buffer\n")));
        goto error;
      }
      usb_fill_bulk_urb(dev->write_urb, dev->udev,
                        usb_sndbulkpipe(dev->udev,
                                        endpoint->bEndpointAddress),
                        dev->bulk_out_buffer, buffer_size,
                        usbcan_write_bulk_callback, vCard);
    }
  }
  if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
    DEBUGPRINT(1, (TXT("Couldn't find both bulk-in and bulk-out endpoints\n")));
    goto error;
  }


  // Allow device read, write and ioctl
  dev->present = 1;

  // We can register the device now, as it is ready
  usb_set_intfdata(interface, vCard);
  dev->vCard = vCard;

  // Set the number on the channels
  for (i = 0; i < MAX_CHANNELS; i++) {
    VCanChanData   *vChd = vCard->chanData[i];
    vChd->channel  = i;
  }


  // Start up vital stuff
  usbcan_start(vCard);

  // Let the user know what node this device is now attached to
  DEBUGPRINT(2, (TXT("------------------------------\n")));
  DEBUGPRINT(2, (TXT("USBcanII device %d now attached\n"),
                 driverData.noOfDevices));
  for (i = 0; i < MAX_CHANNELS; i++) {
    DEBUGPRINT(2, (TXT("With minor number %d \n"), vCard->chanData[i]->minorNr));
  }
  DEBUGPRINT(2, (TXT("using driver built %s\n"), TXT2(__TIME__)));
  DEBUGPRINT(2, (TXT("on %s\n"), TXT2(__DATE__)));
  DEBUGPRINT(2, (TXT("------------------------------\n")));

  return 0;

error:
  DEBUGPRINT(2, (TXT("_deallocate from usbcan_plugin\n")));
  usbcan_deallocate(vCard);

  return retval;
} // usbcan_plugin



//========================================================================
//
// Init stuff, called from end of _plugin
//
static void DEVINIT usbcan_start (VCanCardData *vCard)
{
  UsbcanCardData *dev = (UsbcanCardData *)vCard->hwCardData;
  OS_IF_THREAD rx_thread;
  unsigned int i;

  DEBUGPRINT(3, (TXT("usbcan: _start\n")));

  // Initialize queues/waitlists for commands
  os_if_spin_lock_init(&dev->replyWaitListLock);

  INIT_LIST_HEAD(&dev->replyWaitList);
  queue_init(&dev->txCmdQueue, KV_USBCAN_TX_CMD_BUF_SIZE);

  // Init the lock for the hi part of the timestamps
  os_if_spin_lock_init(&dev->timeHi_lock);

  // Set spinlocks for the outstanding tx
  for (i = 0; i < MAX_CHANNELS; i++) {
    VCanChanData    *vChd    = vCard->chanData[i];
    UsbcanChanData  *usbChan = vChd->hwChanData;
    os_if_spin_lock_init(&usbChan->outTxLock);
  }

  os_if_init_sema(&dev->write_finished);
  os_if_up_sema(&dev->write_finished);

  os_if_init_task(&dev->txWork, &usbcan_send, vCard);
  dev->txTaskQ = os_if_declare_task("usbcan_tx", &dev->txWork);

  rx_thread = os_if_kernel_thread(usbcan_rx_thread, vCard);

  // Gather some card info
  usbcan_get_card_info(vCard);
  DEBUGPRINT(2, (TXT("vcard chnr: %d\n"), vCard->nrChannels));
  vCard->driverData = &driverData;
  vCanInitData(vCard);
} // _start


//========================================================================
//
// Allocates space for card structs
//
static int DEVINIT usbcan_allocate (VCanCardData **in_vCard)
{
  // Helper struct for allocation
  typedef struct {
    VCanChanData    *dataPtrArray[MAX_CHANNELS];
    VCanChanData    vChd[MAX_CHANNELS];
    UsbcanChanData  hChd[MAX_CHANNELS];
  } ChanHelperStruct;

  int              chNr;
  ChanHelperStruct *chs;
  VCanCardData     *vCard;

  DEBUGPRINT(3, (TXT("usbcan: _allocate\n")));

  // Allocate data area for this card
  vCard = os_if_kernel_malloc(sizeof(VCanCardData) + sizeof(UsbcanCardData));
  DEBUGPRINT(2, (TXT("MALLOC _allocate\n")));
  if (!vCard) {
    DEBUGPRINT(1, (TXT("alloc error\n")));
    goto card_alloc_err;
  }
  memset(vCard, 0, sizeof(VCanCardData) + sizeof(UsbcanCardData));

  // hwCardData is directly after VCanCardData
  vCard->hwCardData = vCard + 1;

  // Allocate memory for n channels
  chs = os_if_kernel_malloc(sizeof(ChanHelperStruct));
  DEBUGPRINT(2, (TXT("MALLOC _alloc helperstruct\n")));
  if (!chs) {
    DEBUGPRINT(1, (TXT("chan alloc error\n")));
    goto chan_alloc_err;
  }
  memset(chs, 0, sizeof(ChanHelperStruct));


  // Init array and hwChanData
  for (chNr = 0; chNr < MAX_CHANNELS; chNr++) {
    chs->dataPtrArray[chNr]    = &chs->vChd[chNr];
    chs->vChd[chNr].hwChanData = &chs->hChd[chNr];
    chs->vChd[chNr].minorNr    = -1;   // No preset minor number
  }
  vCard->chanData = chs->dataPtrArray;

  os_if_spin_lock(&driverData.canCardsLock);
  // Insert into list of cards
  vCard->next = driverData.canCards;
  driverData.canCards = vCard;
  os_if_spin_unlock(&driverData.canCardsLock);

  *in_vCard = vCard;

  return VCAN_STAT_OK;

chan_alloc_err:
  os_if_kernel_free(vCard);

card_alloc_err:

  return VCAN_STAT_NO_MEMORY;
} // _allocate




//============================================================================
// usbcan_deallocate
//
static void DEVEXIT usbcan_deallocate (VCanCardData *vCard)
{
  UsbcanCardData *dev = (UsbcanCardData *)vCard->hwCardData;
  VCanCardData *local_vCard;
  int i;

  DEBUGPRINT(3, (TXT("usbcan: _deallocate\n")));

  // Make sure all workqueues are finished
  //flush_workqueue(&dev->txTaskQ);

  if (dev->bulk_in_buffer != NULL) {
    DEBUGPRINT(2, (TXT("Free bulk_in_buffer\n")));
    os_if_kernel_free(dev->bulk_in_buffer);
    dev->bulk_in_buffer = NULL;
  }
#  if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35))
  usb_buffer_free(dev->udev, dev->bulk_out_size,
                  dev->bulk_out_buffer,
                  dev->write_urb->transfer_dma);
#  else
  usb_free_coherent(dev->udev, dev->bulk_out_size,
                    dev->bulk_out_buffer,
                    dev->write_urb->transfer_dma);
#  endif
  usb_free_urb(dev->write_urb);

  os_if_spin_lock(&driverData.canCardsLock);

  // qqq Check for open files
  local_vCard = driverData.canCards;

  // Identify the card to remove in the global list

  if (local_vCard == vCard) {
    // The first entry is the one to remove
    driverData.canCards = local_vCard->next;
  }
  else {
    while (local_vCard) {
      if (local_vCard->next == vCard) {
        // We have found it!
        local_vCard->next = vCard->next;
        break;
      }

      local_vCard = local_vCard->next;
    }

    // If vCard isn't found in the list we ignore the removal from the list
    // but we still deallocate vCard - fall through
    if (!local_vCard) {
      DEBUGPRINT(1, (TXT("Error: Bad vCard in usbcan_dealloc()\n")));
    }
  }

  os_if_spin_unlock(&driverData.canCardsLock);

  for(i = 0; i < MAX_CHANNELS; i++) {
    VCanChanData *vChd      = vCard->chanData[i];
    UsbcanChanData *usbChan = vChd->hwChanData;
    if (usbChan->objbufs) {
      DEBUGPRINT(2, (TXT("Free vCard->chanData[i]->hwChanData->objbufs[%d]\n"), i));
      os_if_kernel_free(usbChan->objbufs);
      usbChan->objbufs = NULL;
    }
  }
  if (vCard->chanData != NULL) {
    DEBUGPRINT(2, (TXT("Free vCard->chanData\n")));
    os_if_kernel_free(vCard->chanData);
    vCard->chanData = NULL;
  }
  if (vCard != NULL) {
    DEBUGPRINT(2, (TXT("Free vCard\n")));
    os_if_kernel_free(vCard);    // Also frees hwCardData (allocated together)
    vCard = NULL;
  }
} // _deallocate


#  if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 8))
#    define USB_KILL_URB(x) usb_unlink_urb(x)
#  else
#    define USB_KILL_URB(x) usb_kill_urb(x)
#  endif

//============================================================================
//     usbcan_remove
//
//     Called by the usb core when the device is removed from the system.
//
//     This routine guarantees that the driver will not submit any more urbs
//     by clearing dev->udev.  It is also supposed to terminate any currently
//     active urbs.  Unfortunately, usb_bulk_msg(), does not provide any way
//     to do this.  But at least we can cancel an active write.
//
static void DEVEXIT usbcan_remove (struct usb_interface *interface)
{
  VCanCardData *vCard;
  VCanChanData *vChan;
  UsbcanCardData *dev;
  unsigned int i;

  DEBUGPRINT(3, (TXT("usbcan: _remove\n")));

  vCard = usb_get_intfdata(interface);
  usb_set_intfdata(interface, NULL);

  dev = vCard->hwCardData;

  // Prevent device read, write and ioctl
  // Needs to be done here, or some commands will seem to
  // work even though the device is no longer present.
  dev->present = 0;

  for (i = 0; i < MAX_CHANNELS; i++) {
    vChan = vCard->chanData[i];
    DEBUGPRINT(3, (TXT("Waiting for all closed on minor %d\n"), vChan->minorNr));
    while (atomic_read(&vChan->fileOpenCount) > 0) {
      os_if_wait_for_event_timeout_simple(10);
    }
  }

  // Terminate workqueues
  flush_scheduled_work();

  // Give back our minor
  //usb_deregister_dev(interface, &usbcan_class);

  // Terminate an ongoing write
  DEBUGPRINT(6, (TXT("Ongoing write terminated\n")));
  USB_KILL_URB(dev->write_urb);
  DEBUGPRINT(6, (TXT("Unlinking urb\n")));
  os_if_down_sema(&dev->write_finished);

  // Remove spin locks
  for (i = 0; i < MAX_CHANNELS; i++) {
    VCanChanData   *vChd    = vCard->chanData[i];
    UsbcanChanData *usbChan = vChd->hwChanData;
    os_if_spin_lock_remove(&usbChan->outTxLock);
  }

  // Flush and destroy tx workqueue
  DEBUGPRINT(2, (TXT("destroy_workqueue\n")));
  os_if_destroy_task(dev->txTaskQ);

  os_if_delete_sema(&dev->write_finished);
  os_if_spin_lock_remove(&dev->replyWaitListLock);

  driverData.noOfDevices -= vCard->nrChannels;

  // Deallocate datastructures
  usbcan_deallocate(vCard);

  DEBUGPRINT(2, (TXT("USBcanII device removed. USBcanII devices present (%d)\n"),
                 driverData.noOfDevices));

} // _remove



//======================================================================
//
// Set bit timing
//
static int usbcan_set_busparams (VCanChanData *vChan, VCanBusParams *par)
{
  heliosCmd        cmd;
  VCanCardData     *vCard = vChan->vCard;
  UsbcanCardData   *dev = vCard->hwCardData;
  uint32_t         tmp;
  int              ret;

  DEBUGPRINT(3, (TXT("usbcan: _set_busparam\n")));

  cmd.setBusparamsReq.cmdNo   = CMD_SET_BUSPARAMS_REQ;
  cmd.setBusparamsReq.cmdLen  = sizeof(cmdSetBusparamsReq);
  cmd.setBusparamsReq.bitRate = par->freq;
  cmd.setBusparamsReq.sjw     = (unsigned char)par->sjw;
  cmd.setBusparamsReq.tseg1   = (unsigned char)par->tseg1;
  cmd.setBusparamsReq.tseg2   = (unsigned char)par->tseg2;
  cmd.setBusparamsReq.channel = (unsigned char)vChan->channel;
  cmd.setBusparamsReq.noSamp  = 1; // qqq Can't be trusted: (BYTE) pi->chip_param.samp3

  // Check bus parameters
  tmp = par->freq * (par->tseg1 + par->tseg2 + 1);
  if (tmp == 0) {
    DEBUGPRINT(1, (TXT("usbcan: _set_busparams() tmp == 0!\n")));
    return VCAN_STAT_BAD_PARAMETER;
  }
  if ((8000000 / tmp) > 16) {
    DEBUGPRINT(1, (TXT("usbcan: _set_busparams() prescaler wrong\n")));
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(5, (TXT ("usbcan_set_busparams: Chan(%d): Freq (%d) SJW (%d) TSEG1 (%d) TSEG2 (%d) ")
                 TXT2("Samp (%d)\n"),
                 cmd.setBusparamsReq.channel,
                 cmd.setBusparamsReq.bitRate,
                 cmd.setBusparamsReq.sjw,
                 cmd.setBusparamsReq.tseg1,
                 cmd.setBusparamsReq.tseg2,
                 cmd.setBusparamsReq.noSamp));

  ret = usbcan_queue_cmd(vCard, &cmd, 5 /* There is no response */);

  if (ret == VCAN_STAT_OK) {
    // Store locally since getBusParams not correct
    dev->freq    = (uint32_t)par->freq;
    dev->sjw     = (unsigned char)par->sjw;
    dev->tseg1   = (unsigned char)par->tseg1;
    dev->tseg2   = (unsigned char)par->tseg2;
    dev->samples = 1; // Always 1
  }

  return ret;
} // _set_busparams



//======================================================================
//
//  Get bit timing
//  GetBusParams doesn't return any values.
//
static int usbcan_get_busparams (VCanChanData *vChan, VCanBusParams *par)
{
  UsbcanCardData   *dev = vChan->vCard->hwCardData;

  DEBUGPRINT(4, (TXT("usbcan: _getbusparam\n")));

  par->freq  = dev->freq;
  par->sjw   = dev->sjw;
  par->tseg1 = dev->tseg1;
  par->tseg2 = dev->tseg2;
  par->samp3 = dev->samples; // Always 1

  // This returns wrong values...
  // ret = usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
  //                                  CMD_GET_SOFTWARE_INFO_RESP,
  //                                  CMD_GET_SOFTWARE_INFO_REQ);

  return VCAN_STAT_OK;
} // _get_busparams


//======================================================================
//
//  Set silent or normal mode
//
static int usbcan_set_silent (VCanChanData *vChan, int silent)
{
  // UsbcanCardData  *vCard   =  vChan->hwChanData;
  heliosCmd cmd;
  int       ret;

  DEBUGPRINT(3, (TXT("usbcan: _set_silent\n")));

  cmd.setDrivermodeReq.cmdNo      = CMD_SET_DRIVERMODE_REQ;
  cmd.setDrivermodeReq.cmdLen     = sizeof(cmdSetDrivermodeReq);
  cmd.setDrivermodeReq.channel    = (unsigned char)vChan->channel;
  cmd.setDrivermodeReq.driverMode = silent ? DRIVERMODE_SILENT :
                                             DRIVERMODE_NORMAL;

  ret = usbcan_queue_cmd(vChan->vCard, &cmd, 5 /* There is no response */);

  return ret;
} // _set_silent


//======================================================================
//
//  Line mode
//
static int usbcan_set_trans_type (VCanChanData *vChan, int linemode, int resnet)
{
  // qqq Not implemented
  DEBUGPRINT(3, (TXT("usbcan: _set_trans_type is NOT implemented!\n")));

  return VCAN_STAT_OK;
} // _set_trans_type


//======================================================================
//
//  Query chip status
//
static int usbcan_get_chipstate (VCanChanData *vChan)
{
  VCanCardData   *vCard = vChan->vCard;
  //VCAN_EVENT     msg;
  heliosCmd      cmd;
  heliosCmd      reply;
  int            ret;
  //UsbcanCardData *dev = vCard->hwCardData;

  DEBUGPRINT(3, (TXT("usbcan: _getchipstate\n")));

  cmd.head.cmdNo              = CMD_GET_CHIP_STATE_REQ;
  cmd.getChipStateReq.cmdLen  = sizeof(cmdGetChipStateReq);
  cmd.getChipStateReq.channel = (unsigned char)vChan->channel;
  cmd.getChipStateReq.transId = (unsigned char)vChan->channel;

  ret = usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
                                   CMD_CHIP_STATE_EVENT,
                                   cmd.getChipStateReq.transId);

  return ret;
} // _get_chipstate


//======================================================================
//
//  Go bus on
//
static int usbcan_bus_on (VCanChanData *vChan)
{
  VCanCardData    *vCard    = vChan->vCard;
  UsbcanChanData  *usbChan  = vChan->hwChanData;
  heliosCmd cmd;
  heliosCmd reply;
  int ret;

  DEBUGPRINT(3, (TXT("usbcan: _bus_on\n")));

  memset(((UsbcanChanData *)vChan->hwChanData)->current_tx_message, 0, sizeof(((UsbcanChanData *)vChan->hwChanData)->current_tx_message));
  atomic_set(&vChan->transId, 1);
  os_if_spin_lock(&usbChan->outTxLock);
  usbChan->outstanding_tx = 0;
  os_if_spin_unlock(&usbChan->outTxLock);

  cmd.head.cmdNo            = CMD_START_CHIP_REQ;
  cmd.startChipReq.cmdLen   = sizeof(cmdStartChipReq);
  cmd.startChipReq.channel  = (unsigned char)vChan->channel;
  cmd.startChipReq.transId  = (unsigned char)vChan->channel;

  DEBUGPRINT(5, (TXT("bus on called - ch %d\n"), cmd.startChipReq.channel));

  ret = usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
                                   CMD_START_CHIP_RESP,
                                   cmd.startChipReq.transId);
  if (ret == VCAN_STAT_OK) {
    vChan->isOnBus = 1;

    usbcan_get_chipstate(vChan);
  }

  return ret;
} // _bus_on


//======================================================================
//
//  Go bus off
//
static int usbcan_bus_off (VCanChanData *vChan)
{
  VCanCardData *vCard = vChan->vCard;
  UsbcanChanData *usbChan = vChan->hwChanData;

  heliosCmd cmd;
  heliosCmd reply;
  int ret;

  DEBUGPRINT(3, (TXT("usbcan: _bus_off\n")));

  cmd.head.cmdNo            = CMD_STOP_CHIP_REQ;
  cmd.startChipReq.cmdLen   = sizeof(cmdStartChipReq);
  cmd.startChipReq.channel  = (unsigned char)vChan->channel;
  cmd.startChipReq.transId  = (unsigned char)vChan->channel;

  ret = usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
                                   CMD_STOP_CHIP_RESP, cmd.startChipReq.transId);
  if (ret == VCAN_STAT_OK) {
    usbcan_get_chipstate(vChan);

    DEBUGPRINT(5, (TXT("bus off channel %d\n"), cmd.startChipReq.channel));

    vChan->isOnBus = 0;
    vChan->chipState.state = CHIPSTAT_BUSOFF;
    memset(usbChan->current_tx_message, 0, sizeof(usbChan->current_tx_message));

    os_if_spin_lock(&usbChan->outTxLock);
    usbChan->outstanding_tx = 0;
    os_if_spin_unlock(&usbChan->outTxLock);

    atomic_set(&vChan->transId, 1);
  }

  return ret;
} // _bus_off



//======================================================================
//
//  Clear send buffer on card
//
static int usbcan_flush_tx_buffer (VCanChanData *vChan)
{
  UsbcanChanData     *usbChan  = vChan->hwChanData;
  //UsbcanCardData *dev      = vChan->vCard->hwCardData;
  //VCanCardData   *vCard    = vChan->vCard;
  heliosCmd cmd;
  int ret;

  DEBUGPRINT(3, (TXT("usbcan: _flush_tx_buffer - %d\n"), vChan->channel));

  cmd.head.cmdNo         = CMD_FLUSH_QUEUE;
  cmd.flushQueue.cmdLen  = sizeof(cmd.flushQueue);
  cmd.flushQueue.channel = (unsigned char)vChan->channel;
  cmd.flushQueue.flags   = 0;

  // We _must_ clear the queue before outstanding_tx!
  // Otherwise, the transmit code could be in the process of sending
  // a message from the queue, increasing outstanding_tx after our clear.
  // With a cleared queue, the transmit code will not be doing anything.
  // qqq Consider a different handle being used to transmit.
  queue_reinit(&vChan->txChanQueue);
  os_if_spin_lock(&usbChan->outTxLock);
  usbChan->outstanding_tx = 0;
  os_if_spin_unlock(&usbChan->outTxLock);

  ret = usbcan_queue_cmd(vChan->vCard, &cmd, 5 /* There is no response */);

  if (ret == VCAN_STAT_OK) {
    atomic_set(&vChan->transId, 1);
    queue_reinit(&vChan->txChanQueue);
    os_if_spin_lock(&usbChan->outTxLock);
    usbChan->outstanding_tx = 0;
    os_if_spin_unlock(&usbChan->outTxLock);
  }

  return ret;
} // _flush_tx_buffer


//======================================================================
//
// Request send
//
static void usbcan_schedule_send (VCanCardData *vCard, VCanChanData *vChan)
{
  UsbcanCardData *dev = vCard->hwCardData;

  DEBUGPRINT(3, (TXT("usbcan: _schedule_send\n")));

  if (usbcan_tx_available(vChan) && dev->present) {
    os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);
  }
#if DEBUG
  else {
    DEBUGPRINT(3, (TXT("SEND FAILED \n")));
  }
#endif
} // _schedule_send



//======================================================================
//  Read transmit error counter
//
static int usbcan_get_tx_err (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("usbcan: _get_tx_err\n")));

  usbcan_get_chipstate(vChan);

  return vChan->chipState.txerr;
  //return vChan->txErrorCounter;
} //_get_tx_err


//======================================================================
//  Read transmit error counter
//
static int usbcan_get_rx_err (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("usbcan: _get_rx_err\n")));

  usbcan_get_chipstate(vChan);

  return vChan->chipState.rxerr;
  //return vChan->rxErrorCounter;
} // _get_rx_err


//======================================================================
//  Read receive queue length in hardware/firmware
//
static unsigned long usbcan_get_hw_rx_q_len (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("usbcan: _get_hw_rx_q_len\n")));

  // qqq Why _tx_ channel buffer?
  return queue_length(&vChan->txChanQueue);
} // _get_hw_rx_q_len


//======================================================================
//  Read transmit queue length in hardware/firmware
//
static unsigned long usbcan_get_hw_tx_q_len (VCanChanData *vChan)
{
  UsbcanChanData *hChd  = vChan->hwChanData;
  unsigned int res;

  DEBUGPRINT(3, (TXT("usbcan: _get_hw_tx_q_len\n")));

  os_if_spin_lock(&hChd->outTxLock);
  res = hChd->outstanding_tx;
  os_if_spin_unlock(&hChd->outTxLock);

  return res;
} // _get_hw_tx_q_len





//======================================================================
//
// Run when driver is loaded
//
static int INIT usbcan_init_driver (void)
{
  int result;

  DEBUGPRINT(3, (TXT("usbcan: _init_driver\n")));

  driverData.deviceName = DEVICE_NAME_STRING;

  // Register this driver with the USB subsystem
  result = usb_register(&usbcan_driver);
  if (result) {
    DEBUGPRINT(1, (TXT("usbcan: usb_register failed. Error number %d\n"),
                   result));
    return result;
  }

  return 0;
} // _init_driver



//======================================================================
// Run when driver is unloaded
//
static int EXIT usbcan_close_all (void)
{
  DEBUGPRINT(2, (TXT("usbcan: _close_all (deregister driver..)\n")));
  usb_deregister(&usbcan_driver);

  return 0;
} // _close_all



//======================================================================
// proc read function
//
static int usbcan_proc_read (struct seq_file* m, void* v)
{
  int            channel  = 0;
  VCanCardData  *cardData = driverData.canCards;

  seq_printf(m, "\ntotal channels %d\n", driverData.noOfDevices);
  seq_puts(m, "minor numbers");
  while (NULL != cardData) {
    for (channel = 0; channel < cardData->nrChannels; channel++) {
    	seq_printf(m, " %d", cardData->chanData[channel]->minorNr);
    }
    cardData = cardData->next;
  }

  return 0;
} // _proc_read


//======================================================================
//  Can we send now?
//
static int usbcan_tx_available (VCanChanData *vChan)
{
  UsbcanChanData     *usbChan  = vChan->hwChanData;
  VCanCardData       *vCard    = vChan->vCard;
  UsbcanCardData     *dev      = vCard->hwCardData;
  unsigned int       res;

  DEBUGPRINT(3, (TXT("usbcan: _tx_available %d (%d)!\n"),
                 usbChan->outstanding_tx, dev->max_outstanding_tx));

  os_if_spin_lock(&usbChan->outTxLock);
  res = usbChan->outstanding_tx;
  os_if_spin_unlock(&usbChan->outTxLock);

  return (res < dev->max_outstanding_tx);
} // _tx_available


//======================================================================
//  Are all sent msg's received?
//
static int usbcan_outstanding_sync (VCanChanData *vChan)
{
  UsbcanChanData     *usbChan  = vChan->hwChanData;
  unsigned int     res;

  DEBUGPRINT(3, (TXT("usbcan: _outstanding_sync (%d)\n"),
                 usbChan->outstanding_tx));

  os_if_spin_lock(&usbChan->outTxLock);
  res = usbChan->outstanding_tx;
  os_if_spin_unlock(&usbChan->outTxLock);

  return (res == 0);
} // _outstanding_sync



//======================================================================
// Get time
//
static int usbcan_get_time (VCanCardData *vCard, unsigned long *time)
{
  heliosCmd cmd;
  heliosCmd reply;
  int ret;

  DEBUGPRINT(3, (TXT("usbcan: _get_time\n")));

  memset(&cmd, 0, sizeof(cmd));
  cmd.head.cmdNo           = CMD_READ_CLOCK_REQ;
  cmd.readClockReq.cmdLen  = sizeof(cmd.readClockReq);
  cmd.readClockReq.flags   = 0;

  // CMD_READ_CLOCK_RESP seem to always return 0 as transid
  ret = usbcan_send_and_wait_reply(vCard, (heliosCmd *)&cmd, &reply,
                                   CMD_READ_CLOCK_RESP, 0);

  if (ret == VCAN_STAT_OK) {
    *time = reply.readClockResp.time[1];
    *time = *time << 16;
    *time += reply.readClockResp.time[0];
    *time = *time /USBCANII_TICKS_PER_10US;

    //DEBUGPRINT(2, (TXT("time %d\n"), (time / USBCANII_TICKS_PER_10US)));
  }

  return ret;
}


/***************************************************************************/
/* Free an object buffer (or free all) */
static int usbcan_objbuf_free (VCanChanData *chd, int bufType, int bufNo)
{
  int ret;
  heliosCmd cmd;
  int start, stop, i;
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    if (bufNo == -1) {
      // All buffers.. cleanup in progress, so we are happy.
      return VCAN_STAT_OK;
    }
    // Tried to free a nonexistent buffer; this is an error.
    return VCAN_STAT_BAD_PARAMETER;
  }
  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_free\n")));

  if (bufNo == -1) {
    start = 0;
    stop  = dev->autoTxBufferCount;  // ci->cc.auto...
  } else {
    start = bufNo;
    stop  = bufNo + 1;
  }

  for (i = start; i < stop; i++) {
    usbChan->objbufs[i].in_use = 0;

    memset(&cmd, 0, sizeof(cmd));
    cmd.autoTxBufferReq.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
    cmd.autoTxBufferReq.cmdLen      = sizeof(cmd.autoTxBufferReq);
    cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_DEACTIVATE;
    cmd.autoTxBufferReq.channel     = (unsigned char)chd->channel;
    cmd.autoTxBufferReq.bufNo       = (unsigned char)i;

    ret = usbcan_queue_cmd(vCard, &cmd, USBCAN_Q_CMD_WAIT_TIME);
    if (ret != VCAN_STAT_OK) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  return VCAN_STAT_OK;
}


/***************************************************************************/
/* Allocate an object buffer */
static int usbcan_objbuf_alloc (VCanChanData *chd, int bufType, int *bufNo)
{
  int i;
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (dev->autoTxBufferCount == 0 || dev->autoTxBufferResolution == 0) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_alloc\n")));

  if (!usbChan->objbufs) {
    DEBUGPRINT(4, (TXT("Allocating usbChan->objbufs[]\n")));
    usbChan->objbufs = os_if_kernel_malloc(sizeof(OBJECT_BUFFER) * dev->autoTxBufferCount);
    if (!usbChan->objbufs) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  for (i = 0; i < dev->autoTxBufferCount; i++) {
    if (!usbChan->objbufs[i].in_use) {
      usbChan->objbufs[i].in_use = 1;
      *bufNo = i;
      return VCAN_STAT_OK;
    }
  }

  return VCAN_STAT_NO_MEMORY;
}


/***************************************************************************/
/* Write data to an object buffer */
static int usbcan_objbuf_write (VCanChanData *chd, int bufType, int bufNo,
                                int id, int flags, int dlc, unsigned char *data)
{
  int ret;
  heliosCmd cmd;
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs[bufNo].in_use) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_write, id=0x%x flags=0x%x dlc=%d\n"),
                 id, flags, dlc));

  usbChan->objbufs[bufNo].msg.id     = id;
  usbChan->objbufs[bufNo].msg.flags  = (unsigned char)flags;
  usbChan->objbufs[bufNo].msg.length = (unsigned char)dlc;
  memcpy(usbChan->objbufs[bufNo].msg.data, data, (dlc > 8) ? 8 : dlc);

  memset(&cmd, 0, sizeof(cmd));
  cmd.setAutoTxBuffer.cmdNo   = CMD_SET_AUTO_TX_BUFFER;
  cmd.setAutoTxBuffer.cmdLen  = sizeof(cmd.setAutoTxBuffer);
  cmd.setAutoTxBuffer.channel = (unsigned char)chd->channel;
  cmd.setAutoTxBuffer.bufNo   = (unsigned char)bufNo;

  cmd.setAutoTxBuffer.flags = 0;
  if (id & EXT_MSG) {
    cmd.setAutoTxBuffer.flags        |= AUTOTXBUFFER_MSG_EXT;
    cmd.setAutoTxBuffer.rawMessage[0] = (unsigned char)((id >> 24) & 0x1F);
    cmd.setAutoTxBuffer.rawMessage[1] = (unsigned char)((id >> 18) & 0x3F);
    cmd.setAutoTxBuffer.rawMessage[2] = (unsigned char)((id >> 14) & 0x0F);
    cmd.setAutoTxBuffer.rawMessage[3] = (unsigned char)((id >>  6) & 0xFF);
    cmd.setAutoTxBuffer.rawMessage[4] = (unsigned char)((id      ) & 0x3F);
  } else {
    cmd.setAutoTxBuffer.rawMessage[0] = (unsigned char)((id >>  6) & 0x1F);
    cmd.setAutoTxBuffer.rawMessage[1] = (unsigned char)((id      ) & 0x3F);
  }
  cmd.setAutoTxBuffer.rawMessage[5] = dlc & 0x0F;
  memcpy(&cmd.setAutoTxBuffer.rawMessage[6], data, 8);

  if (flags & MSGFLAG_REMOTE_FRAME) {
    cmd.setAutoTxBuffer.flags |= AUTOTXBUFFER_MSG_REMOTE_FRAME;
  }

  ret = usbcan_queue_cmd(vCard, &cmd, USBCAN_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}


/***************************************************************************/
/* Set filters on an object buffer */
static int usbcan_objbuf_set_filter (VCanChanData *chd, int bufType, int bufNo,
                                     int code, int mask)
{
  UsbcanChanData  *usbChan = chd->hwChanData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_set_filter\n")));
  // This operation is irrelevant, so we fail.

  return VCAN_STAT_BAD_PARAMETER;
}


/***************************************************************************/
/* Set flags on an object buffer */
static int usbcan_objbuf_set_flags (VCanChanData *chd, int bufType, int bufNo,
                                    int flags)
{
  UsbcanChanData  *usbChan = chd->hwChanData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_set_flags\n")));
  // This operation is irrelevant.

  return VCAN_STAT_BAD_PARAMETER;
}


/***************************************************************************/
/* Enable/disable an object buffer (or enable/disable all) */
static int usbcan_objbuf_enable (VCanChanData *chd, int bufType, int bufNo,
                                 int enable)
{
  int ret;
  heliosCmd cmd;
  int start, stop, i;
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_enable\n")));

  if (bufNo == -1) {
    start = 0;
    stop  = dev->autoTxBufferCount;
  } else {
    start = bufNo;
    stop  = bufNo + 1;
    if (!usbChan->objbufs[start].in_use) {
      return VCAN_STAT_BAD_PARAMETER;
    }
  }

  for (i = start; i < stop; i++) {
    usbChan->objbufs[i].active = enable;

    memset(&cmd, 0, sizeof(cmd));
    cmd.autoTxBufferReq.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
    cmd.autoTxBufferReq.cmdLen      = sizeof(cmd.autoTxBufferReq);
    cmd.autoTxBufferReq.requestType = enable ? AUTOTXBUFFER_CMD_ACTIVATE :
                                               AUTOTXBUFFER_CMD_DEACTIVATE;
    cmd.autoTxBufferReq.channel     = (unsigned char)chd->channel;
    cmd.autoTxBufferReq.bufNo       = (unsigned char)i;

    ret = usbcan_queue_cmd(vCard, &cmd, USBCAN_Q_CMD_WAIT_TIME);
    if (ret != VCAN_STAT_OK) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  return VCAN_STAT_OK;
}


/***************************************************************************/
/* Set the transmission interval (in microseconds) for an object buffer */
static int usbcan_objbuf_set_period (VCanChanData *chd, int bufType, int bufNo,
                                     int period)
{
  int ret;
  heliosCmd cmd;
  unsigned int interval;
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!usbChan->objbufs[bufNo].in_use) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (dev->autoTxBufferCount == 0 || dev->autoTxBufferResolution == 0) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  ret = 0;
  interval = (period + dev->autoTxBufferResolution / 2) /
             dev->autoTxBufferResolution;
  if (interval == 0) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_set_period period=%d (scaled)interval=%d\n"),
                 period, interval));

  usbChan->objbufs[bufNo].period = period;

  memset(&cmd, 0, sizeof(cmd));
  cmd.autoTxBufferReq.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
  cmd.autoTxBufferReq.cmdLen      = sizeof(cmd.autoTxBufferReq);
  cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_SET_INTERVAL;
  cmd.autoTxBufferReq.channel     = (unsigned char)chd->channel;
  cmd.autoTxBufferReq.bufNo       = (unsigned char)bufNo;
  cmd.autoTxBufferReq.interval    = interval;

  ret = usbcan_queue_cmd(vCard, &cmd, USBCAN_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}




/***************************************************************************/
static int usbcan_objbuf_exists (VCanChanData *chd, int bufType, int bufNo)
{
  UsbcanChanData  *usbChan = chd->hwChanData;
  VCanCardData    *vCard   = chd->vCard;
  UsbcanCardData  *dev     = (UsbcanCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return 0;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return 0;
  }

  if (!usbChan->objbufs) {
    return 0;
  }

  if (!usbChan->objbufs[bufNo].in_use) {
    return 0;
  }

  return 1;
}



INIT int init_module (void)
{
  driverData.hwIf = &hwIf;
  return vCanInit (&driverData, MAX_CHANNELS);
}

EXIT void cleanup_module (void)
{
  vCanCleanup (&driverData);
}
