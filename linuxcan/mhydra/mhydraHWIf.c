/*
**                Copyright 2012 by Kvaser AB, M�lndal, Sweden
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
// Linux Mhydra driver
//

#  include <linux/version.h>
#  include <linux/usb.h>
#  include <linux/types.h>
#  include <linux/seq_file.h>


// Non system headers
#include "osif_kernel.h"
#include "osif_functions_kernel.h"
#include "VCanOsIf.h"
#include "mhydraHWIf.h"
#include "hydra_host_cmds.h"
#include "queue.h"
#include "debug.h"
#include "hwnames.h"
#include "vcan_ioctl.h"



// Get a minor range for your devices from the usb maintainer
// Use a unique set for each driver
#define USB_MHYDRA_MINOR_BASE   80

    MODULE_LICENSE("Dual BSD/GPL");
    MODULE_AUTHOR("KVASER");
    MODULE_DESCRIPTION("Mhydra CAN module.");


//----------------------------------------------------------------------------
// If you do not define MHYDRA_DEBUG at all, all the debug code will be
// left out.  If you compile with MHYDRA_DEBUG=0, the debug code will
// be present but disabled -- but it can then be enabled for specific
// modules at load time with a 'pc_debug=#' option to insmod.
//
#  ifdef MHYDRA_DEBUG
      static int pc_debug = MHYDRA_DEBUG;
      MODULE_PARM_DESC(pc_debug, "Mhydra debug level");
      module_param(pc_debug, int, 0644);
#     define DEBUGPRINT(n, arg)     if (pc_debug >= (n)) { DEBUGOUT(n, arg); }
#  else
#     define DEBUGPRINT(n, arg)     if ((n) == 1) { DEBUGOUT(n, arg); }
#  endif
//----------------------------------------------------------------------------



//======================================================================
// HW function pointers
//======================================================================

static int INIT mhydra_init_driver(void);
static int mhydra_set_busparams(VCanChanData *vChd, VCanBusParams *par);
static int mhydra_get_busparams(VCanChanData *vChd, VCanBusParams *par);
static int mhydra_set_silent(VCanChanData *vChd, int silent);
static int mhydra_set_trans_type(VCanChanData *vChd, int linemode, int resnet);
static int mhydra_bus_on(VCanChanData *vChd);
static int mhydra_bus_off(VCanChanData *vChd);
static int mhydra_get_tx_err(VCanChanData *vChd);
static int mhydra_get_rx_err(VCanChanData *vChd);
static int mhydra_outstanding_sync(VCanChanData *vChan);
static int EXIT mhydra_close_all(void);
static int mhydra_proc_read(struct seq_file* m, void* v);
static int mhydra_get_chipstate(VCanChanData *vChd);
static int mhydra_get_time(VCanCardData *vCard, unsigned long *time);
static int mhydra_flush_tx_buffer(VCanChanData *vChan);
static void mhydra_schedule_send(VCanCardData *vCard, VCanChanData *vChan);
static unsigned long mhydra_get_hw_rx_q_len(VCanChanData *vChan);
static unsigned long mhydra_get_hw_tx_q_len(VCanChanData *vChan);
static int mhydra_objbuf_exists(VCanChanData *chd, int bufType, int bufNo);
static int mhydra_objbuf_free(VCanChanData *chd, int bufType, int bufNo);
static int mhydra_objbuf_alloc(VCanChanData *chd, int bufType, int *bufNo);
static int mhydra_objbuf_write(VCanChanData *chd, int bufType, int bufNo,
                             int id, int flags, int dlc, unsigned char *data);
static int mhydra_objbuf_enable(VCanChanData *chd, int bufType, int bufNo,
                              int enable);
static int mhydra_objbuf_set_filter(VCanChanData *chd, int bufType, int bufNo,
                                  int code, int mask);
static int mhydra_objbuf_set_flags(VCanChanData *chd, int bufType, int bufNo,
                                 int flags);
static int mhydra_objbuf_set_period(VCanChanData *chd, int bufType, int bufNo,
                                  int period);
static int mhydra_objbuf_set_msg_count(VCanChanData *chd, int bufType, int bufNo,
                                     int count);
static int mhydra_objbuf_send_burst(VCanChanData *chd, int bufType, int bufNo,
                                  int burstLen);

static VCanDriverData driverData;

static VCanHWInterface hwIf = {
  .initAllDevices    = mhydra_init_driver,
  .setBusParams      = mhydra_set_busparams,
  .getBusParams      = mhydra_get_busparams,
  .setOutputMode     = mhydra_set_silent,
  .setTranceiverMode = mhydra_set_trans_type,
  .busOn             = mhydra_bus_on,
  .busOff            = mhydra_bus_off,
  .txAvailable       = mhydra_outstanding_sync,            // This isn't really a function thats checks if tx is available!
  .procRead          = mhydra_proc_read,
  .closeAllDevices   = mhydra_close_all,
  .getTime           = mhydra_get_time,
  .flushSendBuffer   = mhydra_flush_tx_buffer,
  .getRxErr          = mhydra_get_rx_err,
  .getTxErr          = mhydra_get_tx_err,
  .rxQLen            = mhydra_get_hw_rx_q_len,
  .txQLen            = mhydra_get_hw_tx_q_len,
  .requestChipState  = mhydra_get_chipstate,
  .requestSend       = mhydra_schedule_send,
  .objbufExists      = mhydra_objbuf_exists,
  .objbufFree        = mhydra_objbuf_free,
  .objbufAlloc       = mhydra_objbuf_alloc,
  .objbufWrite       = mhydra_objbuf_write,
  .objbufEnable      = mhydra_objbuf_enable,
  .objbufSetFilter   = mhydra_objbuf_set_filter,
  .objbufSetFlags    = mhydra_objbuf_set_flags,
  .objbufSetPeriod   = mhydra_objbuf_set_period,
  .objbufSetMsgCount = mhydra_objbuf_set_msg_count,
  .objbufSendBurst   = mhydra_objbuf_send_burst
};



//======================================================================
// Static declarations


// USB packet size
#define MAX_PACKET_OUT      3072        // To device
#define MAX_PACKET_IN       3072        // From device

#define TIMESTAMP_AS_UINT64(X) ((*(uint64_t *)(X)) & 0X0000FFFFFFFFFFFFULL)


//===========================================================================
static unsigned long timestamp_in_10us(VCanCardData *vCard,
                                       unsigned short *tics)
{
  unsigned long  ulTemp;
  unsigned short resTime[3];
  uint64_t timestamp;
  MhydraCardData *dev = (MhydraCardData *)vCard->hwCardData;
  unsigned int hires_timer_fq = dev->hires_timer_fq;
  unsigned int uiDivisor = 10 * hires_timer_fq;


  timestamp = TIMESTAMP_AS_UINT64(tics);
  tics[0] = timestamp;
  tics[1] = timestamp >> 16;
  tics[2] = timestamp >> 32;

  resTime[0] = tics[2] / uiDivisor;
  ulTemp     = (tics[2] % uiDivisor) << 16;

  if (resTime[0] > 0) {
    // The timer overflows - Ignore this
    DEBUGPRINT(6, (TXT(">>>>>>>>>> Timer overflow\n")));
  }

  resTime[1] = (unsigned short)((ulTemp + tics[1]) / uiDivisor);
  ulTemp     = ((ulTemp + tics[1]) % uiDivisor) << 16;

  resTime[2] = (unsigned short)((ulTemp + tics[0]) / uiDivisor);

  return (unsigned long)((resTime[1] << 16) + resTime[2]);
}


#define NUMBER_OF_BITS_FROM_ACK_TO_VALID_MSG    8


//======================================================================
// Prototypes
static int    mhydra_plugin(struct usb_interface *interface,
                          const struct usb_device_id *id);
static void   mhydra_remove(struct usb_interface *interface);

// Interrupt handler prototype changed in 2.6.19.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
static void   mhydra_write_bulk_callback(struct urb *urb, struct pt_regs *regs);
 #else
static void   mhydra_write_bulk_callback(struct urb *urb);
 #endif



static int    mhydra_allocate(VCanCardData **vCard);
static void   mhydra_deallocate(VCanCardData *vCard);

static void   mhydra_start(VCanCardData *vCard);

static int    mhydra_tx_available(VCanChanData *vChan);
static int    mhydra_transmit(VCanCardData *vCard);
static int    mhydra_send_and_wait_reply(VCanCardData *vCard, hydraHostCmd *cmd,
                                       hydraHostCmd *replyPtr,
                                       unsigned char cmdNr,
                                       unsigned char transId);
static int    mhydra_queue_cmd(VCanCardData *vCard, hydraHostCmd *cmd,
                             unsigned int timeout);

static void   mhydra_handle_command(hydraHostCmd *cmd, VCanCardData *vCard);
static unsigned short    mhydra_get_trans_id(hydraHostCmd *cmd);

static int    mhydra_fill_usb_buffer(VCanCardData *vCard,
                                   unsigned char *buffer, int maxlen);
static void   mhydra_translate_can_msg(VCanChanData *vChan,
                                     hydraHostCmd *hydra_msg, CAN_MSG *can_msg);

static void   mhydra_get_card_info(VCanCardData *vCard);
//----------------------------------------------------------------------



//----------------------------------------------------------------------------
// Supported KVASER hardware
#define KVASER_VENDOR_ID                0x0bfd
#define USB_EAGLE_PRODUCT_ID              256 // Kvaser Eagle
#define USB_BLACKBIRD_V2_PRODUCT_ID       258 // Kvaser BlackBird v2
#define USB_MEMO_PRO_5HS_PRODUCT_ID       260 // Kvaser Memorator Pro 5xHS
#define USB_USBCAN_PRO_5HS_PRODUCT_ID     261 // Kvaser USBcan Pro 5xHS
#define USB_USBCAN_LIGHT_4HS_PRODUCT_ID   262 // Kvaser USBcan Light 4xHS (00831-1)
#define USB_LEAF_PRO_HS_V2_PRODUCT_ID     263 // Kvaser Leaf Pro HS v2 (00843-4)
#define USB_USBCAN_PRO_2HS_V2_PRODUCT_ID  264 // Kvaser USBcan Pro 2xHS v2 (00752-9)
#define USB_MEMO_2HS_PRODUCT_ID           265 // Kvaser Memorator 2xHS v2 (00821-2)
#define USB_MEMO_PRO_2HS_V2_PRODUCT_ID    266 // Kvaser Memorator Pro 2xHS v2 (00819-9)

// Table of devices that work with this driver
static struct usb_device_id mhydra_table [] = {
  { USB_DEVICE(KVASER_VENDOR_ID, USB_EAGLE_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_BLACKBIRD_V2_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_MEMO_PRO_5HS_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_USBCAN_PRO_5HS_PRODUCT_ID) },
  { USB_DEVICE(KVASER_VENDOR_ID, USB_USBCAN_LIGHT_4HS_PRODUCT_ID)},
  { USB_DEVICE(KVASER_VENDOR_ID, USB_LEAF_PRO_HS_V2_PRODUCT_ID)},
  { USB_DEVICE(KVASER_VENDOR_ID, USB_USBCAN_PRO_2HS_V2_PRODUCT_ID)},
  { USB_DEVICE(KVASER_VENDOR_ID, USB_MEMO_2HS_PRODUCT_ID)},
  { USB_DEVICE(KVASER_VENDOR_ID, USB_MEMO_PRO_2HS_V2_PRODUCT_ID)},
  { }  // Terminating entry
};

MODULE_DEVICE_TABLE(usb, mhydra_table);

//
// USB class driver info in order to get a minor number from the usb core,
// and to have the device registered with devfs and the driver core
//



// USB specific object needed to register this driver with the usb subsystem
static struct usb_driver mhydra_driver = {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15))
  .owner      =    THIS_MODULE,
#endif
  .name       =    "mhydra",
  .probe      =    mhydra_plugin,
  .disconnect =    mhydra_remove,
  .id_table   =    mhydra_table,
};

//============================================================================





//------------------------------------------------------
//
//    ---- CALLBACKS ----
//
//------------------------------------------------------

//============================================================================
//  mhydra_write_bulk_callback
//
// Interrupt handler prototype changed in 2.6.19.
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19))
static void mhydra_write_bulk_callback (struct urb *urb, struct pt_regs *regs)
#else
static void mhydra_write_bulk_callback (struct urb *urb)
#endif
{
  VCanCardData *vCard = (VCanCardData *)urb->context;
  MhydraCardData *dev   = (MhydraCardData *)vCard->hwCardData;

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
// mhydra_rx_thread
//
static int mhydra_rx_thread (void *context)
{
  VCanCardData *vCard   = (VCanCardData *)context;
  MhydraCardData *dev     = (MhydraCardData *)vCard->hwCardData;
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

    len = 0;
    // Do a blocking bulk read to get data from the device
    // Timeout after 30 seconds
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
      DEBUGPRINT(2, (TXT ("usb_bulk_msg error (%d)"), ret));

      if (ret != -ETIMEDOUT) {
        // Save if runaway
        if (ret == -EILSEQ || ret == -ESHUTDOWN || ret == -EINVAL) {
          DEBUGPRINT(2, (TXT ("usb_bulk_msg error (%d) - Device probably ")
                         TXT2("removed, closing down\n"), ret));
          dev->present = 0;
          result = -ENODEV;
          break;
        }

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
      hydraHostCmd        *cmd;
      int            loopCounter = 1000; // qqq, dev->bulk_in_size+1??
      unsigned int   count       = 0;


      while (count < len) {
        // A loop counter as a safety measure.
        if (--loopCounter == 0) {
          DEBUGPRINT(2, (TXT("ERROR mhydra_rx_thread() LOOPMAX. \n")));
          break;
        }

        // A command will never straddle a bulk_in_MaxPacketSize byte boundary.
        // The firmware will place a zero in the buffer to indicate that
        // the next command will follow after the next
        // bulk_in_MaxPacketSize bytes boundary.

        cmd = (hydraHostCmd *)&buffer[count];
        if (cmd->cmdNo == 0) {
          count += dev->bulk_in_MaxPacketSize;
          count &= -(dev->bulk_in_MaxPacketSize);
          continue;
        }
        else {
          count += HYDRA_CMD_SIZE;
        }

        mhydra_handle_command(cmd, vCard);
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


// This should compile to nothing on an x86,
// which is a little endian CPU.
static void le_to_cpu (hydraHostCmd *cmd)
{
  switch (cmd->cmdNo) {
  case CMD_LOG_MESSAGE:
    le16_to_cpus(&cmd->logMessage.time[0]);
    le16_to_cpus(&cmd->logMessage.time[1]);
    le16_to_cpus(&cmd->logMessage.time[2]);
    le32_to_cpus(&cmd->logMessage.id);
    break;
  case CMD_LOG_TRIG:
    le16_to_cpus(&cmd->logTrig.time[0]);
    le16_to_cpus(&cmd->logTrig.time[1]);
    le16_to_cpus(&cmd->logTrig.time[2]);
    le32_to_cpus(&cmd->logTrig.preTrigger);
    le32_to_cpus(&cmd->logTrig.postTrigger);
    break;
  case CMD_LOG_RTC_TIME:
    le32_to_cpus(&cmd->logRtcTime.unixTime);
    break;
  case CMD_RX_STD_MESSAGE:
  case CMD_RX_EXT_MESSAGE:
    le16_to_cpus(&cmd->logMessage.time[0]);
    le16_to_cpus(&cmd->logMessage.time[1]);
    le16_to_cpus(&cmd->logMessage.time[2]);
    break;
  case CMD_TX_ACKNOWLEDGE:
    le16_to_cpus(&cmd->txAck.time[0]);
    le16_to_cpus(&cmd->txAck.time[1]);
    le16_to_cpus(&cmd->txAck.time[2]);
    break;
  case CMD_TX_REQUEST:
    le16_to_cpus(&cmd->txRequest.time[0]);
    le16_to_cpus(&cmd->txRequest.time[1]);
    le16_to_cpus(&cmd->txRequest.time[2]);
    break;
  case CMD_SET_BUSPARAMS_REQ:
    le32_to_cpus(&cmd->setBusparamsReq.bitRate);
    break;
  case CMD_GET_BUSPARAMS_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_BUSPARAMS_RESP:
    // Nothing to translate.
    le32_to_cpus(&cmd->getBusparamsResp.bitRate);
    break;
  case CMD_GET_CHIP_STATE_REQ:
    // Nothing to translate.
    break;
  case CMD_CHIP_STATE_EVENT:
    le16_to_cpus(&cmd->chipStateEvent.time[0]);
    le16_to_cpus(&cmd->chipStateEvent.time[1]);
    le16_to_cpus(&cmd->chipStateEvent.time[2]);
    break;
  case CMD_SET_DRIVERMODE_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_DRIVERMODE_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_DRIVERMODE_RESP:
    // Nothing to translate.
    break;
  case CMD_RESET_CHIP_REQ:
    // Nothing to translate.
    break;
  case CMD_RESET_CARD_REQ:
    // Nothing to translate.
    break;
  case CMD_START_CHIP_REQ:
    // Nothing to translate.
    break;
  case CMD_START_CHIP_RESP:
    // Nothing to translate.
    break;
  case CMD_STOP_CHIP_REQ:
    // Nothing to translate.
    break;
  case CMD_STOP_CHIP_RESP:
    // Nothing to translate.
    break;
  case CMD_READ_CLOCK_REQ:
    // Nothing to translate.
    break;
  case CMD_READ_CLOCK_RESP:
    le16_to_cpus(&cmd->readClockResp.time[0]);
    le16_to_cpus(&cmd->readClockResp.time[1]);
    le16_to_cpus(&cmd->readClockResp.time[2]);
    break;
  case CMD_SELF_TEST_REQ:
    // Nothing to translate.
    break;
  case CMD_SELF_TEST_RESP:
    le32_to_cpus(&cmd->selfTestResp.results);
    break;
  case CMD_GET_CARD_INFO_2:
    le32_to_cpus(&cmd->getCardInfo2Resp.oem_unlock_code);
    break;
  case CMD_GET_CARD_INFO_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_CARD_INFO_RESP:
    le32_to_cpus(&cmd->getCardInfoResp.serialNumber);
    le32_to_cpus(&cmd->getCardInfoResp.clockResolution);
    le32_to_cpus(&cmd->getCardInfoResp.mfgDate);
    le32_to_cpus(&cmd->getCardInfoResp.hwRevision);

    break;
  case CMD_GET_INTERFACE_INFO_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_INTERFACE_INFO_RESP:
    le32_to_cpus(&cmd->getInterfaceInfoResp.channelCapabilities);
    break;
  case CMD_GET_SOFTWARE_INFO_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_SOFTWARE_INFO_RESP:
    le16_to_cpus(&cmd->getSoftwareInfoResp.maxOutstandingTx);
    break;
  case CMD_GET_BUSLOAD_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_BUSLOAD_RESP:
    le16_to_cpus(&cmd->getBusLoadResp.time[0]);
    le16_to_cpus(&cmd->getBusLoadResp.time[1]);
    le16_to_cpus(&cmd->getBusLoadResp.time[2]);
    le16_to_cpus(&cmd->getBusLoadResp.sample_interval);
    le16_to_cpus(&cmd->getBusLoadResp.active_samples);
    le16_to_cpus(&cmd->getBusLoadResp.delta_t);
    break;
  case CMD_RESET_STATISTICS:
    // Nothing to translate.
    break;
  case CMD_CHECK_LICENSE_REQ:
    // Nothing to translate.
    break;
  case CMD_CHECK_LICENSE_RESP:
    le32_to_cpus(&cmd->checkLicenseResp.licenseMask);
    le32_to_cpus(&cmd->checkLicenseResp.kvaserLicenseMask);
    break;
  case CMD_ERROR_EVENT:
    le16_to_cpus(&cmd->errorEvent.time[0]);
    le16_to_cpus(&cmd->errorEvent.time[1]);
    le16_to_cpus(&cmd->errorEvent.time[2]);
    le16_to_cpus(&cmd->errorEvent.addInfo1);
    le16_to_cpus(&cmd->errorEvent.addInfo2);
    break;
  case CMD_FLUSH_QUEUE:
    // Nothing to translate.
    break;
  case CMD_RESET_ERROR_COUNTER:
    // Nothing to translate.
    break;
  case CMD_CAN_ERROR_EVENT:
    le16_to_cpus(&cmd->canErrorEvent.time[0]);
    le16_to_cpus(&cmd->canErrorEvent.time[1]);
    le16_to_cpus(&cmd->canErrorEvent.time[2]);
    break;
  case CMD_SET_TRANSCEIVER_MODE_REQ:
    // Nothing to translate.
    break;
  case CMD_INTERNAL_DUMMY:
    le16_to_cpus(&cmd->internalDummy.time);
    break;
  case CMD_SOUND:
    le16_to_cpus(&cmd->sound.freq);
    le16_to_cpus(&cmd->sound.duration);
    break;
  case CMD_SET_IO_PORTS_REQ:
    le32_to_cpus(&cmd->setIoPortsReq.portVal);
    break;
  case CMD_GET_IO_PORTS_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_IO_PORTS_RESP:
    le32_to_cpus(&cmd->getIoPortsResp.portVal);
    le16_to_cpus(&cmd->getIoPortsResp.status);
    break;
  case CMD_GET_TRANSCEIVER_INFO_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_TRANSCEIVER_INFO_RESP:
    le32_to_cpus(&cmd->getTransceiverInfoResp.transceiverCapabilities);
    break;
  case CMD_SET_HEARTBEAT_RATE_REQ:
    le16_to_cpus(&cmd->setHeartbeatRateReq.rate);
    break;
  case CMD_HEARTBEAT_RESP:
    le16_to_cpus(&cmd->heartbeatResp.time[0]);
    le16_to_cpus(&cmd->heartbeatResp.time[1]);
    le16_to_cpus(&cmd->heartbeatResp.time[2]);
    break;
  case CMD_AUTO_TX_BUFFER_REQ:
    le32_to_cpus(&cmd->autoTxBufferReq.interval);
    break;
  case CMD_AUTO_TX_BUFFER_RESP:
    le32_to_cpus(&cmd->autoTxBufferResp.timerResolution);
    le16_to_cpus(&cmd->autoTxBufferResp.capabilities);
    break;
  case CMD_TREF_SOFNR:
    le16_to_cpus(&cmd->trefSofSeq.sofNr);
    le16_to_cpus(&cmd->trefSofSeq.time[0]);
    le16_to_cpus(&cmd->trefSofSeq.time[1]);
    le16_to_cpus(&cmd->trefSofSeq.time[2]);
    break;
  case CMD_SOFTSYNC_ONOFF:
    le16_to_cpus(&cmd->softSyncOnOff.onOff);
    break;
  case CMD_USB_THROTTLE:
    le16_to_cpus(&cmd->usbThrottle.throttle);
    break;
  case CMD_GET_SOFTWARE_DETAILS_REQ:
    // Nothing to translate.
    break;
  case CMD_GET_SOFTWARE_DETAILS_RESP:
    le32_to_cpus(&cmd->getSoftwareDetailsResp.swOptions);
    le32_to_cpus(&cmd->getSoftwareDetailsResp.swVersion);
    DEBUGPRINT(4, (TXT("translate getSoftwareDetailsResp **** %d ****\n"), cmd->getSoftwareDetailsResp.swVersion));
    le32_to_cpus(&cmd->getSoftwareDetailsResp.swName);
    le32_to_cpus(&cmd->getSoftwareDetailsResp.EAN[0]);
    le32_to_cpus(&cmd->getSoftwareDetailsResp.EAN[1]);
    break;

  default:
    DEBUGPRINT(4, (TXT("translate **** %d ****\n"), cmd->cmdNo));
    break;
  }
}


static void cpu_to_le (hydraHostCmd *cmd)
{
  // qqq For now, assume cpu_to_le is always the same as le_to_cpu.
  le_to_cpu(cmd);
}


//============================================================================
//
// mhydra_handle_command
// Handle a received hydraHostCmd.
//
static void mhydra_handle_command (hydraHostCmd *cmd, VCanCardData *vCard)
{
  MhydraCardData     *dev = (MhydraCardData *)vCard->hwCardData;
  struct list_head *currHead;
  struct list_head *tmpHead;
  WaitNode         *currNode;
  VCAN_EVENT       e;
  unsigned long    irqFlags;
  int              srcHE;

  le_to_cpu(cmd);

  srcHE = (cmd->cmdIOP.srcChannel << 4) | cmd->cmdIOPSeq.srcHE;

  DEBUGPRINT(2, (TXT("*** mhydra_handle_command %d\n"), cmd->cmdNo));
  switch (cmd->cmdNo) {



    case CMD_GET_BUSPARAMS_RESP:
    {
      unsigned int  chan = dev->he2channel[srcHE];

      if (chan < (unsigned)vCard->nrChannels) {

        DEBUGPRINT(4, (TXT("CMD_GET_BUSPARAMS_RESP\n")));
        dev->freq    = cmd->getBusparamsResp.bitRate;
        dev->sjw     = cmd->getBusparamsResp.sjw;
        dev->tseg1   = cmd->getBusparamsResp.tseg1;
        dev->tseg2   = cmd->getBusparamsResp.tseg2;
        dev->samples = 1;

        DEBUGPRINT(5, (TXT ("CMD_GET_BUSPARAMS_RESP Chan(%d): Freq (%d) SJW (%d) TSEG1 (%d) TSEG2 (%d)\n"),
                     chan,
                     cmd->getBusparamsResp.bitRate,
                     cmd->getBusparamsResp.sjw,
                     cmd->getBusparamsResp.tseg1,
                     cmd->getBusparamsResp.tseg2));
      }
      break;
    }

    case CMD_CHIP_STATE_EVENT:
    {
      unsigned int  chan = dev->he2channel[srcHE];
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
        DEBUGPRINT(6, (TXT("CMD_CHIP_STATE_EVENT, chan %d - busStatus 0x%02X\n"), chan, cmd->chipStateEvent.busStatus));
        // ".busStatus" is the contents of the CnSTRH register.
        switch (cmd->chipStateEvent.busStatus &
                (M32C_BUS_PASSIVE | M32C_BUS_OFF)) {
          case 0:
            vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
            break;

          case M32C_BUS_PASSIVE:
            vChd->chipState.state = CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            break;

          case M32C_BUS_OFF:
            vChd->chipState.state = CHIPSTAT_BUSOFF;
            break;

          case (M32C_BUS_PASSIVE | M32C_BUS_OFF):
            vChd->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE |
                                    CHIPSTAT_ERROR_WARNING;
            break;
        }

        // Reset is treated like bus-off
        if (cmd->chipStateEvent.busStatus & M32C_BUS_RESET) {
          vChd->chipState.state = CHIPSTAT_BUSOFF;
          vChd->chipState.txerr = 0;
          vChd->chipState.rxerr = 0;
        }

        e.tag       = V_CHIP_STATE;
        e.timeStamp = timestamp_in_10us(vCard, cmd->chipStateEvent.time);
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
      DEBUGPRINT(4, (TXT("CMD_READ_CLOCK_RESP - Ignored\n")));
      break;

    case CMD_GET_CARD_INFO_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_CARD_INFO_RESP\n")));

      memcpy(vCard->ean, &cmd->getCardInfoResp.EAN[0], 8);
      vCard->serialNumber = cmd->getCardInfoResp.serialNumber;

      vCard->hwRevisionMajor = cmd->getCardInfoResp.hwRevision;
      vCard->hwRevisionMinor = 0;
      vCard->hw_type = cmd->getCardInfoResp.hwType;

      vCard->nrChannels = cmd->getCardInfoResp.channelCount;

      DEBUGPRINT(4, (TXT("CMD_GET_CARD_INFO_RESP vCard->nrChannels (%d)\n"), vCard->nrChannels));

      vCard->capabilities =     VCAN_CHANNEL_CAP_SEND_ERROR_FRAMES |
                                VCAN_CHANNEL_CAP_RECEIVE_ERROR_FRAMES |
                                VCAN_CHANNEL_CAP_TIMEBASE_ON_CARD |
                                VCAN_CHANNEL_CAP_BUSLOAD_CALCULATION |
                                VCAN_CHANNEL_CAP_ERROR_COUNTERS |
                                VCAN_CHANNEL_CAP_EXTENDED_CAN |
                                VCAN_CHANNEL_CAP_TXREQUEST |
                                VCAN_CHANNEL_CAP_TXACKNOWLEDGE |
                                VCAN_CHANNEL_CAP_ERROR_COUNTERS;
      break;

    case CMD_GET_CARD_INFO_2:
      DEBUGPRINT(4, (TXT("CMD_GET_CARD_INFO_2 - Ignored\n")));
      break;

    case CMD_GET_INTERFACE_INFO_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_INTERFACE_INFO_RESP - Ignored\n")));
      break;

    case CMD_GET_SOFTWARE_INFO_RESP:
    {
      DEBUGPRINT(4, (TXT("CMD_GET_SOFTWARE_INFO_RESP\n")));

      dev->max_outstanding_tx = cmd->getSoftwareInfoResp.maxOutstandingTx;
      if (dev->max_outstanding_tx > HYDRA_MAX_OUTSTANDING_TX) {
        dev->max_outstanding_tx = HYDRA_MAX_OUTSTANDING_TX;
      }
      dev->max_outstanding_tx--;   // Can't use all elements!

      break;
    }

    case CMD_AUTO_TX_BUFFER_RESP:
    {
      if (cmd->autoTxBufferResp.responseType == AUTOTXBUFFER_CMD_GET_INFO) {
        dev->autoTxBufferCount      = cmd->autoTxBufferResp.bufferCount;
        dev->autoTxBufferResolution = cmd->autoTxBufferResp.timerResolution;
        DEBUGPRINT(2, (TXT("AUTOTXBUFFER_CMD_GET_INFO: count=%d resolution=%d\n"),
                       dev->autoTxBufferCount, dev->autoTxBufferResolution));
      }
      break;
    }

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
      unsigned int  chan = dev->he2channel[srcHE];
      VCanChanData  *vChan = vCard->chanData[chan];

      DEBUGPRINT(4, (TXT("CMD_TX_REQUEST\n")));

      if (chan < (unsigned)vCard->nrChannels) {
        // A TxReq. Take the current tx message, modify it to a
        // receive message and send it back.
        transId = getSEQ(cmd);
        if ((transId == 0) || (transId > dev->max_outstanding_tx)) {
          DEBUGPRINT(6, (TXT ("CMD_TX_REQUEST chan %d ")
                         TXT2("ERROR transid too high %d\n"), chan, transId));
          break;
        }

        if (((MhydraChanData *)vChan->hwChanData)->current_tx_message[transId - 1].flags & VCAN_MSG_FLAG_TXRQ) {
          VCAN_EVENT *e = (VCAN_EVENT *)&((MhydraChanData *)vChan->hwChanData)->current_tx_message[transId - 1];
          e->tag = V_RECEIVE_MSG;
          e->timeStamp = timestamp_in_10us(vCard, cmd->txRequest.time);
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
      unsigned int  transId;
      unsigned int  chan = dev->he2channel[srcHE];

      VCanChanData  *vChan    = vCard->chanData[chan];
      MhydraChanData  *mhydraChan = vChan->hwChanData;
      MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

      DEBUGPRINT(4, (TXT("CMD_TX_ACKNOWLEDGE\n")));

      DEBUGOUT(ZONE_ERR, (TXT("TXACK:%d %d 0x%x %d\n"), chan,
                          getSEQ(cmd),
                          ((MhydraChanData *)vChan->hwChanData)->
                           current_tx_message[getSEQ(cmd)].id,
                          mhydraChan->outstanding_tx));

      if (chan < (unsigned)vCard->nrChannels) {
        DEBUGPRINT(4, (TXT ("CMD_TX_ACKNOWLEDGE on ch %d ")
                       TXT2("(outstanding tx = %d)\n"),
                       chan, mhydraChan->outstanding_tx));
        transId = getSEQ(cmd);
        if ((transId == 0) || (transId > dev->max_outstanding_tx)) {
          DEBUGPRINT(2, (TXT("CMD_TX_ACKNOWLEDGE chan %d ERROR transid %d\n"),
                         chan, transId));
          break;
        }

        if (((MhydraChanData *)vChan->hwChanData)->current_tx_message[transId - 1].flags & VCAN_MSG_FLAG_TXACK) {
          VCAN_EVENT *e = (VCAN_EVENT *)&((MhydraChanData *)vChan->hwChanData)->current_tx_message[transId - 1];
          e->tag = V_RECEIVE_MSG;
          e->timeStamp = timestamp_in_10us(vCard, cmd->txAck.time);

          if (!(e->tagData.msg.flags & VCAN_MSG_FLAG_ERROR_FRAME)) {
            e->timeStamp += mhydraChan->timestamp_correction_value;
          }

          e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXRQ; // qqq TXRQ???
          if (cmd->txAck.flags & MSGFLAG_NERR) {
            // A lowspeed transceiver may report NERR during TX
            e->tagData.msg.flags |= VCAN_MSG_FLAG_NERR;
            DEBUGPRINT(6, (TXT("txack flag=%x\n"), cmd->txAck.flags));
          }

          vCanDispatchEvent(vChan, e);
        }

        os_if_spin_lock(&mhydraChan->outTxLock);
        // If we have cleared the transmit queue, we might get a TX_ACK
        // that should not be counted.
        if (mhydraChan->outstanding_tx) {
          mhydraChan->outstanding_tx--;
        }

        // Outstanding are changing from *full* to at least one open slot?
        if (mhydraChan->outstanding_tx >= (dev->max_outstanding_tx - 1)) {
          os_if_spin_unlock(&mhydraChan->outTxLock);
          DEBUGPRINT(6, (TXT("Buffer in chan %d not full (%d) anymore\n"),
                         chan, mhydraChan->outstanding_tx));
          os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);
        }

        // Check if we should *wake* canwritesync
        else if ((mhydraChan->outstanding_tx == 0) && txQEmpty(vChan) &&
                 test_and_clear_bit(0, &vChan->waitEmpty)) {
          os_if_spin_unlock(&mhydraChan->outTxLock);
          os_if_wake_up_interruptible(&vChan->flushQ);
          DEBUGPRINT(6, (TXT("W%d\n"), chan));
        }
        else {
#if DEBUG
          if (mhydraChan->outstanding_tx < 4)
            DEBUGPRINT(6, (TXT("o%d ql%d we%d s%d\n"),
                           mhydraChan->outstanding_tx,
                           queue_length(&vChan->txChanQueue),
                           constant_test_bit(0, &vChan->waitEmpty),
                           dev->max_outstanding_tx));
#endif
          os_if_spin_unlock(&mhydraChan->outTxLock);
        }

        DEBUGPRINT(6, (TXT("X%d\n"), chan));
      }
      break;
    }

    case CMD_CAN_ERROR_EVENT:
    {
      int             errorCounterChanged;
      unsigned int  chan = dev->he2channel[srcHE];
      VCanChanData  *vChan    = vCard->chanData[chan];

      DEBUGPRINT(4, (TXT("CMD_CAN_ERROR_EVENT\n")));

      // It's an error frame if any of our error counters has
      // increased..
      errorCounterChanged  = (cmd->canErrorEvent.txErrorCounter >
                              vChan->chipState.txerr);
      errorCounterChanged |= (cmd->canErrorEvent.rxErrorCounter >
                              vChan->chipState.rxerr);
      // It's also an error frame if we have seen a bus error.
      errorCounterChanged |= (cmd->canErrorEvent.busStatus & M32C_BUS_ERROR);

      vChan->chipState.txerr = cmd->canErrorEvent.txErrorCounter;
      vChan->chipState.rxerr = cmd->canErrorEvent.rxErrorCounter;


      switch (cmd->canErrorEvent.busStatus & (M32C_BUS_PASSIVE | M32C_BUS_OFF)) {
        case 0:
          vChan->chipState.state = CHIPSTAT_ERROR_ACTIVE;
          break;

        case M32C_BUS_PASSIVE:
          vChan->chipState.state = CHIPSTAT_ERROR_PASSIVE |
                                  CHIPSTAT_ERROR_WARNING;
          break;

        case M32C_BUS_OFF:
          vChan->chipState.state = CHIPSTAT_BUSOFF;
          errorCounterChanged = 0;
          break;

        case (M32C_BUS_PASSIVE | M32C_BUS_OFF):
          vChan->chipState.state = CHIPSTAT_BUSOFF | CHIPSTAT_ERROR_PASSIVE |
                                  CHIPSTAT_ERROR_WARNING;
          errorCounterChanged = 0;
          break;

        default:
          break;
      }

      // Reset is treated like bus-off
      if (cmd->canErrorEvent.busStatus & M32C_BUS_RESET) {
        vChan->chipState.state = CHIPSTAT_BUSOFF;
        vChan->chipState.txerr = 0;
        vChan->chipState.rxerr = 0;
        errorCounterChanged = 0;
      }

      // Dispatch can event

      e.tag = V_CHIP_STATE;

      e.timeStamp = timestamp_in_10us(vCard, cmd->canErrorEvent.time);

      e.transId = 0;
      e.tagData.chipState.busStatus      = vChan->chipState.state;
      e.tagData.chipState.txErrorCounter = vChan->chipState.txerr;
      e.tagData.chipState.rxErrorCounter = vChan->chipState.rxerr;
      vCanDispatchEvent(vChan, &e);

      if (errorCounterChanged) {
        e.tag               = V_RECEIVE_MSG;
        e.transId           = 0;
        e.timeStamp = timestamp_in_10us(vCard, cmd->canErrorEvent.time);

        e.tagData.msg.id    = 0;
        e.tagData.msg.flags = VCAN_MSG_FLAG_ERROR_FRAME;

        if (cmd->canErrorEvent.flags & MSGFLAG_NERR) {
          // A lowspeed transceiver may report NERR during error
          // frames
          e.tagData.msg.flags |= VCAN_MSG_FLAG_NERR;
        }

        e.tagData.msg.dlc   = 0;
        vCanDispatchEvent(vChan, &e);
      }
      break;
    }

    case CMD_FLUSH_QUEUE_RESP:
        DEBUGPRINT(4, (TXT("CMD_FLUSH_QUEUE_RESP - Ignored\n")));
        break;

    case CMD_USB_THROTTLE:
      DEBUGPRINT(4, (TXT("CMD_USB_THROTTLE - Ignored\n")));
      break;

    case CMD_TREF_SOFNR:
      DEBUGPRINT(4, (TXT("CMD_TREF_SOFNR - Ignored\n")));
      break;

    case CMD_LOG_MESSAGE:
    {
      unsigned int  chan = dev->he2channel[srcHE];
      VCanChanData  *vChd = NULL;

      DEBUGPRINT(4, (TXT("CMD_LOG_MESSAGE\n")));
      if (chan < vCard->nrChannels) {
        vChd = vCard->chanData[chan];

        e.tag               = V_RECEIVE_MSG;
        e.transId           = 0;
        e.timeStamp         = timestamp_in_10us(vCard, cmd->logMessage.time);

        e.timeStamp        += ((MhydraChanData *)vChd->hwChanData)->timestamp_correction_value;
        e.tagData.msg.id    = cmd->logMessage.id;
        e.tagData.msg.flags = cmd->logMessage.flags;
        e.tagData.msg.dlc   = cmd->logMessage.dlc;

        memcpy(e.tagData.msg.data, cmd->logMessage.data, 8);

        vCanDispatchEvent(vChd, &e);
      }
      break;
    }

    case CMD_CHECK_LICENSE_RESP:
      DEBUGPRINT(4, (TXT("CMD_CHECK_LICENCE_RESP - Ignore\n")));
      break;

    case CMD_GET_TRANSCEIVER_INFO_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_TRANSCEIVER_INFO_RESP - Ignore\n")));
      break;

    case CMD_SELF_TEST_RESP:
      DEBUGPRINT(4, (TXT("CMD_SELF_TEST_RESP - Ignore\n")));
      break;

    case CMD_LED_ACTION_RESP:
      DEBUGPRINT(4, (TXT("CMD_LED_ACTION_RESP - Ignore\n")));
      break;

    case CMD_GET_IO_PORTS_RESP:
      DEBUGPRINT(4, (TXT("CMD_GET_IO_PORTS_RESP - Ignore\n")));
      break;

    case CMD_HEARTBEAT_RESP:
      DEBUGPRINT(4, (TXT("CMD_HEARTBEAT_RESP - Ignore\n")));
      break;

    case CMD_SOFTSYNC_ONOFF:
      printk("CMD_SOFTSYNC_ONOFF - Ignore\n");
      DEBUGPRINT(4, (TXT("CMD_SOFTSYNC_ONOFF - Ignore\n")));
      break;


    case CMD_SET_BUSPARAMS_RESP:
      DEBUGPRINT(4, (TXT("CMD_SET_BUSPARAMS_RESP - Ignore\n")));
      break;

    case CMD_MAP_CHANNEL_RESP:
    {
      uint8_t  he, chan;
      uint16_t transId;

      transId = getSEQ(cmd);
      if (transId > 0x7f || transId < 0x40) {
        DEBUGPRINT(4, (TXT("Error: CMD_MAP_CHANNEL_RESP, invalid transId: 0x%x\n"),
                cmd->transId));
        break;
      }

      switch (transId & 0xff0) {
      case 0x40:
        chan = transId & 0x00f;
        he   = cmd->mapChannelResp.heAddress;
        dev->channel2he[chan] = he;
        dev->he2channel[he]   = chan;
        DEBUGPRINT(4, (TXT("CMD_MAP_CHANNEL_RESP, dev->he2channel[he]: %d dev->channel2he[chan] %d\n"),
            dev->he2channel[he], dev->channel2he[chan]));
        break;

      default:
        DEBUGPRINT(4, (TXT("Warning: Ignored CMD_MAP_CHANNEL_RESP, id:0x%x\n"),
                cmd->transId));
        break;
      }
      break;
    }

    case CMD_GET_SOFTWARE_DETAILS_RESP:
    {
      DEBUGPRINT(4, (TXT("CMD_GET_SOFTWARE_DETAILS_RESP\n")));
      vCard->firmwareVersionMajor = (cmd->getSoftwareDetailsResp.swVersion >> 24) & 0xff;
      vCard->firmwareVersionMinor = (cmd->getSoftwareDetailsResp.swVersion >> 16) & 0xff;
      vCard->firmwareVersionBuild = (cmd->getSoftwareDetailsResp.swVersion) & 0xffff;

      if (cmd->getSoftwareDetailsResp.swOptions & SWOPTION_BAD_MOOD) {
        DEBUGPRINT(2, (TXT("%s: Firmware configuration error!\n"),
                       driverData.deviceName));
        vCard->card_flags |= DEVHND_CARD_REFUSE_TO_USE_CAN;
      }

      if (cmd->getSoftwareDetailsResp.swOptions & SWOPTION_BETA) {
        vCard->card_flags |= DEVHND_CARD_FIRMWARE_BETA;
      }

      if (cmd->getSoftwareDetailsResp.swOptions & SWOPTION_RC) {
        vCard->card_flags |= DEVHND_CARD_FIRMWARE_RC;
      }

      if (cmd->getSoftwareDetailsResp.swOptions & SWOPTION_AUTO_TX_BUFFER) {
        vCard->card_flags |= DEVHND_CARD_AUTO_TX_OBJBUFS;
      }

      if ((cmd->getSoftwareDetailsResp.swOptions & SWOPTION_CPU_FQ_MASK) == SWOPTION_80_MHZ_CLK) {
        dev->hires_timer_fq = 80;
      }
      else if ((cmd->getSoftwareDetailsResp.swOptions & SWOPTION_CPU_FQ_MASK) == SWOPTION_24_MHZ_CLK) {
        dev->hires_timer_fq = 24;
      }
      else {
        dev->hires_timer_fq = 1;
      }
      break;
    }

    default:
      DEBUGPRINT(2, (TXT("UNKNOWN COMMAND - %d\n"), cmd->cmdNo));
  }

  //
  // Wake up those who are waiting for a resp
  //

  os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
  list_for_each_safe(currHead, tmpHead, &dev->replyWaitList)
  {
    currNode = list_entry(currHead, WaitNode, list);
      if (currNode->cmdNr == cmd->cmdNo &&
          mhydra_get_trans_id(cmd) == (currNode->transId & SEQ_MASK)) {
        memcpy(currNode->replyPtr, cmd, HYDRA_CMD_SIZE);
        DEBUGPRINT(4, (TXT ("Match: cN->cmdNr(%d) == cmd->cmdNo(%d) && ")
                       TXT2("_get_trans_id(%d) == cN->transId(%d)\n"),
                       currNode->cmdNr, cmd->cmdNo,
                       mhydra_get_trans_id(cmd), (currNode->transId & SEQ_MASK)));
        os_if_up_sema(&currNode->waitSemaphore);
      }
#if DEBUG
      else {
        DEBUGPRINT(4, (TXT ("No match: cN->cmdNr(%d) == cmd->cmdNo(%d) && ")
                       TXT2("_get_trans_id(%d) == cN->transId(%d)\n"),
                       currNode->cmdNr, cmd->cmdNo,
                       mhydra_get_trans_id(cmd), (currNode->transId & SEQ_MASK)));
      }
#endif

  }
  os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);
} // _handle_command


//============================================================================
// _get_trans_id
//
static unsigned short mhydra_get_trans_id (hydraHostCmd *cmd)
{
  return getSEQ(cmd);
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
static void mhydra_send (void *context)
#else
static void mhydra_send (OS_IF_TASK_QUEUE_HANDLE *work)
#endif
{
  unsigned int     i;
#if USE_CONTEXT
  VCanCardData     *vCard     = (VCanCardData *)context;
  MhydraCardData     *dev       = (MhydraCardData *)vCard->hwCardData;
#else
  MhydraCardData     *dev       = container_of(work, MhydraCardData, txWork);
  VCanCardData     *vCard     = dev->vCard;
#endif
  VCanChanData     *vChan     = NULL;
  int              tx_needed  = 0;

  DEBUGPRINT(4, (TXT("dev = 0x%p  vCard = 0x%p\n"), dev, vCard));

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

      // Test if queue is empty or Mhydra has sent "queue high"qqq?
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

    if ((result = mhydra_transmit(vCard)) <= 0) {
      // The transmission failed - mark write as finished
      os_if_up_sema(&dev->write_finished);
    }

    // Wake up those who are waiting to send a cmd or msg
    // It seems rather likely that we emptied all our queues, and if not,
    // the awoken threads will go back to sleep again, anyway.
    // A better solution would be to do this inside mhydra_fill_usb_buffer,
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
// translate from CAN_MSG to hydraHostCmd
//
static void mhydra_translate_can_msg (VCanChanData *vChan,
                                    hydraHostCmd *hydra_msg,
                                    CAN_MSG *can_msg)
{

  MhydraCardData *dev = vChan->vCard->hwCardData;
  uint32_t id = can_msg->id;

  // Save a copy of the message.
  ((MhydraChanData *)vChan->hwChanData)->current_tx_message[atomic_read(&vChan->transId) - 1] = *can_msg;

  hydra_msg->txCanMessage.channel = (unsigned char)vChan->channel;
  hydra_msg->txCanMessage.transId = (unsigned char)atomic_read(&vChan->transId);

  DEBUGPRINT(5, (TXT("can mesg channel:%d transid %d\n"),
                 hydra_msg->txCanMessage.channel,
                 hydra_msg->txCanMessage.transId));

  hydra_msg->cmdNo = CMD_TX_CAN_MESSAGE;
  setDST(hydra_msg, dev->channel2he[vChan->channel]);
  setSEQ(hydra_msg, (unsigned char)atomic_read(&vChan->transId));

  hydra_msg->txCanMessage.id = id;
  hydra_msg->txCanMessage.dlc = can_msg->length;

  memcpy(&hydra_msg->txCanMessage.data, can_msg->data, 8);

  DEBUGPRINT(5, (TXT("outstanding(%d)++ id: %d\n"),
                 ((MhydraChanData *)vChan->hwChanData)->outstanding_tx, id));
  DEBUGPRINT(5, (TXT("Trans %d, jif %ld\n"),
                 hydra_msg->txCanMessage.transId, jiffies));

  hydra_msg->txCanMessage.flags = can_msg->flags & (VCAN_MSG_FLAG_TX_NOTIFY   |
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
static int mhydra_fill_usb_buffer (VCanCardData *vCard, unsigned char *buffer,
                                 int maxlen)
{
  int           cmd_bwp = 0;
  int           msg_bwp = 0;
  unsigned int  j;
  int           more_messages_to_send;
  hydraHostCmd       command;
  MhydraCardData  *dev   = (MhydraCardData *)vCard->hwCardData;
  VCanChanData  *vChan;
  int           len;
  int           queuePos;

  // Fill buffer with commands
  while (!queue_empty(&dev->txCmdQueue)) {
    hydraHostCmd   *commandPtr;
    int       len;

    queuePos = queue_front(&dev->txCmdQueue);
    if (queuePos < 0) {   // Did we actually get anything from queue?
      queue_release(&dev->txCmdQueue);
      break;
    }
    commandPtr = &dev->txCmdBuffer[queuePos];

    len = HYDRA_CMD_SIZE;

    DEBUGPRINT(5, (TXT("fill buf with cmd nr %d\n"), commandPtr->cmdNo));


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

    cpu_to_le(commandPtr);

    memcpy(&buffer[cmd_bwp], commandPtr, len);
    cmd_bwp += len;


    queue_pop(&dev->txCmdQueue);
  } // end while

  msg_bwp = cmd_bwp;

  DEBUGPRINT(5, (TXT("bwp: (%d)\n"), msg_bwp));

  // Add the messages

  // qqq G�r "kommandon" och "meddelanden" ut separat????!!

  DEBUGPRINT(5, (TXT("vCard->nrChannels: (%d)\n"), vCard->nrChannels));

  for (j = 0; j < vCard->nrChannels; j++) {

    MhydraChanData *mhydraChan;
    vChan    = (VCanChanData *)vCard->chanData[j];
    mhydraChan = vChan->hwChanData;

    if (vChan->minorNr < 0) {  // Channel not initialized?
      continue;
    }

    more_messages_to_send = 1;

    while (more_messages_to_send) {
      more_messages_to_send = !queue_empty(&vChan->txChanQueue);

      // Make sure we dont write more messages than
      // we are allowed to the mhydra
      if (!mhydra_tx_available(vChan)) {
        DEBUGPRINT(3, (TXT("Too many outstanding packets\n")));
        return msg_bwp;
      }

      if (more_messages_to_send == 0)
        break;

      // Get and translate message
      queuePos = queue_front(&vChan->txChanQueue);
      if (queuePos < 0) {   // Did we actually get anything from queue?
        queue_release(&vChan->txChanQueue);
        break;
      }
      mhydra_translate_can_msg(vChan, &command, &vChan->txChanBuffer[queuePos]);

      len = HYDRA_CMD_SIZE;

      // Any space left in the usb buffer?
      if (len > (maxlen - msg_bwp)) {
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
        queue_release(&vChan->txChanQueue);
        continue;
      }


      memcpy(&buffer[msg_bwp], &command, len);
      msg_bwp += len;
      DEBUGPRINT(5, (TXT("memcpy cmdno %d, len %d (%d)\n"),
                     command.cmdNo, len, msg_bwp));
      DEBUGPRINT(5, (TXT("x\n")));

      // qqq This is not atomic (but it should not matter)
      if ((atomic_read(&vChan->transId) + 1u) > dev->max_outstanding_tx) {
        atomic_set(&vChan->transId, 1);
      }
      else {
        atomic_inc(&vChan->transId);
      }

      // Have to be here (after all the breaks and continues)
      os_if_spin_lock(&mhydraChan->outTxLock);
      mhydraChan->outstanding_tx++;
      os_if_spin_unlock(&mhydraChan->outTxLock);

      DEBUGPRINT(5, (TXT("t mhydra, chan %d, out %d\n"),
                     j, mhydraChan->outstanding_tx));
                   
      queue_pop(&vChan->txChanQueue);
    } // while (more_messages_to_send)
  }

  return msg_bwp;
} // _fill_usb_buffer



//============================================================================
// The actual sending
//
static int mhydra_transmit (VCanCardData *vCard /*, void *cmd*/)
{
  MhydraCardData   *dev     = (MhydraCardData *)vCard->hwCardData;
  int            retval   = 0;
  int            fill     = 0;

  fill = mhydra_fill_usb_buffer(vCard, dev->write_urb->transfer_buffer,
                              MAX_PACKET_OUT);

  if (fill == 0) {
    // No data to send...
    DEBUGPRINT(5, (TXT("Couldn't send any messages\n")));
    return 0;
  }

  dev->write_urb->transfer_buffer_length = fill;

  if (!dev->present) {
    // The device was unplugged before the file was released.
    // We cannot deallocate here, it shouldn't be done from here
    return VCAN_STAT_NO_DEVICE;
  }

  retval = usb_submit_urb(dev->write_urb, GFP_KERNEL);
  if (retval) {
    DEBUGPRINT(1, (TXT("%s - failed submitting write urb, error %d"),
                   __FUNCTION__, retval));
    retval = VCAN_STAT_FAIL;
  }
  else {
    // The semaphore is released on successful transmission
    retval = sizeof(hydraHostCmd);
  }

  return retval;
} // _transmit


//============================================================================
// _map_channels
//
static int mhydra_map_channels (VCanCardData* vCard) {
  MhydraCardData  *dev = (MhydraCardData *)vCard->hwCardData;
  hydraHostCmd    cmd;
  hydraHostCmd    reply;
  uint16_t        transId;
  int             i, r;

  strcpy(cmd.mapChannelReq.name, "CAN");
  cmd.cmdNo = CMD_MAP_CHANNEL_REQ;

  memset(dev->channel2he, ILLEGAL_HE, sizeof(dev->channel2he));
  memset(dev->he2channel, 0xff, sizeof(dev->he2channel));

  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    setDST(&cmd, ROUTER_HE);
    cmd.mapChannelReq.channel = (unsigned char)i;
    transId = (0x40 | i);

    setSEQ(&cmd, transId);

    r = mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
        CMD_MAP_CHANNEL_RESP, getSEQ(&cmd));

    DEBUGPRINT(2, (TXT("Map he address 0x%02X position %d flags 0x%02X\n"),
                   reply.mapChannelResp.heAddress,
                   reply.mapChannelResp.position,
                   reply.mapChannelResp.flags));

    if (r != 0) {
      DEBUGPRINT(2, (TXT("CMD_MAP_CHANNEL_REQ - failed, chan=%d, stat=%d\n"),
           i, r));
      return r;
    }
  }

  return 0;
}



//============================================================================
// _get_card_info
//
static void mhydra_get_card_info (VCanCardData* vCard)
{
  MhydraCardData *dev = (MhydraCardData *)vCard->hwCardData;
  hydraHostCmd cmd;
  hydraHostCmd reply;

  DEBUGPRINT(3, (TXT("mhydra: mhydra_get_card_info\n")));

  memset(&cmd, 0, sizeof(cmd));
  memset(&reply, 0, sizeof(reply));

  //
  // CMD_GET_CARD_INFO_REQ
  //
  cmd.cmdNo = CMD_GET_CARD_INFO_REQ;
  setDST(&cmd, ILLEGAL_HE);

  mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
                  CMD_GET_CARD_INFO_RESP, getSEQ(&cmd));

  //
  // CMD_GET_SOFTWARE_INFO_REQ
  //
  cmd.cmdNo = CMD_GET_SOFTWARE_INFO_REQ;
  setDST(&cmd, ILLEGAL_HE);

  mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
      CMD_GET_SOFTWARE_INFO_RESP, getSEQ(&cmd));

  //
  // CMD_GET_SOFTWARE_DETAILS_REQ
  //
  cmd.cmdNo = CMD_GET_SOFTWARE_DETAILS_REQ;
  setDST(&cmd, ILLEGAL_HE);

  mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
      CMD_GET_SOFTWARE_DETAILS_RESP, getSEQ(&cmd));

  if (vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
    //
    // Optionally get the auto transmit/response buffer info
    //

    cmd.cmdNo = CMD_AUTO_TX_BUFFER_REQ;
    setDST(&cmd, dev->channel2he[0]);
    cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_GET_INFO;

    mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
                        CMD_AUTO_TX_BUFFER_RESP, getSEQ(&cmd));

  }

} // _get_card_info


//============================================================================
//  response_timeout
//  Used in mhydra_send_and_wait_reply
static void mhydra_response_timer (unsigned long voidWaitNode)
{
  WaitNode *waitNode = (WaitNode *)voidWaitNode;
  waitNode->timedOut = 1;
  os_if_up_sema(&waitNode->waitSemaphore);
} // response_timeout


//============================================================================
//  mhydra_send_and_wait_reply
//  Send a hydraHostCmd and wait for the mhydra to answer.
//
static int mhydra_send_and_wait_reply (VCanCardData *vCard, hydraHostCmd *cmd,
                                     hydraHostCmd *replyPtr, unsigned char cmdNr,
                                     unsigned char transId)
{
  MhydraCardData       *dev = vCard->hwCardData;
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

  ret = mhydra_queue_cmd(vCard, cmd, MHYDRA_Q_CMD_WAIT_TIME);
  if (ret != 0) {
    os_if_spin_lock_irqsave(&dev->replyWaitListLock, &irqFlags);
    list_del(&waitNode.list);
    os_if_spin_unlock_irqrestore(&dev->replyWaitListLock, irqFlags);
    return ret;
  }

  DEBUGPRINT(5, (TXT("b4 init timer\n")));
  init_timer(&waitTimer);
  waitTimer.function  = mhydra_response_timer;
  waitTimer.data      = (unsigned long)&waitNode;
  waitTimer.expires   = jiffies + msecs_to_jiffies(MHYDRA_CMD_RESP_WAIT_TIME);
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
//  mhydra_queue_cmd
//  Put the command in the command queue
//
// The unrolled sleep is used to catch a missing position in the queue
// qqq Protect the filling of the buffer with a semaphore
static int mhydra_queue_cmd (VCanCardData *vCard, hydraHostCmd *cmd,
                           unsigned int timeout)
{
  hydraHostCmd *bufCmdPtr = NULL;
  MhydraCardData *dev  = (MhydraCardData *)vCard->hwCardData;
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
  bufCmdPtr = (hydraHostCmd *)&dev->txCmdBuffer[queuePos];

  memcpy(bufCmdPtr, cmd, HYDRA_CMD_SIZE);
  queue_push(&dev->txCmdQueue);

  // Wake up the tx-thread
  os_if_queue_task_not_default_queue(dev->txTaskQ, &dev->txWork);

  return VCAN_STAT_OK;
} // _queue_cmd


//============================================================================
//  mhydra_plugin
//
//  Called by the usb core when a new device is connected that it thinks
//  this driver might be interested in.
//  Also allocates card info struct mem space and starts workqueues
//
static int mhydra_plugin (struct usb_interface *interface,
                        const struct usb_device_id *id)
{
  struct usb_device               *udev = interface_to_usbdev(interface);
  struct usb_host_interface       *iface_desc;
  struct usb_endpoint_descriptor  *endpoint;
  size_t                          buffer_size;
  unsigned int                    i;
  int                             retval = -ENOMEM;
  VCanCardData                    *vCard;
  MhydraCardData                    *dev;

  DEBUGPRINT(3, (TXT("mhydra: _plugin\n")));

  // See if the device offered us matches what we can accept
  // Add here for more devices
  if (
      (udev->descriptor.idVendor != KVASER_VENDOR_ID) ||
      (
       (udev->descriptor.idProduct != USB_EAGLE_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_BLACKBIRD_V2_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_MEMO_PRO_5HS_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_USBCAN_PRO_5HS_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_USBCAN_LIGHT_4HS_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_LEAF_PRO_HS_V2_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_USBCAN_PRO_2HS_V2_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_MEMO_2HS_PRODUCT_ID) &&
       (udev->descriptor.idProduct != USB_MEMO_PRO_2HS_V2_PRODUCT_ID)
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
  switch (udev->descriptor.idProduct) {
    case USB_EAGLE_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Eagle plugged in\n")));
      break;
    case USB_BLACKBIRD_V2_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("BlackBird v2 plugged in\n")));
      break;
    case USB_MEMO_PRO_5HS_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Memorator Pro 5xHS plugged in\n")));
      break;
    case USB_USBCAN_PRO_5HS_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("USBcan Pro 5xHS plugged in\n")));
      break;
    case USB_USBCAN_LIGHT_4HS_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("USBcan Light 4xHS (00831-1) plugged in\n")));
      break;
    case USB_LEAF_PRO_HS_V2_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Leaf Pro HS v2 (00843-4) plugged in\n")));
      break;
    case USB_USBCAN_PRO_2HS_V2_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("USBcan Pro 2xHS v2 (00752-9) plugged in\n")));
      break;
    case USB_MEMO_2HS_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Memorator 2xHS v2 (00821-2) plugged in\n")));
      break;
    case USB_MEMO_PRO_2HS_V2_PRODUCT_ID:
      DEBUGPRINT(2, (TXT("\nKVASER ")));
      DEBUGPRINT(2, (TXT("Memorator Pro 2xHS v2 (00819-9) plugged in\n")));
      break;

    default:
      DEBUGPRINT(2, (TXT("UNKNOWN product plugged in\n")));
      break;
  }
#endif

  // Allocate datastructures for the card
  if (mhydra_allocate(&vCard) != VCAN_STAT_OK) {
    // Allocation failed
    return -ENOMEM;
  }

  dev = vCard->hwCardData;
  os_if_init_sema(&((MhydraCardData *)vCard->hwCardData)->sem);
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
                        mhydra_write_bulk_callback, vCard);
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
  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    VCanChanData   *vChd = vCard->chanData[i];
    vChd->channel  = i;
  }


  // Start up vital stuff
  mhydra_start(vCard);

  // Let the user know what node this device is now attached to
  DEBUGPRINT(2, (TXT("------------------------------\n")));
  DEBUGPRINT(2, (TXT("Mhydra device %d now attached\n"),
                 driverData.noOfDevices));
  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    DEBUGPRINT(2, (TXT("With minor number %d \n"), vCard->chanData[i]->minorNr));
  }
  DEBUGPRINT(2, (TXT("using driver built %s\n"), TXT2(__TIME__)));
  DEBUGPRINT(2, (TXT("on %s\n"), TXT2(__DATE__)));
  DEBUGPRINT(2, (TXT("------------------------------\n")));

  return 0;

error:
  DEBUGPRINT(2, (TXT("_deallocate from mhydra_plugin\n")));
  mhydra_deallocate(vCard);

  return retval;
} // mhydra_plugin



//========================================================================
//
// Init stuff, called from end of _plugin
//
static void mhydra_start (VCanCardData *vCard)
{
  MhydraCardData *dev = (MhydraCardData *)vCard->hwCardData;
  OS_IF_THREAD rx_thread;
  unsigned int i;

  DEBUGPRINT(3, (TXT("mhydra: _start\n")));

  // Initialize queues/waitlists for commands
  os_if_spin_lock_init(&dev->replyWaitListLock);

  INIT_LIST_HEAD(&dev->replyWaitList);
  queue_init(&dev->txCmdQueue, KV_MHYDRA_TX_CMD_BUF_SIZE);

  // Set spinlocks for the outstanding tx
  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    VCanChanData  *vChd     = vCard->chanData[i];
    MhydraChanData  *mhydraChan = vChd->hwChanData;
    os_if_spin_lock_init(&mhydraChan->outTxLock);
  }

  os_if_init_sema(&dev->write_finished);
  os_if_up_sema(&dev->write_finished);

  os_if_init_task(&dev->txWork, &mhydra_send, vCard);
  dev->txTaskQ = os_if_declare_task("mhydra_tx", &dev->txWork);
  
  rx_thread = os_if_kernel_thread(mhydra_rx_thread, vCard);

  mhydra_map_channels(vCard);

  mhydra_get_card_info(vCard);
  
  if (vCard) {
    DEBUGPRINT(2, (TXT("vCard chnr: %d\n"), vCard->nrChannels));
  }
  else {
    DEBUGPRINT(2, (TXT("vCard is NULL\n")));
  }

  vCard->driverData = &driverData;
  vCanInitData(vCard);

} // _start


//========================================================================
//
// Allocates space for card structs
//
static int mhydra_allocate (VCanCardData **in_vCard)
{
  // Helper struct for allocation
  typedef struct {
    VCanChanData  *dataPtrArray[HYDRA_MAX_CHANNELS];
    VCanChanData  vChd[HYDRA_MAX_CHANNELS];
    MhydraChanData  hChd[HYDRA_MAX_CHANNELS];
  } ChanHelperStruct;

  int              chNr;
  ChanHelperStruct *chs;
  VCanCardData     *vCard;

  DEBUGPRINT(3, (TXT("mhydra: _allocate\n")));

  // Allocate data area for this card
  vCard = os_if_kernel_malloc(sizeof(VCanCardData) + sizeof(MhydraCardData));
  DEBUGPRINT(2, (TXT("MALLOC _allocate\n")));
  if (!vCard) {
    DEBUGPRINT(1, (TXT("alloc error\n")));
    goto card_alloc_err;
  }
  memset(vCard, 0, sizeof(VCanCardData) + sizeof(MhydraCardData));

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
  for (chNr = 0; chNr < HYDRA_MAX_CHANNELS; chNr++) {
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
// mhydra_deallocate
//
static void mhydra_deallocate (VCanCardData *vCard)
{
  MhydraCardData *dev = (MhydraCardData *)vCard->hwCardData;
  VCanCardData *local_vCard;
  int i;

  DEBUGPRINT(3, (TXT("mhydra: _deallocate\n")));

  // Make sure all workqueues are finished
  //flush_workqueue(&dev->txTaskQ);

  if (dev->bulk_in_buffer != NULL) {
    os_if_kernel_free(dev->bulk_in_buffer);
    DEBUGPRINT(2, (TXT("Free bulk_in_buffer\n")));
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
      DEBUGPRINT(1, (TXT("Error: Bad vCard in mhydra_dealloc()\n")));
    }
  }

  os_if_spin_unlock(&driverData.canCardsLock);

  for(i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    VCanChanData *vChd     = vCard->chanData[i];
    MhydraChanData *mhydraChan = vChd->hwChanData;
    if (mhydraChan->objbufs) {
      DEBUGPRINT(2, (TXT("Free vCard->chanData[i]->hwChanData->objbufs[%d]\n"), i));
      os_if_kernel_free(mhydraChan->objbufs);
      mhydraChan->objbufs = NULL;
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
//     mhydra_remove
//
//     Called by the usb core when the device is removed from the system.
//
//     This routine guarantees that the driver will not submit any more urbs
//     by clearing dev->udev.  It is also supposed to terminate any currently
//     active urbs.  Unfortunately, usb_bulk_msg(), does not provide any way
//     to do this.  But at least we can cancel an active write.
//
static void mhydra_remove (struct usb_interface *interface)
{
  VCanCardData *vCard;
  VCanChanData *vChan;
  MhydraCardData *dev;
  unsigned int i;

  DEBUGPRINT(3, (TXT("mhydra: _remove\n")));

  vCard = usb_get_intfdata(interface);
  usb_set_intfdata(interface, NULL);


  dev = vCard->hwCardData;

  // Prevent device read, write and ioctl
  // Needs to be done here, or some commands will seem to
  // work even though the device is no longer present.
  dev->present = 0;

  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    vChan = vCard->chanData[i];
    DEBUGPRINT(3, (TXT("Waiting for all closed on minor %d\n"), vChan->minorNr));
    while (atomic_read(&vChan->fileOpenCount) > 0) {
      os_if_set_task_uninterruptible ();
      os_if_wait_for_event_timeout_simple(10);
    }
  }

  // Terminate workqueues
  flush_scheduled_work();


  // Terminate an ongoing write
  DEBUGPRINT(6, (TXT("Ongoing write terminated\n")));
  USB_KILL_URB(dev->write_urb);
  DEBUGPRINT(6, (TXT("Unlinking urb\n")));
  os_if_down_sema(&dev->write_finished);

  // Remove spin locks
  for (i = 0; i < HYDRA_MAX_CHANNELS; i++) {
    VCanChanData   *vChd     = vCard->chanData[i];
    MhydraChanData   *mhydraChan = vChd->hwChanData;
    os_if_spin_lock_remove(&mhydraChan->outTxLock);
  }

  // Flush and destroy tx workqueue
  DEBUGPRINT(2, (TXT("destroy_workqueue\n")));
  os_if_destroy_task(dev->txTaskQ);

  os_if_delete_sema(&dev->write_finished);
  os_if_spin_lock_remove(&dev->replyWaitListLock);

  driverData.noOfDevices -= vCard->nrChannels;

  // Deallocate datastructures
  mhydra_deallocate(vCard);

  DEBUGPRINT(2, (TXT("Mhydra device removed. Mhydra devices present (%d)\n"),
                 driverData.noOfDevices));
} // _remove



//======================================================================
//
// Set bit timing
//
static int mhydra_set_busparams (VCanChanData *vChan, VCanBusParams *par)
{
  hydraHostCmd    cmd, reply;
  uint32_t        tmp, PScl;
  int             ret;

  MhydraCardData *dev = vChan->vCard->hwCardData;


  DEBUGPRINT(4, (TXT("mhydra: _set_busparam\n")));
  memset(&cmd, 0, sizeof(cmd));

  cmd.cmdNo = CMD_SET_BUSPARAMS_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  cmd.setBusparamsReq.bitRate = par->freq;
  cmd.setBusparamsReq.sjw     = (unsigned char)par->sjw;
  cmd.setBusparamsReq.tseg1   = (unsigned char)par->tseg1;
  cmd.setBusparamsReq.tseg2   = (unsigned char)par->tseg2;
  cmd.setBusparamsReq.noSamp  = 1; // qqq Can't be trusted: (BYTE) pi->chip_param.samp3

  // Check bus parameters
  tmp = par->freq * (par->tseg1 + par->tseg2 + 1);
  if (tmp == 0) {
    DEBUGPRINT(1, (TXT("mhydra: _set_busparams() tmp == 0!\n")));
    return VCAN_STAT_BAD_PARAMETER;
  }

  PScl = 16000000UL / tmp;

  // QQQ m32firm will silently make the prescaler even for compatibility with
  // its mcp2515 and old products... Had to put it there instead of here
  // because we send wanted bitrate not actual prescaler.
  if (PScl <= 1 || PScl > 256) {
    DEBUGPRINT(1, (TXT("%s: hwif_set_chip_param() prescaler wrong (%d)\n"),
                   driverData.deviceName, PScl & 1 /* even */));
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(5, (TXT ("mhydra_set_busparams: Chan(%d): Freq (%d) SJW (%d) TSEG1 (%d) TSEG2 (%d) ")
                 TXT2("Samp (%d)\n"),
                 vChan->channel,
                 cmd.setBusparamsReq.bitRate,
                 cmd.setBusparamsReq.sjw,
                 cmd.setBusparamsReq.tseg1,
                 cmd.setBusparamsReq.tseg2,
                 cmd.setBusparamsReq.noSamp));

  ret = mhydra_send_and_wait_reply(vChan->vCard, (hydraHostCmd *)&cmd, &reply,
                                  CMD_SET_BUSPARAMS_RESP, getSEQ(&cmd));

//  ret = mhydra_queue_cmd(vChan->vCard, &cmd, 5 /* There is no response */);

  return ret;
} // _set_busparams



//======================================================================
//
//  Get bit timing
//  GetBusParams doesn't return any values.
//
static int mhydra_get_busparams (VCanChanData *vChan, VCanBusParams *par)
{
  int ret;
  hydraHostCmd  cmd, reply;
  MhydraCardData *dev = vChan->vCard->hwCardData;

  DEBUGPRINT(3, (TXT("mhydra: _get_busparam\n")));
  memset(&cmd, 0, sizeof(cmd));

  cmd.cmdNo = CMD_GET_BUSPARAMS_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  ret = mhydra_send_and_wait_reply(vChan->vCard, (hydraHostCmd *)&cmd, &reply,
                                  CMD_GET_BUSPARAMS_RESP, getSEQ(&cmd));

  if (ret == VCAN_STAT_OK) {
    DEBUGPRINT(5, (TXT ("Chan(%d): Freq (%d) SJW (%d) TSEG1 (%d) TSEG2 (%d)\n"),
                 vChan->channel,
                 reply.getBusparamsResp.bitRate,
                 reply.getBusparamsResp.sjw,
                 reply.getBusparamsResp.tseg1,
                 reply.getBusparamsResp.tseg2));

  }
  else {
    DEBUGPRINT(3, (TXT("mhydra_get_busparam - failed %d\n"),ret));
  }

  par->freq   = reply.getBusparamsResp.bitRate;
  par->sjw    = reply.getBusparamsResp.sjw;
  par->tseg1  = reply.getBusparamsResp.tseg1;
  par->tseg2  = reply.getBusparamsResp.tseg2;
  par->samp3  = reply.getBusparamsResp.noSamp;

  return ret;

} // _get_busparams


//======================================================================
//
//  Set silent or normal mode
//
static int mhydra_set_silent (VCanChanData *vChan, int silent)
{
  hydraHostCmd cmd;
  int ret;
  MhydraCardData *dev = vChan->vCard->hwCardData;

  DEBUGPRINT(3, (TXT("mhydra: _set_silent %d\n"),silent));

  cmd.cmdNo = CMD_SET_DRIVERMODE_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);

  cmd.setDrivermodeReq.driverMode = silent ? DRIVERMODE_SILENT :
                                             DRIVERMODE_NORMAL;

  ret = mhydra_queue_cmd(vChan->vCard, &cmd, 5 /* There is no response */);

  return ret;
} // _set_silent


//======================================================================
//
//  Line mode
//
static int mhydra_set_trans_type (VCanChanData *vChan, int linemode, int resnet)
{
  // qqq Not implemented
  DEBUGPRINT(3, (TXT("mhydra: _set_trans_type is NOT implemented!\n")));

  return VCAN_STAT_OK;
} // _set_trans_type




//======================================================================
//
//  Query chip status
//
static int mhydra_get_chipstate (VCanChanData *vChan)
{
  VCanCardData    *vCard = vChan->vCard;
  MhydraCardData  *dev = vChan->vCard->hwCardData;
  //VCAN_EVENT msg;
  hydraHostCmd    cmd;
  hydraHostCmd    reply;
  int             ret;

  DEBUGPRINT(3, (TXT("mhydra: _get_chipstate\n")));

  cmd.cmdNo = CMD_GET_CHIP_STATE_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  ret = mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
                                 CMD_CHIP_STATE_EVENT,
                                 getSEQ(&cmd));

  return ret;
} // _get_chipstate



//======================================================================
//
//  Go bus on
//
static int mhydra_bus_on (VCanChanData *vChan)
{
  VCanCardData  *vCard    = vChan->vCard;
  MhydraChanData  *mhydraChan = vChan->hwChanData;
  MhydraCardData *dev = vChan->vCard->hwCardData;
  hydraHostCmd cmd;
  hydraHostCmd reply;
  int     ret;

  DEBUGPRINT(3, (TXT("mhydra: _bus_on\n")));

  memset(((MhydraChanData *)vChan->hwChanData)->current_tx_message, 0, sizeof(((MhydraChanData *)vChan->hwChanData)->current_tx_message));
  atomic_set(&vChan->transId, 1);
  os_if_spin_lock(&mhydraChan->outTxLock);
  mhydraChan->outstanding_tx = 0;
  os_if_spin_unlock(&mhydraChan->outTxLock);

  cmd.cmdNo = CMD_START_CHIP_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  ret = mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
      CMD_CHIP_STATE_EVENT, getSEQ(&cmd));

  if (ret == VCAN_STAT_OK) {
    vChan->isOnBus = 1;

    mhydra_get_chipstate(vChan);
  }

  return ret;
} // _bus_on


//======================================================================
//
//  Go bus off
//
static int mhydra_bus_off (VCanChanData *vChan)
{
  VCanCardData *vCard    = vChan->vCard;
  MhydraChanData *mhydraChan = vChan->hwChanData;
  MhydraCardData *dev = vChan->vCard->hwCardData;

  hydraHostCmd cmd;
  hydraHostCmd reply;
  int     ret;

  DEBUGPRINT(3, (TXT("mhydra: _bus_off\n")));

  cmd.cmdNo = CMD_STOP_CHIP_REQ;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  ret = mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
      CMD_CHIP_STATE_EVENT, getSEQ(&cmd));

  if (ret == VCAN_STAT_OK) {
    mhydra_get_chipstate(vChan);

    DEBUGPRINT(5, (TXT("bus off channel %d\n"), vChan->channel));

    vChan->isOnBus = 0;
    vChan->chipState.state = CHIPSTAT_BUSOFF;
    memset(mhydraChan->current_tx_message, 0, sizeof(mhydraChan->current_tx_message));

    os_if_spin_lock(&mhydraChan->outTxLock);
    mhydraChan->outstanding_tx = 0;
    os_if_spin_unlock(&mhydraChan->outTxLock);

    atomic_set(&vChan->transId, 1);
  }

  return ret;
} // _bus_off



//======================================================================
//
//  Clear send buffer on card
//
static int mhydra_flush_tx_buffer (VCanChanData *vChan)
{
  MhydraChanData  *mhydraChan = vChan->hwChanData;
  MhydraCardData  *dev      = vChan->vCard->hwCardData;
  hydraHostCmd    cmd, reply;
  int     ret;

  DEBUGPRINT(3, (TXT("mhydra: _flush_tx_buffer - %d\n"), vChan->channel));
  
  cmd.cmdNo         = CMD_FLUSH_QUEUE;
  setDST(&cmd, dev->channel2he[vChan->channel]);
  setSEQ(&cmd, (unsigned char)atomic_read(&vChan->transId));

  cmd.flushQueue.flags   = 0;

  // We _must_ clear the queue before outstanding_tx!
  // Otherwise, the transmit code could be in the process of sending
  // a message from the queue, increasing outstanding_tx after our clear.
  // With a cleared queue, the transmit code will not be doing anything.
  // qqq Consider a different handle being used to transmit.
  queue_reinit(&vChan->txChanQueue);
  os_if_spin_lock(&mhydraChan->outTxLock);
  mhydraChan->outstanding_tx = 0;
  os_if_spin_unlock(&mhydraChan->outTxLock);

//  ret = mhydra_queue_cmd(vChan->vCard, &cmd, 5 /* There is no response */);

  ret = mhydra_send_and_wait_reply(vChan->vCard, (hydraHostCmd *)&cmd, &reply,
                                  CMD_FLUSH_QUEUE_RESP, getSEQ(&cmd));


  if (ret == VCAN_STAT_OK) {
    atomic_set(&vChan->transId, 1);
    // Do this once more, for good measure.
    queue_reinit(&vChan->txChanQueue);
    os_if_spin_lock(&mhydraChan->outTxLock);
    mhydraChan->outstanding_tx = 0;
    os_if_spin_unlock(&mhydraChan->outTxLock);
  }
  
  return ret;
} // _flush_tx_buffer


//======================================================================
//
// Request send
//
static void mhydra_schedule_send (VCanCardData *vCard, VCanChanData *vChan)
{
  MhydraCardData *dev = vCard->hwCardData;

  DEBUGPRINT(3, (TXT("mhydra: _schedule_send\n")));

  if (mhydra_tx_available(vChan) && dev->present) {
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
static int mhydra_get_tx_err (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("mhydra: _get_tx_err\n")));

  mhydra_get_chipstate(vChan);

  return vChan->chipState.txerr;
  //return vChan->txErrorCounter;
} //_get_tx_err


//======================================================================
//  Read transmit error counter
//
static int mhydra_get_rx_err (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("mhydra: _get_rx_err\n")));

  mhydra_get_chipstate(vChan);

  return vChan->chipState.rxerr;
  //return vChan->rxErrorCounter;
} // _get_rx_err


//======================================================================
//  Read receive queue length in hardware/firmware
//
static unsigned long mhydra_get_hw_rx_q_len (VCanChanData *vChan)
{
  DEBUGPRINT(3, (TXT("mhydra: _get_hw_rx_q_len\n")));

  // qqq Why _tx_ channel buffer?
  return queue_length(&vChan->txChanQueue);
} // _get_hw_rx_q_len


//======================================================================
//  Read transmit queue length in hardware/firmware
//
static unsigned long mhydra_get_hw_tx_q_len (VCanChanData *vChan)
{
  MhydraChanData *hChd  = vChan->hwChanData;
  unsigned int res;

  DEBUGPRINT(3, (TXT("mhydra: _get_hw_tx_q_len\n")));

  os_if_spin_lock(&hChd->outTxLock);
  res = hChd->outstanding_tx;
  os_if_spin_unlock(&hChd->outTxLock);

  return res;
} // _get_hw_tx_q_len





//======================================================================
//
// Run when driver is loaded
//
static int INIT mhydra_init_driver (void)
{
  int result;

  DEBUGPRINT(3, (TXT("mhydra: _init_driver\n")));

  driverData.deviceName = DEVICE_NAME_STRING;

  // Register this driver with the USB subsystem
  result = usb_register(&mhydra_driver);
  if (result) {
    DEBUGPRINT(1, (TXT("mhydra: usb_register failed. Error number %d"),
                   result));
    return result;
  }

  return 0;
} // _init_driver



//======================================================================
// Run when driver is unloaded
//
static int EXIT mhydra_close_all (void)
{
  DEBUGPRINT(2, (TXT("mhydra: _close_all (deregister driver..)\n")));
  usb_deregister(&mhydra_driver);

  return 0;
} // _close_all



//======================================================================
// proc read function
//
static int mhydra_proc_read (struct seq_file* m, void* v)
{
  int            channel  = 0;
  VCanCardData  *cardData = driverData.canCards;

  seq_printf(m,"\ntotal channels %d\n", driverData.noOfDevices);
  seq_puts(m,"minor numbers");
  while (NULL != cardData) {
    for (channel = 0; channel < cardData->nrChannels; channel++) {
        seq_printf(m," %d", cardData->chanData[channel]->minorNr);
    }
    cardData = cardData->next;
  }
  seq_puts(m, "\n");

  return 0;
} // _proc_read


//======================================================================
//  Can we send now?
//
static int mhydra_tx_available (VCanChanData *vChan)
{
  MhydraChanData     *mhydraChan = vChan->hwChanData;
  VCanCardData     *vCard    = vChan->vCard;
  MhydraCardData     *dev      = vCard->hwCardData;
  unsigned int     res;

  DEBUGPRINT(3, (TXT("mhydra: _tx_available %d (%d)!\n"),
                 mhydraChan->outstanding_tx, dev->max_outstanding_tx));

  os_if_spin_lock(&mhydraChan->outTxLock);
  res = mhydraChan->outstanding_tx;
  os_if_spin_unlock(&mhydraChan->outTxLock);

  return (res < dev->max_outstanding_tx);
} // _tx_available


//======================================================================
//  Are all sent msg's received?
//
static int mhydra_outstanding_sync (VCanChanData *vChan)
{
  MhydraChanData     *mhydraChan  = vChan->hwChanData;
  unsigned int     res;

  DEBUGPRINT(3, (TXT("mhydra: _outstanding_sync (%d)\n"),
                 mhydraChan->outstanding_tx));

  os_if_spin_lock(&mhydraChan->outTxLock);
  res = mhydraChan->outstanding_tx;
  os_if_spin_unlock(&mhydraChan->outTxLock);

  return (res == 0);
} // _outstanding_sync



//======================================================================
// Get time
//
 static int mhydra_get_time (VCanCardData *vCard, unsigned long *time)
{
  hydraHostCmd cmd;
  hydraHostCmd reply;
  int ret;

  DEBUGPRINT(3, (TXT("mhydra: _get_time\n")));

  memset(&cmd, 0, sizeof(cmd));
  cmd.cmdNo = CMD_READ_CLOCK_REQ;
  setDST(&cmd, ILLEGAL_HE);

  cmd.readClockReq.flags   = 0;

  // CMD_READ_CLOCK_RESP seem to always return 0 as transid
  ret = mhydra_send_and_wait_reply(vCard, (hydraHostCmd *)&cmd, &reply,
                                 CMD_READ_CLOCK_RESP, getSEQ(&cmd));
  if (ret == VCAN_STAT_OK) {
    *time = timestamp_in_10us(vCard, reply.readClockResp.time);
  }

  return ret;
}


/***************************************************************************/
/* Free an object buffer (or free all) */
static int mhydra_objbuf_free (VCanChanData *chd, int bufType, int bufNo)
{
  int ret;
  hydraHostCmd cmd;
  int start, stop, i;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    if (bufNo == -1) {
      // All buffers.. cleanup in progress, so we are happy.
      return VCAN_STAT_OK;
    }
    // Tried to free a nonexistent buffer; this is an error.
    return VCAN_STAT_BAD_PARAMETER;
  }
  if (!mhydraChan->objbufs) {
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
    mhydraChan->objbufs[i].in_use = 0;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
    setDST(&cmd, dev->channel2he[chd->channel]);

    cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_DEACTIVATE;
    cmd.autoTxBufferReq.bufNo       = (unsigned char)i;

    ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);
    if (ret != VCAN_STAT_OK) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  return VCAN_STAT_OK;
}


/***************************************************************************/
/* Allocate an object buffer */
static int mhydra_objbuf_alloc (VCanChanData *chd, int bufType, int *bufNo)
{
  int i;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (dev->autoTxBufferCount == 0 || dev->autoTxBufferResolution == 0) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_alloc\n")));

  if (!mhydraChan->objbufs) {
    DEBUGPRINT(4, (TXT("Allocating mhydraChan->objbufs[]\n")));
    mhydraChan->objbufs = os_if_kernel_malloc(sizeof(OBJECT_BUFFER) * dev->autoTxBufferCount);
    if (!mhydraChan->objbufs) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  for (i = 0; i < dev->autoTxBufferCount; i++) {
    if (!mhydraChan->objbufs[i].in_use) {
      mhydraChan->objbufs[i].in_use = 1;
      *bufNo = i;
      return VCAN_STAT_OK;
    }
  }

  return VCAN_STAT_NO_MEMORY;
}


/***************************************************************************/
/* Write data to an object buffer */
static int mhydra_objbuf_write (VCanChanData *chd, int bufType, int bufNo,
                              int id, int flags, int dlc, unsigned char *data)
{
  int ret;
  hydraHostCmd cmd;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs[bufNo].in_use) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_write, id=0x%x flags=0x%x dlc=%d\n"),
                 id, flags, dlc));

  mhydraChan->objbufs[bufNo].msg.id     = id;
  mhydraChan->objbufs[bufNo].msg.flags  = (unsigned char)flags;
  mhydraChan->objbufs[bufNo].msg.length = (unsigned char)dlc;
  memcpy(mhydraChan->objbufs[bufNo].msg.data, data, (dlc > 8) ? 8 : dlc);

  memset(&cmd, 0, sizeof(cmd));

  cmd.cmdNo = CMD_AUTO_TX_BUFFER_REQ;
  setDST(&cmd, dev->channel2he[chd->channel]);

  cmd.autoTxBufferReq.bufNo   = (unsigned char)bufNo;
  cmd.autoTxBufferReq.flags = 0;

  if (id & EXT_MSG) {
    cmd.autoTxBufferReq.flags        |= AUTOTXBUFFER_MSG_EXT;
  }

  cmd.autoTxBufferReq.id = id;
  cmd.autoTxBufferReq.dlc = dlc & 0x0F;
  cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_SET_BUFFER;
  memcpy(&cmd.autoTxBufferReq.data, data, 8);

  if (flags & MSGFLAG_REMOTE_FRAME) {
    cmd.autoTxBufferReq.flags |= AUTOTXBUFFER_MSG_REMOTE_FRAME;
  }

  ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}


/***************************************************************************/
/* Set filters on an object buffer */
static int mhydra_objbuf_set_filter (VCanChanData *chd, int bufType, int bufNo,
                                   int code, int mask)
{
  MhydraChanData  *mhydraChan = chd->hwChanData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_set_filter\n")));
  // This operation is irrelevant, so we fail.

  return VCAN_STAT_BAD_PARAMETER;
}


/***************************************************************************/
/* Set flags on an object buffer */
static int mhydra_objbuf_set_flags (VCanChanData *chd, int bufType, int bufNo,
                                  int flags)
{
  MhydraChanData  *mhydraChan = chd->hwChanData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_set_flags\n")));
  // This operation is irrelevant.

  return VCAN_STAT_BAD_PARAMETER;
}


/***************************************************************************/
/* Enable/disable an object buffer (or enable/disable all) */
static int mhydra_objbuf_enable (VCanChanData *chd, int bufType, int bufNo,
                               int enable)
{
  int ret;
  hydraHostCmd cmd;
  int start, stop, i;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_enable\n")));

  if (bufNo == -1) {
    start = 0;
    stop  = dev->autoTxBufferCount;
  } else {
    start = bufNo;
    stop  = bufNo + 1;
    if (!mhydraChan->objbufs[start].in_use) {
      return VCAN_STAT_BAD_PARAMETER;
    }
  }

  for (i = start; i < stop; i++) {
    mhydraChan->objbufs[i].active = enable;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmdNo       = CMD_AUTO_TX_BUFFER_REQ;
    setDST(&cmd, dev->channel2he[chd->channel]);

    cmd.autoTxBufferReq.requestType = enable ? AUTOTXBUFFER_CMD_ACTIVATE :
                                               AUTOTXBUFFER_CMD_DEACTIVATE;
    cmd.autoTxBufferReq.bufNo       = (unsigned char)i;

    ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);
    if (ret != VCAN_STAT_OK) {
      return VCAN_STAT_NO_MEMORY;
    }
  }

  return VCAN_STAT_OK;
}


/***************************************************************************/
/* Set the transmission interval (in microseconds) for an object buffer */
static int mhydra_objbuf_set_period (VCanChanData *chd, int bufType, int bufNo,
                                   int period)
{
  int ret;
  hydraHostCmd cmd;
  unsigned int interval;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs[bufNo].in_use) {
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

  mhydraChan->objbufs[bufNo].period = period;

  memset(&cmd, 0, sizeof(cmd));
  cmd.cmdNo = CMD_AUTO_TX_BUFFER_REQ;
  setDST(&cmd, dev->channel2he[chd->channel]);

  cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_SET_INTERVAL;
  cmd.autoTxBufferReq.bufNo       = (unsigned char)bufNo;
  cmd.autoTxBufferReq.interval    = interval;

  ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}


/***************************************************************************/
/* Set the message count for an object buffer */
static int mhydra_objbuf_set_msg_count (VCanChanData *chd, int bufType, int bufNo,
                                      int count)
{
  int ret;
  hydraHostCmd cmd;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs[bufNo].in_use) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if ((unsigned)count > 0xffff) {
    return VCAN_STAT_BAD_PARAMETER;
  }
  
  memset(&cmd, 0, sizeof(cmd));
  cmd.cmdNo = CMD_AUTO_TX_BUFFER_REQ;
  setDST(&cmd, dev->channel2he[chd->channel]);

  cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_SET_MSG_COUNT;
  cmd.autoTxBufferReq.bufNo       = (unsigned char)bufNo;
  cmd.autoTxBufferReq.interval    = (unsigned short)count;

  ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}


/***************************************************************************/
static int mhydra_objbuf_exists (VCanChanData *chd, int bufType, int bufNo)
{
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return 0;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return 0;
  }

  if (!mhydraChan->objbufs) {
    return 0;
  }

  if (!mhydraChan->objbufs[bufNo].in_use) {
    return 0;
  }

  return 1;
}


/***************************************************************************/
static int mhydra_objbuf_send_burst (VCanChanData *chd, int bufType, int bufNo,
                                   int burstLen)
{
  int ret;
  hydraHostCmd cmd;
  MhydraChanData  *mhydraChan = chd->hwChanData;
  VCanCardData  *vCard    = chd->vCard;
  MhydraCardData  *dev      = (MhydraCardData *)vCard->hwCardData;

  if (bufType != OBJBUF_TYPE_PERIODIC_TX) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (bufNo < 0 || bufNo >= dev->autoTxBufferCount) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (!mhydraChan->objbufs[bufNo].in_use) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  if (dev->autoTxBufferCount == 0 || dev->autoTxBufferResolution == 0) {
    return VCAN_STAT_BAD_PARAMETER;
  }

  DEBUGPRINT(4, (TXT("hwif_objbuf_send_burst len=%d \n"), burstLen));

  memset(&cmd, 0, sizeof(cmd));
  cmd.cmdNo = CMD_AUTO_TX_BUFFER_REQ;
  setDST(&cmd, dev->channel2he[chd->channel]);

  cmd.autoTxBufferReq.requestType = AUTOTXBUFFER_CMD_GENERATE_BURST;
  cmd.autoTxBufferReq.bufNo       = (unsigned char)bufNo;
  cmd.autoTxBufferReq.interval    = burstLen;

  ret = mhydra_queue_cmd(vCard, &cmd, MHYDRA_Q_CMD_WAIT_TIME);

  return (ret != VCAN_STAT_OK) ? VCAN_STAT_NO_MEMORY : VCAN_STAT_OK;
}




INIT int init_module (void)
{
  driverData.hwIf = &hwIf;
  return vCanInit (&driverData, HYDRA_MAX_CHANNELS);
}

EXIT void cleanup_module (void)
{
  vCanCleanup (&driverData);
}
