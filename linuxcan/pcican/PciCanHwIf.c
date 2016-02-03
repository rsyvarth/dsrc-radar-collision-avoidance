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
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------

//
// Kvaser CAN driver PCIcan hardware specific parts
// PCIcan functions
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
#include <asm/io.h>
#include <linux/seq_file.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#   include <asm/system.h>
#endif
#include <asm/bitops.h>
#include <asm/uaccess.h>

#include <asm/delay.h>

// Kvaser definitions
#include "VCanOsIf.h"
#include "dallas.h"
#include "PciCanHwIf.h"
#include "osif_kernel.h"
#include "osif_functions_kernel.h"
#include "queue.h"
#include "debug.h"
#include "util.h"
#include "vcan_ioctl.h"

#include "sja1000.h"
#include "amcc5920.h"
#include "hwnames.h"


MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("KVASER");
MODULE_DESCRIPTION("PCIcan CAN module.");

//
// If you do not define PCICAN_DEBUG at all, all the debug code will be
// left out.  If you compile with PCICAN_DEBUG=0, the debug code will
// be present but disabled -- but it can then be enabled for specific
// modules at load time with a 'debug_level=#' option to insmod.
// i.e. >insmod kvpcican debug_level=#
//

#ifdef PCICAN_DEBUG
static int debug_level = PCICAN_DEBUG;
    MODULE_PARM_DESC(debug_level, "PCIcan debug level");
    module_param(debug_level, int, 0644);
#   define DEBUGPRINT(n, args...) if (debug_level>=(n)) printk("<" #n ">" args)
#else
#   define DEBUGPRINT(n, args...) if ((n) == 1) printk("<" #n ">" args)
#endif

//======================================================================
// HW function pointers
//======================================================================

static int INIT pciCanInitAllDevices(void);
static int pciCanSetBusParams (VCanChanData *vChd, VCanBusParams *par);
static int pciCanGetBusParams (VCanChanData *vChd, VCanBusParams *par);
static int pciCanSetOutputMode (VCanChanData *vChd, int silent);
static int pciCanSetTranceiverMode (VCanChanData *vChd, int linemode, int resnet);
static int pciCanBusOn (VCanChanData *vChd);
static int pciCanBusOff (VCanChanData *vChd);
static int pciCanGetTxErr(VCanChanData *vChd);
static int pciCanGetRxErr(VCanChanData *vChd);
static int pciCanTxAvailable (VCanChanData *vChd);
static int EXIT pciCanCloseAllDevices(void);
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
    .txAvailable        = pciCanTxAvailable,
    .procRead           = pciCanProcRead,
    .closeAllDevices    = pciCanCloseAllDevices,
    .getTime            = vCanTime,
    .flushSendBuffer    = vCanFlushSendBuffer,
    .getTxErr           = pciCanGetTxErr,
    .getRxErr           = pciCanGetRxErr,
    .rxQLen             = pciCanRxQLen,
    .txQLen             = pciCanTxQLen,
    .requestChipState   = pciCanRequestChipState,
    .requestSend        = pciCanRequestSend
};


static int DEVINIT pciCanInitOne(struct pci_dev *dev, const struct pci_device_id *id);
static void DEVEXIT pciCanRemoveOne(struct pci_dev *dev);

static struct pci_device_id id_table[] DEVINITDATA = {
  {
    .vendor    = PCICAN_VENDOR,
    .device    = PCICAN_ID,
    .subvendor = PCI_ANY_ID,
    .subdevice = PCI_ANY_ID
  },
  {
    .vendor    = KVASER_VENDOR,
    .device    = PCIECAN_ID,
    .subvendor = PCI_ANY_ID,
    .subdevice = PCI_ANY_ID
  },
  {0,},
};

static struct pci_driver pcican_tbl = {
  .name     = "kvpcican",
  .id_table = id_table,
  .probe    = pciCanInitOne,
  .remove   = DEVEXITP(pciCanRemoveOne),
};

//MODULE_DEVICE_TABLE(pci, pcican_tbl);


//======================================================================
// Wrapper for common function
//======================================================================
static unsigned long getTime(VCanCardData *vCard)
{
  unsigned long time;

  hwIf.getTime(vCard, &time);

  return time;
}


//======================================================================
// /proc read function
//======================================================================
static int pciCanProcRead (struct seq_file* m, void* v)
{
	seq_printf(m, "\ntotal channels %d\n",
                   driverData.noOfDevices);

    return 0;
}

//======================================================================
//  Can we send now?
//======================================================================
static int pciCanTxAvailable (VCanChanData *vChd)
{
    return vChd->currentTxMsg == NULL;
} // pciCanTxAvailable


//======================================================================
//  Check sja1000 health
// This is only called before a card is initialized, so no one can
// interfere with the accesses (lock not even initialized at this point).
//======================================================================
static int DEVINIT pciCanProbeChannel (VCanChanData *chd)
{
    unsigned int port, tmp;

    PciCanChanData *hChd = chd->hwChanData;

    // First, reset the chip.
    iowrite8(0x01, hChd->sja1000 + PCAN_MOD);
    port = ioread8(hChd->sja1000 + PCAN_MOD);

    // If we don't read 0x01 back, then it isn't an sja1000.
    if ((port & 0x01) == 0) {
        return VCAN_STAT_NO_DEVICE;
    }

    // 0xff is not a valid answer.
    if (port == 0xFF) {
        return VCAN_STAT_NO_DEVICE;
    }

    // Try to set the Pelican bit.
    port = ioread8(hChd->sja1000 + PCAN_CDR);
    iowrite8((unsigned char)(port | PCAN_PELICAN), hChd->sja1000 + PCAN_CDR);
    port = ioread8(hChd->sja1000 + PCAN_CDR);
    if ((port & PCAN_PELICAN) == 0) {
        return VCAN_STAT_NO_DEVICE;
    }

    // Reset it..
    iowrite8((unsigned char)(port & ~PCAN_PELICAN), hChd->sja1000 + PCAN_CDR);
    port = ioread8(hChd->sja1000 + PCAN_CDR);
    if ((port & PCAN_PELICAN) != 0) {
        return VCAN_STAT_NO_DEVICE;
    }

    // Check that bit 5 in the 82c200 control register is always 1
    tmp = ioread8(hChd->sja1000 + 0);
    if ((tmp & 0x20) == 0) {
        return VCAN_STAT_NO_DEVICE;
    }
    iowrite8((unsigned char)(tmp | 0x20), hChd->sja1000 + 0);
    tmp = ioread8(hChd->sja1000 + 0);
    if ((tmp & 0x20) == 0) {
        return VCAN_STAT_NO_DEVICE;
    }

    // The 82c200 command register is always read as 0xFF
    tmp = ioread8(hChd->sja1000 + 1);
    if (tmp != 0xFF) {
        return VCAN_STAT_NO_DEVICE;
    }
    iowrite8(0, hChd->sja1000 + 1);
    tmp = ioread8(hChd->sja1000 + 1);
    if (tmp != 0xFF) {
        return VCAN_STAT_NO_DEVICE;
    }

    // Set the Pelican bit.
    port = ioread8(hChd->sja1000 + PCAN_CDR);
    iowrite8((unsigned char)(port | PCAN_PELICAN), hChd->sja1000 + PCAN_CDR);
    port = ioread8(hChd->sja1000 + PCAN_CDR);
    if ((port & PCAN_PELICAN) == 0) {
        return VCAN_STAT_NO_DEVICE;
    }

    return VCAN_STAT_OK;
} // pciCanProbeChannel


//======================================================================
// Find out some info about the H/W
// (*cd) must have pciIf, xilinx and sjaBase initialized
// This is only called before a card is initialized, so no one can
// interfere with the accesses (lock not even initialized at this point).
//======================================================================
static int DEVINIT pciCanProbe (VCanCardData *vCd)
{
    PciCanCardData *hCd = vCd->hwCardData;
    int i;
    void __iomem *addr;
    int xilinxRev;

    // Set (one) Wait State needed(?) by the CAN chip on the ADDON bus of S5920
    // WritePortUlong(ci->cc.s5920_address + S5920_PTCR, 0x81818181L );

    // Assert PTADR# - we're in passive mode so the other bits are not important
    iowrite32(0x80808080L, hCd->pciIf + S5920_PTCR);

    xilinxRev = ioread8(hCd->xilinx + XILINX_VERINT) >> 4;

    vCd->firmwareVersionMajor = xilinxRev;
    vCd->firmwareVersionMinor = 0;
    vCd->firmwareVersionBuild = 0;

    vCd->hwRevisionMajor = 0;
    vCd->hwRevisionMinor = 0;

    hCd->cardEeprom.address_out = hCd->xilinx + XILINX_OUTA;
    hCd->cardEeprom.address_in  = hCd->xilinx + XILINX_INA;
    hCd->cardEeprom.in_mask = 0x80;
    hCd->cardEeprom.out_mask = 0x80;


    // Check for piggybacks and other data stored in the Dallas memories.
    ds_init(&(hCd->cardEeprom));

    if (ds_check_for_presence(&(hCd->cardEeprom))) {
        unsigned char buf[32];
        unsigned long boardSerialNumber;
        unsigned int mfgDate;

        memset(buf, 0, sizeof(buf));

        ds_read_rom_64bit(&(hCd->cardEeprom), 0, buf);

        if (*buf == DS2431_CODE) {
            //
            // This is a 2431 memory, so the date is in the second page...
            //
            ds_read_memory(DS2431_CODE, 
                           &(hCd->cardEeprom), 0x20, buf, sizeof(buf));
            mfgDate = *(unsigned short *)&buf[5];
            if (mfgDate != 0xffff) {
            }

            // ...and the rest of the data in the first page.
            ds_read_memory(DS2431_CODE, 
                           &(hCd->cardEeprom), 0, buf, sizeof(buf));
            memcpy(&boardSerialNumber, &buf[6], sizeof(boardSerialNumber));
            if (boardSerialNumber != 0xffffffff) {
                vCd->serialNumber = boardSerialNumber;
            }
        } else {
            //
            // This is most likely a ds2430.
            //

            ds_read_memory(DS2430_CODE,
                           &(hCd->cardEeprom), 0, buf, sizeof(buf));
            memcpy(&boardSerialNumber, &buf[6], sizeof(boardSerialNumber));

            if (boardSerialNumber != 0xffffffff) {
                unsigned char buf[8];
                unsigned int mfgDate;
                //
                // The serial number makes sense so it's really a ds2430
                //
                vCd->serialNumber = boardSerialNumber;

                // Read the date; it's in the 8-byte application area
                ds_read_application_area(&(hCd->cardEeprom), 0,
                                         buf, sizeof(buf));
                mfgDate = *(unsigned short *)&buf[5];
                if (mfgDate != 0xffff) {
                }
            }
        }

        // qqq This should be per channel!
        vCd->capabilities = VCAN_CHANNEL_CAP_RECEIVE_ERROR_FRAMES |
                            VCAN_CHANNEL_CAP_ERROR_COUNTERS       |
                            VCAN_CHANNEL_CAP_EXTENDED_CAN         |
                            VCAN_CHANNEL_CAP_TXREQUEST            |
                            VCAN_CHANNEL_CAP_TXACKNOWLEDGE;

        vCd->hw_type      = HWTYPE_PCICAN;

        memset(vCd->ean, 0, sizeof(vCd->ean));
        if (memcmp(&buf[1], "\xff\xff\xff\xff\xff\xff\xff\xff", 6) != 0) {
            packed_EAN_to_BCD_with_csum(&buf[1], vCd->ean);
        }
        {
            unsigned int sum = 0, n;
            char hex[3 * 32 + 1];

            DEBUGPRINT(1, "EEPROM Board: ");
            for (n = 0; n < 32; ++n) {
                sprintf(&hex[n * 3], "%02x ", buf[n]);
                sum += buf [n];
            }
            DEBUGPRINT(1, "%s, sum=%02x\n", hex, sum);

            DEBUGPRINT(1, "serial number = 0x%04x (%u)\n",
                       vCd->serialNumber, vCd->serialNumber);

            DEBUGPRINT(1, "ean code = ");
            for (n = 0; n < sizeof(vCd->ean); n++) {
              sprintf(&hex[n * 2], "%02x", vCd->ean[n]);
            }
            DEBUGPRINT(1, "%s\n", hex);
        }
    }

    ds_shutdown(&(hCd->cardEeprom));

    for (i = 0; i < MAX_CHANNELS; i++) {
        VCanChanData   *vChd = vCd->chanData[i];
        PciCanChanData *hChd = vChd->hwChanData;

        if (vChd->chipType == CAN_CHIP_TYPE_UNKNOWN) {
            // This does not work, if the card is probed the second time
            // (e.g. after wakeup from sleep mode), and there are less
            // than MAX_CHANNELS available.

            // Each controller has PCICAN_BYTES_PER_CIRCUIT bytes.
            // This is "hardcoded" on the PCB and in the Xilinx.
            addr                         = hCd->sjaBase + (i * PCICAN_BYTES_PER_CIRCUIT);
            vChd->channel                = i;
            hChd->sja1000                = addr;
            hChd->chanEeprom.address_out = 0;
            hChd->chanEeprom.address_in  = 0;
            hChd->chanEeprom.in_mask     = 0;
            hChd->chanEeprom.out_mask    = 0;

            hChd->xilinxAddressOut       = 0;
            hChd->xilinxAddressCtrl      = 0;
            hChd->xilinxAddressIn        = 0;

            if (vChd->channel == 0) {
                hChd->chanEeprom.address_out = hCd->xilinx + XILINX_OUTA;
                hChd->chanEeprom.address_in  = hCd->xilinx + XILINX_INA;
                hChd->chanEeprom.in_mask     = 0x01;
                hChd->chanEeprom.out_mask    = 0x01;

                hChd->xilinxAddressOut       = hCd->xilinx + XILINX_OUTA;
                hChd->xilinxAddressCtrl      = hCd->xilinx + XILINX_CTRLA;
                hChd->xilinxAddressIn        = hCd->xilinx + XILINX_INA;
            }
            else if (vChd->channel == 1) {
                hChd->chanEeprom.address_out = hCd->xilinx + XILINX_OUTB;
                hChd->chanEeprom.address_in  = hCd->xilinx + XILINX_INB;
                hChd->chanEeprom.in_mask     = 0x01;
                hChd->chanEeprom.out_mask    = 0x01;

                hChd->xilinxAddressOut       = hCd->xilinx + XILINX_OUTB;
                hChd->xilinxAddressCtrl      = hCd->xilinx + XILINX_CTRLB;
                hChd->xilinxAddressIn        = hCd->xilinx + XILINX_INB;
            }
        }
        else {
            DEBUGPRINT(1, "Channel %d already detected - don't calculate address\n", vChd->channel);
        }

        if (pciCanProbeChannel(vChd) == VCAN_STAT_OK) {
            vChd->chipType = CAN_CHIP_TYPE_SJA1000;
        } else {
            // Exit from loop
            break;
        }
    }

    vCd->nrChannels = i;
    DEBUGPRINT(1, "Kvaser PCIcan with %d channels found\n", vCd->nrChannels);

    // Init Dallas ports so we can read the memories on the piggybacks, if
    // they are present.

    iowrite8(1, hCd->xilinx + XILINX_CTRLA);
    iowrite8(1, hCd->xilinx + XILINX_CTRLB);

    for (i = 0; i < vCd->nrChannels; ++i) {
        VCanChanData   *vChd = vCd->chanData[i];
        PciCanChanData *hChd = vChd->hwChanData;

        vChd->transType = VCAN_TRANSCEIVER_TYPE_251;

        ds_init(&hChd->chanEeprom);

        memset(vChd->ean, 0, sizeof(vChd->ean));
        vChd->serialLow  = 0;
        vChd->serialHigh = 0;

        if (ds_check_for_presence(&hChd->chanEeprom)) {

            unsigned char buf[32];

            memset(buf, 0, sizeof(buf));
            ds_read_rom_64bit(&hChd->chanEeprom, 0, buf);

            // *buf is DS2431_CODE or something else
            ds_read_memory(*buf, &hChd->chanEeprom, 0, buf, sizeof(buf));

            if ((buf[9] == VCAN_TRANSCEIVER_TYPE_SWC)       ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_DNOPTO)    ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_251)       ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_252)       ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_1054_OPTO) ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_SWC_OPTO)  ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_1050)      ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_1050_OPTO) ||
                (buf[9] == VCAN_TRANSCEIVER_TYPE_GAL)
                ) {
                vChd->transType = buf[9];
                DEBUGPRINT(3, "Channel %d piggy type %d\n", i, buf[9]);
            } else {
                DEBUGPRINT(1, "Channel %d strange piggy type %d\n", i, buf[9]);
            }

            // qqq The following fields should probably also be filled in:
            // unsigned int  transceiver_type;
            // unsigned int  transceiver_state;
            // unsigned long serial_number_low,
            // serial_number_high;
            // unsigned long transceiver_capabilities;
            // char          ean[16];
            // unsigned int  linemode,
            // resnet;

            memset(vChd->ean, 0, sizeof(vChd->ean));
            memset(vCd->ean, 0, sizeof(vCd->ean));
            packed_EAN_to_BCD_with_csum(&buf[0], vChd->ean);
        }

        ds_shutdown(&(hCd->cardEeprom));

        switch (vChd->transType) {

            case VCAN_TRANSCEIVER_TYPE_SWC:
            case VCAN_TRANSCEIVER_TYPE_SWC_OPTO:
                vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_SWC_NORMAL;
                break;

            case VCAN_TRANSCEIVER_TYPE_251:
            case VCAN_TRANSCEIVER_TYPE_NONE:
            case VCAN_TRANSCEIVER_TYPE_DNOPTO:
            case VCAN_TRANSCEIVER_TYPE_1050:
            case VCAN_TRANSCEIVER_TYPE_1050_OPTO:
                vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_NORMAL;
                break;

            default:
                vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_NA;
                break;
        }
    }

    // Now restore the control registers in the Xilinx to the default config;
    // needed for talking to a PCIcan-Q. If we have piggys present, they will
    // be setup again in hermes_setup_transceiver().

    if (vCd->nrChannels <= 2) {
        // PCIcan-S, -D. Disable the ununsed interrupts so Galathea will work.
        // This is done by writing 0x40 to the control ports.
        // Also, for Galathea, set bit 1 to 1 in the control registers.
        iowrite8(0x42, hCd->xilinx + XILINX_CTRLA);
        iowrite8(0x42, hCd->xilinx + XILINX_CTRLB);
    } else {
        // Xilinx setup for PCIcan-Q.
        iowrite8(0, hCd->xilinx + XILINX_CTRLA);
        iowrite8(0, hCd->xilinx + XILINX_CTRLB);
    }


    if (i == 0) {
        // No channels found
        vCd->cardPresent = 0;
        return VCAN_STAT_NO_DEVICE;
    }

    vCd->cardPresent = 1;

    return VCAN_STAT_OK;
} // pciCanProbe


//======================================================================
// Perform transceiver-specific setup on PCIcan with piggybacks
// Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanSetupTransceiver (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned char tmp;

    if (hChd->xilinxAddressCtrl == 0) {
        return;
    }

    switch (vChd->transType) {

        case VCAN_TRANSCEIVER_TYPE_NONE:
        case VCAN_TRANSCEIVER_TYPE_251:
        case VCAN_TRANSCEIVER_TYPE_DNOPTO:
        case VCAN_TRANSCEIVER_TYPE_1050:
        case VCAN_TRANSCEIVER_TYPE_1050_OPTO:
            // Piggyback pins all inputs.
            tmp = ioread8(hChd->xilinxAddressCtrl);
            tmp &= 0xe1;
            iowrite8(tmp, hChd->xilinxAddressCtrl);
            break;

        case VCAN_TRANSCEIVER_TYPE_SWC:
        case VCAN_TRANSCEIVER_TYPE_SWC_OPTO:
            tmp = ioread8(hChd->xilinxAddressCtrl);
            tmp &= 0xe1;
            tmp |= 0x18;
            iowrite8(tmp, hChd->xilinxAddressCtrl);
            // qqq we should implement a "sticky" input register in the Xilinx
            // qqq so we can see if HVOLT has gone low.
            break;

            //
            // The 252/1053 code is not yet tested. qqq
            //
        case VCAN_TRANSCEIVER_TYPE_252:
        case VCAN_TRANSCEIVER_TYPE_1054_OPTO:
            tmp = ioread8(hChd->xilinxAddressCtrl);
            tmp &= 0xe1;
            tmp |= 0x1c;
            iowrite8(tmp, hChd->xilinxAddressCtrl);
            // qqq we should implement a "sticky" input register in the Xilinx
            // qqq so we can see if NERR has gone low.
            break;
        case VCAN_TRANSCEIVER_TYPE_GAL:
        {
            // Activate the transceiver. OUTx register bit 1 high then low

            unsigned char tmp;
            tmp = ioread8(hChd->xilinxAddressOut);
            iowrite8((unsigned char)(tmp | 2), hChd->xilinxAddressOut);
            iowrite8((unsigned char)(tmp & ~2), hChd->xilinxAddressOut);

        }
        break;
        default:
            // qqq unsupported piggy
            DEBUGPRINT(1, "PCIcan: unknown piggy on chan %d\n", vChd->channel);
            break;
    }
} // pciCanSetupTransceiver


//======================================================================
// This sets the transceiver to the specified line mode.
// It's a no-op for 251-type tranceivers, but for e.g. SWCs the current
// line mode could be WAKEUP.
// Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanActivateTransceiver (VCanChanData *vChd, int linemode)
{
    PciCanChanData *hChd = vChd->hwChanData;

    if (hChd->xilinxAddressOut == 0) {
        return;
    }

    switch (vChd->transType) {

        case VCAN_TRANSCEIVER_TYPE_SWC:
        case VCAN_TRANSCEIVER_TYPE_SWC_OPTO:
            switch (linemode) {

                case VCAN_TRANSCEIVER_LINEMODE_SWC_SLEEP:
                    iowrite8(0x00, hChd->xilinxAddressOut);
                    break;

                case VCAN_TRANSCEIVER_LINEMODE_SWC_NORMAL:
                    iowrite8(0x18, hChd->xilinxAddressOut);
                    break;

                case VCAN_TRANSCEIVER_LINEMODE_SWC_FAST:
                    iowrite8(0x08, hChd->xilinxAddressOut);
                    break;

                case VCAN_TRANSCEIVER_LINEMODE_SWC_WAKEUP:
                    iowrite8(0x10, hChd->xilinxAddressOut);
                    break;
            }
            break;

        case VCAN_TRANSCEIVER_TYPE_252:
        case VCAN_TRANSCEIVER_TYPE_1054_OPTO:
            // EN=1, STB#=1, WAK#=1
            iowrite8(0x1c, hChd->xilinxAddressOut);
            break;
        case VCAN_TRANSCEIVER_TYPE_GAL:

            break;
        default:
            break;
    }
} // pciCanActivateTransceiver


//======================================================================
// Enable bus error interrupts, and reset the
// counters which keep track of the error rate
// Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanResetErrorCounter (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned char ier;
    ier = ioread8(hChd->sja1000 + PCAN_IER);
    iowrite8(ier | PCAN_BEIE, hChd->sja1000 + PCAN_IER);
    vChd->errorCount = 0;

    vChd->errorTime = getTime(vChd->vCard);
} // pciCanResetErrorCounter


//======================================================================
//  Set bit timing
//======================================================================
static int pciCanSetBusParams (VCanChanData *vChd, VCanBusParams *par)
{
    PciCanChanData  *hChd = vChd->hwChanData;
    unsigned int    quantaPerCycle;
    unsigned long   brp;
    unsigned char   cbt0;
    unsigned char   cbt1;
    unsigned char   tmp;
    int             resetStatus;
    unsigned long   freq;
    unsigned char   tseg1;
    unsigned char   tseg2;
    unsigned char   sjw;
    unsigned char   sam;
    void __iomem    *circAddr = hChd->sja1000;
    unsigned long   irqFlags;

    freq  = par->freq;
    sjw   = par->sjw;
    tseg1 = par->tseg1;
    tseg2 = par->tseg2;
    sam   = 1; // Always 1

    quantaPerCycle = tseg1 + tseg2 + 1;

    if (quantaPerCycle == 0 || freq == 0) {
        return VCAN_STAT_BAD_PARAMETER;
    }

    brp = (8000000L * 64) / (freq * quantaPerCycle);
    if ((brp & 0x3F) != 0) {
        // Fraction != 0 : not divisible.
        return VCAN_STAT_BAD_PARAMETER;
    }
    brp = brp >> 6;
    if (brp   < 1 || brp   > 64 ||
        sjw   < 1 || sjw   >  4 ||
        tseg1 < 1 || tseg1 > 16 ||
        tseg2 < 1 || tseg2 > 8) {
        return VCAN_STAT_BAD_PARAMETER;
    }

    cbt0 = ((sjw - 1) << 6) + (brp - 1);
    cbt1 = ((sam == 3 ? 1 : 0) << 7) + ((tseg2 - 1) << 4) + (tseg1 - 1);


    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    // Put the circuit in Reset Mode
    tmp = ioread8(circAddr + PCAN_MOD);

    // Always set the AFM bit.
    tmp |= PCAN_AFM;
    resetStatus = tmp & PCAN_RM;
    iowrite8(tmp | PCAN_RM | PCAN_AFM, circAddr + PCAN_MOD);

    iowrite8(cbt0, circAddr + PCAN_BTR0);
    iowrite8(cbt1, circAddr + PCAN_BTR1);

    if (!resetStatus) {
        tmp = ioread8(circAddr + PCAN_MOD);
        iowrite8(tmp & ~PCAN_RM, circAddr + PCAN_MOD);
    }

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return VCAN_STAT_OK;
} // pciCanSetBusParams


//======================================================================
//  Get bit timing
//======================================================================
static int pciCanGetBusParams (VCanChanData *vChd, VCanBusParams *par)
{
    PciCanChanData *hChd = vChd->hwChanData;

    unsigned int  quantaPerCycle;
    unsigned char cbt0;
    unsigned char cbt1;
    unsigned long brp;
    unsigned long irqFlags;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    cbt0 = ioread8(hChd->sja1000 + PCAN_BTR0);
    cbt1 = ioread8(hChd->sja1000 + PCAN_BTR1);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    par->sjw = 1 + (cbt0 >> 6);
    par->samp3 = (cbt1 >> 7) == 1 ? 3 : 1;

    par->tseg1 = 1 + (cbt1 & 0xf);
    par->tseg2 = 1 + ((cbt1 >> 4) & 0x7);

    quantaPerCycle = par->tseg1 + par->tseg2 + 1;
    brp = 1 + (cbt0 & 0x3f);

    par->freq = 8000000L / (quantaPerCycle * brp);

    return VCAN_STAT_OK;
} // pciCanGetBusParams


//======================================================================
//  Set silent or normal mode
//======================================================================
static int pciCanSetOutputMode (VCanChanData *vChd, int silent)
{
    PciCanChanData *hChd = vChd->hwChanData;

    unsigned char driver;
    unsigned char tmp;
    unsigned long irqFlags;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    // Save control register
    tmp = ioread8(hChd->sja1000 + PCAN_MOD);
    // Put the circuit in Reset Mode
    iowrite8(tmp | PCAN_RM , hChd->sja1000 + PCAN_MOD);
    // Always set the AFM bit.
    tmp |= PCAN_AFM;

    if (vChd->transType == VCAN_TRANSCEIVER_TYPE_GAL) {
        driver = OCR_DEFAULT_GAL;
    } else {
        driver = OCR_DEFAULT_STD;
    }

    if (silent) {
        tmp |= PCAN_LOM;
    } else {
        tmp &= ~PCAN_LOM;
    }

    // Set the output control
    iowrite8(driver, hChd->sja1000 + PCAN_OCR);
    // Restore control register
    iowrite8(tmp, hChd->sja1000 + PCAN_MOD);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return VCAN_STAT_OK;
} // pciCanSetOutputMode


//======================================================================
//  Line mode
//======================================================================
static int pciCanSetTranceiverMode (VCanChanData *vChd, int linemode, int resnet)
{
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned long irqFlags;

    vChd->lineMode = linemode;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    pciCanActivateTransceiver(vChd, vChd->lineMode);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return VCAN_STAT_OK;
} // pciCanSetTranceiverMode


//======================================================================
// Query chip status (internal)
// Must be called with channel locked (to avoid access interference).
//======================================================================
static int pciCanRequestChipState_internal (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;
    VCAN_EVENT msg;
    unsigned char sr, cr;

    sr = ioread8(hChd->sja1000 + PCAN_SR);
    cr = ioread8(hChd->sja1000 + PCAN_MOD);

    vChd->chipState.txerr = ioread8(hChd->sja1000 + PCAN_TXERR);
    vChd->chipState.rxerr = ioread8(hChd->sja1000 + PCAN_RXERR);

    switch (sr & (PCAN_BS | PCAN_ES)) {
        case PCAN_BS:
            vChd->chipState.state = CHIPSTAT_BUSOFF;
            break;

        case PCAN_BS | PCAN_ES:
            vChd->chipState.state = CHIPSTAT_BUSOFF;
            break;

        case PCAN_ES:
            vChd->chipState.state = CHIPSTAT_ERROR_WARNING;
            if ((vChd->chipState.txerr > 127) ||
                (vChd->chipState.rxerr > 127)) {
                vChd->chipState.state |= CHIPSTAT_ERROR_PASSIVE;
            }
            break;

        case 0:
            vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;
            break;
    }

    if (cr & PCAN_RM) {
        // It's in reset mode. We report BUSOFF but should really be
        // reporting "inactive" or so. qqq
        vChd->chipState.state = CHIPSTAT_BUSOFF;
    }

    msg.tag                              = V_CHIP_STATE;
    msg.timeStamp                        = getTime(vChd->vCard);
    msg.transId                          = 0;
    msg.tagData.chipState.busStatus      = (unsigned char)vChd->chipState.state;
    msg.tagData.chipState.txErrorCounter = (unsigned char)vChd->chipState.txerr;
    msg.tagData.chipState.rxErrorCounter = (unsigned char)vChd->chipState.rxerr;
    vCanDispatchEvent(vChd, &msg);

    return VCAN_STAT_OK;
} // pciCanRequestChipState


//======================================================================
//  Query chip status
//======================================================================
static int pciCanRequestChipState (VCanChanData *vChd)
{
  PciCanChanData *hChd = vChd->hwChanData;
  int ret;
  unsigned long irqFlags;

  os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

  ret = pciCanRequestChipState_internal(vChd);

  os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

  return ret;
}


//======================================================================
//  Go bus on
//======================================================================
static int pciCanBusOn (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned int   tmp;
    unsigned long  irqFlags;

    switch (vChd->transType) {
        case VCAN_TRANSCEIVER_TYPE_SWC:
        case VCAN_TRANSCEIVER_TYPE_SWC_OPTO:
            vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_SWC_NORMAL;
            break;

        case VCAN_TRANSCEIVER_TYPE_251:
        case VCAN_TRANSCEIVER_TYPE_NONE:
        case VCAN_TRANSCEIVER_TYPE_DNOPTO:
        case VCAN_TRANSCEIVER_TYPE_1050:
        case VCAN_TRANSCEIVER_TYPE_1050_OPTO:
            vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_NORMAL;
            break;

        default:
            vChd->lineMode = VCAN_TRANSCEIVER_LINEMODE_NA;
            break;
    }

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    pciCanSetupTransceiver(vChd);
    pciCanActivateTransceiver(vChd, vChd->lineMode);

    vChd->isOnBus = 1;
    vChd->overrun = 0;
    pciCanResetErrorCounter(vChd);

    // Go on bus
    tmp = ioread8(hChd->sja1000 + PCAN_MOD);
    // Always set the AFM bit.
    tmp |= PCAN_AFM;
    iowrite8(PCAN_CDO, hChd->sja1000 + PCAN_CMR);
    iowrite8(tmp | PCAN_RM, hChd->sja1000 + PCAN_MOD);
    iowrite8(0, hChd->sja1000 + PCAN_TXERR);
    iowrite8(0, hChd->sja1000 + PCAN_RXERR);
    (void)ioread8(hChd->sja1000 + PCAN_ECC);

    // Write the hardware filters once again. They might have been corrupted
    // if we tried to write a message to the transmit buffer at the same
    // time as the sja1000 decided to go bus off due to e.g. excessive errors.
    iowrite8(0,    hChd->sja1000 + PCAN_ACR0);
    iowrite8(0,    hChd->sja1000 + PCAN_ACR1);
    iowrite8(0,    hChd->sja1000 + PCAN_ACR2);
    iowrite8(0,    hChd->sja1000 + PCAN_ACR3);
    iowrite8(0xFF, hChd->sja1000 + PCAN_AMR0);
    iowrite8(0xFF, hChd->sja1000 + PCAN_AMR1);
    iowrite8(0xFF, hChd->sja1000 + PCAN_AMR2);
    iowrite8(0xFF, hChd->sja1000 + PCAN_AMR3);

    iowrite8(tmp & ~PCAN_RM, hChd->sja1000 + PCAN_MOD);

    vChd->chipState.state = CHIPSTAT_ERROR_ACTIVE;

    pciCanRequestChipState_internal(vChd);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return VCAN_STAT_OK;
} // pciCanBusOn


//======================================================================
//  Go bus off
//======================================================================
static int pciCanBusOff (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned int   tmp;
    unsigned long  irqFlags;

    vChd->isOnBus = 0;
    // since we are bus off we have no msg *on the way*
    vChd->currentTxMsg = NULL;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    tmp = ioread8(hChd->sja1000 + PCAN_MOD);
    iowrite8(tmp | PCAN_RM, hChd->sja1000 + PCAN_MOD);

    pciCanRequestChipState_internal(vChd);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return VCAN_STAT_OK;
} // pciCanBusOff


//======================================================================
//  Disable Card
//======================================================================
static void pciCanResetCard (VCanCardData *vChd)
{
    PciCanCardData *hChd = vChd->hwCardData;
    unsigned int tmp;

    // The card must be present!
    if (!vChd->cardPresent) {
        return;
    }

    // Disable interrupts from card
    tmp = ioread32(hChd->pciIf + S5920_INTCSR);
    tmp &= ~INTCSR_ADDON_INTENABLE_M;
    iowrite32(tmp, hChd->pciIf + S5920_INTCSR);
} // pciCanResetCard


//======================================================================
//  Interrupt handling functions
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanReceiveIsr (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;

    void __iomem       *circAddr = hChd->sja1000;
    VCAN_EVENT         e;
    int                i;
    int                r;
    unsigned char      dlc, data_len;
    unsigned char      flags;
    unsigned char      *p;
    void __iomem       *data;
    unsigned char      SR;
    WL                 id;
    unsigned int       bytes;
    static unsigned int overrun_occured = 0;

    SR = ioread8(circAddr + PCAN_SR);

    if (overrun_occured) {
      overrun_occured--;
    }
    bytes = 0;
    while (SR & PCAN_RBS) {
        unsigned char tmp;

        e.timeStamp = getTime(vChd->vCard);

        tmp = ioread8(circAddr + PCAN_MSGBUF);
        data_len = dlc = (unsigned char)(tmp & 0x0F);

        if (dlc > 8) {
            data_len = 8;
        }
        flags = (unsigned char)((tmp & PCAN_FF_REMOTE) ? VCAN_MSG_FLAG_REMOTE_FRAME : 0);
        id.L = 0;


        // Extended CAN
        if (tmp & PCAN_FF_EXTENDED) {
            bytes += 1 + data_len + 4;
            id.B.b3 = ioread8(circAddr + PCAN_XID0);
            id.B.b2 = ioread8(circAddr + PCAN_XID1);
            id.B.b1 = ioread8(circAddr + PCAN_XID2);
            id.B.b0 = ioread8(circAddr + PCAN_XID3);
            id.L >>= 3;
            id.L |= VCAN_EXT_MSG_ID;
            data = circAddr + PCAN_XDATA;
        }
        // Standard CAN
        else {
            bytes += 1 + data_len + 2;
            id.B.b1 = ioread8(circAddr + PCAN_SID0);
            id.B.b0 = ioread8(circAddr + PCAN_SID1);
            id.L >>= 5;
            data = circAddr + PCAN_SDATA;
        }

        p = e.tagData.msg.data;

        for (i = 0; i < data_len; i++) {
            *p++ = ioread8(data++);
        }

        if (SR & PCAN_DOS) {
            DEBUGPRINT(2, "Overrun for: %08x\n", id.L);
            iowrite8(PCAN_CDO, hChd->sja1000 + PCAN_CMR);
            flags |= VCAN_MSG_FLAG_OVERRUN;
            overrun_occured = 2;
        }
        if (overrun_occured) {
            if (tmp & PCAN_FF_EXTENDED) {
                DEBUGPRINT(4, "RX %08x %04x %02x %d\n", id.L, flags, SR, bytes);
            } else {
                DEBUGPRINT(4, "RX %03x %04x %02x %d\n", id.L, flags, SR, bytes);
            }
        } else {
        }

        e.tag               = V_RECEIVE_MSG;
        e.transId           = 0;
        e.tagData.msg.id    = id.L;
        e.tagData.msg.flags = flags;
        e.tagData.msg.dlc   = dlc;

        r = vCanDispatchEvent(vChd, &e);
        // Release receive buffer
        iowrite8(PCAN_RRB, circAddr + PCAN_CMR);

        SR = ioread8(circAddr + PCAN_SR);
    }

    if (vChd->errorCount > 0) {
        pciCanResetErrorCounter(vChd);
    }
} // pciCanReceiveIsr


//======================================================================
//  Transmit interrupt handler
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanTransmitIsr (VCanChanData *vChd)
{
    PciCanChanData *hChd = vChd->hwChanData;

    pciCanActivateTransceiver(vChd, vChd->lineMode);

    // "send" a transmit ack.
    if (vChd->currentTxMsg && vChd->currentTxMsg->flags & VCAN_MSG_FLAG_TXACK) {
        VCAN_EVENT *e = (VCAN_EVENT *)vChd->currentTxMsg;
        e->tag = V_RECEIVE_MSG;
        e->timeStamp = getTime(vChd->vCard);
        e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXRQ;
        vCanDispatchEvent(vChd, e);
    }
    
    // Flag that the card can accept msg's again. 
    vChd->currentTxMsg = NULL;

    // Send next message in queue
# if !defined(TRY_RT_QUEUE)
    os_if_queue_task(&hChd->txTaskQ);
# else
    os_if_queue_task_not_default_queue(hChd->txTaskQ, &hChd->txWork);
# endif
} // pciCanTransmitIsr


//======================================================================
//  Handle error interrupts. Happens when the bus status or error status
//  bits in the status register changes.
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanErrorIsr (VCanChanData *vChd)
{
    DEBUGPRINT(3, "pciCanErrorIsr\n");
    pciCanRequestChipState_internal(vChd);
} // pciCanErrorIsr


//======================================================================
//  Overrun interrupt handler
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanOverrunIsr (VCanChanData *vChd)
{
    DEBUGPRINT(3, "pciCanOverrunIsr\n");
    pciCanReceiveIsr(vChd);
} // pciCanOverrunIsr


//======================================================================
//  Bus error interrupt handler
//  Must be called with channel locked (to avoid access interference).
//======================================================================
static void pciCanBusErrorIsr (VCanChanData *vChd)
{

    PciCanChanData  *hChd = vChd->hwChanData;
    VCAN_EVENT      e;
    unsigned char   ECC;
    int             r;

    DEBUGPRINT(3, "Bus error\n");
    ECC                  = ioread8(hChd->sja1000 + PCAN_ECC);
    e.timeStamp          = getTime(vChd->vCard);
    e.tag                = V_RECEIVE_MSG;
    e.transId            = 0;
    e.tagData.msg.id     = 0x800 + ECC;
    e.tagData.msg.flags  = VCAN_MSG_FLAG_ERROR_FRAME;
    e.tagData.msg.dlc    = 0;

    r = vCanDispatchEvent(vChd, &e);

    // Muffle the sja1000 if we get too many errors.
    // qqq this is not done right
    //

    vChd->errorCount++;
    if (vChd->errorCount == MAX_ERROR_COUNT / 2) {
        // Half done, store current time
        vChd->errorTime = e.timeStamp;
    }
    else if (vChd->errorCount > MAX_ERROR_COUNT) {
        if ((e.timeStamp - vChd->errorTime) > ERROR_RATE / 10) {
            // Error rate reasonable, restart counters
            vChd->errorCount = 0;
            vChd->errorTime = e.timeStamp;
        }
        else {
            unsigned char ier;
            ier = ioread8(hChd->sja1000 + PCAN_IER);
            iowrite8((unsigned char)(ier & ~PCAN_BEIE), hChd->sja1000 + PCAN_IER);
        }
    }
} // pciCanBusErrorIsr


// Must be called with channel locked (to avoid access interference).
static void pciCanErrorPassiveIsr (VCanChanData *vChd)
{
    DEBUGPRINT(3, "pciCanErrorPassiveIsr\n");
    pciCanRequestChipState_internal(vChd);
} // pciCanErrorPassiveIsr


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
    PciCanCardData  *hCd     = vCard->hwCardData;
    VCanChanData    *vChd;
    PciCanChanData  *hChd;
    unsigned int    loopmax  = 1000;
    unsigned int    ireg;
    int             i;
    int             handled = 0;
    unsigned long   irqFlags;

    // Read interrupt status
    while (ioread32(hCd->pciIf + S5920_INTCSR) & INTCSR_INTERRUPT_ASSERTED_M) {
        handled = 1;

        if (--loopmax == 0) {
            // Kill the card.
            DEBUGPRINT(1, "PCIcan runaway, shutting down card!!");
            pciCanResetCard(vCard);
            return IRQ_HANDLED; // qqq ???
        }

        // Handle all channels
        for (i = 0; i < vCard->nrChannels; i++) {
            vChd = vCard->chanData[i];
            hChd = vChd->hwChanData;

            os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

            // Reading clears interrupt flag
            while ((ireg = ioread8(hChd->sja1000 + PCAN_IR))) {
                if (ireg & PCAN_RI)  pciCanReceiveIsr(vChd);
                if (ireg & PCAN_TI)  pciCanTransmitIsr(vChd);
                if (ireg & PCAN_EI)  pciCanErrorIsr(vChd);
                if (ireg & PCAN_DOI) pciCanOverrunIsr(vChd);
                if (ireg & PCAN_WUI) {
                  DEBUGPRINT(1, "PCIcan: Huh? Wakeup Interrupt!\n");
                }
//                if (ireg & PCAN_AUI) {
//                    DEBUGPRINT(1, "PCIcan: Huh? Arbitration Lost Interrupt!\n");
//                }
                if (ireg & PCAN_BEI) pciCanBusErrorIsr(vChd);
                if (ireg & PCAN_EPI) pciCanErrorPassiveIsr(vChd);
            }

            os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);
        }
    }

    return IRQ_RETVAL(handled);
} // pciCanInterrupt


//======================================================================
//  pcicanTransmit
//======================================================================
static int pciCanTransmitMessage (VCanChanData *vChd, CAN_MSG *m)
{
    PciCanChanData *hChd = vChd->hwChanData;
    void __iomem   *p;
    void __iomem   *circAddr = hChd->sja1000;
    unsigned char  *msg = m->data;
    signed long    ident = m->id;
    unsigned char  flags = m->flags;
    unsigned char  dlc, data_len, x;
    int            i;
    unsigned long  irqFlags;
    VCAN_EVENT     e;

    x = data_len = dlc = m->length & 0x0f;
    if (data_len > 8) {
        data_len = 8;
    }
    if (flags & VCAN_MSG_FLAG_REMOTE_FRAME) {
        x |= PCAN_FF_REMOTE;
    }

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    // We have to have a txAvailable check within the spinlock...
    if (!hwIf.txAvailable(vChd)) {
      DEBUGPRINT(2, "Not available %p\n", vChd->currentTxMsg);
      os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);
      return VCAN_STAT_NO_RESOURCES;
    }


    // Set special transceiver modes
    switch (vChd->transType) {

        case VCAN_TRANSCEIVER_TYPE_SWC:
        case VCAN_TRANSCEIVER_TYPE_SWC_OPTO:
            if (flags & VCAN_MSG_FLAG_WAKEUP) {
                pciCanActivateTransceiver(vChd,
                                          VCAN_TRANSCEIVER_LINEMODE_SWC_WAKEUP);
            }
            break;

        case VCAN_TRANSCEIVER_TYPE_251:
        case VCAN_TRANSCEIVER_TYPE_NONE:
            break;

        default:
            break;
    }

    if (ident & VCAN_EXT_MSG_ID) { // Extended CAN
        WL id;

        id.L = ident & ~VCAN_EXT_MSG_ID;
        id.L <<= 3;

        x |= PCAN_FF_EXTENDED;

        iowrite8(x,       circAddr + PCAN_MSGBUF);
        iowrite8(id.B.b3, circAddr + PCAN_XID0);
        iowrite8(id.B.b2, circAddr + PCAN_XID1);
        iowrite8(id.B.b1, circAddr + PCAN_XID2);
        iowrite8(id.B.b0, circAddr + PCAN_XID3);

        p = circAddr + PCAN_XDATA;
    }
    else { // Standard CAN
        iowrite8(x,                           circAddr + PCAN_MSGBUF);
        iowrite8((unsigned char)(ident >> 3), circAddr + PCAN_SID0);
        iowrite8((unsigned char)(ident << 5), circAddr + PCAN_SID1);

        p = circAddr + PCAN_SDATA;
    }

    for (i = 0; i < data_len; i++) {
        iowrite8(*msg++, p++);
    }

    vChd->currentTxMsg = m;

    if (flags & VCAN_MSG_FLAG_TXRQ) {
        e = *((VCAN_EVENT *)vChd->currentTxMsg);
        e.timeStamp = getTime(vChd->vCard);
    }

    iowrite8(PCAN_TR, circAddr + PCAN_CMR);

    if (vChd->errorCount > 0) {
        pciCanResetErrorCounter(vChd);
    }

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    if (flags & VCAN_MSG_FLAG_TXRQ) {
        e.tagData.msg.flags &= ~(VCAN_MSG_FLAG_TX_NOTIFY | VCAN_MSG_FLAG_TX_START);

        // Fix invalid messages
        e.tagData.msg.flags &= (VCAN_MSG_FLAG_ERROR_FRAME |
                                VCAN_MSG_FLAG_REMOTE_FRAME);

        if (e.tagData.msg.id & VCAN_EXT_MSG_ID) {
            e.tagData.msg.id &= 0x1fffffff | VCAN_EXT_MSG_ID;
        }
        else {
            e.tagData.msg.id &= 0x07ff;
        }

        e.tag = V_RECEIVE_MSG;

        e.tagData.msg.flags |= VCAN_MSG_FLAG_TX_START;
        vCanDispatchEvent(vChd, &e);
    }

    return VCAN_STAT_OK;
} // pciCanTransmitMessage


//======================================================================
//  Read transmit error counter
//======================================================================
static int pciCanGetTxErr (VCanChanData *vChd)
{
    int            ret;
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned long  irqFlags;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    ret = ioread8(hChd->sja1000 + PCAN_TXERR);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return ret;
}


//======================================================================
//  Read transmit error counter
//======================================================================
static int pciCanGetRxErr (VCanChanData *vChd)
{
    int            ret;
    PciCanChanData *hChd = vChd->hwChanData;
    unsigned long  irqFlags;

    os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

    ret = ioread8(hChd->sja1000 + PCAN_RXERR);

    os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    return ret;
}


//======================================================================
//  Read receive queue length in hardware/firmware
//======================================================================
static unsigned long pciCanRxQLen (VCanChanData *vChd)
{
  // qqq Why _tx_ channel buffer?
    return queue_length(&vChd->txChanQueue);
}


//======================================================================
//  Read transmit queue length in hardware/firmware
//======================================================================
static unsigned long pciCanTxQLen (VCanChanData *vChd)
{
    int qLen = 0;
    //if ((vChd->chipState.state != CHIPSTAT_BUSOFF) && !hwIf.txAvailable(vChd)) qLen++;

    // return zero because we don't have any hw-buffers.
    return qLen;
}


//======================================================================
//  Initialize H/W
//  This is only called before a card is initialized, so no one can
//  interfere with the accesses.
//======================================================================
static int DEVINIT pciCanInitHW (VCanCardData *vCard)
{
    int             chNr;
    unsigned int    tmp;
    PciCanCardData  *hCard = vCard->hwCardData;
    unsigned long   irqFlags;

    // The card must be present!
    if (!vCard->cardPresent) {
        return VCAN_STAT_NO_DEVICE;
    }

    for (chNr = 0; chNr < vCard->nrChannels; chNr++){
        VCanChanData   *vChd = vCard->chanData[chNr];
        PciCanChanData *hChd = vChd->hwChanData;

        void __iomem *addr = hChd->sja1000;
        if (!addr) {
            return VCAN_STAT_FAIL;
        }

        os_if_spin_lock_irqsave(&hChd->lock, &irqFlags);

        // Reset the circuit...
        iowrite8(PCAN_RM, addr + PCAN_MOD);
        //
        // ...goto Pelican mode...
        iowrite8(PCAN_PELICAN | PCAN_CBP, addr + PCAN_CDR);
        //
        // ...and set the filter mode
        iowrite8(PCAN_RM | PCAN_AFM, addr + PCAN_MOD);

        // Activate almost all interrupt sources.
        iowrite8(PCAN_BEIE | PCAN_EPIE | PCAN_DOIE | PCAN_EIE | PCAN_TIE | PCAN_RIE,
             addr + PCAN_IER);

        // Accept all messages by default.

        iowrite8(0,    addr + PCAN_ACR0);
        iowrite8(0,    addr + PCAN_ACR1);
        iowrite8(0,    addr + PCAN_ACR2);
        iowrite8(0,    addr + PCAN_ACR3);
        iowrite8(0xFF, addr + PCAN_AMR0);
        iowrite8(0xFF, addr + PCAN_AMR1);
        iowrite8(0xFF, addr + PCAN_AMR2);
        iowrite8(0xFF, addr + PCAN_AMR3);

        // Default 125 kbit/s, pushpull.
        iowrite8(0x07, addr + PCAN_BTR0);
        iowrite8(0x23, addr + PCAN_BTR1);

        if (vChd->transType == VCAN_TRANSCEIVER_TYPE_GAL) {
            iowrite8(OCR_DEFAULT_GAL, addr + PCAN_OCR);
        } else {
            iowrite8(OCR_DEFAULT_STD, addr + PCAN_OCR);
        }

        os_if_spin_unlock_irqrestore(&hChd->lock, irqFlags);

    }

    // Enable interrupts from card
    tmp  = ioread32(hCard->pciIf + S5920_INTCSR);
    tmp |= INTCSR_ADDON_INTENABLE_M;
    iowrite32(tmp, hCard->pciIf + S5920_INTCSR);

    return VCAN_STAT_OK;
}


//======================================================================
//  Find out addresses for one card
//======================================================================
static int DEVINIT readPCIAddresses (struct pci_dev *dev, VCanCardData *vCard)
{
  PciCanCardData *hCd = vCard->hwCardData;

  if (pci_request_regions(dev, "Kvaser PCIcan")) {
    DEBUGPRINT(1, "request regions failed\n");
    return VCAN_STAT_FAIL;
  }

  if (pci_enable_device(dev)){
    DEBUGPRINT(1, "enable device failed\n");
    pci_release_regions(dev);
    return VCAN_STAT_NO_DEVICE;
  }

  hCd->irq     = dev->irq;
  hCd->pciIf   = pci_iomap(dev, 0, 0);
  hCd->sjaBase = pci_iomap(dev, 1, 0);
  hCd->xilinx  = pci_iomap(dev, 2, 0);

  if (!hCd->pciIf || !hCd->sjaBase || !hCd->xilinx) {
    DEBUGPRINT(1, "pci_iomap failed\n");
    if (hCd->pciIf)   pci_iounmap(dev, hCd->pciIf);
    if (hCd->sjaBase) pci_iounmap(dev, hCd->sjaBase);
    if (hCd->xilinx)  pci_iounmap(dev, hCd->xilinx);
    pci_disable_device(dev);
    pci_release_regions(dev);
    return VCAN_STAT_FAIL;
  }

  DEBUGPRINT(2, "irq     = %d\n", hCd->irq);
  DEBUGPRINT(2, "pciIf   = %lx\n", (long)hCd->pciIf);
  DEBUGPRINT(2, "sjaBase = %lx\n", (long)hCd->sjaBase);
  DEBUGPRINT(2, "xilinx  = %lx\n", (long)hCd->xilinx);

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
        DEBUGPRINT(3, "SEND FAILED \n");
    }
#endif
}


//======================================================================
//  Process send Q - This function is called from the immediate queue
//======================================================================
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
void pciCanSend (void *void_chanData)
#else
void pciCanSend (OS_IF_TASK_QUEUE_HANDLE *work)
#endif

{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
    VCanChanData   *chd = (VCanChanData *)void_chanData;
#else
# if !defined(TRY_RT_QUEUE)
    PciCanChanData *devChan = container_of(work, PciCanChanData, txTaskQ);
# else
    PciCanChanData *devChan = container_of(work, PciCanChanData, txWork);
# endif
    VCanChanData   *chd = devChan->vChan;
#endif

    int queuePos;

    if (!hwIf.txAvailable(chd)) {
        return;
    }

    if (chd->minorNr < 0) {  // Channel not initialized?
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
            queue_release(&chd->txChanQueue);
        }
    } else {
        queue_release(&chd->txChanQueue);
        if (test_and_clear_bit(0, &chd->waitEmpty)) {
            wake_up_interruptible(&chd->flushQ);
        }
    }

    return;
}


//======================================================================
//  Initialize H/W specific data
//======================================================================
static void DEVINIT pciCanInitData (VCanCardData *vCard)
{
    int chNr;
    vCard->driverData = &driverData;
    vCanInitData(vCard);
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
    PciCanCardData *hCd;

    int chNr;
    VCanCardData *vCard;

    // Allocate data area for this card
    vCard = os_if_kernel_malloc(sizeof(VCanCardData) + sizeof(PciCanCardData));
    if (!vCard) {
        goto card_alloc_err;
    }
    memset(vCard, 0, sizeof(VCanCardData) + sizeof(PciCanCardData));

    // hwCardData is directly after VCanCardData
    vCard->hwCardData = vCard + 1;
    hCd = vCard->hwCardData;

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

    // Get PCI controller, SJA1000 base and Xilinx addresses
    if (readPCIAddresses(dev, vCard) != VCAN_STAT_OK) {
        DEBUGPRINT(1, "readPCIAddresses failed");
        goto pci_err;
    }

    // Find out type of card i.e. N/O channels etc
    if (pciCanProbe(vCard) != VCAN_STAT_OK) {
        DEBUGPRINT(1, "pciCanProbe failed");
        goto probe_err;
    }

    // Init channels
    pciCanInitData(vCard);

    pci_set_drvdata(dev, vCard);

    // ISR
    // SA_* changed name in 2.6.18 and the old ones were to go away January '07
    if (request_irq(hCd->irq, pciCanInterrupt,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18))
                    SA_SHIRQ,
#else
                    IRQF_SHARED,
#endif
                    "Kvaser PCIcan", vCard)) {
        DEBUGPRINT(1, "request_irq failed");
        goto irq_err;
    }

    // Init h/w  & enable interrupts in PCI Interface
    if (pciCanInitHW(vCard) != VCAN_STAT_OK) {
        DEBUGPRINT(1, "pciCanInitHW failed");
        goto inithw_err;
    }

    // Insert into list of cards
    os_if_spin_lock(&driverData.canCardsLock);
    vCard->next = driverData.canCards;
    driverData.canCards = vCard;
    os_if_spin_unlock(&driverData.canCardsLock);

    return VCAN_STAT_OK;

inithw_err:
    free_irq(hCd->irq, vCard);
irq_err:
    pci_set_drvdata(dev, NULL);
    for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
      PciCanChanData *hChd = vCard->chanData[chNr]->hwChanData;
//#if 0  // No task to destroy since none declared (using standard work queue).
#if defined(TRY_RT_QUEUE)
      os_if_destroy_task(hChd->txTaskQ);
#endif
      os_if_spin_lock_remove(&hChd->lock);
      os_if_spin_lock_remove(&vCard->chanData[chNr]->openLock);
    }
probe_err:
    pci_iounmap(dev, hCd->pciIf);
    pci_iounmap(dev, hCd->sjaBase);
    pci_iounmap(dev, hCd->xilinx);
    pci_disable_device(dev);
    pci_release_regions(dev);
pci_err:
    os_if_kernel_free(vCard->chanData);
chan_alloc_err:
    os_if_kernel_free(vCard);
card_alloc_err:

    return VCAN_STAT_FAIL;
} // pciCanInitOne


//======================================================================
// Shut down the HW for one card
//======================================================================
static void DEVEXIT pciCanRemoveOne (struct pci_dev *dev)
{
  VCanCardData *vCard, *lastCard;
  VCanChanData *vChan;
  PciCanCardData *hCd;
  int chNr;

  vCard = pci_get_drvdata(dev);
  hCd   = vCard->hwCardData;
  pci_set_drvdata(dev, NULL);

  pciCanResetCard(vCard);

  free_irq(hCd->irq, vCard);

  // qqq Mark as not working somehow!

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    vChan = vCard->chanData[chNr];
    DEBUGPRINT(3, "Waiting for all closed on minor %d\n", vChan->minorNr);
    while (atomic_read(&vChan->fileOpenCount) > 0) {
      os_if_set_task_uninterruptible ();
      os_if_wait_for_event_timeout_simple(10);
    }
  }

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    PciCanChanData *hChd = vCard->chanData[chNr]->hwChanData;
//#if 0  // No task to destroy since none declared (using standard work queue).
#if defined(TRY_RT_QUEUE)
    os_if_destroy_task(hChd->txTaskQ);
#endif
    os_if_spin_lock_remove(&hChd->lock);
    os_if_spin_lock_remove(&vCard->chanData[chNr]->openLock);
  }

  pci_iounmap(dev, hCd->pciIf);
  pci_iounmap(dev, hCd->sjaBase);
  pci_iounmap(dev, hCd->xilinx);
  pci_disable_device(dev);
  pci_release_regions(dev);

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

  os_if_kernel_free(vCard->chanData);
  os_if_kernel_free(vCard);
}


//======================================================================
// Find and initialize all cards
//======================================================================
static int INIT pciCanInitAllDevices (void)
{
    int found;

    driverData.deviceName = DEVICE_NAME_STRING;

    found = pci_register_driver(&pcican_tbl);
    DEBUGPRINT(1, "pciCanInitAllDevices %d\n", found);

    // We need to find at least one
    return (found < 0) ? found : 0;
} // pciCanInitAllDevices


//======================================================================
// Shut down and free resources before unloading driver
//======================================================================
static int EXIT pciCanCloseAllDevices (void)
{
    // qqq check for open files
    DEBUGPRINT(1, "pciCanCloseAllDevices\n");
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
