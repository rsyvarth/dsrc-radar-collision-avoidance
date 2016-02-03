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

//////////////////////////////////////////////////////////////////////////////////
// FILE: memQ.c                                /
//////////////////////////////////////////////////////////////////////////////////
// memQ  --  Structs and functions for manipulating memQ             /
//////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------


#include <asm/io.h>

#include "osif_functions_kernel.h"
#include "helios_cmds.h"
#include "memq.h"

/////////////////////////////////////////////

#define MEM_Q_TX_BUFFER       1
#define MEM_Q_RX_BUFFER       2

/////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////

int MemQSanityCheck (PciCan2CardData *ci)
{
    uint32_t p;
    unsigned long irqFlags;

    os_if_spin_lock_irqsave(&ci->memQLock, &irqFlags);

    p = ioread32(ci->baseAddr + DPRAM_HOST_WRITE_PTR);
    if (p > 10000)
        goto error;

    p = ioread32(ci->baseAddr + DPRAM_HOST_READ_PTR);
    if (p > 10000)
        goto error;

    p = ioread32(ci->baseAddr + DPRAM_M16C_WRITE_PTR);
    if (p > 10000)
        goto error;

    p = ioread32(ci->baseAddr + DPRAM_M16C_READ_PTR);
    if (p > 10000)
        goto error;

    os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);

    return 1;

error:
    os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);

    return 0;
}

/////////////////////////////////////////////////////////////////////////
//help fkt for Tx/Rx- FreeSpace(...)
/////////////////////////////////////////////////////////////////////////

static int AvailableSpace (unsigned int  cmdLen, unsigned long rAddr,  
                           unsigned long wAddr,  unsigned long bufStart, 
                           unsigned int bufSize) 
{
    int remaining;
    //cmdLen in bytes

    // Note that "remaining" is not actually number of remaining bytes.
    // Also, the writer never wraps for a single message (it actually
    // guarantees that the write pointer will never be too close to
    // the top of the buffer, so the first case below will always be ok).
    if (wAddr >= rAddr) {
      remaining = bufSize - (wAddr - bufStart) - cmdLen;
    } else {
      remaining = (rAddr - wAddr - 1) - cmdLen;
    }

    // Shout if enough space not available!
    if (remaining < 0) {
      static int count = 0;
      if (count++ % 1000 == 0) {
        printk("<1>memQ: Too little space remaining (%d): %d\n",
               remaining, count);
      }
    }

    return remaining >= 0;
}

////////////////////////////////////////////////////////////////////////////
//returns true if there is enough room for cmd in Tx-memQ, else false
////////////////////////////////////////////////////////////////////////////
static int TxAvailableSpace (PciCan2CardData *ci, unsigned int cmdLen)
{
    unsigned long hwp, crp;
    int      tmp;

    hwp = ioread32(ci->baseAddr + DPRAM_HOST_WRITE_PTR);
    crp = ioread32(ci->baseAddr + DPRAM_M16C_READ_PTR);

    tmp = AvailableSpace(cmdLen,
                        (unsigned long)(ci->baseAddr + crp),
                        (unsigned long)(ci->baseAddr + hwp),
                        (unsigned long)(ci->baseAddr + DPRAM_TX_BUF_START),
                        DPRAM_TX_BUF_SIZE);

    return tmp;
}


//////////////////////////////////////////////////////////////////////////

typedef struct _tmp_context {
    PciCan2CardData *ci;
    heliosCmd *cmd;
    int res;
} TMP_CONTEXT;


int QCmd (PciCan2CardData *ci, heliosCmd *cmd)
{
    int           i;
    void __iomem  *p;
    uint32_t      hwp, crp;
    uint32_t      *tmp;
    void __iomem  *addr = ci->baseAddr;
    unsigned long irqFlags;

    os_if_spin_lock_irqsave(&ci->memQLock, &irqFlags);

    if (!TxAvailableSpace(ci, cmd->head.cmdLen)) {
        os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);
        return MEM_Q_FULL;
    }

    hwp = ioread32(addr + DPRAM_HOST_WRITE_PTR);
    crp = ioread32(addr + DPRAM_M16C_READ_PTR);

    p = addr + hwp;
    tmp = (uint32_t *)cmd;

    for (i = 0; i < cmd->head.cmdLen; i += 4) {
        iowrite32(*tmp++, p);
        p += 4;
    }

    hwp += cmd->head.cmdLen;
 
    if ((hwp + MAX_CMD_LEN) > DPRAM_TX_BUF_END) {
        hwp = DPRAM_TX_BUF_START;
    }

    iowrite32(hwp, addr + DPRAM_HOST_WRITE_PTR);

    os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);

    return MEM_Q_SUCCESS;
}


/////////////////////////////////////////////////////////////////////
int GetCmdFromQ (PciCan2CardData *ci, heliosCmd *cmdPtr)
{
    void __iomem  *p;
    uint32_t      hrp, cwp;
    uint32_t      *tmp;
    int           empty;
    void __iomem  *addr = ci->baseAddr;
    unsigned long irqFlags;

    os_if_spin_lock_irqsave(&ci->memQLock, &irqFlags);

    hrp = ioread32(addr + DPRAM_HOST_READ_PTR);
    cwp = ioread32(addr + DPRAM_M16C_WRITE_PTR);

    empty = (hrp == cwp);

    if (!empty) {
        int len;

        p = addr + hrp;
        tmp = (uint32_t *)cmdPtr;
        *tmp++ = ioread32(p);
        len = cmdPtr->head.cmdLen - 4;
        p += 4;

        while (len > 0) {
            *tmp++ = ioread32(p);
            len -= 4;
            p += 4;
        }

        hrp += cmdPtr->head.cmdLen;

        if ((hrp + MAX_CMD_LEN) > DPRAM_RX_BUF_END) {
            hrp = DPRAM_RX_BUF_START;
        }

        iowrite32(hrp, addr + DPRAM_HOST_READ_PTR);

        os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);

        return MEM_Q_SUCCESS;
    }
    else {
        os_if_spin_unlock_irqrestore(&ci->memQLock, irqFlags);

        return MEM_Q_EMPTY;
    }
}
