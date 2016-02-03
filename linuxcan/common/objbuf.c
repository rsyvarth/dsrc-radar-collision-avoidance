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


#  include <linux/version.h>

#include "debug.h"
#include "vcanevt.h"
#include "VCanOsIf.h"
#include "queue.h"
#include "objbuf.h"

// Returns bitmap with matching buffers, or 0 if no match.
// "id" is assumed to have bit 31 set if it's extended.
// 
unsigned int objbuf_filter_match (OBJECT_BUFFER *buf, unsigned int id,
                                  unsigned int flags)
{
  unsigned int result = 0;
  int i;
  
  if (!buf) {
    return 0;
  }

  for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
    if (buf[i].in_use && buf[i].active) {
      if (buf[i].flags & OBJBUF_AUTO_RESPONSE_RTR_ONLY) {
        if ((flags & VCAN_MSG_FLAG_REMOTE_FRAME) == 0) {
          continue;
        }
      }
  
      if ((id & buf[i].acc_mask) == buf[i].acc_code) {
        result |= 1 << i;
      }
    }
  }

  DEBUGOUT(2, (TXT("objbuf_filter_match = %04x\n"), result));
  
  return result;
}


# if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
#  define USE_CONTEXT 1
# else
#  define USE_CONTEXT 0
# endif

#if USE_CONTEXT
static void objbuf_write_all (void *context)
#else
static void objbuf_write_all (OS_IF_TASK_QUEUE_HANDLE *work)
#endif
{
  unsigned int     i;
  uint32_t         active_mask, done_mask;
#if USE_CONTEXT
  VCanOpenFileNode *fileNodePtr = (VCanOpenFileNode *)context;
#else
  VCanOpenFileNode *fileNodePtr = container_of(work, VCanOpenFileNode, objbufWork);
#endif
  VCanChanData     *vChan       = fileNodePtr->chanData;
  VCanCardData     *vCard       = vChan->vCard;
  CAN_MSG          *bufMsgPtr;
  int              queuePos;

#ifdef __arm__
  unsigned int rd;
  unsigned int new_rd;
#endif

  DEBUGOUT(2, (TXT("objbuf_write_all: dev = 0x%p  vCard = 0x%p\n"),
               vChan, vCard));

  // Use semaphore to enforce mutual exclusion
  // for a specific file descriptor.
  os_if_down_sema(&fileNodePtr->ioctl_mutex);

  // qqq Would be good to check "present" here, but that is device specific!
  // qqq The same for bus_on, perhaps.
  if (!fileNodePtr->objbuf) {
    // The device was unplugged before the file was released
    // We cannot deallocate here, it is too early and handled elsewhere
    os_if_up_sema(&fileNodePtr->ioctl_mutex);
    return;
  }

  active_mask = atomic_read (&fileNodePtr->objbufActive);
  if (!active_mask) {
    os_if_up_sema(&fileNodePtr->ioctl_mutex);
    return;
  }

  done_mask = 0;
  for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
    if ((active_mask & (1 << i)) == 0) {
      continue;
    }
    if (!fileNodePtr->objbuf[i].in_use || !fileNodePtr->objbuf[i].active) {
      continue;
    }

    // qqq Would be good to check "present" here, but that is device specific!

    queuePos = queue_back(&vChan->txChanQueue);
    if (queuePos < 0) {
      queue_release(&vChan->txChanQueue);
      break;
    }
    bufMsgPtr = &vChan->txChanBuffer[queuePos];

    memcpy(bufMsgPtr, &fileNodePtr->objbuf[i].msg, sizeof(CAN_MSG));

    // This is for keeping track of the originating fileNode
    bufMsgPtr->user_data = fileNodePtr->transId;
    bufMsgPtr->flags &= ~(VCAN_MSG_FLAG_TX_NOTIFY | VCAN_MSG_FLAG_TX_START);
    if (fileNodePtr->modeTx || (atomic_read(&vChan->fileOpenCount) > 1)) {
      bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_NOTIFY;
    }
    if (fileNodePtr->modeTxRq) {
      bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_START;
    }

    queue_push(&vChan->txChanQueue);

    done_mask |= (1 << i);
  }
#ifdef __arm__
  do {
    rd = atomic_read(&fileNodePtr->objbufActive);
    new_rd = rd & ~done_mask;
  } while (atomic_cmpxchg(&fileNodePtr->objbufActive, rd, new_rd) != rd);
#else
  atomic_clear_mask(done_mask, &fileNodePtr->objbufActive);
#endif

  if (active_mask != done_mask) {
    // Give ourselves a little extra work in case all the sends could not
    // be handled this time.
    // qqq Should probably be delayed (or waiting until space)
    os_if_queue_task_not_default_queue(fileNodePtr->objbufTaskQ,
                                       &fileNodePtr->objbufWork);
  }

  if (done_mask) {
    vChan->vCard->driverData->hwIf->requestSend(vCard, vChan); // Ok to fail ;-)
  }

  os_if_up_sema(&fileNodePtr->ioctl_mutex);

  DEBUGOUT(2, (TXT("objbuf_write_all: %04x/%04x\n"), done_mask, active_mask));
}


void objbuf_init (VCanOpenFileNode *fileNodePtr)
{
  os_if_init_task(&fileNodePtr->objbufWork, objbuf_write_all, fileNodePtr);
  fileNodePtr->objbufTaskQ = os_if_declare_task("objbuf",
                                                &fileNodePtr->objbufWork);
}


void objbuf_shutdown (VCanOpenFileNode *fileNodePtr)
{
  os_if_destroy_task(fileNodePtr->objbufTaskQ);
}
