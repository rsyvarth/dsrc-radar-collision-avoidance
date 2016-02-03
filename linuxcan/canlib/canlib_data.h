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

/* Kvaser Linux Canlib */

#ifndef _CANLIB_DATA_H_
#define _CANLIB_DATA_H_


#include "linkedlist.h"

#include "osif_user.h"


#include "vcanevt.h"

#define N_O_CHANNELS 2

typedef LinkedList HandleList;

#include <canlib.h>
#include <canlib_version.h>

#define LAPCAN_TICKS_PER_MS 125

#define OPEN_AS_CAN          0 
#define OPEN_AS_CANFD_ISO    1
#define OPEN_AS_CANFD_NONISO 2

struct CANops;

// This struct is associated with each handle
// returned by canOpenChannel
typedef struct HandleData
{
  OS_IF_FILE_HANDLE fd;
  char               deviceName[DEVICE_NAME_LEN];
  char               deviceOfficialName[150];
  int                channelNr; // Absolute ch nr i.e. it can be >2 for lapcan
  canHandle          handle;
  unsigned char      isExtended;
  unsigned char      fdMode;
  unsigned char      acceptLargeDlc;
  unsigned char      wantExclusive;
  unsigned char      acceptVirtual;
  unsigned char      readIsBlock;
  unsigned char      writeIsBlock;
  long               readTimeout;
  long               writeTimeout;
  unsigned long      currentTime;
  DWORD              timerResolution;
  double             timerScale;
  void               (*callback)(canNotifyData *);
  void               (*callback2)(CanHandle hnd, void* ctx, unsigned int event);
  canNotifyData      notifyData;
  OS_IF_FILE_HANDLE  notifyFd;
  pthread_t          notifyThread;
  unsigned int       notifyFlags;
  struct CANOps      *canOps;
  unsigned char      syncPressent;
  long               syncId;
  unsigned char      syncMsg[MAX_MSG_LEN];
  unsigned int       syncDlc;
  unsigned int       syncFlag;
  unsigned long      syncTime;
} HandleData;


/* Hardware dependent functions that do the actual work with the card
 * The functions are given a HandleData struct */
//typedef struct HWOps
typedef struct CANOps
{
  /* Read a channel and flags e.g canWANT_EXCLUSIVE and return a file descriptor
   * to read messages from.
   */
  canStatus (*openChannel)(HandleData *);
  /* Read a callback function and flags that defines which events triggers it */
  canStatus (*setNotify)(HandleData *,
             /*void (*callback)(canNotifyData *)*/
             OS_IF_SET_NOTIFY_PARAM, unsigned int);
  canStatus (*busOn)(HandleData *);
  canStatus (*busOff)(HandleData *);
  canStatus (*setBusParams)(HandleData *hData, long freq, unsigned int tseg1, 
			    unsigned int tseg2, unsigned int sjw, unsigned int noSamp,
                            long freq_brs, unsigned int tseg1_brs, unsigned int tseg2_brs, 
			    unsigned int sjw_brs, unsigned int syncmode);
  canStatus (*getBusParams)(HandleData *hData, long *freq, unsigned int *tseg1, 
			    unsigned int *tseg2, unsigned int *sjw, unsigned int *noSamp,
                            long *freq_brs, unsigned int *tseg1_brs, unsigned int *tseg2_brs,
                            unsigned int *sjw_brs, unsigned int *syncmode);
  canStatus (*read)(HandleData *, long *, void *, unsigned int *,
                    unsigned int *, unsigned long *);
  canStatus (*readWait)(HandleData *, long *, void *, unsigned int *,
                        unsigned int *, unsigned long *, long);
  canStatus (*setBusOutputControl)(HandleData *, unsigned int);
  canStatus (*getBusOutputControl)(HandleData *, unsigned int *);
  canStatus (*accept)(HandleData *, const long, const unsigned int);
  canStatus (*write)(HandleData *, long, void *, unsigned int, unsigned int);
  canStatus (*writeWait)(HandleData *, long, void *,
                         unsigned int, unsigned int, long);
  canStatus (*writeSync)(HandleData *, unsigned long);
  canStatus (*getNumberOfChannels)(HandleData *, int *);
  canStatus (*readTimer)(HandleData *, unsigned long *);
  canStatus (*readErrorCounters)(HandleData *, unsigned int *,
                                 unsigned int *, unsigned int *);
  canStatus (*readStatus)(HandleData *, unsigned long *);
  canStatus (*getChannelData)(char *, int, void *, size_t);
  canStatus (*ioCtl)(HandleData * , unsigned int, void *, size_t);
  canStatus (*objbufFreeAll)(HandleData *hData);
  canStatus (*objbufAllocate)(HandleData *hData, int type, int *number);
  canStatus (*objbufFree)(HandleData *hData, int idx);
  canStatus (*objbufWrite)(HandleData *hData, int idx, int id, void* msg,
                           unsigned int dlc, unsigned int flags);
  canStatus (*objbufSetFilter)(HandleData *hData, int idx,
                               unsigned int code, unsigned int mask);
  canStatus (*objbufSetFlags)(HandleData *hData, int idx, unsigned int flags);
  canStatus (*objbufSetPeriod)(HandleData *hData, int idx, unsigned int period);
  canStatus (*objbufSetMsgCount)(HandleData *hData, int idx, unsigned int count);
  canStatus (*objbufSendBurst)(HandleData *hData, int idx, unsigned int burstLen);
  canStatus (*objbufEnable)(HandleData *hData, int idx);
  canStatus (*objbufDisable)(HandleData *hData, int idx);
} CANOps;


#endif
