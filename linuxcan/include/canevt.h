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

#ifndef _CANEVT_H_
#define _CANEVT_H_

#include <pshpack1.h>

#define CANEVT_NO_COMMAND            0

  // Events
#define CANEVT_RECEIVE_MSG           1
#define CANEVT_STATISTIC_STD         2
#define CANEVT_STATISTIC_EXT         3
#define CANEVT_CHIP_STATE            4
#define CANEVT_CLOCK_OVERFLOW        5
#define CANEVT_TRIGGER               6
#define CANEVT_ERROR                 7
#define CANEVT_TIMER                 8
#define CANEVT_TRANSCEIVER           9

  // Commands
#define CANEVT_TRANSMIT_MSG          10
#define CANEVT_SET_TIMER             11
#define CANEVT_RESET_CLOCK           12
#define CANEVT_INIT_CHIP             13
#define CANEVT_START_CHIP            14
#define CANEVT_STOP_CHIP             15
#define CANEVT_RESET_CHIP            16
#define CANEVT_SET_ACCEPTANCE        17
//#define CANEVT_SET_DRIVERMODE        18
#define CANEVT_GET_STATISTIC         19
#define CANEVT_GET_CHIP_STATE        20
#define CANEVT_SET_OUTPUT_MODE       21
#define CANEVT_FUNCTION              22
#define CANEVT_GET_TRANSCEIVER       23
#define CANEVT_GET_TRANSCEIVER_RESP  24
#define CANEVT_SET_TRANSCEIVER       25
// 26, 27, 28 reserved
#define CANEVT_INIT_HARDWARE         29
#define CANEVT_EXIT_HARDWARE         30
#define CANEVT_READ_CLOCK            31

// TODO: Change to 64 for CAN FD
#define MAX_MSG_LEN 8
#define EXT_MSG                     0x80000000  // signs an extended identifier

#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_OVERRUN             0x02        // Msgs following this has been lost
#define MSGFLAG_NERR                0x04        // NERR active during this msg
#define MSGFLAG_WAKEUP              0x08        // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#define MSGFLAG_RESERVED_1          0x20        // Reserved for future usage
#define MSGFLAG_TX                  0x40        // TX acknowledge
#define MSGFLAG_TXRQ                0x80        // TX request

typedef struct {
  unsigned long id;
  unsigned char flags;
  unsigned char dlc;
  unsigned char data[MAX_MSG_LEN];
} CanEventMessage;


#define CHIPSTAT_BUSOFF              0x01
#define CHIPSTAT_ERROR_PASSIVE       0x02
#define CHIPSTAT_ERROR_WARNING       0x04
#define CHIPSTAT_ERROR_ACTIVE        0x08

typedef struct {
  unsigned char busStatus;
  unsigned char txErrorCounter;
  unsigned char rxErrorCounter;
} CanEventChipState;

typedef struct {
  unsigned long  stdData;
  unsigned long  stdRemote;
  unsigned long  errFrame;
  unsigned short busLoad; // 0.00-100.00%
} CanEventStatisticsStd;

typedef struct {
  unsigned long  extData;
  unsigned long  extRemote;
  unsigned long  ovrFrame;
} CanEventStatisticsExt;

typedef struct {
  unsigned char code; // qqq
} CanEventErrorCode;

//------------------------------------------------------------------------------
// commands
//------------------------------------------------------------------------------

typedef struct {
  unsigned long code;
  unsigned long mask;
} CanEventAcceptanceFilter;

typedef struct {
  unsigned char mode;
} CanEventDriverMode;

typedef struct {
  unsigned long rate;
} CanEventSetTimer;

typedef struct {
  unsigned char level;
} CanEventSetDebugLevel;

#define OUTPUT_MODE_SILENT 0
#define OUTPUT_MODE_NORMAL 1

typedef struct {
  unsigned char mode;
} CanEventSetOutputMode;

typedef struct {
  unsigned char code;
} CanEventFunction;

// The following definitions are copied from lapcmds.h !!

#define TRANSCEIVER_EVENT_ERROR   1
#define TRANSCEIVER_EVENT_CHANGED 2

// event
#define TRANSCEIVER_EVENT_NONE           0
#define TRANSCEIVER_EVENT_NERR           1
#define TRANSCEIVER_EVENT_REMOVED        2
#define TRANSCEIVER_EVENT_DETECTED       3

// type
#define TRANSCEIVER_TYPE_NONE     0
#define TRANSCEIVER_TYPE_251      1
#define TRANSCEIVER_TYPE_252      2
#define TRANSCEIVER_TYPE_DNOPTO   3
#define TRANSCEIVER_TYPE_W210     4
#define TRANSCEIVER_TYPE_SWC_PROTO       5  // Prototype. Driver may latch-up.
#define TRANSCEIVER_TYPE_SWC             6


// lineMode
#define TRANSCEIVER_LINEMODE_NA       0
#define TRANSCEIVER_LINEMODE_TWO_LINE 1
#define TRANSCEIVER_LINEMODE_CAN_H    2
#define TRANSCEIVER_LINEMODE_CAN_L    3
#define TRANSCEIVER_LINEMODE_SWC_SLEEP   4  // SWC Sleep Mode.
#define TRANSCEIVER_LINEMODE_SWC_NORMAL  5  // SWC Normal Mode.
#define TRANSCEIVER_LINEMODE_SWC_FAST    6  // SWC High-Speed Mode.
#define TRANSCEIVER_LINEMODE_SWC_WAKEUP  7  // SWC Wakeup Mode.
#define TRANSCEIVER_LINEMODE_SLEEP    8
#define TRANSCEIVER_LINEMODE_NORMAL   9


// resNet
#define TRANSCEIVER_RESNET_NA          0
#define TRANSCEIVER_RESNET_MASTER      1
#define TRANSCEIVER_RESNET_MASTER_STBY 2
#define TRANSCEIVER_RESNET_SLAVE       3

typedef struct {
  unsigned char event;       // TRANSCEIVER_EVENT_xxx
} CanEventTransceiver;

typedef struct {
  unsigned char type;        // TRANSCEIVER_TYPE_xxx
  unsigned char lineMode;    // TRANSCEIVER_LINEMODE_xxx
  unsigned char resNet;      // TRANSCEIVER_RESNET_xxx
} CanEventGetTransceiverResponse;

typedef struct {
  unsigned char type;       // TRANSCEIVER_TYPE_xxx
  unsigned char lineMode;   // TRANSCEIVER_LINEMODE_xxx
  unsigned char resNet;     // TRANSCEIVER_RESNET_xxx
} CanEventSetTransceiver;


typedef union {
    CanEventMessage                 canEventMessage;
    CanEventChipState               canEventChipState;
    CanEventStatisticsStd           canEventStatisticsStd;
    CanEventStatisticsExt           canEventStatisticsExt;
    CanEventErrorCode               canEventErrorCode;
    CanEventAcceptanceFilter        canEventAcceptanceFilter;
    CanEventDriverMode              canEventDriverMode;
    CanEventSetTimer                canEventSetTimer;
    CanEventSetDebugLevel           canEventSetDebugLevel;
    CanEventSetOutputMode           canEventSetOutputMode;
    CanEventFunction                canEventFunction;
    CanEventTransceiver             canEventTransceiver;
    CanEventGetTransceiverResponse  canEventGetTransceiverResponse;
    CanEventSetTransceiver          canEventSetTransceiver;
} CanEventUnion;


typedef struct {
  unsigned char tag;
  unsigned char chanIndex;
  unsigned char _internal1;
  unsigned char _internal2;
  unsigned long timeStamp;
  CanEventUnion x;
} CanEvent, *PCanEvent;


#include <poppack.h>

#endif

