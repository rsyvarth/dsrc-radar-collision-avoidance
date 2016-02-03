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

/*
** Description:
** Driver ioctl commands for Linux
**
*/

#ifndef _VCAN_IOCTL_H
#define _VCAN_IOCTL_H

#   include <asm/ioctl.h>

#   define VCAN_IOC_MAGIC 'v'



// See /Documentation/ioctl-number.txt
#define VCAN_IOC_SENDMSG            _IOW(VCAN_IOC_MAGIC, 101, int)
#define VCAN_IOC_RECVMSG            _IOR(VCAN_IOC_MAGIC, 102, int)
#define VCAN_IOC_OPEN_CHAN          _IO(VCAN_IOC_MAGIC,107)
#define VCAN_IOC_OPEN_TRANSP        _IO(VCAN_IOC_MAGIC,106)
#define VCAN_IOC_OPEN_EXCL          _IO(VCAN_IOC_MAGIC,105)
#define VCAN_IOC_GET_NRCHANNELS     _IO(VCAN_IOC_MAGIC,104)
#define VCAN_IOC_WAIT_EMPTY         _IO(VCAN_IOC_MAGIC,109)
#define VCAN_IOC_FLUSH_RCVBUFFER    _IO(VCAN_IOC_MAGIC,112)
#define VCAN_IOC_GET_STAT           _IO(VCAN_IOC_MAGIC,113)
#define VCAN_IOC_SET_WRITE_BLOCK    _IO(VCAN_IOC_MAGIC,114)
#define VCAN_IOC_SET_READ_BLOCK     _IO(VCAN_IOC_MAGIC,115)
#define VCAN_IOC_SET_WRITE_TIMEOUT  _IO(VCAN_IOC_MAGIC,116)
#define VCAN_IOC_SET_READ_TIMEOUT   _IO(VCAN_IOC_MAGIC,117)
#define VCAN_IOC_FLUSH_SENDBUFFER   _IO(VCAN_IOC_MAGIC,118)
#define VCAN_IOC_BUS_ON             _IO(VCAN_IOC_MAGIC,119)
#define VCAN_IOC_BUS_OFF            _IO(VCAN_IOC_MAGIC,120)
#define VCAN_IOC_SET_BITRATE        _IO(VCAN_IOC_MAGIC,121)
#define VCAN_IOC_GET_BITRATE        _IO(VCAN_IOC_MAGIC,122)
#define VCAN_IOC_SET_OUTPUT_MODE    _IO(VCAN_IOC_MAGIC,123)
#define VCAN_IOC_GET_OUTPUT_MODE    _IO(VCAN_IOC_MAGIC,124)
#define VCAN_IOC_SET_MSG_FILTER     _IO(VCAN_IOC_MAGIC,125)
#define VCAN_IOC_GET_MSG_FILTER     _IO(VCAN_IOC_MAGIC,126)
#define VCAN_IOC_READ_TIMER         _IO(VCAN_IOC_MAGIC,127)
#define VCAN_IOC_GET_TX_ERR         _IO(VCAN_IOC_MAGIC,128)
#define VCAN_IOC_GET_RX_ERR         _IO(VCAN_IOC_MAGIC,129)
#define VCAN_IOC_GET_OVER_ERR       _IO(VCAN_IOC_MAGIC,130)
#define VCAN_IOC_GET_RX_QUEUE_LEVEL _IO(VCAN_IOC_MAGIC,131)
#define VCAN_IOC_GET_TX_QUEUE_LEVEL _IO(VCAN_IOC_MAGIC,132)
#define VCAN_IOC_GET_CHIP_STATE     _IO(VCAN_IOC_MAGIC,133)
#define VCAN_IOC_GET_VERSION        _IO(VCAN_IOC_MAGIC,134)
#define VCAN_IOC_GET_TXACK          _IO(VCAN_IOC_MAGIC,135)
#define VCAN_IOC_SET_TXACK          _IO(VCAN_IOC_MAGIC,136)

#define VCAN_IOC_GET_SERIAL         _IO(VCAN_IOC_MAGIC,138)
#define VCAN_IOC_GET_EAN            _IO(VCAN_IOC_MAGIC,139)
#define VCAN_IOC_GET_CARD_NUMBER    _IO(VCAN_IOC_MAGIC,140)
#define VCAN_IOC_GET_CHAN_NO_ON_CARD _IO(VCAN_IOC_MAGIC,141)
#define VCAN_IOC_GET_FIRMWARE_REV   _IO(VCAN_IOC_MAGIC,142)
#define VCAN_IOC_SET_TIMER_SCALE    _IO(VCAN_IOC_MAGIC,143)
#define VCAN_IOC_GET_TIMER_SCALE    _IO(VCAN_IOC_MAGIC,144)
#define VCAN_IOC_GET_EVENTHANDLE    _IO(VCAN_IOC_MAGIC,145)
#define VCAN_IOC_GET_CHAN_CAP       _IO(VCAN_IOC_MAGIC,146)
#define VCAN_IOC_GET_TRANS_CAP      _IO(VCAN_IOC_MAGIC,147)
#define VCAN_IOC_GET_CHAN_FLAGS     _IO(VCAN_IOC_MAGIC,148)
#define VCAN_IOC_GET_CARD_TYPE      _IO(VCAN_IOC_MAGIC,149)

#define VCAN_IOC_SET_TXRQ           _IO(VCAN_IOC_MAGIC,150)

#define VCAN_IOC_GET_HARDWARE_REV   _IO(VCAN_IOC_MAGIC,151)

#define VCAN_IOC_SET_TXECHO         _IO(VCAN_IOC_MAGIC,152)

#define VCAN_IOC_SSP_AUTO           _IO(VCAN_IOC_MAGIC,154)
#define VCAN_IOC_SSP_GET            _IO(VCAN_IOC_MAGIC,155)
#define VCAN_IOC_SSP_SET            _IO(VCAN_IOC_MAGIC,156)

#define MAX_IOCTL_CARD_NAME       31
#define MAX_IOCTL_DRIVER_NAME     31
#define MAX_IOCTL_VENDOR_NAME     31
#define MAX_IOCTL_CHANNEL_PREFIX  31
#define MAX_IOCTL_CHANNEL_NAME    31

#define VCAN_CHANNEL_CAP_SEND_ERROR_FRAMES      0x00000001
#define VCAN_CHANNEL_CAP_RECEIVE_ERROR_FRAMES   0x00000002
#define VCAN_CHANNEL_CAP_TIMEBASE_ON_CARD       0x00000004
#define VCAN_CHANNEL_CAP_BUSLOAD_CALCULATION    0x00000008
#define VCAN_CHANNEL_CAP_OBJECT_BUFFER_MODE     0x00000010
#define VCAN_CHANNEL_CAP_TIME_SYNC              0x00000020
#define VCAN_CHANNEL_CAP_ABSOLUTE_TIME          0x00000040
#define VCAN_CHANNEL_CAP_EXTENDED_CAN           0x00000080
/* Flags for Kvasers channel capabilities */
#define VCAN_CHANNEL_CAP_ERROR_COUNTERS         0x00010000
#define VCAN_CHANNEL_CAP_CAN_DIAGNOSTICS        0x00020000
#define VCAN_CHANNEL_CAP_TXREQUEST              0x00040000
#define VCAN_CHANNEL_CAP_TXACKNOWLEDGE          0x00080000
#define VCAN_CHANNEL_CAP_VIRTUAL                0x00100000
#define VCAN_CHANNEL_CAP_SIMULATED              0x00200000
#define VCAN_CHANNEL_CAP_REMOTE                 0x00400000  // Device (channel) is remote, e.g. Iris
#define VCAN_CHANNEL_CAP_CANFD                  0x00800000  // Device (channel) supports CAN-FD
#define VCAN_CHANNEL_CAP_CANFD_NONISO           0x01000000  // Device (channel) supports non-ISO CAN-FD

#define VCAN_CHANNEL_STATUS_TIME_SYNC_ENABLED   0x00000001
#define VCAN_CHANNEL_STATUS_TIME_SYNC_RUNNING   0x00000002
#define VCAN_CHANNEL_STATUS_ON_BUS              0x00000004
#define VCAN_CHANNEL_STATUS_ACTIVATED           0x00000008

#endif /* _VCAN_IOCTL_H */
