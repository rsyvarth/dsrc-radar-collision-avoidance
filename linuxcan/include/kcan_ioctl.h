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

/* kcan_ioctl.h: ioctls()'s specific for Kvasers CAN drivers */

#ifndef _KCAN_IOCTL_H
#define _KCAN_IOCTL_H

//#   include <linux/ioctl.h>
#   include <asm/ioctl.h>

#   define KCAN_IOC_MAGIC 'k'

// For compatibility with Windows #define:s below.
#define VCAN_DEVICE      0     // dummy
#define KCAN_IOCTL_START 0
#define METHOD_BUFFERED  0     // dummy
#define FILE_ANY_ACCESS  0

// qqq This likely is not a good idea!

#define CTL_CODE(x,i,y,z) _IO(KCAN_IOC_MAGIC, (i))


#define KCAN_IOCTL_OBJBUF_FREE_ALL              CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 6, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ALLOCATE              CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 7, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_FREE                  CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 8, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_WRITE                 CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 9, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FILTER            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 10, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FLAGS             CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 11, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ENABLE                CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 12, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_DISABLE               CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 13, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_PERIOD            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 22, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SEND_BURST            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 23, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_MSG_COUNT         CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 34, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_CANFD                        CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 69, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_CARDFLAG_FIRMWARE_BETA       0x01  // Firmware is beta
#define KCAN_CARDFLAG_FIRMWARE_RC         0x02  // Firmware is release candidate
#define KCAN_CARDFLAG_AUTO_RESP_OBJBUFS   0x04  // Firmware supports auto-response object buffers
#define KCAN_CARDFLAG_REFUSE_TO_RUN       0x08  // Major problem detected
#define KCAN_CARDFLAG_REFUSE_TO_USE_CAN   0x10  // Major problem detected
#define KCAN_CARDFLAG_AUTO_TX_OBJBUFS     0x20  // Firmware supports periodic transmit object buffers


#define KCAN_DRVFLAG_BETA                 0x01    // Driver is beta

#if defined(DEVHND_DRIVER_IS_BETA)
CompilerAssert(KCAN_DRVFLAG_BETA == DEVHND_DRIVER_IS_BETA);
#endif


#define CAN_CANFD_SUCCESS          0
#define CAN_CANFD_MISMATCH        -1
#define CAN_CANFD_NOT_IMPLEMENTED -2
#define CAN_CANFD_FAILURE         -3
#define CANFD 1
#define CAN   0
#define CAN_CANFD_SET          1
#define CAN_CANFD_READ         2
#define CAN_CANFD_READ_VERSION 3

typedef struct {
  unsigned int fd;   // CANFD, CAN
  unsigned int action; // CAN_CANFD_SET, CAN_CANFD_READ
  unsigned int reply; // reply from read?
  int status; // CAN_CANFD_MATCHING, CAN_CANFD_MISMATCH
  unsigned int unused[8];
} KCAN_IOCTL_CANFD_T;


#endif /* KCANIO_H */

