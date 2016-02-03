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
**   Definitions for the DPRAM in Helios (PCIcan II)
**   This file is intended both for Win32, M16C and Linux; beware.
**
*/

#ifndef _HELIOS_DPRAM_H_
#define _HELIOS_DPRAM_H_

#ifdef HAL_HELIOS
// Compiling for M16C
#  define DPRAM_BASE 0x6000
#else
#  define DPRAM_BASE 0
#endif

/* 
Each address in the DPRAM buffer can be accessed with or without 
mutual exclusion, Bits [11:0]  set the memory address whereas 
bit12 set mutual exclusion. (when high)  
*/
#define DPRAM_MUTEX 0x1000

// Addr (Hex)   Register
// 0-3          Control register
// 4-5          Host rx pointer LOW
// 6-7          Host rx pointer HIGH
// 8-9          Host tx pointer LOW
// A-B          Host tx pointer HIGH
// C-D          M16c rx pointer LOW
// E-F          M16c rx pointer HIGH
// 10-11        M16c tx pointer LOW
// 12-13        M16c tx pointer HIGH

// qq This is not the case anymore.
// 000c - 05ff  Receive buffer (read by host, written by M16C)
// 0600 - 0bff  Transmit buffer (written by host, read by M16C)
// 0c00 - 0c03  Timer
// 0d00 - 0d03  Interrupt control for the host


//DPRAM receive (M16C to host) buffer start

#define DPRAM_RX_BUF_START  (DPRAM_BASE + 0x0100)
#define DPRAM_RX_BUF_END    (DPRAM_BASE + 0x05ff)
#define DPRAM_RX_BUF_SIZE   (DPRAM_RX_BUF_END - DPRAM_RX_BUF_START + 1)


//DPRAM transmit (host to M16C) buffer start
#define DPRAM_TX_BUF_START  (DPRAM_BASE + 0x0600)
#define DPRAM_TX_BUF_END    (DPRAM_BASE + 0x0bff)
#define DPRAM_TX_BUF_SIZE   (DPRAM_TX_BUF_END - DPRAM_TX_BUF_START + 1)

//
// Special purpose addresses in the DPRAM buffer
//

// Special function register
#define DPRAM_CTRLREG     (DPRAM_BASE + DPRAM_MUTEX + 0x0000)

// 32-bit pointers
#define DPRAM_HOST_READ_PTR  (DPRAM_BASE + DPRAM_MUTEX + 0x0004)
#define DPRAM_HOST_WRITE_PTR (DPRAM_BASE + DPRAM_MUTEX + 0x0008)
#define DPRAM_M16C_READ_PTR  (DPRAM_BASE + DPRAM_MUTEX + 0x000C)
#define DPRAM_M16C_WRITE_PTR (DPRAM_BASE + DPRAM_MUTEX + 0x0010)

#ifdef HAL_HELIOS

#define HOST_READ_PTR_LOW   ((unsigned short*)(DPRAM_HOST_READ_PTR))
#define HOST_READ_PTR_HIGH  ((unsigned short*)(DPRAM_HOST_READ_PTR + 2))

#define HOST_WRITE_PTR_LOW ((unsigned short*)(DPRAM_HOST_WRITE_PTR))
#define HOST_WRITE_PTR_HIGH ((unsigned short*)(DPRAM_HOST_WRITE_PTR + 2))

#define M16C_READ_PTR_LOW  ((unsigned short*)(DPRAM_M16C_READ_PTR))
#define M16C_READ_PTR_HIGH  ((unsigned short*)(DPRAM_M16C_READ_PTR + 2))

#define M16C_WRITE_PTR_LOW ((unsigned short*)(DPRAM_M16C_WRITE_PTR))
#define M16C_WRITE_PTR_HIGH ((unsigned short*)(DPRAM_M16C_WRITE_PTR + 2))

#endif

// Interrupt register - 8 bits, read/write, accessible from the host side only
#define DPRAM_INTERRUPT_REG       (DPRAM_BASE + 0xd00)
#define DPRAM_INTERRUPT_DISABLE   0x01    // Set to 0 to enable interrupts
#define DPRAM_INTERRUPT_ACK       0x02    // Set to 1 and back to 0 again to clear interrupt
#define DPRAM_INTERRUPT_ACTIVE    0x04    // 1 when interrupt is active


// Timer register; runs with 1 MHz and can't (currently) be cleared.
#define DPRAM_TIMER_REG   (DPRAM_BASE + 0x0F00)


// Bit definitions for the control register 
#define DPRAM_CTRLREG_BIST_DONE           0x00000001
#define DPRAM_CTRLREG_BIST_OK             0x00000002
#define DPRAM_CTRLREG_FIRMWARE_REV_MASK   0xFF000000
#define DPRAM_CTRLREG_FIRMWARE_REV_SHIFT  24

#endif
