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

#ifndef _TL16564_H_
#define _TL16564_H_

// --- Addresses as seen from the subsystem (and some subsystem-only bit values)
#define TL_CTRL           0x110
#define TL_CTRL_COM1      0x00  /* We're COM1 when in address mode */
#define TL_CTRL_COM2      0x01  /* We're COM2 when in address mode */
#define TL_CTRL_COM3      0x02  /* We're COM3 when in address mode */
#define TL_CTRL_COM4      0x03  /* We're COM4 when in address mode */
#define TL_CTRL_CPUINT    0x04  /* Enable CPU interrupts via STSCHG# */
#define TL_CTRL_ADDRMODE  0x08  /* Enable Address Mode */
#define TL_CTRL_MODE_MASK 0x30
#define TL_CTRL_IO        0x10  /* I/O card */
#define TL_CTRL_MEM_BUSY  0x20  /* Memory card, BUSY */
#define TL_CTRL_MEM_READY 0x30  /* Memory card, READY */
#define TL_CTRL_RESET_IRQ 0x40  /* Reset subsystem IRQ signal */
#define TL_CTRL_BYPASS    0x80  /* Enable serial-bypass mode */

#define TL_PGMCLK     0x120  // WO

#define TL_MCR_B5     0x130  // WO - this is the auto-CTS bit
#define TL_DLL        0x130  // RO
#define TL_IER        0x131  // RO
#define TL_FCR        0x132  // RO
#define TL_LCR        0x133  // RO
#define TL_MCR        0x134  // RO
#define TL_LSR        0x135  // RO
#define TL_MSR        0x136  // RO
#define TL_DLM        0x137  // RO
#define TL_FIFO       0x140


// --- Addresses seen from the host (relative to the FIFO)
#define TLH_FIFO      0x00
#define TLH_IER       0x01
#define TLH_IIR       0x02  // RO
#define TLH_FCR       0x02  // WO
#define TLH_LCR       0x03
#define TLH_MCR       0x04
#define TLH_LSR       0x05
#define TLH_MSR       0x06
#define TLH_SCR       0x07
// When LCR.DLAB is set:
#define TLH_DLL       0x00
#define TLH_DLM       0x01



// Various bit values in the control registers
/* Interrupt Enable Register, IER */
#define IER_ERBI      0x01
#define IER_ETBEI     0x02
#define IER_ELSI      0x04
#define IER_EDSSI     0x08

/* Interrupt Identification Register, IIR */
#define IIR_NOINTPEND     0x01
#define IIR_ID1           0x02
#define IIR_ID2           0x04
#define IIR_ID3           0x08
#define IIR_FIFO_IS_ENA1  0x40
#define IIR_FIFO_IS_ENA2  0x80

// The values for different interrupt reasons
#define IIR_INTID_MASK        0x06
#define INT_RCV_LINE_STATUS   0x06  //  110
#define INT_RCV_DATA          0x04  // .100
#define INT_TX_HOLD_REG_EMPTY 0x02  //  010
#define INT_MODEM_STATUS      0x00  //  000
#define INT_CHAR_TIMEOUT      0x0c  // 1100

/* FIFO Control Register, FCR */
#define FCR_FIFO_ENABLE   0x01
#define FCR_RXFIFO_RESET  0x02
#define FCR_TXFIFO_RESET  0x04
#define FCR_DMAMODE       0x08
#define FCR_FDEPTH64      0x20      // Note: "reserved" in 16C550C
#define FCR_AUTO_RTS      0x10      // Note: "reserved" in 16C550C

/* Line Control Register, LCR */
#define LCR_WLS0          0x01
#define LCR_WLS1          0x02
#define LCR_STOPBITS      0x04
#define LCR_PARITY_ENABLE 0x08
#define LCR_EVEN_PARITY   0x10
#define LCR_STICK_PARITY  0x20
#define LCR_BREAK         0x40
#define LCR_DLAB          0x80

/* Modem Control Register, MCR */
#define MCR_DTR       0x01
#define MCR_RTS       0x02
#define MCR_OUT1      0x04
#define MCR_OUT2      0x08
#define MCR_LOOP      0x10
/* This bit has another meaning in the 564:
#define MCR_AFE       0x20
*/
/* This definition isn't meaningful in the 564:
#define MCR_AUTOFLOW  0x22 // Both AFE and RTS
*/
#define MCR_RESERVED1 0x20


/* Line Status Register, LSR */
#define LSR_DATA_READY    0x01
#define LSR_OVERRUN       0x02
#define LSR_PARITY_ERR    0x04
#define LSR_FRAMING_ERR   0x08
#define LSR_BREAK_INT     0x10
#define LSR_THRE          0x20
#define LSR_TEMT          0x40
#define LSR_RCVERR        0x80

/* Modem Status Register, MSR */
#define MSR_DELTA_CTS   0x01
#define MSR_DELTA_DSR   0x02
#define MSR_TERI        0x04
#define MSR_DELTA_DCD   0x08
#define MSR_CTS         0x10
#define MSR_DSR         0x20
#define MSR_RI          0x40
#define MSR_DCD         0x80

#endif // _TL16564_H_
