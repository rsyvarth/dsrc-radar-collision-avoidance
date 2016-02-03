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

/* Kvaser CAN driver PCIcan hardware specific parts                    
** SJA1000 data                                                     
*/

#ifndef _SJA1000_H_
#define _SJA1000_H_

/* Mode Register */
#define PCAN_MOD             0
#define PCAN_SM              0x10
#define PCAN_AFM             0x08
#define PCAN_STM             0x04
#define PCAN_LOM             0x02
#define PCAN_RM              0x01

/* Command Register */
#define PCAN_CMR             1
#define PCAN_SRR             0x10
#define PCAN_CDO             0x08
#define PCAN_RRB             0x04
#define PCAN_AT              0x02
#define PCAN_TR              0x01

#define PCAN_CMD_SINGLESHOT       (PCAN_AT|PCAN_TR)
#define PCAN_CMD_SINGLESHOT_SELF  (PCAN_SRR|PCAN_AT)

/* Status Register */
#define PCAN_SR              2
#define PCAN_BS              0x80
#define PCAN_ES              0x40
#define PCAN_TS              0x20
#define PCAN_RS              0x10
#define PCAN_TCS             0x08
#define PCAN_TBS             0x04
#define PCAN_DOS             0x02
#define PCAN_RBS             0x01

/* Interrupt Register */
#define PCAN_IR              3
#define PCAN_BEI             0x80
#define PCAN_ALI             0x40
#define PCAN_EPI             0x20
#define PCAN_WUI             0x10
#define PCAN_DOI             0x08
#define PCAN_EI              0x04
#define PCAN_TI              0x02
#define PCAN_RI              0x01

/* Interrupt Enable Register */
#define PCAN_IER             4
#define PCAN_BEIE            0x80
#define PCAN_ALIE            0x40
#define PCAN_EPIE            0x20
#define PCAN_WUIE            0x10
#define PCAN_DOIE            0x08
#define PCAN_EIE             0x04
#define PCAN_TIE             0x02
#define PCAN_RIE             0x01


/* Bus Timing Register 0 */
#define PCAN_BTR0            6

/* Bus Timing Register 1 */
#define PCAN_BTR1            7

/* Output Control Register */
#define PCAN_OCR             8
#define PCAN_OCTP1           0x80
#define PCAN_OCTN1           0x40
#define PCAN_OCPOL1          0x20
#define PCAN_OCTP0           0x10
#define PCAN_OCTN0           0x08
#define PCAN_OCPOL0          0x04
#define PCAN_OCM1            0x02
#define PCAN_OCM0            0x01

#define PCAN_ALC             11

#define PCAN_ECC             12
#define PCAN_ECC_ERROR_MASK  0xC0
#define PCAN_ECC_BIT_ERROR   0x00
#define PCAN_ECC_FORM_ERROR  0x40
#define PCAN_ECC_STUFF_ERROR 0x80
#define PCAN_ECC_OTHER_ERROR 0xc0
#define PCAN_ECC_DIR_MASK    0x20
#define PCAN_ECC_DIR_TX      0x00
#define PCAN_ECC_DIR_RX      0x20
#define PCAN_ECC_SEGMENT_MASK 0x1F

#define PCAN_EWLR            13
#define PCAN_RXERR           14
#define PCAN_TXERR           15

#define PCAN_ACR0            16
#define PCAN_ACR1            17
#define PCAN_ACR2            18
#define PCAN_ACR3            19
#define PCAN_AMR0            20
#define PCAN_AMR1            21
#define PCAN_AMR2            22
#define PCAN_AMR3            23

#define PCAN_RMC             29
#define PCAN_RBSA            30
#define PCAN_CDR             31
#define PCAN_PELICAN         0x80
#define PCAN_CBP             0x40
#define PCAN_RXINTEN         0x20
#define PCAN_CLOCKOFF        0x08

#define PCAN_FF_EXTENDED     0x80
#define PCAN_FF_REMOTE       0x40

#define PCAN_MSGBUF          16
#define PCAN_SID0            17
#define PCAN_SID1            18
#define PCAN_XID0            17
#define PCAN_XID1            18
#define PCAN_XID2            19
#define PCAN_XID3            20
#define PCAN_SDATA           19
#define PCAN_XDATA           21

#define PCAN_RXFIFO          32
#define PCAN_RXFIFO_SIZE     64

typedef struct {
    unsigned char mode;
    unsigned char command;
    unsigned char status;
    unsigned char intr;
    unsigned char intrEnable;
    unsigned char _reserved0;
    unsigned char btr0;
    unsigned char btr1;
    unsigned char outputCtrl;
    unsigned char test;
    unsigned char _reserved1;
    unsigned char arbLost;
    unsigned char errorCode;
    unsigned char errorWarningLimit;
    unsigned char rxError;
    unsigned char txError;
    unsigned char frameFormat;
    union {
        struct {
            unsigned char id0;
            unsigned char id1;
            unsigned char data[8];
        } std;
        struct {
            unsigned char id0;
            unsigned char id1;
            unsigned char id2;
            unsigned char id3;
            unsigned char data[8];
        } ext;
    } msg;
    unsigned char _reserved3;
    unsigned char rxBufStart;
    unsigned char clockDivider;
} __sja1000;

#define PCAN_EPOS_START_OF_FRAME        0x03
#define PCAN_EPOS_ID28_ID21             0x02
#define PCAN_EPOS_ID20_ID18             0x06
#define PCAN_EPOS_SRTR                  0x04
#define PCAN_EPOS_IDE                   0x05
#define PCAN_EPOS_ID17_ID13             0x07
#define PCAN_EPOS_ID12_ID5              0x0F
#define PCAN_EPOS_ID4_ID0               0x0E
#define PCAN_EPOS_RTR                   0x0C
#define PCAN_EPOS_RESVD_1               0x0D
#define PCAN_EPOS_RESVD_2               0x09
#define PCAN_EPOS_DLC                   0x0B
#define PCAN_EPOS_DATA                  0x0A
#define PCAN_EPOS_CRC                   0x08
#define PCAN_EPOS_CRC_DELIM             0x18
#define PCAN_EPOS_ACK                   0x19
#define PCAN_EPOS_ACK_DELIM             0x1B
#define PCAN_EPOS_EOF                   0x1A
#define PCAN_EPOS_INTERMISSION          0x12
#define PCAN_EPOS_ACTIVE_ERROR          0x11
#define PCAN_EPOS_PASSIVE_ERROR         0x16
#define PCAN_EPOS_TOLERATE_DOM          0x13
#define PCAN_EPOS_ERROR_DELIM           0x17
#define PCAN_EPOS_OVERLOAD              0x1C


#endif
