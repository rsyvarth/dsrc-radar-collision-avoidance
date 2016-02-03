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

/* hwnames.h - defines for names and numbers for the different types of hardware */
#ifndef _HWNAMES_H_
#define _HWNAMES_H_


#ifdef HWTYPE_NONE
#  error HWTYPE_NONE is already defined.
#endif

//
// Hardware types
//
#define HWTYPE_NONE            0
#define HWTYPE_VIRTUAL         1
#define HWTYPE_CANCARDX        2
#define HWTYPE_CANPARI         3
#define HWTYPE_CANDONGLE       4
#define HWTYPE_CANAC2          5
#define HWTYPE_CANAC2PCI       6
#define HWTYPE_CANCARD         7
#define HWTYPE_PCCAN           8
#define HWTYPE_HERMES          9  // also the deceased ISAcan
#define HWTYPE_PCICAN          HWTYPE_HERMES
#define HWTYPE_NEWPCMCIA      10  // Was Mars; now unused.
#define HWTYPE_DAPHNE         11  // also HWTYPE_NEWUSB
#define HWTYPE_CANCARDY       12  // the one-channel CANcardX
/* 13-39 are reserved to Vector, just to be sure */
/* Vector will use all odd numbers and Kvaser all even numbers, as per a
   semi-official agreement */
// Vector CANcardXL           15  // Information only.
// Vector CANcaseXL           21  // Information only
#define HWTYPE_HELIOS         40  // Helios / PCIcan II
#define HWTYPE_PCICAN_II      HWTYPE_HELIOS
// Reserved to Vector      41
#define HWTYPE_DEMETER        42  // USBcan II, Memorator et al
#define HWTYPE_SIMULATION     44  // kcanc for Creator
#define HWTYPE_AURORA         46
#define HWTYPE_FILO           48  // Leaf
#define HWTYPE_LEAF           HWTYPE_FILO
#define HWTYPE_PC104_PLUS     50
#define HWTYPE_PCICANX_II     52
#define HWTYPE_MEMORATOR_II   54
#define HWTYPE_MEMORATOR_PRO  HWTYPE_MEMORATOR_II
#define HWTYPE_USBCAN_PRO     56
#define HWTYPE_BLACKBIRD      58
#define HWTYPE_IRIS           HWTYPE_BLACKBIRD
#define HWTYPE_MEMORATOR_LIGHT  60  // Memorator Light
#define HWTYPE_MINIHYDRA      62
#define HWTYPE_EAGLE          62
#define HWTYPE_BAGEL          64
#define HWTYPE_MINIPCIE       66
#define HWTYPE_USBCAN_KLINE   68
#define HWTYPE_ETHERCAN       70
#define HWTYPE_USBCAN_LIGHT   72
#define HWTYPE_USBCAN_PRO2    74
#define HWTYPE_PCIE_V2        76
#define HWTYPE_PCIECANFD      HWTYPE_PCIE_V2
#define HWTYPE_MEMORATOR_PRO2 78
#define HWTYPE_LEAF2              80  ///< Kvaser Leaf Pro HS (2nd generation)
#define HWTYPE_MEMORATOR_V2   82  ///< Kvaser Memorator (2nd generation)


/* Transceiver types */
#ifdef VCAN_TRANSCEIVER_TYPE_NONE
#  error VCAN_TRANSCEIVER_TYPE_NONE is already defined.
#endif
#define VCAN_TRANSCEIVER_TYPE_NONE              0
#define VCAN_TRANSCEIVER_TYPE_251               1
#define VCAN_TRANSCEIVER_TYPE_252               2
#define VCAN_TRANSCEIVER_TYPE_DNOPTO            3
#define VCAN_TRANSCEIVER_TYPE_W210              4
#define VCAN_TRANSCEIVER_TYPE_SWC_PROTO         5  // Prototype. Driver may latch-up.
#define VCAN_TRANSCEIVER_TYPE_SWC               6
#define VCAN_TRANSCEIVER_TYPE_FIBER             8
#define VCAN_TRANSCEIVER_TYPE_K                10  // K-line, without CAN
#define VCAN_TRANSCEIVER_TYPE_1054_OPTO        11  // 1054 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_SWC_OPTO         12  // SWC with optical isolation
#define VCAN_TRANSCEIVER_TYPE_B10011S          13  // B10011S truck-and-trailer
#define VCAN_TRANSCEIVER_TYPE_1050             14  // 1050
#define VCAN_TRANSCEIVER_TYPE_1050_OPTO        15  // 1050 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_1041             16  // 1041
#define VCAN_TRANSCEIVER_TYPE_1041_OPTO        17  // 1041 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_RS485            18  // J1708
#define VCAN_TRANSCEIVER_TYPE_LIN              19  // LIN
#define VCAN_TRANSCEIVER_TYPE_KONE             20  // KONE piggyback
#define VCAN_TRANSCEIVER_TYPE_GAL              VCAN_TRANSCEIVER_TYPE_KONE  // Galathea piggyback
// Agreement: Vector to use all odd numbers, Kvaser all even numbers
#define VCAN_TRANSCEIVER_TYPE_CANFD            22  // Generic CAN-FD capable transceiver
#define VCAN_TRANSCEIVER_TYPE_LINX_LIN         64
#define VCAN_TRANSCEIVER_TYPE_LINX_J1708       66
#define VCAN_TRANSCEIVER_TYPE_LINX_K           68
#define VCAN_TRANSCEIVER_TYPE_LINX_SWC         70
#define VCAN_TRANSCEIVER_TYPE_LINX_LS          72


/* old style transciver type names */
#define TRANSCEIVER_TYPE_NONE           VCAN_TRANSCEIVER_TYPE_NONE
#define TRANSCEIVER_TYPE_251            VCAN_TRANSCEIVER_TYPE_251
#define TRANSCEIVER_TYPE_252            VCAN_TRANSCEIVER_TYPE_252
#define TRANSCEIVER_TYPE_DNOPTO         VCAN_TRANSCEIVER_TYPE_DNOPTO
#define TRANSCEIVER_TYPE_W210           VCAN_TRANSCEIVER_TYPE_W210
#define TRANSCEIVER_TYPE_SWC_PROTO      VCAN_TRANSCEIVER_TYPE_SWC_PROTO
#define TRANSCEIVER_TYPE_SWC            VCAN_TRANSCEIVER_TYPE_SWC

#endif
