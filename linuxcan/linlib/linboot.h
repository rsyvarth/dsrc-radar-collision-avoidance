/**
 *                Copyright 2012 by Kvaser AB, Mölndal, Sweden
 *                        http://www.kvaser.com
 *
 * \section LICENSE
 * This software is dual licensed under the following two licenses:
 * BSD-new and GPLv2. You may use either one. See the included
 * COPYING file for details.
 *
 * License: BSD-new
 * ===============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License: GPLv2
 * ===============================================================================
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * ---------------------------------------------------------------------------
 */

#ifndef __LINBOOT_H
#define __LINBOOT_H

/****************************************************************************/
/* Memory layout. The bootloader entry and the firmware entry must be at fixed
 * positions.
 * Erasing the flash memory must be done in blocks of 64 bytes, so the firmware
 * can't start below address 0x40. The code at the interrupt vectors at 0x08
 * and 0x18 is contained in the boot code which jumps to application code.
 * No registers are saved besides the automatic save done for hig priority
 * interrupts; if we used RAM locations to do this, there could be colissions
 * between the boot code and the application code.
 */

/* Adress  Len   File    Description
 * 0x0     6     boot.c  Reset code. Jumps to boot loader.
 * 0x8     <=16  lin.c   Interrupt vector
 * 0x18    <=40
 * 0x40          lin.c   High priority interrupts
 * 0x60          lin.c   Low priority interrupts. Must save registers
 * 0x80          lin.c   Firmware main
 *               boot.c  Bootloader warm start
 *               boot.c  Bootloader cold start
 */
#define ORIGIN_FW_INT_HIGH      0x40
#define ORIGIN_FW_INT_LOW       0x60
#define ORIGIN_FIRMWARE_MAIN    0x80
#define ORIGIN_FIRMWARE ORIGIN_FW_INT_HIGH // Start of firmware code area
#if ICD2 // Debugger uses 576 bytes flash at address 0x7dc0..0x7fff
  #define ORIGIN_BOOTLOADER_WARM  0x7100 // It is possible to jump here for a warm start from the firmware
  #define ORIGIN_BOOTLOADER ORIGIN_BOOTLOADER_WARM // Start of boot code area (besides reset and interrupt vectors at address 0)
#else
  #define ORIGIN_BOOTLOADER_WARM  0x7400 // It is possible to jump here for a warm start from the firmware
  #define ORIGIN_BOOTLOADER ORIGIN_BOOTLOADER_WARM // Start of boot code area
#endif

// We use some bits in the CAN id to signal things about the messages.
// The parity bits are not used for parity, but for status information.
// For messages received from the host, we strip the high bits including the
// parity bits which we recalculates ourselves.
// A valid LIN id is in the range 0x00..0x3f.
// The highest bit (0x400) is set for commands.
#define LINID_DONGLE_RCV 0x40 // Added to messages the dongle receives from the LIN bus.
#define LINID_DONGLE_TX 0x80 // When the slave is polled for a message, or the master transmits a LIN message, the same is transmitted to the host with this bit set.
#define LINID_CSUM_ERROR 0x100 // Checksum or parity error. The actual parity and checksum we received can be found in the timing message.
#define LINID_NO_DATA 0x200 // A header was received but no data (and no checksum). No slave filled in the data portion.


/* CAN-IDs */
#define CAN_ID_COMMAND          0x7ff /* CMD, CCNT, .... */
#define CAN_ID_RESPONSE         0x7fe /* CMD, CCNT, STATUS, .... */
#define CAN_ID_STATUS           0x7fd
#define CAN_ID_TIMING           0x7fc
#define CAN_ID_TIMING0          0x7f8
#define CAN_ID_TX_WAKEUP        0x7fb // Transmitted to a slave dongle, the slave will transmit a wake-up signal. Optionally two parameters, {count, interval}
#define CAN_ID_RX_WAKEUP        0x7f9 // Transmitted to Navigator in when receiving a wake up frame.
#define CAN_ID_TXE_WAKEUP       0x7f4 // Transmitted to Navigator as an echo of a transmitted wake up frame.
#define CAN_ID_SYNCHERROR       0x7fa // Messagecan contain one data byte with a LIN_SYNCHERROR_- error code.
#define CAN_ID_TIMING_SYNCH     0x7f7 // Contains four 16-bit timer values for the Rec->Dom edges in the synch field
#define CAN_ID_TIMING_DATA0     0x7f6 // Contains 16-bit timer values for the start bits of the message
#define CAN_ID_TIMING_DATA1     0x7f5
#define CAN_ID_TERM             0x7f2

#define CAN_ID_PASSTHRU_RX      0x7f0
#define CAN_ID_PASSTHRU_TX      0x7f1

#define CAN_ID_DEBUG            0x201


// Different synch errors:
#define LIN_SYNCHERROR_SYNCH_BREAK 1 // Timeout while synch break (too long synch break)
#define LIN_SYNCHERROR_NO_SYNCH_FIELD 2 // Timeout while waiting for the synch field to begin
#define LIN_SYNCHERROR_SYNCH_FIELD 3 // Timeout while waiting for the synch field to complete
#define LIN_SYNCHERROR_SHORT_SYNCH_BREAK 4 // Wrong combination of synch break length and syncg field length

/* Commands
* The message format is
*  {cmd, ccnt, ?,?,?,?,?,?}
*/
#define COMMAND_RESET                   0x00 /* {CMD, CCNT, 0,0,0,0,0,0} -> - */
#define COMMAND_GET_VERSION             0x01 /* {CMD, CCNT, 0,0,0,0,0,0} -> {CMD,CCNT,flag, Major,Minor,Build, day:5,month:4,year:7} */
#define COMMAND_GET_STATUS              0x02 /* {CMD, CCNT, 0,0,0,0,0,0} -> {CMD,CCNT,flag, appFlag, ...*/

// Values for appFlag: (for boot versions before 2.2 (2002-10-07),
// appFlag is in the range 0..3).
#define LINSTAT_BOOT    0x80 // The boot loop is running; status message ends with {..., bitStat, boot_loop_reason, loop_count.lo,loop_count.hi}
#define LINSTAT_APP     0x81 // The application is running; status message ends with {..., lin_operation_mode, 0,0,0}
#define LINSTAT_BOOTUPD 0x82 // The boot updater is running; status message ends with {..., 0,0,0,0}


#define COMMAND_HELLO                   0x03 /* {CMD, CCNT, 0, 0,0,0,0,0} -> {CMD, CCNT, flag, cv0,cv1,cv2,cv3,cv5} */
#define COMMAND_CERTIFY                 0x04 /* {CMD, CCNT, 0, rv0,rv1,rv2,rv3,rv4} -> {CMD, CCNT, flag, rv0,rv1,rv2,rv3,rv5} */

#define COMMAND_GET_BOOT_CHECKSUM       0x05 /* {CMD, CCNT, 0,0,0,0,0,0} -> {CMD,CCNT,flag, CsumLo,Csum,Csum,CsumHi, 0} */
#define COMMAND_GET_APP_CHECKSUM        0x06 /* {CMD, CCNT, 0,0,0,0,0,0} -> {CMD,CCNT,flag, CsumLo,Csum,Csum,CsumHi, 0} */

#define COMMAND_SETUP_ILLEGAL_MESSAGE   0x07 /* {CMD, CCNT, 0, id, cFlag, delay, 0,0} -> {CMD, CCNT, flag, ... } */
// Bits in cFlag:
#define LIN_MSG_DISTURB_CSUM   1
#define LIN_MSG_DISTURB_PARITY 2 // Used only in master mode

#define COMMAND_SETUP_LIN               0x08 /* {CMD, CCNT, 0, lFlags, bittimeLow,bittimeHigh, 0,0} -> {CMD, CCNT, lFlags, bittimeLow,bittimeHigh, 0,0} */
// Bits in lFlags
#define LIN_ENHANCED_CHECKSUM  1 // 0 - off (default), 1 - on
#define LIN_VARIABLE_DLC       2 // 0 - off, 1 - on (default)
// If the bittime is non-zero, it is applied at once if in master mode.

#define COMMAND_START_APPLICATION       0x10 /* {CMD, CCNT, 0,0,0,0,0,0} -> - */
#define COMMAND_START_BOOTLOADER        0x11 /* {CMD, CCNT, 0,0,0,0,0,0} -> - */
// #define COMMAND_LIN_MASTER      0x01
// #define COMMAND_LIN_SLAVE       0x01
// #define COMMAND_LIN_PASSTHRU    0x02

#define COMMAND_FLASH_DL_START          0x40 /* {CMD, CCNT, 0, PASSWD0,PASSWD1,PASSWD2,PASSWD3, 0} --> {CMD, CCNT, flag, PASSWD0,PASSWD1,PASSWD2,PASSWD3, 0}*/
#define COMMAND_FLASH_DL_END            0x41 /* {CMD, CCNT, 0,0,0,0,0,0} --> {CMD, CCNT, flag, 0,0,0,0,0} */
//#define COMMAND_FLASH_DL_PROG           0x42 /* CMD, CCNT, ADDRH,ADDRL, DATAH,DATAL, 0,0 */
#define COMMAND_FLASH_DL_READ           0x43 /* {CMD, CCNT, 0, ADDRL,ADDRM,ADDRH, 0,0}  -->  {CMD, CCNT, flag, d0,d1,d2,d4, 0} */

/* Programming of an 18f458 must be done 8 bytes at a time, and erase 64 bytes
* at a time. Start by specifying an address (the three LSB of which must be
* zero), then a sequence of one or more data block pairs. The response to the
* last message in a pair contains the address the data was written to. Each
* time a new 64-byte block is accessed, it is erased, so programming must be
* done in sequence.
*/

#define COMMAND_FLASH_DL_ADDRESS        0x44 /* {CMD, CCNT, 0, ADDRL,ADDRM,ADDRH, 0,0}  -->  {CMD, CCNT, flag, ADDRL,ADDRM,ADDRH, 0,0} */
#define COMMAND_FLASH_DL_WRITE0         0x45 /* {CMD, CCNT, 0, d0,d1,d2,d3, 0}  -->  {CMD, CCNT, flag, 0,0,0, 0,0} */
#define COMMAND_FLASH_DL_WRITE1         0x46 /* {CMD, CCNT, 0, d4,d5,d6,d7, 0}  -->  {CMD, CCNT, flag, ADDRL,ADDRM,ADDRH, 0,0} (address written to) */

// We support the old format for a while
#define COMMAND_EEPROM_READ_OLD         0x50 /* {CMD, CCNT, ADDR, 0,0,0,0,0} --> {CMD, CCNT, flag, d, 0,0,0,0} */
#define COMMAND_EEPROM_WRITE_OLD        0x51 /* {CMD, CCNT, ADDR, DATA, PASSWD0,PASSWD1, 0,0} --> {CMD, CCNT, flag, 0,0,0,0,0}*/

#define COMMAND_EEPROM_READ             0x52 /* {CMD, CCNT, 0, ADDR, 0,0,0,0} --> {CMD, CCNT, flag, ADDR, d, 0,0,0} */
#define COMMAND_EEPROM_WRITE            0x53 /* {CMD, CCNT, 0, ADDR, DATA, PASSWD0,PASSWD1, 0} --> {CMD, CCNT, flag, ADDR, DATA, PASSWD0,PASSWD1, 0}*/

#define FLASH_PROG_PASSWD0              0x14
#define FLASH_PROG_PASSWD1              0x11
#define FLASH_PROG_PASSWD2              0x19
#define FLASH_PROG_PASSWD3              0x67

#define EEPROM_PROG_PASSWD0             0x01
#define EEPROM_PROG_PASSWD1             0x03


/* Reasons for entering bootloader loop */
#define BOOTLOOP_REASON_LIN_RX_LOW      0x01
#define BOOTLOOP_REASON_INVALID_CODE    0x02
#define BOOTLOOP_REASON_INVALID_EEPROM  0x03
#define BOOTLOOP_REASON_NO_AUTOSTART    0x04
#define BOOTLOOP_REASON_ON_COMMAND      0x05

/* EEPROM locations */
#define EEPROM_BOOT_PARAM_SIZE            16
#define EEPROM_LOC_TAG1                 0x00
#define EEPROM_LOC_TAG2                 0x01
#define EEPROM_LOC_TAG3                 0x02
#define EEPROM_LOC_VER_MINOR            0x03
#define EEPROM_LOC_VER_MAJOR            0x04
#define EEPROM_LOC_BOOT_PARAM_VALID     0x05
#define EEPROM_LOC_AUTOSTART            0x06

/* EEPROM defines for LIN firmware */
#define EEPROM_LOC_APPL_PARAM_VALID     0x0f // Placed here so we don't need to change the addresses below
#define EEPROM_LOC_OPERATION_MODE       0x10
#define EEPROM_LOC_BITTIME_LOW          0x11
#define EEPROM_LOC_BITTIME_HIGH         0x12

/* Operation modes */
#define EEPROM_OP_MODE_NONE             0x00
#define EEPROM_OP_MODE_LIN_MASTER       0x01
#define EEPROM_OP_MODE_LIN_SLAVE        0x02
//#define EEPROM_OP_MODE_LIN_PASS_THRU    0x03

/* Bitrates */
//#define EEPROM_BITRATE_AUTO             0x00
//#define EEPROM_BITRATE_12_5             0x01
//#define EEPROM_BITRATE_25               0x02
//#define EEPROM_BITRATE_50               0x03
//#define EEPROM_BITRATE_75               0x04
//#define EEPROM_BITRATE_150              0x05
//#define EEPROM_BITRATE_300              0x06
//#define EEPROM_BITRATE_600              0x07
//#define EEPROM_BITRATE_1200             0x08
//#define EEPROM_BITRATE_2400             0x09
//#define EEPROM_BITRATE_4800             0x0a
//#define EEPROM_BITRATE_9600             0x0b
//#define EEPROM_BITRATE_19200            0x0c
//#define EEPROM_BITRATE_38400            0x0d
//
//#define EEPROM_BITRATE_10000            0x10
//#define EEPROM_BITRATE_20000            0x11
//
//#define EEPROM_BITRATE_BITTIME          0xff


/* Using
#define JUMP_TO_BOOTLOADER      #asm \
                                 goto ORIGIN_BOOTLOADER_WARM \
                                #endasm \
doesn't compile. */
#define JUMP_TO_BOOTLOADER      PCLATH = (ORIGIN_BOOTLOADER_WARM >> 8); \
                                PCL = (ORIGIN_BOOTLOADER_WARM & 0xff);

#define JUMP_TO_FIRMWARE        PCLATH = (ORIGIN_FIRMWARE_MAIN >> 8); \
                                PCL = (ORIGIN_FIRMWARE_MAIN & 0xff);

/*
#define JUMP_TO_RESET           PCLATH = 0; \
                                PCL = 0;
*/

// It is not possible to use goto 0 in inline assembler. So we just insert the
// machine code.
#define JUMP_TO_RESET \
#asm        \
  dw 0xef00 \
  dw 0xf000 \
#endasm \

   

#ifdef LIN_FIRMWARE
unsigned char   bootloader_mode @ 0x20;
bit             bootloader_mode_stay_in_bootloop  @ bootloader_mode.0;
#endif


/****************************************************************************/

#endif // __LINBOOT_H
