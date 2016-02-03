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

#ifndef FILO_CMDS_H_
#define FILO_CMDS_H_


#include "debug.h"
#ifdef FILO_PRIVATE
#   include "filo_private.h"
#endif

#   include <linux/types.h>

#define CMD_RX_STD_MESSAGE                12
#define CMD_TX_STD_MESSAGE                13
#define CMD_RX_EXT_MESSAGE                14
#define CMD_TX_EXT_MESSAGE                15
#define CMD_SET_BUSPARAMS_REQ             16
#define CMD_GET_BUSPARAMS_REQ             17
#define CMD_GET_BUSPARAMS_RESP            18
#define CMD_GET_CHIP_STATE_REQ            19
#define CMD_CHIP_STATE_EVENT              20
#define CMD_SET_DRIVERMODE_REQ            21
#define CMD_GET_DRIVERMODE_REQ            22
#define CMD_GET_DRIVERMODE_RESP           23
#define CMD_RESET_CHIP_REQ                24
#define CMD_RESET_CARD_REQ                25
#define CMD_START_CHIP_REQ                26
#define CMD_START_CHIP_RESP               27
#define CMD_STOP_CHIP_REQ                 28
#define CMD_STOP_CHIP_RESP                29
#define CMD_READ_CLOCK_REQ                30
#define CMD_READ_CLOCK_RESP               31
#define CMD_GET_CARD_INFO_2               32
// 33 may be used
#define CMD_GET_CARD_INFO_REQ             34
#define CMD_GET_CARD_INFO_RESP            35
#define CMD_GET_INTERFACE_INFO_REQ        36
#define CMD_GET_INTERFACE_INFO_RESP       37
#define CMD_GET_SOFTWARE_INFO_REQ         38
#define CMD_GET_SOFTWARE_INFO_RESP        39
#define CMD_GET_BUSLOAD_REQ               40
#define CMD_GET_BUSLOAD_RESP              41
#define CMD_RESET_STATISTICS              42
#define CMD_CHECK_LICENSE_REQ             43
#define CMD_CHECK_LICENSE_RESP            44
#define CMD_ERROR_EVENT                   45
// 46, 47 reserved
#define CMD_FLUSH_QUEUE                   48
#define CMD_RESET_ERROR_COUNTER           49
#define CMD_TX_ACKNOWLEDGE                50
#define CMD_CAN_ERROR_EVENT               51
#define CMD_MEMO_WRITE_SECTOR_REQ                    52
#define CMD_MEMO_WRITE_SECTOR_RESP                   53
#define CMD_MEMO_ERASE_SECTOR_REQ                    54
#define CMD_MEMO_WRITE_CONFIG_REQ                    55
#define CMD_MEMO_WRITE_CONFIG_RESP                   56
#define CMD_MEMO_READ_CONFIG_REQ                     57
#define CMD_MEMO_READ_CONFIG_RESP                    58           
#define CMD_MEMO_ERASE_SECTOR_RESP                   59
#define CMD_TX_REQUEST                               60
#define CMD_SET_HEARTBEAT_RATE_REQ                   61
#define CMD_HEARTBEAT_RESP                           62
#define CMD_SET_AUTO_TX_BUFFER                       63
#define CMD_MEMO_GET_FILESYSTEM_INFO_STRUCT_REQ      64
#define CMD_MEMO_GET_FILESYSTEM_INFO_STRUCT_RESP     65
#define CMD_MEMO_GET_DISK_INFO_STRUCT_REQ            66
#define CMD_MEMO_GET_DISK_INFO_STRUCT_RESP           67
#define CMD_MEMO_GET_DISK_HW_INFO_STRUCT_REQ         68
#define CMD_MEMO_GET_DISK_HW_INFO_STRUCT_RESP        69
#define CMD_MEMO_FORMAT_DISK_REQ                     70
#define CMD_MEMO_FORMAT_DISK_RESP                    71
#define CMD_AUTO_TX_BUFFER_REQ                       72
#define CMD_AUTO_TX_BUFFER_RESP                      73
#define CMD_SET_TRANSCEIVER_MODE_REQ                 74
#define CMD_TREF_SOFNR                               75
#define CMD_SOFTSYNC_ONOFF                           76
#define CMD_USB_THROTTLE                             77
#define CMD_SOUND                                    78

// 79 may be used

#define CMD_SELF_TEST_REQ                            80
#define CMD_SELF_TEST_RESP                           81
#define CMD_MEMO_GET_RTC_REQ                         82
#define CMD_MEMO_GET_RTC_RESP                        83
#define CMD_MEMO_SET_RTC_REQ                         84
#define CMD_MEMO_SET_RTC_RESP                        85

#define CMD_SET_IO_PORTS_REQ                         86
#define CMD_GET_IO_PORTS_REQ                         87
#define CMD_GET_IO_PORTS_RESP                        88

#define CMD_MEMO_READ_SECTOR_REQ                     89
#define CMD_MEMO_READ_SECTOR_RESP                    90

#define CMD_GET_TRANSCEIVER_INFO_REQ                 97
#define CMD_GET_TRANSCEIVER_INFO_RESP                98

#define CMD_MEMO_CONFIG_MODE_REQ                     99
#define CMD_MEMO_CONFIG_MODE_RESP                   100

#define CMD_LED_ACTION_REQ                          101
#define CMD_LED_ACTION_RESP                         102

#define CMD_INTERNAL_DUMMY                          103
#define CMD_MEMO_REINIT_DISK_REQ                    104
#define CMD_MEMO_CPLD_PRG                           105
#define CMD_LOG_MESSAGE                             106
#define CMD_LOG_TRIG                                107
#define CMD_LOG_RTC_TIME                            108


#define SOUND_SUBCOMMAND_INIT         0
#define SOUND_SUBCOMMAND_BEEP         1                     
#define SOUND_SUBCOMMAND_NOTE         2              
#define SOUND_SUBCOMMAND_DISABLE      3              

#define LED_SUBCOMMAND_ALL_LEDS_ON    0
#define LED_SUBCOMMAND_ALL_LEDS_OFF   1  
#define LED_SUBCOMMAND_LED_0_ON       2
#define LED_SUBCOMMAND_LED_0_OFF      3
#define LED_SUBCOMMAND_LED_1_ON       4
#define LED_SUBCOMMAND_LED_1_OFF      5
#define LED_SUBCOMMAND_LED_2_ON       6
#define LED_SUBCOMMAND_LED_2_OFF      7
#define LED_SUBCOMMAND_LED_3_ON       8
#define LED_SUBCOMMAND_LED_3_OFF      9

//////////////////////////////////////////////////////////
// subcommands & other Memorator stuff
#define MEMO_SUBCOMMAND_START_CONFIG                  6
#define MEMO_SUBCOMMAND_BULK_CONFIG                   7
#define MEMO_SUBCOMMAND_STOP_CONFIG                   8

#define MEMO_SUBCOMMAND_GET_FILESYSTEM_INFO_STRUCT    9
#define MEMO_SUBCOMMAND_GET_DISK_INFO_STRUCT          10
#define MEMO_SUBCOMMAND_FORMAT_DISK                   12
#define MEMO_SUBCOMMAND_GET_RTC                       13
#define MEMO_SUBCOMMAND_SET_RTC                       14
#define MEMO_SUBCOMMAND_REINIT_DISK                   15

#define MEMO_SUBCOMMAND_BULK_SECTOR                   16
#define MEMO_SUBCOMMAND_STOP_SECTOR                   17

#define MEMO_SUBCOMMAND_START_PHYSICAL_SECTOR         19
#define MEMO_SUBCOMMAND_STREAM_SECTOR                 20
#define MEMO_SUBCOMMAND_START_LOGICAL_SECTOR          21


#define CONFIG_DATA_CHUNK                             24

#define MEMO_CONFIG_SUBCMD_SET_INTERVAL                0
#define MEMO_CONFIG_SUBCMD_GET_INTERVAL                1 
#define MEMO_CONFIG_SUBCMD_UPDATE_TIMEOUT              2 

///////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////
//FLAGS
/////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////
// CAN message flags
#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_OVERRUN             0x02        // Msgs following this has been lost            
#define MSGFLAG_NERR                0x04        // NERR active during this msg
#define MSGFLAG_WAKEUP              0x08        // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#define MSGFLAG_RESERVED_1          0x20        // Reserved for future usage
#define MSGFLAG_TX                  0x40        // TX acknowledge
#define MSGFLAG_TXRQ                0x80        // TX request


// This one is added to the identifier to mean an extended CAN identifier
// #define DRIVER_EXT_FLAG             0x80000000


/////////////////////////
// Chip status flags
#define BUSSTAT_BUSOFF              0x01
#define BUSSTAT_ERROR_PASSIVE       0x02
#define BUSSTAT_ERROR_WARNING       0x04
#define BUSSTAT_ERROR_ACTIVE        0x08
#define BUSSTAT_BUSOFF_RECOVERY     0x10
#define BUSSTAT_IGNORING_ERRORS     0x20


////////////////////////
// Driver modes
#define DRIVERMODE_NORMAL           0x01
#define DRIVERMODE_SILENT           0x02
#define DRIVERMODE_SELFRECEPTION    0x03
#define DRIVERMODE_OFF              0x04

////////////////////////
// Transceiver (logical) types. Must be the same as TRANSCEIVER_xxx in e.g. canlib.h.
#define FILO_TRANSCEIVER_TYPE_UNKNOWN         0
#define FILO_TRANSCEIVER_TYPE_251             1
#define FILO_TRANSCEIVER_TYPE_252             2 // 252/1053/1054 w/o opto
#define FILO_TRANSCEIVER_TYPE_SWC             6 // J2411 type
#define FILO_TRANSCEIVER_TYPE_1054_OPTO      11 // 1054 with optical isolation
#define FILO_TRANSCEIVER_TYPE_SWC_OPTO       12 // J2411 type with optical isolation
#define FILO_TRANSCEIVER_TYPE_1050           14 // TJA1050
#define FILO_TRANSCEIVER_TYPE_1050_OPTO      15 // TJA1050 with optical isolation
#define FILO_TRANSCEIVER_TYPE_LIN            19 // LIN

// Transceiver line modes. Must be the same as in canlib.h
#define FILO_TRANSCEIVER_LINEMODE_NA          0  // Not Affected/Not available.
#define FILO_TRANSCEIVER_LINEMODE_SWC_SLEEP   4  // SWC Sleep Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_NORMAL  5  // SWC Normal Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_FAST    6  // SWC High-Speed Mode.
#define FILO_TRANSCEIVER_LINEMODE_SWC_WAKEUP  7  // SWC Wakeup Mode.
#define FILO_TRANSCEIVER_LINEMODE_SLEEP       8  // Sleep mode for those supporting it.
#define FILO_TRANSCEIVER_LINEMODE_NORMAL      9  // Normal mode (the inverse of sleep mode) for those supporting it.
#define FILO_TRANSCEIVER_LINEMODE_STDBY      10  // Standby for those who support it
// Transceiver resnet modes. Not supported.
#define FILO_TRANSCEIVER_RESNET_NA            0


////////////////////////////
// Error codes.
// Used in CMD_ERROR_EVENT.
#define FIRMWARE_ERR_OK               0     // No error.
#define FIRMWARE_ERR_CAN              1     // CAN error, addInfo1 contains error code.
#define FIRMWARE_ERR_NVRAM_ERROR      2     // Flash error
#define FIRMWARE_ERR_NOPRIV           3     // No privilege for attempted operation
#define FIRMWARE_ERR_ILLEGAL_ADDRESS  4     // Illegal RAM/ROM address specified
#define FIRMWARE_ERR_UNKNOWN_CMD      5     // Unknown command or subcommand
#define FIRMWARE_ERR_FATAL            6     // A severe error. addInfo1 contains error code.
#define FIRMWARE_ERR_CHECKSUM_ERROR   7     // Downloaded code checksum mismatch
#define FIRMWARE_ERR_QUEUE_LEVEL      8     // Tx queue levels (probably driver error)
#define FIRMWARE_ERR_PARAMETER        9     // Parameter error, addInfo1 contains offending command

// Maximum length of a command. A change here will make the firmware incompatible
// with the driver, unless both are recompiled and replaced.
#define MAX_CMD_LEN                   32


// For CMD_READ_CLOCK_REQ
#define READ_CLOCK_NOW                0x01  // Provide a fast, unsynchronized response.

// For CMD_GET_SOFTWARE_OPTIONS (max 32 flags here)
#define SWOPTION_CONFIG_MODE          0x01L // Memorator in config mode.
#define SWOPTION_AUTO_TX_BUFFER       0x02L // Firmware has auto tx buffers
#define SWOPTION_BETA                 0x04L // Firmware is a beta release
#define SWOPTION_RC                   0x08L // Firmware is a release candidate
#define SWOPTION_BAD_MOOD             0x10L // Firmware detected config error or the like
#define SWOPTION_CPU_FQ_MASK          0x60L
#define SWOPTION_16_MHZ_CLK           0x00L // hires timers run at 16 MHZ
#define SWOPTION_32_MHZ_CLK           0x20L // hires timers run at 32 MHZ
#define SWOPTION_24_MHZ_CLK           0x40L // hires timers run at 24 MHZ
#define SWOPTION_XX_MHZ_CLK           0x60L // hires timers run at xx MHZ
#define SWOPTION_TIMEOFFSET_VALID     0x80L // the timeOffset field in txAcks and logMessages is vaild


// CMD_SET_AUTO_TX_REQ and _RESP enum values
#define AUTOTXBUFFER_CMD_GET_INFO     1     // Get implementation information
#define AUTOTXBUFFER_CMD_CLEAR_ALL    2     // Clear all buffers on a channel
#define AUTOTXBUFFER_CMD_ACTIVATE     3     // Activate a specific buffer
#define AUTOTXBUFFER_CMD_DEACTIVATE   4     // Dectivate a specific buffer
#define AUTOTXBUFFER_CMD_SET_INTERVAL 5     // Set tx buffer transmission interval
#define AUTOTXBUFFER_CMD_GENERATE_BURST 6   // Generate a burst of messages
#define AUTOTXBUFFER_CMD_SET_MSG_COUNT 7    // Set tx buffer message count

// CMD_SET_AUTO_TX_RESP bit values for automatic tx buffer capabilities
#define AUTOTXBUFFER_CAP_TIMED_TX         0x01    // Periodic transmission
#define AUTOTXBUFFER_CAP_AUTO_RESP_DATA   0x02    // Auto response to data frames
#define AUTOTXBUFFER_CAP_AUTO_RESP_RTR    0x04    // Auto response to RTR

// Use these message flags with cmdSetAutoTxBuffer.flags
#define AUTOTXBUFFER_MSG_REMOTE_FRAME     0x10    // Msg is a remote frame
#define AUTOTXBUFFER_MSG_EXT              0x80    // Extended identifier

// For CMD_SOFTSYNC_ONOFF
#define SOFTSYNC_OFF          0
#define SOFTSYNC_ON           1
#define SOFTSYNC_NOT_STARTED  2


#define LOG_FILE_MAJOR_VERSION 3     // Must fit in one byte
#define LOG_FILE_MINOR_VERSION 0     // Must fit in one byte
#define LOG_FILE_VERSION32 (((uint32_t)LOG_FILE_MAJOR_VERSION << 24)+((uint32_t)LOG_FILE_MINOR_VERSION << 16)) // Room for e.g. a build number

/*
 * Trigger types (used in TriggerEvent.type). This is a bit mask.
 */
#define LOG_TRIG_LOG_ALL             1
#define LOG_TRIG_ON_EXTERNAL         2
#define LOG_TRIG_ON_MESSAGE          4
#define LOG_TRIG_LOG_FIFO            0x80

#if defined(TRIG_LOG_ALL)
// LOG_TRIG_xxx symbols must be congruent with TRIG_xxx in the parameter
// file, so make a sanity check here.
CompilerAssert(TRIG_LOG_ALL == LOG_TRIG_LOG_ALL);
CompilerAssert(TRIG_ON_EXTERNAL == LOG_TRIG_ON_EXTERNAL);
CompilerAssert(TRIG_ON_MESSAGE == LOG_TRIG_ON_MESSAGE);
CompilerAssert(TRIG_LOG_FIFO == LOG_TRIG_LOG_FIFO);
#endif




/*
** Command Structs
*/
#if !defined(__IAR_SYSTEMS_ICC__)
#  include <pshpack1.h>
#endif


// Header for every command.
typedef struct {
  unsigned char cmdLen;  // The length of the whole packet (i.e. including this byte)
  unsigned char cmdNo;
} cmdHead;


//#define TIMESTAMP_AS_LONG(x) (*(uint32_t*) x.time)

/*
** Keep the following structs longword aligned (to allow for future
** DPRAM access over the PCI bus.)
*/

typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  channel;
    unsigned char  flags;     // MSGFLAG_*
    unsigned short time[3];
    unsigned char  dlc;
    unsigned char  timeOffset;
    uint32_t       id;        // incl. CAN_IDENT_IS_EXTENDED
    unsigned char  data[8];
} cmdLogMessage;

typedef struct
{
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  type;
    unsigned char  trigNo;      // A bit mask showing which trigger was activated
    unsigned short time[3];
    unsigned short padding;
    uint32_t       preTrigger;
    uint32_t       postTrigger;
} cmdLogTrigger;

typedef struct
{
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  verMajor;            // File version number
    unsigned char  verMinor;            // File version number
    unsigned short date;                // Real time date (YMD)
    unsigned short time;                // Real time time (HMS)
    unsigned char  canDriverType0;      // Transceiver type for channel 0.
    unsigned char  canDriverType1;      // Transceiver type for channel 1.
    unsigned char  hiresTimerFqMHz;     // High-resolution timer frequency, in MHz
    unsigned char  hardwareType;        // Our HWTYPE_xxx.
    unsigned short bitrate0L;           // Bit rate channel 0, low word
    unsigned char  bitrate0H;           // Bit rate channel 0, high byte
    unsigned char  padding1;
    unsigned short bitrate1L;           // Bit rate channel 1, low word
    unsigned char  bitrate1H;           // Bit rate channel 1, high byte
    unsigned char  padding2;
} cmdLogRtcTime;

typedef struct { 
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  channel;
    unsigned char  flags;
    unsigned short time[3];
    unsigned char  rawMessage[14];
} cmdRxCanMessage;

typedef struct { 
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  channel;
    unsigned char  transId;
    unsigned char  rawMessage[14];
    unsigned char  _padding0;
    unsigned char  flags;
} cmdTxCanMessage;

typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  channel;
    unsigned char  transId;
    unsigned short time[3];
    unsigned char  flags;     // Flags detected during tx - currently only NERR
    unsigned char  timeOffset;
} cmdTxAck;

typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  channel;
    unsigned char  transId;
    unsigned short time[3];
    unsigned short padding;
} cmdTxRequest;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    uint32_t      bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
} cmdSetBusparamsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetBusparamsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    uint32_t      bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
} cmdGetBusparamsResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetChipStateReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned short time[3];
    unsigned char txErrorCounter; 
    unsigned char rxErrorCounter; 
    unsigned char busStatus;
    unsigned char padding;
    unsigned short padding2;
} cmdChipStateEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char driverMode;
    unsigned char padding;
    unsigned short  padding2;
} cmdSetDrivermodeReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetDrivermodeReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char driverMode;
    unsigned char padding;
    unsigned short  padding2;
} cmdGetDrivermodeResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdResetChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdResetCardReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStartChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStartChipResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStopChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStopChipResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char flags;
} cmdReadClockReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
    unsigned short time[3];
    unsigned short padding2;
} cmdReadClockResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdSelfTestReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
    uint32_t      results;
} cmdSelfTestResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
      signed char dataLevel;
} cmdGetCardInfo2Req;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char reserved0;      // Unused right now
    unsigned char pcb_id[24];
    uint32_t      oem_unlock_code;
} cmdGetCardInfo2Resp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
      signed char dataLevel;
} cmdGetCardInfoReq;

typedef struct {
    unsigned char cmdLen;       
    unsigned char cmdNo;        
    unsigned char transId;      
    unsigned char channelCount; 
    uint32_t      serialNumber; 
    uint32_t      padding1;       // Unused right now
    uint32_t      clockResolution;
    uint32_t      mfgDate;
    unsigned char EAN[8];         // LSB..MSB, then the check digit.
    unsigned char hwRevision;
    unsigned char usbHsMode;
    unsigned char hwType;         // HWTYPE_xxx (only f/w 1.2 and after)
    unsigned char canTimeStampRef;
} cmdGetCardInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetInterfaceInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    uint32_t      channelCapabilities;
    unsigned char canChipType;
    unsigned char canChipSubType;
    unsigned short  padding;
} cmdGetInterfaceInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdGetSoftwareInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding0;
    uint32_t      swOptions;
    uint32_t      firmwareVersion;
    unsigned short maxOutstandingTx;
    unsigned short padding1;      // Currently unused
    uint32_t      padding[4];     // Currently unused
} cmdGetSoftwareInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetBusLoadReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned short time[3];         // "Absolute" timestamp
    unsigned short sample_interval; // Sampling interval in microseconds
    unsigned short active_samples;  // Number of samples where tx or rx was active
    unsigned short delta_t;         // Milliseconds since last response
} cmdGetBusLoadResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdResetStatisticsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdCheckLicenseReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
    uint32_t      licenseMask;
    uint32_t      kvaserLicenseMask;
} cmdCheckLicenseResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char errorCode;
    unsigned short time[3];
    unsigned short padding;
    unsigned short  addInfo1;
    unsigned short  addInfo2;
} cmdErrorEvent;


typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  transId;
    unsigned char  channel;
    unsigned char  flags;
    unsigned char  padding;
    unsigned short padding2;
} cmdFlushQueue;


typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdNoCommand;


typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdResetErrorCounter;

typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  transId;
    unsigned char  flags;         // Currently only NERR (.._ERROR_FRAME is implicit)
    unsigned short time[3];
    unsigned char  channel;
    unsigned char  padding;
    unsigned char  txErrorCounter;
    unsigned char  rxErrorCounter;
    unsigned char  busStatus;
    unsigned char  errorFactor;
} cmdCanErrorEvent; 

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char lineMode;
    unsigned char resistorNet;      // Not used in Filo (left-over from LAPcan)
    unsigned short padding;
} cmdSetTransceiverModeReq;

typedef struct { 
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned short time;
} cmdInternalDummy;

//##################################################################
// Memorator specific commands
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdMemoGetFilesystemInfoStructReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;

    unsigned short fat_size;                // Size of the FAT, in sectors
    unsigned short fat_type;                // 12 or 16 depending on FAT type.
    unsigned short dir_entries;             // Number of directory entries in the root dir
    unsigned short cluster_size;            // Two-logarithm of the cluster size in sectors
    uint32_t       fat1_start;              // First FAT starts in this sector
    uint32_t       first_data_sector;       // First sector available for data
    uint32_t       last_data_sector;  
} cmdMemoGetFilesystemInfoStructResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdMemoGetDiskInfoStructReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char product_revision;
    unsigned char oem_id[2];
    char          product_name[10];
    uint32_t      m_id;
    uint32_t      serial_number;
    unsigned short date_code;
    unsigned short padding;
} cmdMemoGetDiskInfoStructResp;


typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdMemoGetDiskHWInfoStructReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
    
    unsigned char disk_type;
    unsigned char version;
    unsigned char read_time;
    unsigned char wr_factor;
    
    unsigned char file_format;
    unsigned char erase_value;
    unsigned short read_blk_size;

    unsigned short wr_blk_size;
    unsigned short trans_speed;

    uint32_t      data_size;
} cmdMemoGetDiskHWInfoStructResp;


typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    unsigned char data[CONFIG_DATA_CHUNK];
} cmdMemoWriteConfigReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char status;
} cmdMemoCpldPrgResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    unsigned char data[CONFIG_DATA_CHUNK];
} cmdMemoCpldPrgReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char status;
} cmdMemoWriteConfigResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    uint32_t      dataChunkReqNo;
} cmdMemoReadConfigReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    unsigned char status;
    unsigned char padding[3];
    unsigned char data[CONFIG_DATA_CHUNK];
} cmdMemoReadConfigResp;

// Format disk
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdMemoFormatDiskReq;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char padding;
  unsigned short formatStatus;
  unsigned short padding2;
} cmdMemoFormatDiskResp;

// Read RTC
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char padding;
} cmdMemoGetRTCReq;

// Read RTC - the response
typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   second;
    unsigned char   minute;
    unsigned char   hour;
    unsigned char   day;
    unsigned char   month;
    unsigned char   year;
    unsigned char   padding;
    unsigned short  padding2;
} cmdMemoGetRTCResp;

typedef struct {
    unsigned char  cmdLen;
    unsigned char  cmdNo;
    unsigned char  transId;
    unsigned char  second;
    unsigned char  minute;
    unsigned char  hour;
    unsigned char  day;
    unsigned char  month;
    unsigned char  year;
    unsigned char  padding;
    unsigned short padding2;
} cmdMemoSetRTCReq;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   padding;
} cmdMemoSetRTCResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    signed short  timeout;
    signed short  padding;
} cmdLedActionReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
} cmdLedActionResp;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   subCmd;
    signed short    reqInterval;  // The interval (milliseconds) that Memorator should wait for the next config msg.
    unsigned char   padding[2];
} cmdMemoConfigModeReq;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   subCmd;
    signed short    interval;  // The actual interval (milliseconds) that Memorator should wait for the next config msg.
    unsigned char   diskStat;     // true if the disk is there.
    unsigned char   configMode;   // true if we're in config mode.
} cmdMemoConfigModeResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char subCmd;
    unsigned short freq;
    unsigned short duration;
    uint32_t      padding[2];    
}  cmdSound;
    

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   portNo;             // Hardware-specific port #
    uint32_t        portVal;            // Hardware-specific port value
} cmdSetIoPortsReq;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   portNo;             // Hardware-specific port #
} cmdGetIoPortsReq;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   transId;
    unsigned char   portNo;             // Hardware-specific port #
    uint32_t        portVal;            // Hardware-specific port value
    uint32_t        padding;
    unsigned short  status;
    unsigned short  padding2;
} cmdGetIoPortsResp;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  uint32_t      sectorNo;
  uint32_t      dataChunkReqNo;
} cmdMemoReadSectorReq;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  unsigned char status;
  unsigned char padding[3];
  unsigned char data[CONFIG_DATA_CHUNK];
} cmdMemoReadSectorResp;


typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  uint32_t      sectorNo;
  unsigned char data[CONFIG_DATA_CHUNK];
} cmdMemoWriteSectorReq;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  unsigned char status;
  unsigned char padding[3];
} cmdMemoWriteSectorResp;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  uint32_t      sectorNo;
  uint32_t      count;
} cmdMemoEraseSectorReq;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char subCmd;
  unsigned char status;
  unsigned char padding[3];
} cmdMemoEraseSectorResp;


typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char channel;
} cmdGetTransceiverInfoReq;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char channel;
  uint32_t      transceiverCapabilities;
  unsigned char transceiverStatus;
  unsigned char transceiverType;
  unsigned char padding[2];
} cmdGetTransceiverInfoResp;


typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char transId;
  unsigned char pad1;
  unsigned short rate;
  unsigned short pad2;
} cmdSetHeartbeatRateReq;

typedef struct {             // Could be 8 bytes but is 12 because the low bits
  unsigned char  cmdLen;     // of the time should be long aligned and found on
  unsigned char  cmdNo;      // the same place in all timestamped commands.
  unsigned short padding;
  unsigned short time[3];
  unsigned short padding2;
} cmdHeartbeatResp;

typedef struct { 
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned char  channel;
  unsigned char  bufNo;
  unsigned char  rawMessage[14];
  unsigned char  flags;
  unsigned char _padding0;
} cmdSetAutoTxBuffer;

typedef struct {
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned char  requestType;
  unsigned char  channel;       // For certain requests only
  uint32_t       interval;      // D:o
  unsigned char  bufNo;         // D:o
  unsigned char  padding[3];
} cmdAutoTxBufferReq;

typedef struct {
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned char  responseType;
  unsigned char  bufferCount;
  uint32_t       timerResolution;
  unsigned short capabilities;
  unsigned short padding0;
} cmdAutoTxBufferResp;

typedef struct {
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned short sofNr;
  unsigned short time[3];
  unsigned short padding;
} cmdTrefSofSeq;

typedef struct {
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned short onOff;
} cmdSoftSyncOnOff;

typedef struct {
  unsigned char  cmdLen;
  unsigned char  cmdNo;
  unsigned short throttle;
} cmdUsbThrottle;

typedef union {
  cmdHead                      head;

  cmdLogMessage                logMessage;
  cmdLogTrigger                logTrigger;
  cmdLogRtcTime                logRtcTime;
  cmdRxCanMessage              rxCanMessage;
  cmdTxCanMessage              txCanMessage;
  cmdTxAck                     txAck;
  cmdTxRequest                 txRequest;
  cmdSetBusparamsReq           setBusparamsReq;
  cmdGetBusparamsReq           getBusparamsReq;
  cmdGetBusparamsResp          getBusparamsResp;
  cmdGetChipStateReq           getChipStateReq;
  cmdChipStateEvent            chipStateEvent;
  cmdSetDrivermodeReq          setDrivermodeReq;
  cmdGetDrivermodeReq          getDrivermodeReq;
  cmdGetDrivermodeResp         getDrivermodeResp;
  cmdResetChipReq              resetChipReq;
  cmdResetCardReq              resetCardReq;
  cmdStartChipReq              startChipReq;
  cmdStartChipResp             startChipResp;
  cmdStopChipReq               stopChipReq;
  cmdStopChipResp              stopChipResp;
  cmdGetCardInfo2Req           getCardInfo2Req;
  cmdGetCardInfo2Resp          getCardInfo2Resp;
  cmdGetCardInfoReq            getCardInfoReq;
  cmdGetCardInfoResp           getCardInfoResp;
  cmdGetInterfaceInfoReq       getInterfaceInfoReq;
  cmdGetInterfaceInfoResp      getInterfaceInfoResp;
  cmdGetSoftwareInfoReq        getSoftwareInfoReq;
  cmdGetSoftwareInfoResp       getSoftwareInfoResp;
  cmdGetBusLoadReq             getBusLoadReq;
  cmdGetBusLoadResp            getBusLoadResp;
  cmdResetStatisticsReq        resetStatisticsReq;
  cmdErrorEvent                errorEvent;
  cmdFlushQueue                flushQueue;
  cmdNoCommand                 noCommand;
  cmdResetErrorCounter         resetErrorCounter;
  cmdCanErrorEvent             canErrorEvent;
  cmdCheckLicenseReq           checkLicenseReq;
  cmdCheckLicenseResp          checkLicenseResp;
  cmdReadClockReq              readClockReq;
  cmdReadClockResp             readClockResp;
  cmdSelfTestReq               selfTestReq;
  cmdSelfTestResp              selfTestResp;
  cmdSetTransceiverModeReq     setTransceiverModeReq;
  cmdInternalDummy             internalDummy;
  
  cmdMemoGetFilesystemInfoStructReq   memoGetFilesystemInfoStructReq;
  cmdMemoGetFilesystemInfoStructResp  memoGetFilesystemInfoStructResp;
  cmdMemoGetDiskInfoStructReq         memoGetDiskInfoStructReq;
  cmdMemoGetDiskInfoStructResp        memoGetDiskInfoStructResp;
  cmdMemoGetDiskHWInfoStructReq       memoGetDiskHWInfoStructReq;
  cmdMemoGetDiskHWInfoStructResp      memoGetDiskHWInfoStructResp;
  
  cmdMemoWriteConfigReq       memoWriteConfigReq;
  cmdMemoWriteConfigResp      memoWriteConfigResp;
  cmdMemoReadConfigReq        memoReadConfigReq;
  cmdMemoReadConfigResp       memoReadConfigResp;

  cmdMemoCpldPrgReq           memoCpldPrgReq;
  cmdMemoCpldPrgResp          memoCpldPrgResp;

  cmdMemoFormatDiskReq        memoFormatDiskReq;
  cmdMemoFormatDiskResp       memoFormatDiskResp;
  cmdMemoGetRTCReq            memoGetRTCReq;
  cmdMemoGetRTCResp           memoGetRTCResp;
  cmdMemoSetRTCReq            memoSetRTCReq;
  cmdMemoSetRTCResp           memoSetRTCResp;
  cmdLedActionReq             ledActionReq;
  cmdLedActionResp            ledActionResp;
  cmdMemoConfigModeReq        memoConfigModeReq;
  cmdMemoConfigModeResp       memoConfigModeResp;

  cmdSetIoPortsReq            setIoPortsReq;
  cmdGetIoPortsReq            getIoPortsReq;
  cmdGetIoPortsResp           getIoPortsResp;

  cmdMemoReadSectorReq        memoReadSectorReq;
  cmdMemoReadSectorResp       memoReadSectorResp;
  cmdMemoWriteSectorReq       memoWriteSectorReq;
  cmdMemoWriteSectorResp      memoWriteSectorResp;
  cmdMemoEraseSectorReq       memoEraseSectorReq;
  cmdMemoEraseSectorResp      memoEraseSectorResp;

  cmdGetTransceiverInfoReq    getTransceiverInfoReq;
  cmdGetTransceiverInfoResp   getTransceiverInfoResp;

  cmdSetHeartbeatRateReq      setHeartbeatRateReq;
  cmdHeartbeatResp            heartbeatResp;

  cmdSetAutoTxBuffer          setAutoTxBuffer;
  cmdAutoTxBufferReq          autoTxBufferReq;
  cmdAutoTxBufferResp         autoTxBufferResp;

  cmdTrefSofSeq               trefSofSeq;
  cmdSoftSyncOnOff            softSyncOnOff;
  cmdUsbThrottle              usbThrottle;
  cmdSound                    sound;

#ifdef FILO_PRIVATE
  cmdFiloOtherCommand       o;
#endif
} filoCmd;


// Union for all *synchronized* responses from the card.
// The receive queue uses this union, so it is to be kept small.
typedef union {
  cmdHead                     head;
  cmdLogMessage               logMessage;
  cmdLogTrigger               logTrigger;
  cmdLogRtcTime               logRtcTime;
  cmdRxCanMessage             rxCanMessage;
  cmdTxCanMessage             txCanMessage;
  cmdTxAck                    txAck;
  cmdTxRequest                txRequest;
  cmdChipStateEvent           chipStateEvent;
  cmdGetBusLoadResp           getBusLoadResp;
  cmdStartChipResp            startChipResp;
  cmdStopChipResp             stopChipResp;
  cmdCanErrorEvent            canErrorEvent;
  cmdHeartbeatResp            heartbeatResp;
  cmdReadClockResp            readClockResp;
  cmdSelfTestResp             selfTestResp;
  cmdTrefSofSeq               trefSofSeq;
} filoResponse;          

#if !defined(__IAR_SYSTEMS_ICC__)
#  include <poppack.h>
#endif

#if defined(CompilerAssert)
CompilerAssert(sizeof(filoResponse) == 24);
// A basic sanity check of all structs:
#define CHECK_ALIGNMENT(X) CompilerAssert((sizeof(X) % 4) == 0)
CHECK_ALIGNMENT(cmdLogMessage                );
CHECK_ALIGNMENT(cmdLogTrigger                );
CHECK_ALIGNMENT(cmdLogRtcTime                );
CHECK_ALIGNMENT(cmdRxCanMessage              );
CHECK_ALIGNMENT(cmdTxCanMessage              );
CHECK_ALIGNMENT(cmdTxAck                     );
CHECK_ALIGNMENT(cmdTxRequest                 );
CHECK_ALIGNMENT(cmdSetBusparamsReq           );
CHECK_ALIGNMENT(cmdGetBusparamsReq           );
CHECK_ALIGNMENT(cmdGetBusparamsResp          );
CHECK_ALIGNMENT(cmdGetChipStateReq           );
CHECK_ALIGNMENT(cmdChipStateEvent            );
CHECK_ALIGNMENT(cmdSetDrivermodeReq          );
CHECK_ALIGNMENT(cmdGetDrivermodeReq          );
CHECK_ALIGNMENT(cmdGetDrivermodeResp         );
CHECK_ALIGNMENT(cmdResetChipReq              );
CHECK_ALIGNMENT(cmdResetCardReq              );
CHECK_ALIGNMENT(cmdStartChipReq              );
CHECK_ALIGNMENT(cmdStartChipResp             );
CHECK_ALIGNMENT(cmdStopChipReq               );
CHECK_ALIGNMENT(cmdStopChipResp              );
CHECK_ALIGNMENT(cmdGetCardInfo2Req           );
CHECK_ALIGNMENT(cmdGetCardInfo2Resp          );
CHECK_ALIGNMENT(cmdGetCardInfoReq            );
CHECK_ALIGNMENT(cmdGetCardInfoResp           );
CHECK_ALIGNMENT(cmdGetInterfaceInfoReq       );
CHECK_ALIGNMENT(cmdGetInterfaceInfoResp      );
CHECK_ALIGNMENT(cmdGetSoftwareInfoReq        );
CHECK_ALIGNMENT(cmdGetSoftwareInfoResp       );
CHECK_ALIGNMENT(cmdGetBusLoadReq             );
CHECK_ALIGNMENT(cmdGetBusLoadResp            );
CHECK_ALIGNMENT(cmdResetStatisticsReq        );
CHECK_ALIGNMENT(cmdErrorEvent                );
CHECK_ALIGNMENT(cmdFlushQueue                );
CHECK_ALIGNMENT(cmdNoCommand                 );
CHECK_ALIGNMENT(cmdResetErrorCounter         );
CHECK_ALIGNMENT(cmdCanErrorEvent             );
CHECK_ALIGNMENT(cmdCheckLicenseReq           );
CHECK_ALIGNMENT(cmdCheckLicenseResp          );
CHECK_ALIGNMENT(cmdReadClockReq              );
CHECK_ALIGNMENT(cmdReadClockResp             );
CHECK_ALIGNMENT(cmdSelfTestReq               );
CHECK_ALIGNMENT(cmdSelfTestResp              );
CHECK_ALIGNMENT(cmdSetTransceiverModeReq     );

CHECK_ALIGNMENT(cmdMemoGetFilesystemInfoStructReq  );
CHECK_ALIGNMENT(cmdMemoGetFilesystemInfoStructResp );
CHECK_ALIGNMENT(cmdMemoGetDiskInfoStructReq        );
CHECK_ALIGNMENT(cmdMemoGetDiskInfoStructResp       );
CHECK_ALIGNMENT(cmdMemoGetDiskHWInfoStructReq      );
CHECK_ALIGNMENT(cmdMemoGetDiskHWInfoStructResp     );

CHECK_ALIGNMENT(cmdMemoWriteConfigReq        );
CHECK_ALIGNMENT(cmdMemoWriteConfigResp       );
CHECK_ALIGNMENT(cmdMemoReadConfigReq         );
CHECK_ALIGNMENT(cmdMemoReadConfigResp        );

CHECK_ALIGNMENT(cmdMemoCpldPrgReq            );
CHECK_ALIGNMENT(cmdMemoCpldPrgResp           );

CHECK_ALIGNMENT(cmdMemoFormatDiskReq         );
CHECK_ALIGNMENT(cmdMemoFormatDiskResp        );
                                             
CHECK_ALIGNMENT(cmdMemoGetRTCReq             );
CHECK_ALIGNMENT(cmdMemoGetRTCResp            );
CHECK_ALIGNMENT(cmdMemoSetRTCReq             );
CHECK_ALIGNMENT(cmdMemoSetRTCResp            );
CHECK_ALIGNMENT(cmdLedActionReq              );
CHECK_ALIGNMENT(cmdLedActionResp             );

CHECK_ALIGNMENT(cmdMemoConfigModeReq         );
CHECK_ALIGNMENT(cmdMemoConfigModeResp        );
                                             
CHECK_ALIGNMENT(cmdSetIoPortsReq             );
CHECK_ALIGNMENT(cmdGetIoPortsReq             );
CHECK_ALIGNMENT(cmdGetIoPortsResp            );

CHECK_ALIGNMENT(cmdMemoReadSectorReq         );
CHECK_ALIGNMENT(cmdMemoReadSectorResp        );
CHECK_ALIGNMENT(cmdMemoWriteSectorReq        );
CHECK_ALIGNMENT(cmdMemoWriteSectorResp       );
CHECK_ALIGNMENT(cmdMemoEraseSectorReq        );
CHECK_ALIGNMENT(cmdMemoEraseSectorResp       );

CHECK_ALIGNMENT(cmdGetTransceiverInfoReq     );
CHECK_ALIGNMENT(cmdGetTransceiverInfoResp    );

CHECK_ALIGNMENT(cmdSetHeartbeatRateReq       );
CHECK_ALIGNMENT(cmdHeartbeatResp             );

CHECK_ALIGNMENT(cmdSetAutoTxBuffer           );
CHECK_ALIGNMENT(cmdAutoTxBufferReq           );
CHECK_ALIGNMENT(cmdAutoTxBufferResp          );

CHECK_ALIGNMENT(cmdTrefSofSeq                );
CHECK_ALIGNMENT(cmdSoftSyncOnOff             );
CHECK_ALIGNMENT(cmdUsbThrottle               );


CompilerAssert(sizeof(filoCmd) <= MAX_CMD_LEN);
#endif


#endif //_FILO_CMDS_H_
