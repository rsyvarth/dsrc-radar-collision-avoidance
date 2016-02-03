/*
**                Copyright 2014 by Kvaser AB, M�lndal, Sweden
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
#ifndef HYDRA_HOST_CMDS_H_
#define HYDRA_HOST_CMDS_H_

#include "debug.h"

#ifdef HYDRA_PRIVATE
#   include "hydra_host_private_cmds.h"
#endif

#   include <linux/types.h>

#define CMD_RX_STD_MESSAGE                12
#define CMD_TX_CAN_MESSAGE                33
#define CMD_RX_EXT_MESSAGE                14
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
// 33 may be used - NOT see CMD_TX_CAN_MESSAGE
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

#define CMD_MEMO_GET_DATA                 52
#define CMD_MEMO_PUT_DATA                 53
#define CMD_MEMO_PUT_DATA_START           54
#define CMD_MEMO_ASYNCOP_START            55
#define CMD_MEMO_ASYNCOP_GET_DATA         56
#define CMD_MEMO_ASYNCOP_CANCEL           57
#define CMD_MEMO_ASYNCOP_FINISHED         58
#define CMD_DISK_FULL_INFO                59
#define CMD_TX_REQUEST                               60
#define CMD_SET_HEARTBEAT_RATE_REQ                   61
#define CMD_HEARTBEAT_RESP                           62
#define CMD_SET_AUTO_TX_BUFFER                       63
#define CMD_GET_EXTENDED_INFO                        64
#define CMD_TCP_KEEPALIVE                            65
#define CMD_FLUSH_QUEUE_RESP                         66
#define CMD_HYDRA_TX_INTERVAL_REQ                    67
#define CMD_HYDRA_TX_INTERVAL_RESP                   68
#define CMD_SET_BUSPARAMS_FD_REQ                     69
#define CMD_SET_BUSPARAMS_FD_RESP                    70
// 71 can be reused
#define CMD_AUTO_TX_BUFFER_REQ                       72
#define CMD_AUTO_TX_BUFFER_RESP                      73
#define CMD_SET_TRANSCEIVER_MODE_REQ                 74
#define CMD_TREF_SOFNR                               75
#define CMD_SOFTSYNC_ONOFF                           76
#define CMD_USB_THROTTLE                             77
#define CMD_SOUND                                    78
#define CMD_LOG_TRIG_STARTUP                         79
#define CMD_SELF_TEST_REQ                            80
#define CMD_SELF_TEST_RESP                           81
// 82-84 can be reused
#define CMD_SET_BUSPARAMS_RESP                       85
#define CMD_SET_IO_PORTS_REQ                         86
#define CMD_GET_IO_PORTS_REQ                         87
#define CMD_GET_IO_PORTS_RESP                        88
// 89-96 can be used
#define CMD_GET_TRANSCEIVER_INFO_REQ                 97
#define CMD_GET_TRANSCEIVER_INFO_RESP                98
#define CMD_MEMO_CONFIG_MODE                         99
// 100 can be used
#define CMD_LED_ACTION_REQ                          101
#define CMD_LED_ACTION_RESP                         102
#define CMD_INTERNAL_DUMMY                          103
#define CMD_READ_USER_PARAMETER                     104
#define CMD_MEMO_CPLD_PRG                           105
#define CMD_LOG_MESSAGE                             106
#define CMD_LOG_TRIG                                107
#define CMD_LOG_RTC_TIME                            108

// 109 - 118 reserved

#define CMD_IMP_KEY                                 119

#define CMD_PRINTF                                  120
#define RES_PRINTF                                  (CMD_PRINTF + 128)

#define TRP_DATA                                    121

#define CMD_REGISTER_HE_REQ                         122
#define CMD_REGISTER_HE_RESP                        123
#define CMD_QUERY_ADDR_HE_REQ                       124
#define CMD_QUERY_ADDR_HE_RESP                      125
#define CMD_LISTEN_TO_HE_REQ                        126
#define CMD_LISTEN_TO_HE_RESP                       127
#define CMD_QUERY_NEXT_HE_REQ                       128
#define CMD_QUERY_NEXT_HE_RESP                      129

#define CMD_MEMORY_READ_REQ                         130
#define CMD_MEMORY_READ_RESP                        131
#define CMD_MEMORY_WRITE_REQ                        132
#define CMD_MEMORY_WRITE_RESP                       133
#define CMD_MEMORY_SEARCH_REQ                       134
#define CMD_MEMORY_SEARCH_RESP                      135

#define CMD_MEASURE                                 136

#define CMD_FATAL_ERROR                             137

#define CMD_LOG_ACTION                              138

#define CMD_IO_TRIG_REQ                             148
#define CMD_IO_TRIG_RESP                            149
#define CMD_IO_TRIG_MSG                             150

#define CMD_IO_PORT_INFO_REQ                        151
#define CMD_IO_PORT_INFO_RESP                       152
#define CMD_IO_PORT_CTRL_REQ                        153
#define CMD_IO_PORT_CTRL_RESP                       154

#define CMD_GET_FILE_COUNT_REQ                      158
#define CMD_GET_FILE_COUNT_RESP                     159
#define CMD_GET_FILE_NAME_REQ                       160
#define CMD_GET_FILE_NAME_RESP                      161

#define CMD_GET_NETWORK_DEVICE_NAME_REQ             162
#define CMD_GET_NETWORK_DEVICE_NAME_RESP            163

#define CMD_DEVICE_PING_REQ                         164
#define CMD_DEVICE_PING_RESP                        165


#define CMD_MAP_CHANNEL_REQ                         200
#define CMD_MAP_CHANNEL_RESP                        201
#define CMD_GET_SOFTWARE_DETAILS_REQ                202
#define CMD_GET_SOFTWARE_DETAILS_RESP               203

#define CMD_SET_DEVICE_MODE                         204
#define CMD_GET_DEVICE_MODE                         205

#define MEMO_STATUS_SUCCESS         0
#define MEMO_STATUS_MORE_DATA       1
#define MEMO_STATUS_UNKNOWN_COMMAND 2
#define MEMO_STATUS_FAILED          3
#define MEMO_STATUS_EOF             4

typedef struct {
  unsigned char   subCmd;
  unsigned char   reserved;
  unsigned short  data2;           // Sector count for disk reads
  uint32_t        data1;           // Sector number for disk reads
  unsigned char   data[20];
} hcmdMemoGetDataReq;

typedef struct {
  unsigned char   subCmd;
  unsigned char   dataLen;
  unsigned short  offset;
  unsigned char   data[20];        // Data, or status (dioStat in 0 and lioStat in 1)
  unsigned char   status;          // MEMO_STATUS_xxx
  unsigned char   reserved;
} hcmdMemoGetDataResp;

typedef struct {
  unsigned char   subCmd;
  unsigned char   reserved[3];
  uint32_t        data1;           // Sector number for disk reads
  unsigned short  data2;           // Sector count for disk reads
  unsigned char   reserved2[14];
} hcmdMemoPutDataStartReq;

typedef struct {
  unsigned char  subCmd;
  unsigned char  dataLen;
  unsigned short offset;
  unsigned char  data[20];
  unsigned short reserved;
} hcmdMemoPutDataReq;

typedef struct {
  unsigned char  subCmd;
  unsigned char  dataLen;
  unsigned char  status;
  unsigned char  reserved;
  unsigned char  data[20];
} hcmdMemoPutDataResp;

typedef struct {
  unsigned char  subCmd;
  unsigned char  reserved[3];
} hcmdMemoAsyncopStartReq;

typedef struct {
  unsigned char  subCmd;
  unsigned char  status;
  unsigned short reserved;
} hcmdMemoAsyncopStartResp;

typedef struct {
  unsigned char  subCmd;
  unsigned char  status;
  unsigned short reserved;
} hcmdMemoAsyncopFinishedResp;

typedef struct {
  unsigned char  subCmd;
  unsigned char  reserved[3];
} hcmdMemoAsyncopGetDataReq;

typedef struct {
  unsigned char  subCmd;
  unsigned char  status;
  unsigned char  reserved[2];
  unsigned char  data[20];
} hcmdMemoAsyncopGetDataResp;

typedef struct {
  unsigned char  subCmd;
  unsigned char  reserved[3];
} hcmdMemoAsyncopCancelReq;


// *All* subcommands to be in one number series
#define MEMO_SUBCMD_GET_FS_INFO       1   // Get DOS filesys info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_A   2   // Get disk info; for get_data
#define MEMO_SUBCMD_GET_DISK_INFO_B   3   // Get logio info; for get_data

#define MEMO_SUBCMD_READ_PHYSICAL_SECTOR         4
#define MEMO_SUBCMD_WRITE_PHYSICAL_SECTOR        5
#define MEMO_SUBCMD_ERASE_PHYSICAL_SECTOR        6
#define MEMO_SUBCMD_READ_LOGICAL_SECTOR          7
#define MEMO_SUBCMD_WRITE_LOGICAL_SECTOR         8
#define MEMO_SUBCMD_ERASE_LOGICAL_SECTOR         9

#define MEMO_SUBCMD_FORMAT_DISK      10   // Format disk (FAT16 or -32) asyncop
#define MEMO_SUBCMD_INIT_DISK        11   // Create logdata.kmf, asyncop
#define MEMO_SUBCMD_CLEAR_DATA       12   // Clear logdata.kmf, asyncop

#define MEMO_SUBCMD_GET_MISC_INFO    13   // for get_data
#define MEMO_SUBCMD_GET_RTC_INFO     14   // for get_data
#define MEMO_SUBCMD_PUT_RTC_INFO     15   // for put_data

#define MEMO_SUBCMD_GET_FS_INFO_B    16   // Get various filesystem info

#define MEMO_SUBCMD_FASTREAD_PHYSICAL_SECTOR         17
#define MEMO_SUBCMD_FASTREAD_LOGICAL_SECTOR          18

#define MEMO_SUBCMD_OPEN_FILE           19
#define MEMO_SUBCMD_READ_FILE           20
#define MEMO_SUBCMD_CLOSE_FILE          21
#define MEMO_SUBCMD_WRITE_FILE          22
#define MEMO_SUBCMD_DELETE_FILE         23


typedef struct {
  unsigned short  fat_size;                // Size of the FAT, in sectors
  unsigned short  fat_type;                // 12 or 16 depending on FAT type.
  unsigned short  dir_entries;             // Number of directory entries in the root dir
  unsigned short  cluster_size;            // Two-logarithm of the cluster size in sectors
  uint32_t        fat1_start;              // First FAT starts in this sector
  uint32_t        first_data_sector;       // First sector available for data
  uint32_t        last_data_sector;        // Last sector available for data
} hMemoDataFsInfo;

typedef struct {
  uint32_t  logfile_sectors;          // Number of sectors in log file
  uint32_t  first_param_sector;       // First sector for param.lif
  uint32_t  last_param_sector;        // Last sector for param.lif
  uint32_t  first_dbase_sector;       // First sector for databases (if any)
  uint32_t  last_dbase_sector;        // Last sector for databases (if any)
} hMemoDataFsInfoB;

typedef struct {
  unsigned char   productRev;
  unsigned char   oemId[2];
  unsigned char   reserved1;
  uint32_t        mfgId;
  char            productName[10];
  unsigned short  dateCode;
} hMemoDataDiskInfoA;

typedef struct {
  uint32_t        serialNumber;
  unsigned char   disk_type;
  unsigned char   version;
  unsigned char   read_time;
  unsigned char   wr_factor;
  unsigned char   file_format;
  unsigned char   erase_value;
  unsigned short  read_blk_size;
  unsigned short  wr_blk_size;
  unsigned short  trans_speed;
  uint32_t        data_size;
} hMemoDataDiskInfoB;


#define MEMO_POWER_BAT_FAULT      0x01    // Battery fault of some kind
#define MEMO_POWER_BAT_CHARGING   0x02    // Battery is charging
#define MEMO_POWER_BAT_POWER_OK   0x04    // Battery power OK
#define MEMO_POWER_EXTPOWER_OK    0x08    // External power OK
#define MEMO_POWER_USBPOWER_OK    0x10    // USB power OK
#define MEMO_POWER_BAT_FAULT_NTC  0x20    // NTC Battery fault

#define MEMO_TEMPERATURE_FAILURE  0       // Failed measuring temperature
#define MEMO_TEMPERATURE_MEMO2    1       // Temperature is encoded as in Memo2 (NTC 10K connected to 3.3V)
#define MEMO_TEMPERATURE_MHYDRA   2

typedef struct {
  unsigned char   diskPresent;
  unsigned char   configMode;
  unsigned char   diskWriteProtected;
  unsigned char   temperatureEncoding;    // Encoding of 'temperature' field
  unsigned short  powerStatus;            // MEMO_POWER_xxx flags
  unsigned short  temperature;            // Temperature, raw value
  unsigned char   cpldVersion;            // CPLD version number
  unsigned char   reserved[1];
  unsigned short  reserved1;
  unsigned short  battery;                // Battery voltage, raw value
  unsigned char   reserved2[6];
} hMemoDataMiscInfo;

typedef struct {
  unsigned char   second;
  unsigned char   minute;
  unsigned char   hour;
  unsigned char   day;
  unsigned char   month;
  unsigned char   year;
  unsigned short  padding2;
} hMemoDataRtcInfo;

typedef struct {
  uint32_t      maxDataSize;
  uint32_t      dbaseSpace;
  uint32_t      reserveSpace;
  unsigned char fileSystem;
  unsigned char reserved[7];
} hMemoInitDiskReq;

typedef struct {
  unsigned char dioStatus;
  unsigned char lioStatus;
  unsigned char reserved[10];
} hMemoInitDiskResp;

typedef struct hcanErrorFrameData_s {
  unsigned char busStatus;
  unsigned char errorFactor;
  unsigned char txErrorCounter;
  unsigned char rxErrorCounter;
} hcanErrorFrameData_t;



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
#define LED_SUBCOMMAND_LED_4_ON       10
#define LED_SUBCOMMAND_LED_4_OFF      11
#define LED_SUBCOMMAND_LED_5_ON       12
#define LED_SUBCOMMAND_LED_5_OFF      13
#define LED_SUBCOMMAND_LED_6_ON       14
#define LED_SUBCOMMAND_LED_6_OFF      15

#define CONFIG_DATA_CHUNK                             24


//===========================================================================
// Flags
//===========================================================================

//////////////////////////
// CAN message flags
#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_OVERRUN             0x02        // Some kind of overrun occured
#define MSGFLAG_NERR                0x04        // NERR active during this msg
#define MSGFLAG_WAKEUP              0x08        // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#define MSGFLAG_RESERVED_1          0x20        // Reserved for future usage
#define MSGFLAG_TX                  0x40        // TX acknowledge
#define MSGFLAG_TXRQ                0x80        // TX request

#define MSGFLAG_EXT                 0x20        // Extended message


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
#define HYDRA_TRANSCEIVER_TYPE_UNKNOWN         0
#define HYDRA_TRANSCEIVER_TYPE_251             1
#define HYDRA_TRANSCEIVER_TYPE_252             2 // 252/1053/1054 w/o opto
#define HYDRA_TRANSCEIVER_TYPE_SWC             6 // J2411 type
#define HYDRA_TRANSCEIVER_TYPE_1054_OPTO      11 // 1054 with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_SWC_OPTO       12 // J2411 type with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_1050           14 // TJA1050
#define HYDRA_TRANSCEIVER_TYPE_1050_OPTO      15 // TJA1050 with optical isolation
#define HYDRA_TRANSCEIVER_TYPE_LIN            19 // LIN

// Transceiver line modes. Must be the same as in canlib.h
#define HYDRA_TRANSCEIVER_LINEMODE_NA          0  // Not Affected/Not available.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_SLEEP   4  // SWC Sleep Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_NORMAL  5  // SWC Normal Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_FAST    6  // SWC High-Speed Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SWC_WAKEUP  7  // SWC Wakeup Mode.
#define HYDRA_TRANSCEIVER_LINEMODE_SLEEP       8  // Sleep mode for those supporting it.
#define HYDRA_TRANSCEIVER_LINEMODE_NORMAL      9  // Normal mode (the inverse of sleep mode) for those supporting it.
#define HYDRA_TRANSCEIVER_LINEMODE_STDBY      10  // Standby for those who support it
// Transceiver resnet modes. Not supported.
#define HYDRA_TRANSCEIVER_RESNET_NA            0


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

// Maximum length of a command. Do not change this.
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
#define SWOPTION_80_MHZ_CLK           0x20L // hires timers run at 80 MHZ
#define SWOPTION_24_MHZ_CLK           0x40L // hires timers run at 24 MHZ


// CMD_SET_AUTO_TX_REQ and _RESP enum values
#define AUTOTXBUFFER_CMD_GET_INFO     1     // Get implementation information
#define AUTOTXBUFFER_CMD_CLEAR_ALL    2     // Clear all buffers on a channel
#define AUTOTXBUFFER_CMD_ACTIVATE     3     // Activate a specific buffer
#define AUTOTXBUFFER_CMD_DEACTIVATE   4     // Dectivate a specific buffer
#define AUTOTXBUFFER_CMD_SET_INTERVAL 5     // Set tx buffer transmission interval
#define AUTOTXBUFFER_CMD_GENERATE_BURST 6   // Generate a burst of messages
#define AUTOTXBUFFER_CMD_SET_MSG_COUNT 7    // Set tx buffer message count
#define AUTOTXBUFFER_CMD_SET_BUFFER    8    // Set tx buffer message

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

// For canTimeStampRef in cmdGetCardInfoResp
#define CAN_TIME_STAMP_REF_ACK        0
#define CAN_TIME_STAMP_REF_SOF        1
#define CAN_TIME_STAMP_REF_INTERRUPT  2
#define CAN_TIME_STAMP_REF_CANTIMER   3


/*
** Command Structs
*/

typedef struct {
  unsigned char cmdLen;  // The length of the whole packet (i.e. including this byte)
  unsigned char cmdNo;
} hcmdLogHead;

typedef struct {
  uint32_t       timeL;
  uint32_t       timeH;
  uint8_t        key;
  uint8_t        padding[19];
} hcmdImpKey;

typedef struct {
  uint32_t       timeL;
  uint32_t       timeH;
  unsigned short len;      //  2 byte
  unsigned char  flags;    //  1 byte, 0: tpFollows, 1: Res first, 2: Res last
  unsigned char  reserved; //  1 byte unused
  char  string[16];        // 16 byte
} hcmdPrintf;

typedef struct {
  uint32_t       addr;     //  4 byte
  uint32_t       addr2;    //  4 byte
  unsigned short len;      //  2 byte
  unsigned char  flags;    //  1 byte, 0: tpFollows, 1: Res first, 2: Res last
  unsigned char  reserved; //  1 byte unused
  unsigned char  data[16]; // 16 byte
} hcmdMemory;

typedef struct {
  char           text[28];
} hcmdFatalError;

typedef struct {
  uint32_t       data[7];
} hcmdMeasure;

#define CMDPRINTF_TPFOLLOW_FLAG  0x01
#define CMDPRINTF_RESFIRST_FLAG  0x02
#define CMDPRINTF_RESLAST_FLAG   0x04

typedef struct {
  uint32_t       timeL;
  uint32_t       timeH;
  uint8_t        status;
  uint8_t        padding[19];
} hresPrintf;

#define HM_STATUS_OK 1
#define HM_STATUS_FAILED -1

#define TRPDATA_EOF_FLAG        0x01
#define TRPDATA_ASCII_FLAG      0x02
#define TRPDATA_DIT_FLAG        0x04
#define TRPDATA_RESWANTED_FLAG  0x08
#define TRPDATA_TRUNCATED_FLAG  0x10
#define TRPDATA_RSP_FLAG        0x20  // This TRP is a Response

#define TRPDATA_STATUS_OK          0
#define TRPDATA_STATUS_BUSY        1  // Busy, ask me later with an empty TRPDATA
#define TRPDATA_STATUS_RESEND      2  // Resend all data from last respons
#define TRPDATA_STATUS_ABORT       3  // Error, abort the complete transfer

typedef struct {
  unsigned char len;
  unsigned char flags;   /* 1: EOF, 2: ASCII, 4: DIT, 8: RESW */
  unsigned char status;
  unsigned char padding[1];
  unsigned char data[24];
} htrpData;


/*
** Command Structs
*/
#if !defined(__IAR_SYSTEMS_ICC__)
#  include <pshpack1.h>
#endif

typedef struct {
  uint8_t   he;        // Set to zero if not known
  uint8_t   channels;
  uint16_t  reserved;
  char      name[16];
} hcmdRegisterHeReq;

typedef struct {
  uint8_t he;
} hcmdRegisterHeResp;

struct hydraHostCmd;

typedef struct {
  uint8_t channel;
  uint8_t reserved[3];
  struct  hydraHostCmd *next;
  char    name[16];
} hcmdQueryAddrHeReq;

typedef struct {
  uint8_t he;
} hcmdQueryAddrHeResp;

typedef struct {
  uint8_t enable;
  uint8_t he;
  uint8_t channel;
} hcmdListenToHeReq;

typedef struct {
  uint8_t status;
} hcmdListenToHeResp;

typedef struct {
  uint8_t he;
} hcmdQueryNextHeReq;

typedef struct {
  uint8_t he;
  uint8_t channels;
  uint8_t reserved[2];
  char    name[16];
} hcmdQueryNextHeResp;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   channel;
    unsigned char   flags;
    unsigned short  time[3];
    unsigned char   dlc;
    unsigned char   padding;
    uint32_t        id;
    unsigned char   data[8];
} hcmdLogMessage;

typedef struct {
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   channel;

    unsigned char   action_type;
    uint32_t        action_param;

    unsigned short  statement_idx;
    unsigned short  action_idx;
} hcmdLogAction;

typedef struct
{
    unsigned char   cmdLen;
    unsigned char   cmdNo;
    unsigned char   type;
    unsigned char   padding;
    unsigned short  trigNo;
    unsigned short  time[3];
    uint32_t        preTrigger;
    uint32_t        postTrigger;
} hcmdLogTrig;

typedef struct
{
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char verMajor;            // File version number
    unsigned char verMinor;            // File version number
    unsigned int  unixTime;            // Seconds since 1970
    unsigned char hiresTimerFqMHz;     // High-resolution timer frequency, in MHz
    unsigned char padding[11];
} hcmdLogRtcTime;

typedef struct { 
    uint32_t        id;
    unsigned char   data[8];
    unsigned char   dlc;
    unsigned char   flags;
    unsigned short  transId;
    unsigned char   channel;
    unsigned char   padding[11];
} hcmdTxCanMessage;

typedef struct {
  uint32_t        id;
  unsigned char   data[8];
  unsigned char   dlc;
  unsigned char   flags;
  unsigned short  time[3];
  unsigned char   padding[8];
} hcmdTxAck;

typedef struct {
    unsigned char   reserved1;
    unsigned char   reserved2;
    unsigned short  time[3];
    unsigned short  padding;
} hcmdTxRequest;

typedef struct {
    uint32_t      bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
    unsigned char channel;
    unsigned char padding[19];
} hcmdSetBusparamsReq;

typedef struct {
    unsigned char param_type;
    unsigned char reserved[27];
} hcmdGetBusparamsReq;

typedef struct {
    uint32_t      bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
    unsigned char channel;
    unsigned char padding[19];
} hcmdGetBusparamsResp;

typedef struct {
    unsigned char   reserved;
} hcmdGetChipStateReq;

typedef struct {
    unsigned short time[3];
    unsigned char txErrorCounter;
    unsigned char rxErrorCounter;
    unsigned char busStatus;
    unsigned char channel;
    unsigned char padding[18];
} hcmdChipStateEvent;

typedef struct {
    unsigned char driverMode;
    unsigned char reserved;
} hcmdSetDrivermodeReq;

typedef struct {
    unsigned char reserved;
} hcmdGetDrivermodeReq;

typedef struct {
    unsigned char driverMode;
    unsigned char reserved;
} hcmdGetDrivermodeResp;

typedef struct {
    unsigned char reserved;
} hcmdResetChipReq;

typedef struct {
    unsigned char reserved;
} hcmdResetCardReq;

typedef struct {
    unsigned char reserved;
} hcmdStartChipReq;

typedef struct {
    unsigned char reserved;
} hcmdStartChipResp;

typedef struct {
    unsigned char reserved;
} hcmdStopChipReq;

typedef struct {
    unsigned char reserved;
} hcmdStopChipResp;

typedef struct {
    unsigned char flags;
} hcmdReadClockReq;

typedef struct {
    unsigned short  time[3];
    unsigned short  padding2;
} hcmdReadClockResp;

typedef struct {
    unsigned char reserved;
} hcmdSelfTestReq;

typedef struct {
    uint32_t  results;
} hcmdSelfTestResp;

typedef struct {
      signed char dataLevel;
} hcmdGetCardInfo2Req;

typedef struct {
    unsigned char pcb_id[24];
    uint32_t      oem_unlock_code;
} hcmdGetCardInfo2Resp;

typedef struct {
      signed char dataLevel;
} hcmdGetCardInfoReq;

typedef struct {
    uint32_t      serialNumber;
    uint32_t      clockResolution;
    uint32_t      mfgDate;
    unsigned char EAN[8];
    unsigned char hwRevision;
    unsigned char usbHsMode;
    unsigned char hwType;
    unsigned char canTimeStampRef;
    unsigned char channelCount;
} hcmdGetCardInfoResp;

typedef struct {
    unsigned char reserved;
} hcmdGetInterfaceInfoReq;

typedef struct {
    uint32_t        channelCapabilities;
    unsigned char   canChipType;
    unsigned char   canChipSubType;
    unsigned short  padding;
} hcmdGetInterfaceInfoResp;

typedef struct {
  unsigned char reserved[28];
} hcmdGetSoftwareInfoReq;

typedef struct {
    uint32_t        reserved1;
    uint32_t        reserved2;
    unsigned short  maxOutstandingTx;
    unsigned short  padding1;
    uint32_t        padding[4];
} hcmdGetSoftwareInfoResp;

typedef struct {
  unsigned char reserved[28];
} hcmdGetSoftwareDetailsReq;

typedef struct {
  uint32_t  swOptions;
  uint32_t  swVersion;
  uint32_t  swName;
  uint32_t  EAN[2];
  uint32_t  maxBitrate;
  uint32_t  padding[1];
} hcmdGetSoftwareDetailsResp;


typedef struct {
    unsigned char   reserved;
} hcmdGetBusLoadReq;

typedef struct {
    unsigned short  time[3];         // "Absolute" timestamp
    unsigned short  sample_interval; // Sampling interval in microseconds
    unsigned short  active_samples;  // Number of samples where tx or rx was active
    unsigned short  delta_t;         // Milliseconds since last response
    unsigned char   reserved;
} hcmdGetBusLoadResp;

typedef struct {
    unsigned char reserved;
} hcmdResetStatisticsReq;

typedef struct {
    unsigned char reserved;
} hcmdCheckLicenseReq;

typedef struct {
    uint32_t  licenseMask;
    uint32_t  kvaserLicenseMask;
} hcmdCheckLicenseResp;

typedef struct {
    unsigned short  time[3];
    unsigned char   reserved;
    unsigned char   errorCode;
    unsigned short  addInfo1;
    unsigned short  addInfo2;
} hcmdErrorEvent;

typedef struct {
    unsigned char flags;
    unsigned char reserved;
} hcmdFlushQueue;

typedef struct {
    unsigned char reserved;
} hcmdNoCommand;

typedef struct {
    unsigned char reserved;
} hcmdResetErrorCounter;

typedef struct {
    unsigned short  time[3];
    unsigned char   flags;
    unsigned char   reserved;
    unsigned char   txErrorCounter;
    unsigned char   rxErrorCounter;
    unsigned char   busStatus;
    unsigned char   errorFactor;
} hcmdCanErrorEvent;

typedef struct {
    unsigned char lineMode;
    unsigned char resistorNet;
    unsigned char reserved;
} hcmdSetTransceiverModeReq;

typedef struct { 
    unsigned short  time;
} hcmdInternalDummy;

typedef struct {
    unsigned char status;
} hcmdMemoCpldPrgResp;

typedef struct {
    unsigned char subCmd;
    unsigned char padding;
    unsigned char data[CONFIG_DATA_CHUNK];
} hcmdMemoCpldPrgReq;

typedef struct {
    unsigned char subCmd;
    unsigned char padding;
    signed short  timeout;
} hcmdLedActionReq;

typedef struct {
    unsigned char subCmd;
} hcmdLedActionResp;

typedef struct {
    unsigned char info;
} hcmdDiskFullInfo;

typedef struct {
  unsigned char userNo;
  unsigned char paramNo;
  unsigned char status;
  unsigned char padding;
  unsigned char data[8];
} hcmdReadUserParameter;

typedef struct {
    uint32_t        interval;
    unsigned char   padding[4];
} hcmdMemoConfigModeReq;

typedef struct {
    unsigned char   diskStat;
    unsigned char   configMode;
    unsigned char   reserved[10];
} hcmdMemoConfigModeResp;

typedef struct {
    unsigned char   subCmd;
    unsigned char   padding;
    unsigned short  freq;
    unsigned short  duration;
} hcmdSound;

// port status
#define IO_PORT_STATUS_MISSING  0
#define IO_PORT_STATUS_ENABLED  1
#define IO_PORT_STATUS_DISABLED 2

// port type
#define IO_PORT_TYPE_UNKNOWN      0
#define IO_PORT_TYPE_TRIGGER_IN   1
#define IO_PORT_TYPE_DIGITAL_OUT  2


// Subcommands
#define PORT_DISABLE     0
#define PORT_ENABLE      1
#define PORT_CONFIG_GET  4
#define PORT_CONFIG_SET  5

typedef struct {
  uint32_t        pulseDur;
  uint32_t        activeVal;
  uint32_t        dormantVal;
  uint32_t        portVal;
  unsigned short  type;                // IO_PORT_TYPE_xxx
  unsigned char   status;              // IO_PORT_STATUS_xxx
  unsigned char   subCmd;              // PORT_CONFIG_SET, PORT_xxx
  unsigned char   portNo;
  unsigned char   padding[7];
} hcmdIoPortCtrl;

typedef struct {
    unsigned char   portNo;             // Hardware-specific port #
    unsigned char   padding[3];
    uint32_t        portVal;            // Hardware-specific port value
} hcmdSetIoPortsReq;

typedef struct {
    unsigned char   origReq[12];
    unsigned char   portNo;             // Hardware-specific port #
    unsigned char   padding[15];
} hcmdGetIoPortsReq;

typedef struct {
    uint32_t        portVal;            // Hardware-specific port value
    unsigned char   origReq[12];
    unsigned char   portNo;             // Hardware-specific port #
    unsigned char   status;
    unsigned char   padding[10];
} hcmdGetIoPortsResp;

typedef struct {
    unsigned char mode;                 // Hardware-specific mode value
    unsigned char padding[3];
} hcmdSetDeviceModeReq;

typedef struct {
    unsigned char mode;                 // Hardware-specific mode value
    unsigned char status;
    unsigned char padding[2];
} hcmdGetDeviceModeResp;

typedef struct {
    unsigned char reserved;
} hcmdGetTransceiverInfoReq;

typedef struct {
  uint32_t      transceiverCapabilities;
  unsigned char transceiverStatus;
  unsigned char transceiverType;
  unsigned char reserved;
} hcmdGetTransceiverInfoResp;

typedef struct {
  unsigned short  rate;
  unsigned short  pad2;
} hcmdSetHeartbeatRateReq;

typedef struct {
  unsigned short  time[3];
} hcmdHeartbeatResp;

typedef struct {
  uint32_t        interval;
  unsigned char   requestType;
  unsigned char   bufNo;
  unsigned char   reserved;
  unsigned char   padding;
  uint32_t        id;
  unsigned char   data[8];
  unsigned char   dlc;
  unsigned char   flags;
  unsigned char   padding2[6];
} hcmdAutoTxBufferReq;

typedef struct {
  unsigned char   responseType;
  unsigned char   bufferCount;
  unsigned short  capabilities;
  uint32_t        timerResolution;
  uint32_t        status;
} hcmdAutoTxBufferResp;

typedef struct {
  unsigned short  time[3];
  unsigned short  sofNr;
} hcmdTrefSofSeq;

typedef struct {
  unsigned short  onOff;
} hcmdSoftSyncOnOff;

typedef struct {
  unsigned short  throttle;
} hcmdUsbThrottle;

typedef struct {
  unsigned char   subCmd;
  unsigned char   status;
  unsigned char   first;
  unsigned char   data[14];
  unsigned char   padding[11];
} hcmdGetExtendedInfoReq, hcmdGetExtendedInfoResp;

typedef struct {
  unsigned char   seqNo;
  unsigned char   reserved1;
  unsigned short  reserved2;
  unsigned int    time;
  unsigned char   data[16];
  unsigned char   padding[4];
} hcmdTcpKeepalive;

typedef struct {
  unsigned char   index;
  unsigned char   data[16];
  unsigned char   padding[11];
} hcmdNetworkDeviceName;

typedef struct {
  uint32_t        time_in_us;
  unsigned char   padding[24];
} hcmdDevicePing;

typedef union {
  struct {
    unsigned short  bulkLen;
  } cmdStartSet;
  struct {
    unsigned short  startOffset;
    unsigned short  payloadLengthWanted;
  } cmdStartGet;
} hcommandData;

typedef struct {
  char      name[16];           // CAN, LIN, WHATEVER
  uint8_t   channel;            // 0, 1,... (combine with name)
  uint8_t   reserved[11];
} hcmdMapChannelReq;


typedef struct {
  uint8_t   heAddress;         // Opaque for the driver.
  uint8_t   position;          // Physical "position" in device
  uint16_t  flags;             // Various flags, to be defined
  uint8_t   reserved1[24];
} hcmdMapChannelResp;

typedef struct {
  unsigned short  fileNo;            // File number
  unsigned short  fileCount;         // Number of files on SD card
  uint32_t        fileSize;
  uint32_t        fileTime;
  char            filename[14];        // Only 8.3
  short           status;
} hcmdFileInfo;

// Union for all logged messages.
typedef union {
  hcmdLogHead                  logHead;
  hcmdLogMessage               logMessage;
  hcmdLogTrig                  logTrig;
  hcmdLogRtcTime               logRtcTime;
  hcmdLogAction                logAction;
} hydraHostCmdLog;


// Well-known HEs
#define BROADCAST         0x0f
#define BROADCAST_DEBUG   0x1f
#define BROADCAST_ERROR   0x2f
#define ROUTER_HE         0x00
#define DYNAMIC_HE        ROUTER_HE
#define ILLEGAL_HE        0x3e


#define HYDRA_CMD_SIZE          32

typedef struct hydraHostCmd {
  unsigned char  cmdNo;
  union {
    struct {
      unsigned char dstAddr    : 6;
      unsigned char srcChannel : 2;
    } cmdIOP;
    unsigned char heAddress;    // 0..5: dstHE, 6..7: srcHE-msb-part
  };
  union {
    struct {
      unsigned short transId : 12; //  0..11: Seq
      unsigned short srcHE   : 4;  // 12..15: srcHe-lsb-part
    } cmdIOPSeq;
    unsigned short transId;     // 0..11 transId; 12..15 don't care
  };

  union {
    hcmdFatalError            fatalError;
    hcmdImpKey                keyMsg;
    hcmdPrintf                printfMsg;
    hresPrintf                printfRes;
    htrpData                  trpDataMsg;

    hcmdMemory                memory;
    hcmdMeasure               measure;

    hcmdLogMessage            logMessage;
    hcmdLogTrig               logTrig;
    hcmdLogRtcTime            logRtcTime;
    hcmdLogAction             logAction;
    hcmdTxCanMessage          txCanMessage;
    hcmdTxAck                 txAck;
    hcmdTxRequest             txRequest;
    hcmdSetBusparamsReq       setBusparamsReq;

    hcmdGetBusparamsReq       getBusparamsReq;
    hcmdGetBusparamsResp      getBusparamsResp;
    hcmdGetChipStateReq       getChipStateReq;
    hcmdChipStateEvent        chipStateEvent;
    hcmdSetDrivermodeReq      setDrivermodeReq;
    hcmdGetDrivermodeReq      getDrivermodeReq;

    hcmdResetChipReq          resetChipReq;
    hcmdResetCardReq          resetCardReq;
    hcmdStartChipReq          startChipReq;
    hcmdStartChipResp         startChipResp;
    hcmdStopChipReq           stopChipReq;
    hcmdStopChipResp          stopChipResp;

    hcmdGetBusLoadReq         getBusLoadReq;
    hcmdGetBusLoadResp        getBusLoadResp;
    hcmdCanErrorEvent         canErrorEvent;
    hcmdFlushQueue            flushQueue;
    hcmdResetErrorCounter     resetErrorCounter;

    hcmdGetCardInfo2Req          getCardInfo2Req;
    hcmdGetCardInfo2Resp         getCardInfo2Resp;
    hcmdGetCardInfoReq           getCardInfoReq;
    hcmdGetCardInfoResp          getCardInfoResp;
    hcmdGetInterfaceInfoReq      getInterfaceInfoReq;
    hcmdGetInterfaceInfoResp     getInterfaceInfoResp;
    hcmdGetSoftwareInfoReq       getSoftwareInfoReq;
    hcmdGetSoftwareInfoResp      getSoftwareInfoResp;

    hcmdResetStatisticsReq       resetStatisticsReq;
    hcmdErrorEvent               errorEvent;
    hcmdNoCommand                noCommand;
    hcmdCheckLicenseReq          checkLicenseReq;
    hcmdCheckLicenseResp         checkLicenseResp;
    hcmdReadClockReq             readClockReq;
    hcmdReadClockResp            readClockResp;
    hcmdSelfTestReq              selfTestReq;
    hcmdSelfTestResp             selfTestResp;
    hcmdSetTransceiverModeReq    setTransceiverModeReq;
    hcmdInternalDummy            internalDummy;

    hcmdRegisterHeReq            registerHeReq;
    hcmdRegisterHeResp           registerHeResp;
    hcmdQueryAddrHeReq           queryAddrHeReq;
    hcmdQueryAddrHeResp          queryAddrHeResp;
    hcmdListenToHeReq            listenToHeReq;
    hcmdListenToHeResp           listenToHeResp;
    hcmdQueryNextHeReq           queryNextHeReq;
    hcmdQueryNextHeResp          queryNextHeResp;

    hcmdSetDeviceModeReq         setDeviceModeReq;
    hcmdGetDeviceModeResp        getDeviceModeResp;

    hcmdLedActionReq             ledActionReq;
    hcmdLedActionResp            ledActionResp;

    hcmdIoPortCtrl               ioPortCtrl;

    hcmdSetIoPortsReq            setIoPortsReq;
    hcmdGetIoPortsReq            getIoPortsReq;
    hcmdGetIoPortsResp           getIoPortsResp;

    hcmdGetTransceiverInfoReq    getTransceiverInfoReq;
    hcmdGetTransceiverInfoResp   getTransceiverInfoResp;

    hcmdSetHeartbeatRateReq      setHeartbeatRateReq;
    hcmdHeartbeatResp            heartbeatResp;

    hcmdAutoTxBufferReq          autoTxBufferReq;
    hcmdAutoTxBufferResp         autoTxBufferResp;

    hcmdTrefSofSeq               trefSofSeq;
    hcmdSoftSyncOnOff            softSyncOnOff;
    hcmdUsbThrottle              usbThrottle;
    hcmdSound                    sound;

    hcmdMemoCpldPrgReq           memoCpldPrgReq;
    hcmdMemoCpldPrgResp          memoCpldPrgResp;
    hcmdMemoConfigModeReq        memoConfigModeReq;
    hcmdMemoConfigModeResp       memoConfigModeResp;

    hcmdMemoGetDataReq           memoGetDataReq;
    hcmdMemoGetDataResp          memoGetDataResp;
    hcmdMemoPutDataStartReq      memoPutDataStartReq;
    hcmdMemoPutDataReq           memoPutDataReq;
    hcmdMemoPutDataResp          memoPutDataResp;
    hcmdMemoAsyncopStartReq      memoAsyncopStartReq;
    hcmdMemoAsyncopStartResp     memoAsyncopStartResp;
    hcmdMemoAsyncopGetDataReq    memoAsyncopGetDataReq;
    hcmdMemoAsyncopGetDataResp   memoAsyncopGetDataResp;
    hcmdMemoAsyncopCancelReq     memoAsyncopCancelReq;
    hcmdMemoAsyncopFinishedResp  memoAsyncopFinishedResp;
    hcmdDiskFullInfo             diskFullInfo;

    hcmdReadUserParameter        readUserParameter;

    hcmdGetExtendedInfoReq       getExtendedInfoReq;
    hcmdGetExtendedInfoResp      getExtendedInfoResp;

    hcmdTcpKeepalive             tcpKeepalive;

    hcmdNetworkDeviceName        networkDeviceNameReq;
    hcmdNetworkDeviceName        networkDeviceNameResp;
    hcmdDevicePing               devicePingReq;
    hcmdDevicePing               devicePingResp;

    hcmdMapChannelReq             mapChannelReq;
    hcmdMapChannelResp            mapChannelResp;
    hcmdGetSoftwareDetailsReq     getSoftwareDetailsReq;
    hcmdGetSoftwareDetailsResp    getSoftwareDetailsResp;
    hcmdFileInfo                  fileInfo;


#ifdef HYDRA_PRIVATE
    hcmdHydraOtherCommand        o;
#endif

  } ;
} hydraHostCmd;

#define HE_BITS    4
#define CH_BITS    2
#define SEQ_BITS   8
#if (2 * CH_BITS + HE_BITS) > 8
# error "Too many HE or CH bits!"
#endif
#if (HE_BITS + SEQ_BITS) > 16
# error "Too many HE or SEQ bits!"
#endif
#define ADDR_BITS  (HE_BITS + CH_BITS)
#define CH_COUNT   (1 << CH_BITS)
#define HE_COUNT   (1 << HE_BITS)
#define ADDR_COUNT (1 << (HE_BITS + CH_BITS))
#define SEQ_COUNT  (1 << SEQ_BITS)
#define KLI_COUNT  8
#define CH_MASK    ((CH_COUNT - 1) << HE_BITS)
#define CH_HI_MASK ((CH_COUNT - 1) << (HE_BITS + CH_BITS))
#define HE_MASK    (HE_COUNT - 1)
#define ADDR_MASK  (ADDR_COUNT - 1)
#define SEQ_MASK   (SEQ_COUNT - 1)
#define KLI_MASK   (KLI_COUNT - 1)

#define getSRC(cmd)     (((((cmd)->heAddress) & CH_HI_MASK) >> CH_BITS) |  \
                         (((cmd)->transId) >> SEQ_BITS))
#define getSEQ(cmd)     ((cmd)->transId & SEQ_MASK)
#define setDST(cmd,dst)                                          \
  do { (cmd)->heAddress = ((cmd)->heAddress    & CH_HI_MASK) |   \
                          ((dst)               & ADDR_MASK);     \
  } while (0)
#define setSRC(cmd,src)                                          \
  do { (cmd)->heAddress = ((cmd)->heAddress    & ADDR_MASK)  |   \
                          (((src) << CH_BITS)  & CH_HI_MASK);    \
       (cmd)->transId   = ((cmd)->transId      & SEQ_MASK)   |   \
                          ((src) << SEQ_BITS);                   \
                         } while (0)
#define setSEQ(cmd,seq)                                          \
  do { (cmd)->transId = ((cmd)->transId        & ~SEQ_MASK)  |   \
                         ((seq)                & SEQ_MASK);      \
  } while (0)

// hydraHostCmd must be 32 bytes long!
#if defined(CompilerAssert)
CompilerAssert((sizeof(hydraHostCmd) == HYDRA_CMD_SIZE));
// A basic sanity check of all structs:
#define CHECK_ALIGNMENT(X) CompilerAssert((sizeof(X) % 4) == 0)
CHECK_ALIGNMENT(hydraHostCmd                 );

#endif

//Used when cmdNo is 0xFF (CMD_EXTENDED)
typedef struct {
  unsigned char  cmdNo;
  unsigned char  cmdIOP;
  unsigned short cmdIOPSeq;
  unsigned short cmdLen;
  unsigned char  cmdNoExt;
  unsigned char  Reserved;

  union {
    int q;
  } ;
} hydraHostCmdExt;

#if !defined(__IAR_SYSTEMS_ICC__)
#  include <poppack.h>
#endif

#endif //_HYDRA_HOST_CMDS_H_
