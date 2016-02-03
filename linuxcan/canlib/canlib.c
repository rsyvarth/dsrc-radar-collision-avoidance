/*
**                Copyright 2012 by Kvaser AB, Mölndal, Sweden
**                        http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ============================================================================
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
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
** THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ============================================================================
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

/* Kvaser Linux Canlib */

#include "canlib.h"

#include "canlib_data.h"

#include "osif_user.h"
#include "osif_functions_user.h"

#include "vcan_ioctl.h"    // Need this for IOCtl to check # channels
#include "vcanevt.h"

#include "VCanFunctions.h"
#include "debug.h"

#include <stdio.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <unistd.h>
#   include <errno.h>
#   include <signal.h>
#   include <pthread.h>

#include <string.h>


#   if DEBUG
#      define DEBUGPRINT(args) printf args
#   else
#      define DEBUGPRINT(args)
#   endif



static const char *errorStrings[] = {
  "No error",                        // canOK
  "Error in parameter",              // canERR_PARAM
  "No messages available",           // canERR_NOMSG
  "Specified device not found",      // canERR_NOTFOUND
  "Out of memory",                   // canERR_NOMEM
  "No channels available",           // canERR_NOCHANNELS
  "Interrupted by signal",           // canERR_INTERRUPTED
  "Timeout occurred",                // canERR_TIMEOUT
  "Library not initialized",         // canERR_NOTINITIALIZED
  "No more handles",                 // canERR_NOHANDLES
  "Handle is invalid",               // canERR_INVHANDLE
  "Unknown error (-11)",             // canERR_INIFILE
  "CAN driver type not supported",   // canERR_DRIVER
  "Transmit buffer overflow",        // canERR_TXBUFOFL
  "Unknown error (-14)",             // canERR_RESERVED_1
  "A hardware error was detected",   // canERR_HARDWARE
  "Can not find requested DLL",      // canERR_DYNALOAD
  "DLL seems to be wrong version",   // canERR_DYNALIB
  "Error initializing DLL or driver", // canERR_DYNAINIT
  "Operation not supported by hardware or firmware", // canERR_NOT_SUPPORTED
  "Unknown error (-20)",             // canERR_RESERVED_5
  "Unknown error (-21)",             // canERR_RESERVED_6
  "Unknown error (-22)",             // canERR_RESERVED_2
  "Can not load or open the device driver", // canERR_DRIVERLOAD
  "The I/O request failed, probably due to resource shortage", //canERR_DRIVERFAILED
  "Unknown error (-25)",             // canERR_NOCONFIGMGR
  "Card not found"                   // canERR_NOCARD
  "Unknown error (-27)",             // canERR_RESERVED_7
  "Config not found",                // canERR_REGISTRY
  "The license is not valid",        // canERR_LICENSE
  "Internal error in the driver",    // canERR_INTERNAL
  "Access denied",                   // canERR_NO_ACCESS
  "Not implemented"                  // canERR_NOT_IMPLEMENTED
};

struct dev_descr {
   char * descr_string;
   unsigned int ean[2];
};

// This has to be modified if we add/remove drivers.
static const char *dev_name[] = {"lapcan",   "pcican",   "pcicanII",
                                 "usbcanII", "leaf",     "kvvirtualcan",
                                 "mhydra", "pciefd"};
static const char *off_name[] = {"LAPcan",   "PCIcan",   "PCIcanII",
                                 "USBcanII", "Leaf",     "VIRTUALcan",
                                 "Minihydra", "PCIe CAN"};
static struct dev_descr dev_descr_list[] = {
          {"Kvaser Unknown",                                    {0x00000000, 0x00000000}},
          {"Kvaser Virtual CAN",                                {0x00000000, 0x00000000}},
          {"Kvaser PCIcan-S, 1*HS",                             {0x30000827, 0x00073301}},
          {"Kvaser PCIcan-D, 2*HS",                             {0x30000834, 0x00073301}},
          {"Kvaser PCIcan-Q, 4 *HS drivers",                    {0x30000841, 0x00073301}},
          {"Kvaser PCIcan II S",                                {0x30001565, 0x00073301}},
          {"Kvaser PCIcan II D",                                {0x30001572, 0x00073301}},
          {"Kvaser USBcan II HS (S)",                           {0x30001589, 0x00073301}},
          {"Kvaser USBcan II HS/HS",                            {0x30001596, 0x00073301}},
          {"Kvaser Memorator HS/LS",                            {0x30001701, 0x00073301}},
          {"Kvaser USBcan II HS/LS",                            {0x30001749, 0x00073301}},
          {"Kvaser Memorator HS/HS",                            {0x30001756, 0x00073301}},
          {"Kvaser USBcan Rugged HS",                           {0x30001800, 0x00073301}},
          {"Kvaser USBcan Rugged HS/HS",                        {0x30001817, 0x00073301}},
          {"Kvaser PCIcanP-Swc",                                {0x30002210, 0x00073301}},
          {"Kvaser USBcan II HS-SWC",                           {0x30002319, 0x00073301}},
          {"Kvaser Memorator HS-SWC",                           {0x30002340, 0x00073301}},
          {"Kvaser Leaf Light HS",                              {0x30002418, 0x00073301}},
          {"Kvaser Leaf SemiPro HS",                            {0x30002425, 0x00073301}},
          {"Kvaser Leaf Professional HS",                       {0x30002432, 0x00073301}},
          {"Kvaser Leaf SemiPro LS",                            {0x30002609, 0x00073301}},
          {"Kvaser Leaf Professional LSS",                      {0x30002616, 0x00073301}},
          {"Kvaser Leaf SemiPro SWC",                           {0x30002630, 0x00073301}},
          {"Kvaser Leaf Professional SWC",                      {0x30002647, 0x00073301}},
          {"Kvaser Leaf Professional LIN",                      {0x30002692, 0x00073301}},
          {"Kvaser PCIcanx 4xHS",                               {0x30003309, 0x00073301}},
          {"Kvaser PCIcanx HS/HS",                              {0x30003316, 0x00073301}},
          {"Kvaser PCIcanx HS",                                 {0x30003323, 0x00073301}},
          {"Kvaser PCIcanx II 2*HS",                            {0x30003439, 0x00073301}},
          {"Kvaser PCIcanx II 1*HS, combo",                     {0x30003446, 0x00073301}},
          {"Kvaser Memorator Professional (HS/HS)",             {0x30003514, 0x00073301}},
          {"Kvaser USBcan Professional (HS/HS)",                {0x30003576, 0x00073301}},
          {"Kvaser Leaf Light HS with OBDII connector",         {0x30004023, 0x00073301}},
          {"Kvaser Leaf SemiPro HS with OBDII connector",       {0x30004030, 0x00073301}},
          {"Kvaser Leaf Professional HS with OBDII connector",  {0x30004047, 0x00073301}},
          {"Kvaser PCIEcan HS/HS",                              {0x30004054, 0x00073301}},
          {"Kvaser Leaf Light GI (Galvanic Isolation)",         {0x30004115, 0x00073301}},
          {"Kvaser USBcan Professional HS/HS, with (standard) RJ45 connectors",   {0x30004139, 0x00073301}},
          {"Kvaser Memorator Professional HS/LS",               {0x30004177, 0x00073301}},
          {"Kvaser Leaf Light Rugged HS",                       {0x30004276, 0x00073301}},
          {"Kvaser Leaf Light HS China",                        {0x30004351, 0x00073301}},
          {"Kvaser BlackBird SemiPro HS",                       {0x30004412, 0x00073301}},
          {"Kvaser BlackBird SemiPro 3xHS",                     {0x30004467, 0x00073301}},
          {"Kvaser BlackBird SemiPro HS/HS",                    {0x30004535, 0x00073301}},
          {"Kvaser Memorator R SemiPro",                        {0x30004900, 0x00073301}},
          {"Kvaser Leaf SemiPro Rugged HS",                     {0x30005068, 0x00073301}},
          {"Kvaser Leaf Professional Rugged HS",                {0x30005099, 0x00073301}},
          {"Kvaser Memorator Light HS",                         {0x30005136, 0x00073301}},
          {"Kvaser Memorator Professional CB",                  {0x30005815, 0x00073301}},
          {"Kvaser Eagle",                                      {0x30005679, 0x00073301}},
          {"Kvaser Leaf Light GI (Medical)",                    {0x30005686, 0x00073301}},
          {"Kvaser USBcan Pro SHS/HS",                          {0x30005716, 0x00073301}},
          {"Kvaser USBcan Pro SHS/SHS",                         {0x30005723, 0x00073301}},
          {"Kvaser USBcan R",                                   {0x30005792, 0x00073301}},
          {"Kvaser BlackBird SemiPro",                          {0x30006294, 0x00073301}},
          {"Kvaser BlackBird v2",                               {0x30006713, 0x00073301}},
          {"Kvaser USBcan Professional CB",                     {0x30006843, 0x00073301}},
          {"Kvaser Leaf Light v2",                              {0x30006850, 0x00073301}},
          {"Kvaser Mini PCI Express HS",                        {0x30006881, 0x00073301}},
          {"Kvaser PCIEcan 4xHS",                               {0x30006829, 0x00073301}},
          {"Kvaser Leaf Light HS v2 OEM",                       {0x30007352, 0x00073301}},
          {"Kvaser Ethercan Light HS",                          {0x30007130, 0x00073301}},
          {"Kvaser Mini PCI Express 2xHS",                      {0x30007437, 0x00073301}},
          {"Kvaser USBcan Light 2xHS",                          {0x30007147, 0x00073301}},
          {"Kvaser PCIEcan 4xHS",                               {0x30006836, 0x00073301}},
          {"Kvaser Memorator Pro 5xHS",                         {0x30007789, 0x00073301}},
          {"Kvaser USBcan Pro 5xHS",                            {0x30007796, 0x00073301}},
          {"Kvaser USBcan Light 4xHS",                          {0x30008311, 0x00073301}},
          {"Kvaser Leaf Pro HS v2",                             {0x30008434, 0x00073301}},
          {"Kvaser USBcan Pro 2xHS v2",                         {0x30007529, 0x00073301}},
          {"Kvaser Memorator 2xHS v2",                          {0x30008212, 0x00073301}},
          {"Kvaser Memorator Pro 2xHS v2",                      {0x30008199, 0x00073301}}
};

//******************************************************
// Find out channel specific data
//******************************************************
static
canStatus getDevParams (int channel, char devName[], int *devChannel,
                        CANOps **canOps, char officialName[])
{
  // For now, we just count the number of /dev/%s%d files (see dev_name),
  // where %d is numbers between 0 and 255.
  // This is slow!
  // qqq Maybe we can read the dev names from some struct?

  int         chanCounter = 0;
  int         devCounter  = 0;
  struct stat stbuf;

  int n = 0;

  int CardNo          = -1;
  int ChannelNoOnCard = 0;
  int ChannelsOnCard  = 0;
  int err;
  OS_IF_FILE_HANDLE fd;

  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    CardNo = -1;
    ChannelNoOnCard = 0;
    ChannelsOnCard = 0;

    // There are 256 minor inode numbers
    for(devCounter = 0; devCounter <= 255; devCounter++) {
      snprintf(devName, DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], devCounter);
      if (stat(devName, &stbuf) != -1) {  // Check for existance

        if (!ChannelsOnCard) {
          err = 1;
          fd = open(devName, O_RDONLY);
          if (fd != -1) {
            err = os_if_ioctl_read(fd, VCAN_IOC_GET_NRCHANNELS,
                                   &ChannelsOnCard, sizeof(ChannelsOnCard));
            close(fd);
          }
          if (err) {
            ChannelsOnCard = 1;
          } else {
            ChannelNoOnCard = 0;
             // qqq, CardNo for a given card will decrease if
             //      a card with lower No is removed.
            CardNo++;
          }
        } else {
          ChannelNoOnCard++;
        }
        ChannelsOnCard--;

        if (chanCounter++ == channel) {
          *devChannel = channel;
          *canOps = &vCanOps;
          sprintf(officialName, "KVASER %s channel %d", off_name[n], devCounter);
          *devChannel = ChannelNoOnCard;

          return canOK;
        }
      }
      else {
        // Handle gaps in device numbers
        continue;
      }
    }
  }

  DEBUGPRINT((TXT("return canERR_NOTFOUND\n")));
  devName[0]  = 0;
  *devChannel = -1;
  *canOps     = NULL;

  return canERR_NOTFOUND;
}




//
// API FUNCTIONS
//

//******************************************************
// Open a can channel
//******************************************************
CanHandle CANLIBAPI canOpenChannel (int channel, int flags)
{
  canStatus          status;
  HandleData         *hData;
  CanHandle          hnd;

  hData = (HandleData *)OS_IF_ALLOC_MEM(sizeof(HandleData));
  if (hData == NULL) {
    DEBUGPRINT((TXT("ERROR: cannot allocate memory (%d)\n"),
                (int)sizeof(HandleData)));
    return canERR_NOMEM;
  }

  memset(hData, 0, sizeof(HandleData));

  hData->readIsBlock      = 1;
  hData->writeIsBlock     = 1;
  hData->isExtended       = flags & canOPEN_REQUIRE_EXTENDED;
  if (flags & canOPEN_CAN_FD_NONISO)
    hData->fdMode = OPEN_AS_CANFD_NONISO;
  else if (flags & canOPEN_CAN_FD)
    hData->fdMode = OPEN_AS_CANFD_ISO;
  else
    hData->fdMode = OPEN_AS_CAN;
  hData->acceptLargeDlc   = ((flags & canOPEN_ACCEPT_LARGE_DLC) != 0);
  hData->wantExclusive    = flags & canOPEN_EXCLUSIVE;
  hData->acceptVirtual    = flags & canOPEN_ACCEPT_VIRTUAL;
  hData->notifyFd         = OS_IF_INVALID_HANDLE;

  status = getDevParams(channel,
                        hData->deviceName,
                        &hData->channelNr,
                        &hData->canOps,
                        hData->deviceOfficialName);

  if (status < 0) {
    DEBUGPRINT((TXT("getDevParams ret %d\n"), status));
    OS_IF_FREE_MEM(hData);
    return status;
  }

  status = hData->canOps->openChannel(hData);
  if (status < 0) {
    DEBUGPRINT((TXT("openChannel ret %d\n"), status));
    OS_IF_FREE_MEM(hData);
    return status;
  }

  hnd = insertHandle(hData);

  if (hnd < 0) {
    DEBUGPRINT((TXT("insertHandle ret %d\n"), hnd));
    OS_IF_CLOSE_HANDLE(hData->fd);
    OS_IF_FREE_MEM(hData);
    return canERR_NOMEM;
  }

  return hnd;
}


//******************************************************
// Close can channel
//******************************************************
int CANLIBAPI canClose (const CanHandle hnd)
{
  HandleData *hData;

  // Try to go Bus Off before closing
  canBusOff(hnd);

  hData = removeHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  canSetNotify(hnd, NULL, 0, NULL);
  if (OS_IF_IS_CLOSE_ERROR(OS_IF_CLOSE_HANDLE(hData->fd))) {
    return canERR_INVHANDLE;
  }

  OS_IF_FREE_MEM(hData);

  return canOK;
}


//******************************************************
// Get raw handle/file descriptor to use in system calls
//******************************************************
canStatus CANLIBAPI canGetRawHandle (const CanHandle hnd, void *pvFd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  *(OS_IF_FILE_HANDLE *)pvFd = hData->fd;

  return canOK;
}


//******************************************************
// Go on bus
//******************************************************
canStatus CANLIBAPI canBusOn (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->busOn(hData);
}


//******************************************************
// Go bus off
//******************************************************
canStatus CANLIBAPI canBusOff (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->busOff(hData);
}


//******************************************************
// Try to "reset" the CAN bus.
//******************************************************
canStatus CANLIBAPI canResetBus (const CanHandle hnd)
{
  canStatus stat;
  unsigned long handle_status;

  stat = canReadStatus(hnd, &handle_status);
  if (stat < 0) {
    return stat;
  }
  stat = canBusOff(hnd);
  if (stat < 0) {
    return stat;
  }
  if ((handle_status & canSTAT_BUS_OFF) == 0) {
    stat = canBusOn(hnd);
  }

  return stat;
}



//******************************************************
// Set bus parameters
//******************************************************
canStatus CANLIBAPI
canSetBusParams (const CanHandle hnd, long freq, unsigned int tseg1,
                 unsigned int tseg2, unsigned int sjw,
                 unsigned int noSamp, unsigned int syncmode)
{
  canStatus ret;
  HandleData *hData;
  long freq_brs;
  unsigned int tseg1_brs, tseg2_brs, sjw_brs;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (freq < 0) {
    ret = canTranslateBaud(&freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode);
    if (ret != canOK) {
      return ret;
    }
  }

  ret = hData->canOps->getBusParams(hData, NULL, NULL, NULL, NULL, NULL,
                                    &freq_brs, &tseg1_brs, &tseg2_brs,
                                    &sjw_brs, NULL);
  if (ret != canOK) {
    return ret;
  }

  return hData->canOps->setBusParams(hData, freq, tseg1, tseg2, sjw, noSamp,
                                     freq_brs, tseg1_brs, tseg2_brs, sjw_brs,
                                     syncmode);
}

//******************************************************
// Set CAN FD bus parameters
//******************************************************
canStatus CANLIBAPI
canSetBusParamsFd (const CanHandle hnd, long freq_brs, unsigned int tseg1_brs,
                   unsigned int tseg2_brs, unsigned int sjw_brs)
{
  canStatus ret;
  HandleData *hData;
  long freq;
  unsigned int tseg1, tseg2, sjw, noSamp, syncmode;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  if ((OPEN_AS_CANFD_ISO != hData->fdMode) && (OPEN_AS_CANFD_NONISO != hData->fdMode))
    return canERR_INVHANDLE;

  ret = hData->canOps->getBusParams(hData, &freq, &tseg1, &tseg2, &sjw, &noSamp,
                                    NULL, NULL, NULL, NULL, &syncmode);
  if (ret != canOK) {
    return ret;
  }

  if (freq_brs < 0) {
    ret = canTranslateBaud(&freq_brs, &tseg1_brs, &tseg2_brs, &sjw_brs, &noSamp, &syncmode);
    if (ret != canOK) {
      return ret;
    }
  }

  return hData->canOps->setBusParams(hData, freq, tseg1, tseg2, sjw, noSamp,
                                     freq_brs, tseg1_brs, tseg2_brs, sjw_brs,
                                     syncmode);
}

canStatus CANLIBAPI
canSetBusParamsC200 (const CanHandle hnd, unsigned char btr0, unsigned char btr1)
{
  long bitrate;
  unsigned int tseg1;
  unsigned int tseg2;
  unsigned int sjw;
  unsigned int noSamp;
  unsigned int syncmode = 0;

  sjw     = ((btr0 & 0xc0) >> 6) + 1;
  tseg1   = ((btr1 & 0x0f) + 1);
  tseg2   = ((btr1 & 0x70) >> 4) + 1;
  noSamp  = ((btr1 & 0x80) >> 7) ? 3 : 1;
  bitrate = 8000000L * 64 / (((btr0 & 0x3f) + 1) << 6) /
            (tseg1 + tseg2 + 1);

  return canSetBusParams(hnd, bitrate, tseg1, tseg2, sjw, noSamp, syncmode);
}


//******************************************************
// Get bus parameters
//******************************************************
canStatus CANLIBAPI
canGetBusParams (const CanHandle hnd, long *freq, unsigned int *tseg1,
                 unsigned int *tseg2, unsigned int *sjw,
                 unsigned int *noSamp, unsigned int *syncmode)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->getBusParams(hData, freq, tseg1, tseg2, sjw, noSamp,
                                     NULL, NULL, NULL, NULL, syncmode);
}


//******************************************************
// Get CAN FD bus parameters
//******************************************************
canStatus CANLIBAPI
canGetBusParamsFd (const CanHandle hnd, long *freq, unsigned int *tseg1,
                   unsigned int *tseg2, unsigned int *sjw)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  if (!hData->fdMode) return canERR_INVHANDLE;

  return hData->canOps->getBusParams(hData, NULL, NULL, NULL, NULL, NULL,
                                     freq, tseg1, tseg2, sjw, NULL);
}


//******************************************************
// Set bus output control (silent/normal)
//******************************************************
canStatus CANLIBAPI
canSetBusOutputControl (const CanHandle hnd, const unsigned int drivertype)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (drivertype != canDRIVER_NORMAL && drivertype != canDRIVER_OFF &&
      drivertype != canDRIVER_SILENT && drivertype != canDRIVER_SELFRECEPTION) {
    return canERR_PARAM;
  }

  return hData->canOps->setBusOutputControl(hData, drivertype);
}


//******************************************************
// Get bus output control (silent/normal)
//******************************************************
canStatus CANLIBAPI
canGetBusOutputControl (const CanHandle hnd, unsigned int * drivertype)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->getBusOutputControl(hData, drivertype);
}


//******************************************************
// Set filters
//******************************************************
canStatus CANLIBAPI canAccept (const CanHandle hnd,
                               const long envelope,
                               const unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->accept(hData, envelope, flag);
}


//******************************************************
// Read bus status
//******************************************************
canStatus CANLIBAPI canReadStatus (const CanHandle hnd,
                                   unsigned long *const flags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readStatus(hData, flags);
}


//******************************************************
// Read the error counters
//******************************************************
canStatus CANLIBAPI canReadErrorCounters (const CanHandle hnd,
                                          unsigned int *txErr,
                                          unsigned int *rxErr,
                                          unsigned int *ovErr)
{
  HandleData *hData;
  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readErrorCounters(hData, txErr, rxErr, ovErr);
}


//******************************************************
// Write can message
//******************************************************
canStatus CANLIBAPI
canWrite (const CanHandle hnd, long id, void *msgPtr,
          unsigned int dlc, unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->write(hData, id, msgPtr, dlc, flag);
}


//******************************************************
// Write can message and wait
//******************************************************
canStatus CANLIBAPI
canWriteWait (const CanHandle hnd, long id, void *msgPtr,
              unsigned int dlc, unsigned int flag, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->writeWait(hData, id, msgPtr, dlc, flag, timeout);
}


//******************************************************
// Read can message
//******************************************************
canStatus CANLIBAPI
canRead (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
         unsigned int *flag, unsigned long *time)
{
  int i;
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (hData->syncPressent == 1)
  {
    hData->syncPressent = 0;
    if (id)   *id = hData->syncId;
    if (msgPtr != NULL) {
      int count = hData->syncDlc;
      if (count > 8) {
        count = 8;
      }
      for (i = 0; i < count; i++)
        ((unsigned char *)msgPtr)[i] = hData->syncMsg[i];
    }
    if (dlc)  *dlc = hData->syncDlc;
    if (flag) *flag = hData->syncFlag;
    if (time) *time = hData->syncTime;
    return canOK;
  }
  else
  {
    return hData->canOps->read(hData, id, msgPtr, dlc, flag, time);
  }
}


//*********************************************************
// Read can message or wait until one appears or timeout
//*********************************************************
canStatus CANLIBAPI
canReadWait (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
             unsigned int *flag, unsigned long *time, unsigned long timeout)
{
  int i;
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (hData->syncPressent == 1)
  {
    hData->syncPressent = 0;
    if (id)   *id = hData->syncId;
    if (msgPtr != NULL) {
      int count = hData->syncDlc;
      if (count > 8) {
        count = 8;
      }
      for (i = 0; i < count; i++)
        ((unsigned char *)msgPtr)[i] = hData->syncMsg[i];
    }
    if (dlc)  *dlc = hData->syncDlc;
    if (flag) *flag = hData->syncFlag;
    if (time) *time = hData->syncTime;
    return canOK;
  }
  else
  {
    return hData->canOps->readWait(hData, id, msgPtr, dlc, flag, time, timeout);
  }
}

//*********************************************************
// Waits until the receive buffer contains at least one
// message or a timeout occurs.
//*********************************************************
canStatus CANLIBAPI
canReadSync(const CanHandle hnd, unsigned long timeout)
{
  canStatus CANLIBAPI stat;

  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
	return canERR_INVHANDLE;
  }

  if (hData->syncPressent == 0)
  {
		stat = canReadWait (hnd, &hData->syncId, &hData->syncMsg, &hData->syncDlc,
				  &hData->syncFlag, &hData->syncTime, timeout);
		if (stat != canOK)
		{
		  hData->syncPressent = 0;
		  return canERR_TIMEOUT;
		}
		else
		{
		  hData->syncPressent = 1;
		  return canOK;
		}
  }
  else
  {
	  return canOK;
  }
}
//****************************************************************
// Wait until all can messages on a circuit are sent or timeout
//****************************************************************
canStatus CANLIBAPI
canWriteSync (const CanHandle hnd, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->writeSync(hData, timeout);
}


//******************************************************
// IOCTL
//******************************************************
canStatus CANLIBAPI
canIoCtl (const CanHandle hnd, unsigned int func,
          void *buf, unsigned int buflen)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, func, buf, buflen);
}


//******************************************************
// Read the time from hw
//******************************************************
canStatus CANLIBAPI canReadTimer (const CanHandle hnd, unsigned long *time)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readTimer(hData, time);
}


//******************************************************
// Translate from baud macro to bus params
//******************************************************
canStatus CANLIBAPI canTranslateBaud (long *const freq,
                                      unsigned int *const tseg1,
                                      unsigned int *const tseg2,
                                      unsigned int *const sjw,
                                      unsigned int *const nosamp,
                                      unsigned int *const syncMode)
{
  switch (*freq) {

  case canFD_BITRATE_8M_60P:
      *freq     = 8000000L;
      *tseg1    = 2;
      *tseg2    = 2;
      *sjw      = 1;
      break;

  case canFD_BITRATE_4M_80P:
      *freq     = 4000000L;
      *tseg1    = 7;
      *tseg2    = 2;
      *sjw      = 2;
      break;

  case canFD_BITRATE_2M_80P:
      *freq     = 2000000L;
      *tseg1    = 15;
      *tseg2    = 4;
      *sjw      = 4;
      break;

  case canFD_BITRATE_1M_80P:
      *freq     = 1000000L;
      *tseg1    = 31;
      *tseg2    = 8;
      *sjw      = 8;
      break;

  case canFD_BITRATE_500K_80P:
      *freq     = 500000L;
      *tseg1    = 63;
      *tseg2    = 16;
      *sjw      = 16;
      break;

  case BAUD_1M:
    *freq     = 1000000L;
    *tseg1    = 5;
    *tseg2    = 2;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_500K:
    *freq     = 500000L;
    *tseg1    = 5;
    *tseg2    = 2;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_250K:
    *freq     = 250000L;
    *tseg1    = 5;
    *tseg2    = 2;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_125K:
    *freq     = 125000L;
    *tseg1    = 11;
    *tseg2    = 4;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_100K:
    *freq     = 100000L;
    *tseg1    = 11;
    *tseg2    = 4;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case canBITRATE_83K:
    *freq     = 83333L;
    *tseg1    = 5;
    *tseg2    = 2;
    *sjw      = 2;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_62K:
    *freq     = 62500L;
    *tseg1    = 11;
    *tseg2    = 4;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_50K:
    *freq     = 50000L;
    *tseg1    = 11;
    *tseg2    = 4;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case canBITRATE_10K:
    *freq     = 10000L;
    *tseg1    = 11;
    *tseg2    = 4;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  default:
    return canERR_PARAM;
  }

  return canOK;
}


//******************************************************
// Get error text
//******************************************************
canStatus CANLIBAPI
canGetErrorText (canStatus err, char *buf, unsigned int bufsiz)
{
  signed char code;

  code = (signed char)(err & 0xFF);

  if (!buf || bufsiz == 0) {
    return canOK;
  }
  if ((code <= 0) && (-code < sizeof(errorStrings) / sizeof(char *))) {
    if (errorStrings [-code] == NULL) {
      snprintf(buf, bufsiz, "Unknown error (%d)", (int)code);
    } else {
      strncpy(buf, errorStrings[-code], bufsiz);
    }
  } else {
    strncpy(buf, "This is not an error code", bufsiz);
  }
  buf[bufsiz - 1] = '\0';

  return canOK;
}


//******************************************************
// Get library version
//******************************************************
unsigned short CANLIBAPI canGetVersion (void)
{
  return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;
}


//******************************************************
// Get the total number of channels
//******************************************************
canStatus CANLIBAPI canGetNumberOfChannels (int *channelCount)
{
  // For now, we just count the number of /dev/%s%d files (see dev_name),
  // where %d is numbers between 0 and 255.
  // This is slow!

  int tmpCount = 0;
  int cardNr;
  char filename[DEVICE_NAME_LEN];
  int n = 0;

  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    // There are 256 minor inode numbers
    for(cardNr = 0; cardNr <= 255; cardNr++) {
      snprintf(filename,  DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], cardNr);
      if (os_if_access(filename, F_OK) == 0) {  // Check for existance
        tmpCount++;
      }
      else {
        // Handle gaps in device numbers
        continue;
      }
    }
  }

  *channelCount = tmpCount;

  return canOK;
}



//******************************************************
// Find device description data fom EAN
//******************************************************

canStatus CANLIBAPI
canGetDescrData (void *buffer, unsigned int ean[], int cap)
{
  unsigned int len, i;

  len = sizeof(dev_descr_list)/sizeof(struct dev_descr);

  /* set Unknown device description */
  strcpy(buffer, dev_descr_list[0].descr_string);

  /* check in device is virtual device */
  if ((ean[0] == dev_descr_list[1].ean[0]) &&
      (ean[1] == dev_descr_list[1].ean[1]) &&
      (cap & canCHANNEL_CAP_VIRTUAL))
  {
    strcpy(buffer, dev_descr_list[1].descr_string);
  }
  /* search for description by matching ean number */
  else
  {
    for (i = 2; i<len; i++)
    {
      if ((ean[0] == dev_descr_list[i].ean[0]) && (ean[1] == dev_descr_list[i].ean[1]))
      {
        strcpy(buffer, dev_descr_list[i].descr_string);
        break;
      }
    }
  }
  return canOK;

}

//******************************************************
// Find out channel specific data
//******************************************************
canStatus CANLIBAPI
canGetChannelData (int channel, int item, void *buffer, size_t bufsize)
{
  canStatus status;
  HandleData hData;
  unsigned int ean[2];
  int cap = 0;

  status = getDevParams(channel, hData.deviceName, &hData.channelNr,
                        &hData.canOps, hData.deviceOfficialName);

  if (status < 0) {
    return status;
  }

  switch(item) {
  case canCHANNELDATA_CHANNEL_NAME:
    strcpy(buffer, hData.deviceOfficialName);
    bufsize = strlen(hData.deviceOfficialName);
    return canOK;

  case canCHANNELDATA_CHAN_NO_ON_CARD:
    bufsize = sizeof(hData.channelNr);
    memcpy(buffer, &hData.channelNr, bufsize);
    return canOK;

  case canCHANNELDATA_DEVDESCR_ASCII:
    status = hData.canOps->getChannelData(hData.deviceName,
                                          canCHANNELDATA_CARD_UPC_NO,
                                          &ean,
                                          sizeof(ean));
    if (status != canOK)
    {
      return status;
    }

    status = hData.canOps->getChannelData(hData.deviceName,
                                          canCHANNELDATA_CHANNEL_CAP,
                                          &cap,
                                          sizeof(cap));
    if (status != canOK)
    {
      return status;
    }

    return canGetDescrData(buffer, ean, cap);

  case canCHANNELDATA_MFGNAME_ASCII:
    strcpy(buffer, "KVASER AB");
    return canOK;

  default:
    return hData.canOps->getChannelData(hData.deviceName,
                                        item,
                                        buffer,
                                        bufsize);
  }
}


//===========================================================================
canStatus CANLIBAPI canObjBufFreeAll (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufFreeAll(hData);
}


//===========================================================================
canStatus CANLIBAPI canObjBufAllocate (const CanHandle hnd, int type)
{
  HandleData *hData;
  int number;
  canStatus status;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  status = hData->canOps->objbufAllocate(hData, type, &number);

  return (status == canOK) ? number : status;
}


//===========================================================================
canStatus CANLIBAPI canObjBufFree (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufFree(hData, idx);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufWrite (const CanHandle hnd, int idx, int id, void *msg,
                unsigned int dlc, unsigned int flags)
{
  HandleData *hData;
  unsigned int canio_flags = 0;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }


  if (flags & canMSG_EXT) {
    id |= EXT_MSG;
  }
  if (flags & canMSG_RTR) {
    canio_flags |= VCAN_MSG_FLAG_REMOTE_FRAME;
  }

  return hData->canOps->objbufWrite(hData, idx, id, msg, dlc, canio_flags);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetFilter (const CanHandle hnd, int idx,
                    unsigned int code, unsigned int mask)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetFilter(hData, idx, code, mask);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetFlags (const CanHandle hnd, int idx, unsigned int flags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetFlags(hData, idx, flags);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetPeriod (const CanHandle hnd, int idx, unsigned int period)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetPeriod(hData, idx, period);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetMsgCount (const CanHandle hnd, int idx, unsigned int count)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetMsgCount(hData, idx, count);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSendBurst (const CanHandle hnd, int idx, unsigned int burstLen)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSendBurst(hData, idx, burstLen);
}


//===========================================================================
canStatus CANLIBAPI canObjBufEnable (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufEnable(hData, idx);
}


//===========================================================================
canStatus CANLIBAPI canObjBufDisable (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufDisable(hData, idx);
}


//******************************************************
// Flush receive queue
//******************************************************
canStatus CANLIBAPI
canFlushReceiveQueue (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, canIOCTL_FLUSH_RX_BUFFER, NULL, 0);
}


//******************************************************
// Flush transmit queue
//******************************************************
canStatus CANLIBAPI
canFlushTransmitQueue (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, canIOCTL_FLUSH_TX_BUFFER, NULL, 0);
}




//******************************************************
// Set notification callback
//******************************************************
canStatus CANLIBAPI
canSetNotify (const CanHandle hnd, void (*callback)(canNotifyData *),
              unsigned int notifyFlags, void *tag)
//
// Notification is done by filtering out interesting messages and
// doing a blocked read from a thread.
//
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (notifyFlags == 0 || callback == NULL) {
    if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
      // We want to shut off notification, close file and clear callback

      pthread_cancel(hData->notifyThread);

      // Wait for thread to finish
      pthread_join(hData->notifyThread, NULL);

      if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
        OS_IF_CLOSE_HANDLE(hData->notifyFd);
      }
      hData->notifyFd = OS_IF_INVALID_HANDLE;
    }

    return canOK;
  }

  hData->notifyData.tag = tag;

  return hData->canOps->setNotify(hData, callback, NULL, notifyFlags);
}

kvStatus CANLIBAPI kvSetNotifyCallback(const CanHandle hnd,
                                       kvCallback_t callback, void* context,
                                       unsigned int notifyFlags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (notifyFlags == 0 || callback == NULL) {
    if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
      // We want to shut off notification, close file and clear callback

      pthread_cancel(hData->notifyThread);

      // Wait for thread to finish
      pthread_join(hData->notifyThread, NULL);

      if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
        OS_IF_CLOSE_HANDLE(hData->notifyFd);
      }
      hData->notifyFd = OS_IF_INVALID_HANDLE;
    }

    return canOK;
  }

  hData->notifyData.tag = context;

  return hData->canOps->setNotify(hData, NULL, callback, notifyFlags);
}


//******************************************************
// Initialize library
//******************************************************
void CANLIBAPI canInitializeLibrary (void)
{
  return;
}
