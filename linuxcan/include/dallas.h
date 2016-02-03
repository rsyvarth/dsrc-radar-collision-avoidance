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

#ifndef DALLAS_H
#define DALLAS_H

/*****************************************************************************/

/* Commands to Dallas-type Memories */
#define READ_STATUS                 0x66
#define READ_MEMORY                 0xf0
#define WRITE_SCRATCHPAD            0x0f
#define READ_SCRATCHPAD             0xaa
#define COPY_SCRATCHPAD             0x55
#define WRITE_APPLICATION           0x99
#define READ_APPLICATION            0xc3
#define COPY_AND_LOCK_APPLICATION   0x5a
#define READ_ROM                    0x33
#define MATCH_ROM                   0x55
#define SKIP_ROM                    0xcc
#define SEARCH_ROM                  0xf0

/* Device Types */
#define DS1494_CODE                 0x04
#define DS2430_CODE                 0x14
#define DS2431_CODE                 0x2d

#define COPY_PASSWORD               0xa5
#define STATUS_PASSWORD             0x00

#define MAXMEMORY                   0x04
#define MAXHITLEVEL                 0x04

#define ONE_MEMORY          0x01
#define ALL_FOUND           0x02
#define NOMEMORY            0x03

/*****************************************************************************/

typedef struct s_dallas_context DALLAS_CONTEXT;

struct s_dallas_context {
         void __iomem   *address_out;
         void __iomem   *address_in;
         unsigned int   in_mask;
         unsigned int   out_mask;
};

typedef enum {
          dsError_None = 0,           // 0
          dsError_NoDevice,           // 1
          dsError_WrongDevice,        // 2
          dsError_ScratchPadError,    // 3
          dsError_ROMError,           // 4
          dsError_EEPROMError,        // 5
          dsError_BusStuckLow,        // 6
          dsError_NotProgrammed,      // 7
          dsError_ApplicationLocked,  // 8
          dsError_CRCError            // 9
} dsStatus;

/*****************************************************************************/

dsStatus ds_init(DALLAS_CONTEXT *dc);
void ds_shutdown(DALLAS_CONTEXT *dc);

dsStatus ds_reset(DALLAS_CONTEXT *dc);

int ds_check_for_presence(DALLAS_CONTEXT *dc);

dsStatus ds_read_string(DALLAS_CONTEXT *dc,
                        unsigned int command,
                        unsigned int address,
                        unsigned char *data,
                        int bufsiz);

dsStatus ds_read_memory(unsigned char memory_type,
                        DALLAS_CONTEXT *dc,
                        unsigned int address,
                        unsigned char *data,
                        size_t bufsiz);

dsStatus ds_read_application_area(DALLAS_CONTEXT *dc,
                                  unsigned int address,
                                  unsigned char *data,
                                  int bufsiz);

dsStatus ds_read_rom_64bit(DALLAS_CONTEXT *dc, int port, unsigned char *data);

int ds_verify_area(unsigned char *data, int bufsiz);


/*****************************************************************************/

#endif // DALLAS_H

