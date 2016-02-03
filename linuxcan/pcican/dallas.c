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

/* Kvaser CAN driver PCIcan hardware specific parts                    */
/* Functions for accessing the Dallas circuits.                        */
//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------


#include <linux/delay.h>
#include <linux/types.h>
#include <asm/io.h>
// new
#include "osif_functions_kernel.h"
//
#include "dallas.h"
#include "util.h"

/*****************************************************************************/

#define T_SLOT          120
#define T_LOW            15
#define T_WR_SAMPLE      60
#define T_RELEASE        45

/*****************************************************************************/
// new
static OS_IF_LOCK dallas_lock;
static int dallas_lock_initialized = 0;
//
static void dallas_set_bit (DALLAS_CONTEXT *dc)
{
  unsigned char tmp;
  tmp = ioread8(dc->address_out);
  iowrite8(tmp | dc->out_mask, dc->address_out);
} /* dallas_set_bit */


static void dallas_clr_bit (DALLAS_CONTEXT *dc)
{
  unsigned char tmp;
  tmp = ioread8(dc->address_out);
  iowrite8(tmp & ~dc->out_mask, dc->address_out);
} /* dallas_clr_bit */


static int dallas_bit_value (DALLAS_CONTEXT *dc)
{
  return ioread8(dc->address_in) & dc->in_mask;
} /* dallas_bit_value */


/*****************************************************************************/

/* Write one bit.
 */
static void ds_write_bit (DALLAS_CONTEXT *dc, int value)
{
  unsigned long irqFlags;

  os_if_spin_lock_irqsave(&dallas_lock, &irqFlags);
  
  //REPT
  dallas_clr_bit(dc);

  if (value) {
    /* DS2430: A "one" is a low level; 1us < t < 15 us. */
    /* DS2431: A "one" is a low level; 5us < t < 15 us. */
    udelay(5);
    dallas_set_bit(dc);
    os_if_spin_unlock_irqrestore(&dallas_lock, irqFlags);
    udelay(T_SLOT);
  }
  else {
    /* A "zero" is a low level; 60us < t < 120 us. */
    udelay(T_WR_SAMPLE);
    dallas_set_bit(dc);
    os_if_spin_unlock_irqrestore(&dallas_lock, irqFlags);
    udelay(T_SLOT - T_WR_SAMPLE);
  }
} /* ds_write_bit */



/* Read one bit.
 */
static int ds_read_bit (DALLAS_CONTEXT *dc)
{
  int b;
  unsigned long irqFlags;

  os_if_spin_lock_irqsave(&dallas_lock, &irqFlags);

  //REPT
  dallas_clr_bit(dc);
  udelay(5);
  dallas_set_bit(dc);
  udelay(1);
  b = dallas_bit_value(dc);

  os_if_spin_unlock_irqrestore(&dallas_lock, irqFlags);

  udelay(T_SLOT);

  return b;
} /* ds_read_bit */


/*****************************************************************************/

/* Initialize the I/O port. */
dsStatus ds_init (DALLAS_CONTEXT *dc)
{
  if (!dallas_lock_initialized) {
    os_if_spin_lock_init(&dallas_lock);
    dallas_lock_initialized = 1;
  }
  if (!dc->in_mask) {
    return dsError_NoDevice;
  }
  //dallas_init(dc);
  dallas_set_bit(dc);

  return dsError_None;
} /* ds_init */

void ds_shutdown (DALLAS_CONTEXT *dc)
{
  if (dallas_lock_initialized) {
    os_if_spin_lock_remove(&dallas_lock);
    dallas_lock_initialized = 0;
  }
}

/* Perform a device reset. */
dsStatus ds_reset (DALLAS_CONTEXT *dc)
{
  int i, x;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }

  /* Output a low level */
  dallas_clr_bit(dc);

  udelay(550);

  dallas_set_bit(dc);
  udelay(5);

  /* Wait for presence pulse. Sample two times with 5us between
     each sample. Do not loop more than 150 times. */

  for (i = 0; i < 150; i++) {
    x = 0;
    if (dallas_bit_value(dc) == 0) {
      x++;
    }
    udelay(5);
    if (dallas_bit_value(dc) == 0) {
      x++;
    }
    if (x == 2) {
      break;
    }
  }

  if (x != 2) {
    return dsError_NoDevice;
  }

  /* Wait for the bus to become free. Sample two times with 5us between
     each sample. Do not loop more than 150 times. */

  x = 0;
  for (; i < 150; i++) {
    x = 0;
    if (dallas_bit_value(dc)) {
      x++;
    }
    udelay(5);

    if (dallas_bit_value(dc)) {
      x++;
    }

    if (x == 2) {
      break;
    }
  }

  if (x != 2) {
    return dsError_BusStuckLow;
  }

  return dsError_None;
} /* ds_reset */


/*****************************************************************************/

/* Read a whole byte from the device.
 */
static unsigned int ds_read_byte (DALLAS_CONTEXT *dc)
{
  int x;
  unsigned int data;

  data = 0;
  for (x = 0; x < 8; x++) {
    data |= ((ds_read_bit(dc) ? 1 : 0) << x);
  }

  return data;
} /* ds_read_byte */


/* Write a whole byte to the device.
 */
static void ds_write_byte (DALLAS_CONTEXT *dc, unsigned int data)
{
  int x;
  for (x = 0; x < 8; x++) {
    ds_write_bit(dc, (data >> x) & 1);
  }
} /* ds_write_byte */


/*****************************************************************************/




/* Return 1 if at least one device is present on the bus.
 * Loops 100 times if no device is found.
 */
int ds_check_for_presence (DALLAS_CONTEXT *dc)
{
  int i;

  if (!dc->in_mask) {
    return 0;
  }

  for (i = 0; i < 100; i++) {
    if (ds_reset(dc) == dsError_None) {
      return 1;
    }
  }

  return 0;
} /* ds_check_for_presence */


/* Read data from the memory.
 * This will work for commands READ MEMORY, READ SCRATCHPAD,
 * READ STATUS, READ APPLICATION REG
 */
dsStatus ds_read_string (DALLAS_CONTEXT *dc,
                         unsigned int command,
                         unsigned int address,
                         unsigned char *data,
                         int bufsiz)
{
  unsigned int i;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }

  ds_write_byte(dc, command);
  ds_write_byte(dc, address);

  for (i = 0; i < bufsiz; i++) {
    *(data + i) = (unsigned char)ds_read_byte(dc);
  }

  return ds_reset(dc);
} /* ds_read_string */


/* Read data from the memory.
 */
static dsStatus ds2430_read_memory (DALLAS_CONTEXT *dc,
                                    unsigned int address,
                                    unsigned char *data,
                                    int bufsiz)
{
  dsStatus stat;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }

  address &= 0x1F;
  stat = ds_reset(dc);
  if (stat != dsError_None) {
    return stat;
  }

  /* Only one device; skip ROM check */
  ds_write_byte(dc, SKIP_ROM);

  return ds_read_string(dc, READ_MEMORY, address, data, bufsiz );
} /* ds2430_read_memory */


static dsStatus ds2431_read_memory (DALLAS_CONTEXT *dc,
                                    unsigned int address,
                                    unsigned char *data,
                                    int bufsiz)
{
  dsStatus stat;
  unsigned int i;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }
  if (address + bufsiz > 0x90) {
    return dsError_NoDevice;
  }

  stat = ds_reset(dc);
  if (stat != dsError_None) {
    return stat;
  }

  /* Only one device; skip ROM check */
  ds_write_byte(dc, SKIP_ROM);
  ds_write_byte(dc, READ_MEMORY);
  ds_write_byte(dc, address);
  ds_write_byte(dc, address >> 8);

  for (i = 0; i < bufsiz; i++) {
    data[i] = (unsigned char)ds_read_byte(dc);
  }

  return ds_reset(dc);
} /* ds2431_read_memory */


typedef struct {
  unsigned int crc;
  unsigned int count;
} bufdata_t;


dsStatus ds_read_memory (unsigned char memory_type,
                         DALLAS_CONTEXT *dc,
                         unsigned int address,
                         unsigned char *data,
                         size_t bufsiz)
{
  const int repeatCount = 10;    // Should perhaps be user selectable
  bufdata_t result[repeatCount];
  int found;
  unsigned int maxcount, crc, i, j, maxidx;
  dsStatus stat;

  memset(result, 0, sizeof(result));

  for (i = 0; i < repeatCount; i++) {
    // Read memory and calculate checksum
    if (memory_type == DS2431_CODE) {
      ds2431_read_memory(dc, address, data, bufsiz);
    } else {
      ds2430_read_memory(dc, address, data, bufsiz);
    }
    crc = calculateCRC32(data, bufsiz);

    // Find same checksum in table, or first unused entry
    found = 0;
    for (j = 0; j < repeatCount; j++) {
      if (result[j].count == 0) {
        break;
      } else if (result[j].crc == crc) {
        found = 1;
        result[j].count++;
        break;
      }
    }

    if (!found) {
      // First time this crc was found; store.
      result[j].crc   = crc;
      result[j].count = 1;
    }
    // If we are repeating, sleep for a millisecond or two.
    if (repeatCount > 1) {
      os_if_set_task_uninterruptible ();
      os_if_wait_for_event_timeout_simple(2);
    }
  }

  maxidx = 0;
  maxcount = 0;
  for (i = 0; i < repeatCount; i++) {
    if (result[i].count > maxcount) {
      maxcount = result[i].count;
      // DbgPrint("%s dallas[%d].count=%d\n", hwif.hw_name, i, maxcount);
      maxidx = i;
    }
  }

  // Try to get the most common data again
  for (i = 0; i < repeatCount; i++) {
    // Read memory and calculate checksum
    if (memory_type == DS2431_CODE) {
      ds2431_read_memory(dc, address, data, bufsiz);
    } else {
      ds2430_read_memory(dc, address, data, bufsiz);
    }
    crc = calculateCRC32(data, bufsiz);
    if (crc == result[maxidx].crc) {
      break;
    }
  }

  if (i == repeatCount) {
    // Unable to find selected result again...
    stat = dsError_NotProgrammed;   // For lack of anything better
  } else if (repeatCount > 1) {
    if (maxcount < (repeatCount / 2)) {
      // Majority readings are < 50%, so fail
      stat = dsError_NotProgrammed; // Not entirely correct code, but...
    } else {
      stat = dsError_None;
    }
  } else {
    // We read the contents just once so we don't know if we failed or not.
    stat = dsError_None;
  }


  return stat;
}


/* Read the 64-bit (8-byte) application area of some devices.
 */
dsStatus ds_read_application_area (DALLAS_CONTEXT *dc,
                                   unsigned int address,
                                   unsigned char *data,
                                   int bufsiz)
{
  unsigned int x;
  dsStatus stat;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }

  stat = ds_reset(dc);
  if (stat != dsError_None) {
    return stat;
  }

  ds_write_byte(dc, SKIP_ROM);
  ds_write_byte(dc, READ_STATUS);
  ds_write_byte(dc, 0); /* Validation byte */

  x = ds_read_byte(dc);
  if (x != 0xFC) {
    return dsError_NotProgrammed;
  }

  stat = ds_reset(dc);
  if (stat != dsError_None) {
    return stat;
  }

  ds_write_byte(dc, SKIP_ROM);
  ds_write_byte(dc, READ_APPLICATION);

  /* LSB of address */
  ds_write_byte(dc, address & 0xFF);

  for (x = 0; x < bufsiz; x++) {
    *(data + x) = (unsigned char)ds_read_byte(dc);
  }

  return dsError_None;
} /* ds_read_application_area */


static unsigned char calcCRC8rom (unsigned char *buf)
{
  int i;
  unsigned char crc = 0;

  for (i = 0; i < 8; i++) {
    unsigned char Data = buf[i] ^ crc;
    crc = 0;
    if (Data & 0x01) crc ^= 0x5e;
    if (Data & 0x02) crc ^= 0xbc;
    if (Data & 0x04) crc ^= 0x61;
    if (Data & 0x08) crc ^= 0xc2;
    if (Data & 0x10) crc ^= 0x9d;
    if (Data & 0x20) crc ^= 0x23;
    if (Data & 0x40) crc ^= 0x46;
    if (Data & 0x80) crc ^= 0x8c;
  }

  return crc;
}


/* Read data from the ROM (which contains the S/N).
 */
static dsStatus ds_read_rom_64bit_single (DALLAS_CONTEXT *dc, int port,
                                          unsigned char *data)
{
  int x;
  dsStatus stat;

  if (!dc->in_mask) {
    return dsError_NoDevice;
  }

  stat = ds_reset(dc);
  if (stat != dsError_None) {
    return stat;
  }

  ds_write_byte(dc, READ_ROM);

  for (x = 0; x < 8; x++) {
    *(data + x) = (unsigned char)ds_read_byte(dc);
  }

  if (*data != DS2430_CODE && *data != DS2431_CODE) {
    return dsError_WrongDevice;
  }

  return calcCRC8rom(data) ? dsError_CRCError : dsError_None;
} /* ds_read_rom_64bit_single */


dsStatus ds_read_rom_64bit (DALLAS_CONTEXT *dc, int port, unsigned char *data)
{
  int maxLoop = 10;
  dsStatus stat = dsError_None;

  while (maxLoop--) {
    stat = ds_read_rom_64bit_single(dc, port, data);
    if (stat == dsError_None) {
      break;
    }
  }

  return stat;
} /* ds_read_rom_64bit */


int ds_verify_area (unsigned char *data, int bufsiz)
{
  unsigned int i;
  unsigned char sum = 0;

  for (i = 0; i < bufsiz; i++) {
    sum += data[i];
  }

  return (sum == 0xFF);
} /* ds_verify_area */


/*****************************************************************************/
