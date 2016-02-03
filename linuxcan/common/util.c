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

#include <linux/module.h>
#include "asm/div64.h"
#include "util.h"



/****************************************************************************/
void packed_EAN_to_BCD(unsigned char *ean, unsigned char *bcd)
{
  long long ean64;
  int i;

  //
  // "ean" points to a 40 bit number (LSB first) which is the EAN code.
  // The ean code is followed by the check digit in a separate byte.
  // This is the standard format for DRVcans and LAPcan.
  //
  // "bcd" points to a buffer which will contain the EAN number as
  // a BCD string, LSB first. The buffer must be at least 8 bytes.
  //
  // E.g. 733-0130-00122-0 would be (hex)
  // 20 12 00 30 01 33 07 00
  //
  ean64 = 0;
  for (i = 0; i < 5; i++) {
    ean64 <<= 8;
    ean64 += ean[4 - i];
  }
  ean64 *= 10;
  ean64 += ean[5];

  // Store the EAN number as a BCD string.
  for (i = 0; i < 8; i++) {
    unsigned char c1, c2;
    c1 = do_div(ean64, 10);
    c2 = do_div(ean64, 10);
    *bcd = c1 + (c2 << 4);
    bcd++;
  }
}


/****************************************************************************/
void packed_EAN_to_BCD_with_csum(unsigned char *ean, unsigned char *bcd)
{
  long long ean64, tmp;
  int i;
  unsigned int csum;

  //
  // "ean" points to a 40 bit number (LSB first) which is the EAN code.
  // The ean code is NOT followed by a check digit.
  // This is the standard format for DRVcans and LAPcan.
  //
  // "bcd" points to a buffer which will contain the EAN number as
  // a BCD string, LSB first. The buffer must be at least 8 bytes.
  //
  // E.g. 733-0130-00122-0 would be (hex)
  // 20 12 00 30 01 33 07 00
  //
  ean64 = 0;
  for (i = 0; i < 5; i++) {
    ean64 <<= 8;
    ean64 += ean[4 - i];
  }

  // Calculate checksum
  tmp = ean64;
  csum = 0;
  for (i = 0; i < 12; i++) {
    unsigned int x;
    
    x = do_div(tmp, 10);
    
    if (i % 2 == 0) {
      csum += 3 * x;
    } else {
      csum += x;
    }
  }
  csum = 10 - csum % 10;
  if (csum == 10) {
    csum = 0;
  }

  ean64 *= 10;
  ean64 += csum;

  // Store the EAN number as a BCD string.
  for (i = 0; i < 8; i++) {
    unsigned char c1, c2;
    c1 = do_div(ean64, 10);
    c2 = do_div(ean64, 10);
    *bcd = c1 + (c2 << 4);
    bcd++;
  }
}
EXPORT_SYMBOL(packed_EAN_to_BCD_with_csum);


/****************************************************************************/
/*
** CRC32 routine. It is recreating its table for each calculation.
** This is by design - it is not intended to be used frequently so we
** perfer to save some memory instead.
**
** The following C code (by Rob Warnock <rpw3@sgi.com>) does CRC-32 in
** BigEndian/BigEndian byte/bit order.  That is, the data is sent most
** significant byte first, and each of the bits within a byte is sent most
** significant bit first, as in FDDI. You will need to twiddle with it to do
** Ethernet CRC, i.e., BigEndian/LittleEndian byte/bit order. [Left as an
** exercise for the reader.]
**
** The CRCs this code generates agree with the vendor-supplied Verilog models
** of several of the popular FDDI "MAC" chips.
*/
#define CRC32_POLY    0x04c11db7L     /* AUTODIN II, Ethernet, & FDDI */

unsigned int calculateCRC32(void *buf, unsigned int bufsiz)
{
  unsigned char *p;
  unsigned int crc;
  unsigned int crc32_table[256];
  int i, j;

  for (i = 0; i < 256; ++i) {
    unsigned int c;
    for (c = i << 24, j = 8; j > 0; --j) {
      c = c & 0x80000000L ? (c << 1) ^ CRC32_POLY : (c << 1);
    }
    crc32_table[i] = c;
  }

  crc = 0xffffffffL;      /* preload shift register, per CRC-32 spec */
  for (p = buf; bufsiz > 0; ++p, --bufsiz) {
    crc = (crc << 8) ^ crc32_table[(crc >> 24) ^ *(unsigned char *)p];
  }
  return ~crc;            /* transmit complement, per CRC-32 spec */
}
EXPORT_SYMBOL(calculateCRC32);
