/******************************************************************************
 *                                                                             *
 * License Agreement                                                           *
 *                                                                             *
 * Copyright (c) 2008 Altera Corporation, San Jose, California, USA.           *
 * All rights reserved.                                                        *
 *                                                                             *
 * Permission is hereby granted, free of charge, to any person obtaining a     *
 * copy of this software and associated documentation files (the "Software"),  *
 * to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
 * and/or sell copies of the Software, and to permit persons to whom the       *
 * Software is furnished to do so, subject to the following conditions:        *
 *                                                                             *
 * The above copyright notice and this permission notice shall be included in  *
 * all copies or substantial portions of the Software.                         *
 *                                                                             *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
 * DEALINGS IN THE SOFTWARE.                                                   *
 *                                                                             *
 * This agreement shall be governed in all respects by the laws of the State   *
 * of California and by the laws of the United States of America.              *
 *                                                                             *
 ******************************************************************************/

#include "HAL/inc/epcs_commands.h"
#include "HAL/inc/altera_avalon_spi.h"

uint8_t epcs_read_status_register(void *base)
{
  const uint8_t rdsr = epcs_rdsr;
  uint8_t status;
  alt_avalon_spi_command(
                         base,
                         0,
                         1,
                         &rdsr,
                         1,
                         &status,
                         0
                         );

  return status;
}

static int epcs_test_wip(void *base)
{
  return epcs_read_status_register(base) & 1;
}

static void epcs_await_wip_released(void *base)
{
  /* Wait until the WIP bit goes low. */
  while (epcs_test_wip(base))
    {
    }
}

void epcs_sector_erase(void *base, uint32_t offset)
{
  uint8_t se[4];

  se[0] = epcs_se;
  se[1] = (offset >> 16) & 0xFF;
  se[2] = (offset >> 8) & 0xFF;
  se[3] = offset & 0xFF;

  alt_avalon_spi_command(
                         base,
                         0,
                         sizeof(se) / sizeof(*se),
                         se,
                         0,
                         (uint8_t*)0,
                         0
                         );

  epcs_await_wip_released(base);
}

int32_t epcs_read_buffer(void *base, int offset, uint8_t *dest_addr, int length)
{
  uint8_t read_command[4];

  read_command[0] = epcs_read;
  read_command[1] = (offset >> 16) & 0xFF;
  read_command[2] = (offset >> 8) & 0xFF;
  read_command[3] = offset & 0xFF;

  epcs_await_wip_released(base);

  alt_avalon_spi_command(
                         base,
                         0,
                         sizeof(read_command) / sizeof(*read_command),
                         read_command,
                         length,
                         (uint8_t*)dest_addr,
                         0
                         );

  return length;
}

void epcs_write_enable(void *base)
{
  const uint8_t wren = epcs_wren;
  alt_avalon_spi_command(
                         base,
                         0,
                         1,
                         &wren,
                         0,
                         (uint8_t*)0,
                         0
                         );
}

void epcs_write_status_register(void *base, uint8_t value)
{
  uint8_t wrsr[2];

  wrsr[0] = epcs_wrsr;
  wrsr[1] = value;

  alt_avalon_spi_command(
                         base,
                         0,
                         2,
                         wrsr,
                         0,
                         (uint8_t*)0,
                         0
                         );

  epcs_await_wip_released(base);
}

/* Write a partial or full page, assuming that page has been erased */
int32_t epcs_write_buffer(void *base, int offset, const uint8_t* src_addr, int length)
{
  uint8_t pp[4];

  pp[0] = epcs_pp;
  pp[1] = (offset >> 16) & 0xFF;
  pp[2] = (offset >> 8) & 0xFF;
  pp[3] = offset & 0xFF;

  /* First, WREN */
  epcs_write_enable(base);

  /* Send the PP command */
  alt_avalon_spi_command(
                         base,
                         0,
                         sizeof(pp) / sizeof(*pp),
                         pp,
                         0,
                         (uint8_t*)0,
                         ALT_AVALON_SPI_COMMAND_MERGE
                         );

  /* Send the user's buffer */
  alt_avalon_spi_command(
                         base,
                         0,
                         length,
                         src_addr,
                         0,
                         (uint8_t*)0,
                         0
                         );

  /* Wait until the write is done.  This could be optimized -
   * if the user's going to go off and ignore the flash for
   * a while, its writes could occur in parallel with user code
   * execution.  Unfortunately, I have to guard all reads/writes
   * with wip-tests, to make that happen.
   */
  epcs_await_wip_released(base);

  return length;
}


uint8_t epcs_read_electronic_signature(void *base)
{
  const uint8_t res_cmd[] = {epcs_res, 0, 0, 0};
  uint8_t res;

  alt_avalon_spi_command(
                         base,
                         0,
                         sizeof(res_cmd) / sizeof(*res_cmd),
                         res_cmd,
                         1,
                         &res,
                         0
                         );

  return res;
}

uint8_t epcs_read_device_id(void *base)
{
  const uint8_t rd_id_cmd[] = {epcs_rdid, 0, 0};
  uint8_t res;

  alt_avalon_spi_command(
                         base,
                         0,
                         sizeof(rd_id_cmd) / sizeof(*rd_id_cmd),
                         rd_id_cmd,
                         1,
                         &res,
                         0
                         );

  return res;
}
