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

/*
 * Kvaser Linux Canlib
 * Send a CAN message
 */

#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>


void check (char* id, canStatus stat)
{
  char buf[50];

  buf[0] = '\0';
  canGetErrorText(stat, buf, sizeof(buf));
  if (stat != canOK) {
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  } else {
    printf("%s: OK\n", id);
  }
}

/*
 * Send messages until ctrl-c is pressed 
 */
 
int main (int argc, char *argv[])
{
  canHandle h;
  int channel;
  int bitrate = BAUD_1M;

  errno = 0;
  if (argc != 2 || (channel = atoi(argv[1]), errno) != 0) {
    printf("usage %s channel\n", argv[0]);
    exit(1);
  } else {
    printf("Sending a message on channel %d\n", channel);
  }


  /* Allow signals to interrupt syscalls(e.g in canReadBlock) */
  siginterrupt(SIGINT, 1);
  
  /* Open channel, set parameters and go on bus */

  h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED);
  if (h < 0) {
    printf("canOpenChannel %d failed\n", channel);
    return -1;
  }

  canBusOff(h);
  check("canSetBusParams", canSetBusParams(h, bitrate, 4, 3, 1, 1, 0));
  // Work-around for Leaf bug
  check("canSetBusOutputControl", canSetBusOutputControl(h, canDRIVER_NORMAL));
  check("canBusOn", canBusOn(h));
  
  check("canWrite", canWrite(h, 10000, "Kvaser!", 8, 0));       
  check("canWriteSync", canWriteSync(h, 1000));

  
  //check("canBusOff", canBusOff(h));
  check("canClose", canClose(h));

  return 0;
}





