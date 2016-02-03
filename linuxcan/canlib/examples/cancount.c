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
 * Count the number of messages/second on a CAN channel
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

int i = 0;
int std = 0, ext = 0, rtr = 0, err = 0, over = 0;
unsigned char willExit = 0;

void sighand (int sig)
{
  static int last;

  switch (sig) {
  case SIGINT:
    willExit = 1;
    alarm(0);
    break;
  case SIGALRM:
    if (i-last) {
      printf("msg/s = %d, total=%d, std=%d, ext=%d, err=%d, over=%d\n", 
             i-last, i, std, ext, err, over);
    }
    last = i;
    alarm(1);
    break;
  }
}

 
int main (int argc, char *argv[])
{
  canHandle h;
  int ret = -1;
  long id = 27; 
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;  
  int channel = 0;
  int bitrate = BAUD_1M;

  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);
  alarm(1);

  /* Allow signals to interrupt syscalls(in canReadBlock) */
  siginterrupt(SIGINT, 1);

  errno = 0;
  if (argc != 2 || (channel = atoi(argv[1]), errno) != 0) {
    printf("usage %s channel\n", argv[0]);
    exit(1);
  } else {
    printf("Counting messages on channel %d\n", channel);
  }

  
  /* Open channels, parameters and go on bus */
  h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED);
  if (h < 0) {
    printf("canOpenChannel %d failed\n", channel);
    return -1;
  }
  check("parameters", canSetBusParams(h, bitrate, 4, 3, 1, 1, 0));
  canBusOn(h);

  while (!willExit) {
    do { 
      ret = canReadWait(h, &id, &msg, &dlc, &flag, &time, -1);
      switch (ret){
      case 0:
        if (flag & canMSG_ERROR_FRAME) {
          err++;
        } else {
          if (flag & canMSG_STD)        std++;
          if (flag & canMSG_EXT)        ext++;
          if (flag & canMSG_RTR)        rtr++;
          if (flag & canMSGERR_OVERRUN) over++;
        }
        i++;
        break;
      case canERR_NOMSG:
        break;
      default:
        perror("canReadBlock error");
        break;
      }
    } while (ret == canOK);
    willExit = 1;
  }
   
  sighand(SIGALRM);
  printf("Ready\n");

  return 0;
}





