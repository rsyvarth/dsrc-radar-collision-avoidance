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

#ifndef DEBUGPRN_H
#    define DEBUGPRN_H
/***************************************************************************
* COPYRIGHT:    KVASER AB
* DESIGNED BY:  Lasse Kinnunen
* DESCRIPTION:  Debug print macros.
***************************************************************************/


/***************************************************************************
*   INCLUDES
***************************************************************************/
#ifndef NDEBUG
#    include <stdio.h>
#    if (defined( __BORLANDC__) || defined(_MSC_VER)) && defined( TIMESTAMPED_DEBUGPRINT)
#        include <time.h>
#    endif
#endif


/***************************************************************************
*   DEFINES
***************************************************************************/
#ifndef NDEBUG
#    if (defined( __BORLANDC__) || defined(_MSC_VER)) && defined( TIMESTAMPED_DEBUGPRINT)
#        define POSPRINTF( args)                    \
          {                                         \
              struct tm *time_now;                  \
              time_t secs_now;                      \
              char timestr[80];                     \
              (void) time( &secs_now);              \
              time_now = localtime( &secs_now);     \
              (void) strftime( timestr, 80, "%y-%m-%d %H:%M.%S",    \
                      time_now);                    \
              (void) printf( "%s %d, %s: ",         \
                      __FILE__, __LINE__, timestr); \
          }                             \
          (void) printf args

#        define POSPUTS( str)                       \
          {                                         \
              struct tm *time_now;                  \
              time_t secs_now;                      \
              char timestr[ 80];                    \
              (void) time( &secs_now);              \
              time_now = localtime( &secs_now);     \
              (void) strftime( timestr, 80, "%y-%m-%d %H:%M.%S",    \
                      time_now);                    \
              (void) printf( "%s %d, %s: %s\n",             \
                    __FILE__, __LINE__, timestr, str);      \
          }

#    else

#        define POSPRINTF( args)    (void) printf( "%s %d: ",   \
                                                   __FILE__,    \
                                                   __LINE__);   \
                                    (void) printf args

#        define POSPUTS( str)   (void) printf( "%s %d: %s\n",   \
                                                   __FILE__,    \
                                                   __LINE__,    \
                                                str)

#    endif

#    define PRINTF( args)   (void) printf args
#    define PUTS( str)      (void) puts( str)
#    define PUTCHAR( ch)    (void) putchar( ch)

#    define TRACEBOX(x)     traceBox x
#    define BEEP(freq, duration) Beep((freq), (duration))
#    define MESSAGEBEEP     MessageBeep(0xffff)
extern void traceBox(char *s, ...);

#else

#    define POSPRINTF( args)
#    define POSPUTS( str)
#    define PRINTF( args)
#    define PUTS( str)
#    define PUTCHAR( ch)

#    define TRACEBOX(x)
#    define BEEP(x, y)
#    define MESSAGEBEEP

#endif


/***************************************************************************
*   CONSTANTS
***************************************************************************/
// extern const char * stdErrStr;


#endif  /* #ifndef DEBUGPRN_H */
