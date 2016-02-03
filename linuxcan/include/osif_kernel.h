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

#ifndef OSIF_KERNEL_H_
#define OSIF_KERNEL_H_

//###############################################################

#   include <linux/types.h>
#   include <linux/tty.h>
#   include <linux/module.h>
#   include <linux/spinlock.h>
#   include <linux/workqueue.h>

// DEFINES
#   define OS_IF_TIMEOUT 0
#   define OS_IF_TICK_COUNT jiffies
#   define OS_IF_MOD_INC_USE_COUNT ;
#   define OS_IF_MOD_DEC_USE_COUNT ;

    // TYPEDEFS
    typedef struct timeval        OS_IF_TIME_VAL;
    typedef off_t                 OS_IF_OFFSET;
    typedef unsigned long         OS_IF_SIZE;
    typedef struct task_struct*   OS_IF_THREAD;
    typedef wait_queue_head_t     OS_IF_WAITQUEUE_HEAD;
    typedef wait_queue_t          OS_IF_WAITQUEUE;
    typedef spinlock_t            OS_IF_LOCK;

    typedef struct work_struct      OS_IF_TASK_QUEUE_HANDLE;
    typedef struct workqueue_struct OS_IF_WQUEUE;
    typedef struct completion       OS_IF_SEMAPHORE;



#define INIT           // __init
#define EXIT           // __exit
#define DEVINIT        // __devinit
#define DEVEXIT        // __devexit
#define DEVINITDATA    // __devinitdata
#define DEVEXITP(x) x  // __devexit_p(x)

#endif //OSIF_KERNEL_H_
