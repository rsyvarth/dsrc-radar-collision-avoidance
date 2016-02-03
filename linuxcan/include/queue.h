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

// FIFO
// This abstraction currently only keeps track of indices into a queue,
// the actual data needs to be kept elsewhere.
//
// head points to next place to put new data (back/push)
// tail points to the oldest data (front/pop)
// If they are equal, the queue is empty, so
// only size-1 elements fit.
//
// Without USE_LOCKS defined, the behaviour is
// exactly the same as the old code.

#ifndef QUEUE_H
#define QUEUE_H

#include "osif_functions_kernel.h"
#include "osif_kernel.h"


typedef enum {Normal_lock, Softirq_lock, Irq_lock} Lock_type;
typedef struct {
  int size;
  int head;
  int tail;
  atomic_t length;      // For length queries without locking
  unsigned int flags;   // Only used when holding the lock (Sparc incompatible)!
  OS_IF_WAITQUEUE_HEAD space_event;
  Lock_type lock_type;
  OS_IF_LOCK lock;
  int locked;           // For debugging
  int line;             // For debugging
} Queue;


extern void queue_reinit(Queue *queue);
extern void queue_init(Queue *queue, int size);
extern void queue_irq_lock(Queue *queue);
extern int  queue_length(Queue *queue);
extern int  queue_full(Queue *queue);
extern int  queue_empty(Queue *queue);

// queue_back/front _must_ always be paired with queue_push/pop or _release.
// The first two grab the Queue lock and the last three release it again.
// Make _sure_ not to sleep inbetween and do as little work as possible
// (the interrupts are disabled while holding the lock).
extern int  queue_back(Queue *queue);
extern void queue_push(Queue *queue);
extern int  queue_front(Queue *queue);
extern void queue_pop(Queue *queue);
extern void queue_release(Queue *queue);

extern void queue_add_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_remove_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_add_wait_for_data(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_wakeup_on_space(Queue *queue);
extern void queue_wakeup_on_data(Queue *queue);
extern OS_IF_WAITQUEUE_HEAD *queue_space_event(Queue *queue);

#endif
