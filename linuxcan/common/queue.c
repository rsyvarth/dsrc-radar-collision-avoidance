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

//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------

#include "osif_functions_kernel.h"
#include "osif_kernel.h"
#include "VCanOsIf.h"
#include "queue.h"


// Without this defined, all length queries will use locking
#define ATOMIC_LENGTH

// Lock type to use for queue
#define LOCK_TYPE Softirq_lock


// Some lock debug macros

#define PRINT(t)
//#define PRINT(t) DEBUGOUT(1,t)
//#define PRINTX(t)
#define PRINTX(t) DEBUGOUT(1,t)

#ifdef QUEUE_DEBUG
 #include "debug.h"
 #define QUEUE_DEBUG_RET(ret)                          \
   if (!queue->size) {                                 \
     PRINT((TXT("Using unitialized queue\n")));        \
     return ret;                                       \
   }

 #define QUEUE_DEBUG_LOCK                              \
   if (queue->locked) {                                \
     PRINTX((TXT("Queue already locked (%d/%d)\n"),    \
             __LINE__, queue->line));                  \
     return;                                           \
   } else {                                            \
     queue->line = __LINE__;                           \
   }
 #define QUEUE_DEBUG_LOCK_RET(ret)                     \
   if (queue->locked) {                                \
     PRINTX((TXT("Queue already locked (%d/%d)\n"),    \
             __LINE__, queue->line));                  \
     return ret;                                       \
   } else {                                            \
     queue->line = __LINE__;                           \
   }

 #define QUEUE_DEBUG_UNLOCK                            \
   if (!queue->locked) {                               \
     PRINTX((TXT("Queue already unlocked (%d)\n"),     \
             __LINE__));                               \
     return;                                           \
   }
 #define QUEUE_DEBUG_UNLOCK_RET(ret)                   \
   if (!queue->locked) {                               \
     PRINTX((TXT("Queue already unlocked (%d)\n"),     \
             __LINE__));                               \
     return ret;                                       \
   }
#else
 #define QUEUE_DEBUG
 #define QUEUE_DEBUG_RET(ret)
 #define QUEUE_DEBUG_LOCK
 #define QUEUE_DEBUG_LOCK_RET(ret)
 #define QUEUE_DEBUG_UNLOCK
 #define QUEUE_DEBUG_UNLOCK_RET(ret)
#endif


#define LOCKQ(queue, flags_ptr)                         \
  PRINT((TXT("Locking (%d) at %d\n"),                   \
         queue->lock_type, __LINE__));                  \
  switch (queue->lock_type) {                           \
  case Irq_lock:                                        \
    os_if_spin_lock_irqsave(&queue->lock, flags_ptr);   \
    break;                                              \
  case Softirq_lock:                                    \
    os_if_spin_lock_softirq(&queue->lock);              \
    break;                                              \
  default:                                              \
    os_if_spin_lock(&queue->lock);                      \
    break;                                              \
  }                                                     \
  queue->locked = 1;   /* After acquiring */

#define UNLOCKQ(queue, flags)                           \
  PRINT((TXT("Unlocking (%d) at %d\n"),                 \
         queue->lock_type, __LINE__));                  \
  queue->locked = 0;   /* Before releasing */           \
  switch (queue->lock_type) {                           \
  case Irq_lock:                                        \
    os_if_spin_unlock_irqrestore(&queue->lock, flags);  \
    break;                                              \
  case Softirq_lock:                                    \
    os_if_spin_unlock_softirq(&queue->lock);            \
    break;                                              \
  default:                                              \
    os_if_spin_lock(&queue->lock);                      \
    break;                                              \
  }


void queue_reinit (Queue *queue)
{
  unsigned long flags;

  QUEUE_DEBUG;
  QUEUE_DEBUG_LOCK;
  LOCKQ(queue, &flags);

  queue->head = 0;
  queue->tail = 0;

  atomic_set(&queue->length, 0);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, flags);
}
EXPORT_SYMBOL(queue_reinit);


void queue_init (Queue *queue, int size)
{
  queue->lock_type = LOCK_TYPE;
  os_if_spin_lock_init(&queue->lock);
  queue->size = size;
  os_if_init_waitqueue_head(&queue->space_event);
  queue->locked = 0;
  queue_reinit(queue);
}
EXPORT_SYMBOL(queue_init);


void queue_irq_lock (Queue *queue)
{
  queue->lock_type = Irq_lock;
}
EXPORT_SYMBOL(queue_irq_lock);


int queue_length (Queue *queue)
{
  int length;
#ifndef ATOMIC_LENGTH
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  length = queue->head - queue->tail;
  if (length < 0)
    length += queue->size;

  QUEUE_DEBUG_UNLOCK_RET(0);
  UNLOCKQ(queue, flags);
#else
  length = atomic_read(&queue->length);
#endif

  return length;
}
EXPORT_SYMBOL(queue_length);


int queue_full (Queue *queue)
{
  QUEUE_DEBUG_RET(0);

  return queue_length(queue) >= queue->size - 1;
}


int queue_empty (Queue *queue)
{
  QUEUE_DEBUG_RET(1);

  return queue_length(queue) == 0;
}
EXPORT_SYMBOL(queue_empty);


// Lock will be held when this returns.
// Must be released with a call to queue_push/release()
// as soon as possible. Make _sure_ not to sleep inbetween!
// (Storing irq flags like this is supposedly incompatible
//  with Sparc CPU:s. But that only applies for Irq_lock queues.)
int queue_back (Queue *queue)
{
  int back;
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  back = queue->head;
  // Is there actually any space in the queue?
#ifndef ATOMIC_LENGTH
  // (Holding lock, so can't use queue_full.)
  {
    int length = back - queue->tail;
    if (length < 0)
      length += queue->size;
    if (length >= queue->size - 1) {
      back = -1;
    }
  }
#else
  if (queue_full(queue)) {
    back = -1;
  }
#endif

  queue->flags = flags;

  return back;
}
EXPORT_SYMBOL(queue_back);


// Lock must be held from a previous queue_back().
void queue_push (Queue *queue)
{
  QUEUE_DEBUG;

  queue->head++;
  if (queue->head >= queue->size)
    queue->head = 0;

  atomic_inc(&queue->length);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}
EXPORT_SYMBOL(queue_push);


// Lock will be held when this returns.
// Must be released with a call to queue_pop/release()
// as soon as possible. Make _sure_ not to sleep inbetween!
int queue_front (Queue *queue)
{
  int front;
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  front = queue->tail;
  // Is there actually anything in the queue?
#ifndef ATOMIC_LENGTH
  // (Holding lock, so can't use queue_empty.)
  if (queue->head == front) {
    front = -1;
  }
#else
  if (queue_empty(queue)) {
    front = -1;
  }
#endif

  queue->flags = flags;

  return front;
}
EXPORT_SYMBOL(queue_front);


// Lock must be held from a previous queue_front().
void queue_pop (Queue *queue)
{
  QUEUE_DEBUG;

  queue->tail++;
  if (queue->tail >= queue->size)
    queue->tail = 0;  

  atomic_dec(&queue->length);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}
EXPORT_SYMBOL(queue_pop);


// Lock must be held from a previous queue_front/back().
void queue_release (Queue *queue)
{
  QUEUE_DEBUG;

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}
EXPORT_SYMBOL(queue_release);


void queue_add_wait_for_space (Queue *queue, OS_IF_WAITQUEUE *waiter)
{
  QUEUE_DEBUG;

  os_if_add_wait_queue(&queue->space_event, waiter);
}
EXPORT_SYMBOL(queue_add_wait_for_space);


void queue_remove_wait_for_space (Queue *queue, OS_IF_WAITQUEUE *waiter)
{
  QUEUE_DEBUG;

  os_if_remove_wait_queue(&queue->space_event, waiter);
}
EXPORT_SYMBOL(queue_remove_wait_for_space);




void queue_wakeup_on_space (Queue *queue)
{
  QUEUE_DEBUG;

  os_if_wake_up_interruptible(&queue->space_event);
}
EXPORT_SYMBOL(queue_wakeup_on_space);


OS_IF_WAITQUEUE_HEAD *queue_space_event (Queue *queue)
{
  QUEUE_DEBUG_RET(0);

  return &queue->space_event;
}
