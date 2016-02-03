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

#ifndef OSIF_FUNCTIONS_KERNEL_H_
#define OSIF_FUNCTIONS_KERNEL_H_

#include "osif_kernel.h"


//////////////////////////////////////////////////////////////////////
//
#define os_if_wait_event_interruptible_timeout(wq, condition, timeout)  \
  wait_event_interruptible_timeout(wq, condition, msecs_to_jiffies(timeout) + 1)

//////////////////////////////////////////////////////////////////////
//
void os_if_write_port(unsigned regist, unsigned portAddr);

//////////////////////////////////////////////////////////////////////
//
unsigned int os_if_read_port(unsigned portAddr);

//////////////////////////////////////////////////////////////////////
//
int os_if_queue_task(OS_IF_TASK_QUEUE_HANDLE *hnd);

//////////////////////////////////////////////////////////////////////
//
int os_if_queue_task_not_default_queue(OS_IF_WQUEUE *wq,
                                       OS_IF_TASK_QUEUE_HANDLE *hnd);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_named_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle, char *name);

//////////////////////////////////////////////////////////////////////
//
void os_if_delete_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_waitqueue_entry(OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
void os_if_add_wait_queue(OS_IF_WAITQUEUE_HEAD *waitQ,
                          OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
void os_if_remove_wait_queue(OS_IF_WAITQUEUE_HEAD *waitQ,
                             OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
signed long os_if_wait_for_event_timeout(signed long timeout,
                                         OS_IF_WAITQUEUE *handle);

//////////////////////////////////////////////////////////////////////
//
signed long os_if_wait_for_event_timeout_simple(signed long timeout);

//////////////////////////////////////////////////////////////////////
//
void os_if_wait_for_event(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
//long os_if_wait_event_interruptible_timeout(OS_IF_WAITQUEUE_HEAD handle,
//                                            unsigned long *cond, long time);

//////////////////////////////////////////////////////////////////////
//
void os_if_wake_up_interruptible(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//

//////////////////////////////////////////////////////////////////////
//


//////////////////////////////////////////////////////////////////////
//
void os_if_up_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
void os_if_down_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
int  os_if_down_sema_time(OS_IF_SEMAPHORE *var, int timeout);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
void os_if_delete_sema(OS_IF_SEMAPHORE *var);


//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_interruptible(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_uninterruptible(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_running(void);

//////////////////////////////////////////////////////////////////////
//
unsigned long os_if_get_timeout_time(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_do_get_time_of_day(OS_IF_TIME_VAL *tv);

//////////////////////////////////////////////////////////////////////
//
int os_if_is_rec_busy(int nr, volatile unsigned long *addr);

//////////////////////////////////////////////////////////////////////
//
void os_if_rec_not_busy(int nr, volatile unsigned long *addr);

//////////////////////////////////////////////////////////////////////
//
# define os_if_spin_lock_init(lock) spin_lock_init(lock)

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_remove(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_disable(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_enable(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_save(OS_IF_LOCK *lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_restore(OS_IF_LOCK *lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_irqsave(OS_IF_LOCK *lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock_irqrestore(OS_IF_LOCK *lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_softirq(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock_softirq(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_user_data(void *to, const void *from, OS_IF_SIZE n);

//////////////////////////////////////////////////////////////////////
//
int os_if_set_user_data(void *to, const void *from, OS_IF_SIZE n);

//////////////////////////////////////////////////////////////////////
//
int os_if_set_int(int val, int *dest);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_int(int *val, int *src);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_long(long *val, long *src);

//////////////////////////////////////////////////////////////////////
//
OS_IF_WQUEUE* os_if_declare_task(char *name, OS_IF_TASK_QUEUE_HANDLE *taskQ);

//////////////////////////////////////////////////////////////////////
//
OS_IF_WQUEUE* os_if_declare_rt_task(char *name, OS_IF_TASK_QUEUE_HANDLE *taskQ);

//////////////////////////////////////////////////////////////////////
//
void os_if_destroy_task(OS_IF_WQUEUE *wQueue);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_task(OS_IF_TASK_QUEUE_HANDLE *taskQ, void *function, void *data);

//////////////////////////////////////////////////////////////////////
//
void os_if_rwlock_init(rwlock_t *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_read_lock_irqsave(rwlock_t *rw_lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_read_unlock_irqrestore(rwlock_t *rw_lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_write_lock_irqsave(rwlock_t *rw_lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_write_unlock_irqrestore(rwlock_t *rw_lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_rwlock_remove(rwlock_t *lock);

//////////////////////////////////////////////////////////////////////
//
int os_if_signal_pending(void);

//////////////////////////////////////////////////////////////////////
//
void* os_if_kernel_malloc(size_t buffer_size);

//////////////////////////////////////////////////////////////////////
//
void os_if_kernel_free(void *mem_ptr);

//////////////////////////////////////////////////////////////////////
//
OS_IF_THREAD os_if_kernel_thread(int (*thread)(void *context), void *context);

//////////////////////////////////////////////////////////////////////
//
void os_if_exit_thread(void *module, int result);
#ifndef THIS_MODULE
#define THIS_MODULE 0
#endif


  typedef unsigned long AtomicBit;
  typedef AtomicBit OS_IF_ATOMIC_BIT;
  
  void os_if_init_atomic_bit(OS_IF_ATOMIC_BIT *ab);
  void os_if_remove_atomic_bit(OS_IF_ATOMIC_BIT *ab);

#endif //OSIF_FUNCTIONS_KERNEL_H_
