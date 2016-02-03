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

//
// The following routines for setting the pwm values gives
// a maximum frequency error of about 15384 Hz and a duty cycle
// error of maximum 1%.
// The frequency range is from 250 KHz to 1 MHz, the duty cycle
// is limited to 95%. The duty cycle limit is also enforced during
// frequency and duty cycle changes.
//
//
#include "HAL/inc/pwm_util.h"
#include "inc/pciefd_regs.h"

static int sysClkFreq = 62.5e6;

void pwmInit(int frequency)
{
  sysClkFreq = frequency;
}

// +----------------------------------------------------------------------------
// | Calculates a top value from the frequency
// |
// +----------------------------------------------------------------------------
static int pwmTop(int frequency)
{
  int top;

  top = sysClkFreq / (2*frequency) - 1;

  if (top < 1) {
    return 1;
  }

  if (top > 255) {
    return 255;
  }

  return top;
}

// +----------------------------------------------------------------------------
// | Calculates pwm frequency from the value of top
// |
// +----------------------------------------------------------------------------
static int pwmFrequency(int top)
{
  return sysClkFreq/(2*(top+1));
}

// +----------------------------------------------------------------------------
// | Calculate trigger value from top and duty cycle
// |
// +----------------------------------------------------------------------------
static int pwmTrigger(int top, int duty_cycle)
{
  int trigger;

  trigger = (100*top - duty_cycle*(top+1) + 50)/100; // +50 for rounding instead of truncating

  // Can't really reach 100% because of at least one clock cycle dead band
  if(trigger < 0)
    trigger = 0;

  return trigger;
}

// +----------------------------------------------------------------------------
// | Calculate duty cycle from top and trigger values
// |
// +----------------------------------------------------------------------------
static int pwmDutyCycle(int top, int trigger)
{
  if ( trigger > top ) return 0;

  return (5 + 1000*(top-trigger)/(top+1))/10; //5+ for rounding
}

// +----------------------------------------------------------------------------
// | Get current duty cycle
// |
// +----------------------------------------------------------------------------
int pwmGetDutyCycle(u32 *address)
{
  u32 pwm_ctrl = IORD_PCIEFD_PWM_CTRL(address);
  int top = PCIEFD_PWM_CTRL_TOP_GET(pwm_ctrl);
  int trigger = PCIEFD_PWM_CTRL_TRIGGER_GET(pwm_ctrl);

  return pwmDutyCycle(top,trigger);
}

int pwmSetDutyCycle(u32 *address, int duty_cycle)
{
  u32 pwm_ctrl = IORD_PCIEFD_PWM_CTRL(address);
  int top = PCIEFD_PWM_CTRL_TOP_GET(pwm_ctrl);
  int trigger;

  if ( duty_cycle < 0 ) {
    duty_cycle = 0;
  } else if ( duty_cycle > 100 ) {
    duty_cycle = 100;
  }

  trigger = pwmTrigger(top, duty_cycle);

  pwm_ctrl = PCIEFD_PWM_CTRL_TRIGGER(trigger);
  pwm_ctrl |= PCIEFD_PWM_CTRL_TOP(top);
  IOWR_PCIEFD_PWM_CTRL(address, pwm_ctrl);

  return pwmDutyCycle(top, duty_cycle);
}

// +----------------------------------------------------------------------------
// | Get current frequency
// |
// +----------------------------------------------------------------------------
int pwmGetFrequency(u32 *address)
{
  u32 pwm_ctrl = IORD_PCIEFD_PWM_CTRL(address);
  int top = PCIEFD_PWM_CTRL_TOP_GET(pwm_ctrl);

  return pwmFrequency(top);
}

int pwmSetFrequency(u32 *address, int frequency)
{
  u32 pwm_ctrl;
  int top;
  int duty_cycle = pwmGetDutyCycle(address); // Save current duty cycle

  if (frequency < F_PWM_MIN) {
    frequency = F_PWM_MIN;
  } else if (frequency > F_PWM_MAX) {
    frequency = F_PWM_MAX;
  }

  top = pwmTop(frequency);

  // Set duty cycle to zero during frequency shift
  pwmSetDutyCycle(address,0);

  pwm_ctrl = PCIEFD_PWM_CTRL_TRIGGER(top);
  pwm_ctrl |= PCIEFD_PWM_CTRL_TOP(top);
  IOWR_PCIEFD_PWM_CTRL(address, pwm_ctrl);

  // NOTE: Possibly set top and new trigger in the order giving least overshoot.
  // If top increases, change top first
  // If top decreases, change trigger first

  pwmSetDutyCycle(address, duty_cycle); // Restore duty cycle

  return pwmFrequency(top);
}

