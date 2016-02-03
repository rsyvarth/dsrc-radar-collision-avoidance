
// The PLL average frequency is adjusted by introduced phase shifts causing
// lengthening of clock cycles.
// The phase shift rate is controlled by the scanclk. In the frequency range
// used for the CAN controller (75/80 MHz) 4 clock cycles are necessary to
// do one shift. The phase shift rate is controlled by inserting delay cycles
// between phase shifts.
// To increase the possible offset a larger phase shift can be done by switching
// between two clock phases.
// The number of phase steps done in a clock switch can be set to a value between
// 0 and 7.

// delay cycles = S
// step_size = Z (0-7)
// f is in MHz (scanclk frequency used for pll reconfiguration)
// min_ps is in ps (The minimum phase shift in picoseconds)
// n_phases is the number of phases in one clock period

// The number of phase shifts per us is calculated as
//             f                 Z
// nshifts = ------ * (1 + ---------------)
//           (S+4)          n_phases - 2Z

//                   f                 Z
// ppm = min_ps * ------- * (1 + ---------------)
//                 (S+4)          n_phases - 2Z

//         min_ps*f     n_phases-Z
// ppm = ----------- * -------------
//           S+4        n_phases-2Z


//         min_ps*f
// ppm = ----------- * P
//           4

// P = 4*ppm/min_ps/f

// Set S=0 to get minimum Z to reach requested timing offset.

//
//  ppm*4       n_phases-Z
// --------  = -------------
// min_ps*f     n_phases-2Z
//
//
//  ppm*4*(n_phases-2Z)
// --------------------- = n_phases-Z
//       min_ps*f
//
//
// ppm*4*n_phases - min_ps*f*n_phases  = (ppm*8 - min_ps*f)*Z
//
//
//                 ppm*4 - min_ps*f
// Z = n_phases * ------------------
//                 ppm*8 - min_ps*f
//
//

// Now calculate S when Z is known
//
//
//      n_phases-Z
// P = -------------
//      n_phases-2Z
//
//
//         min_ps*f
// ppm = ----------- * P
//          S+4
//
//
//       min_ps*f
// S = ----------- * P - 4
//        ppm

#include "inc/pll_regs.h"

#ifdef PCIEFD_DEBUG
#define DEBUGPRINT(n, args...) printk("<" #n ">" args)
#else
#define DEBUGPRINT(n, args...)
#endif

static int isIdle(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned int tmp;

  tmp = IORD_PLL_ENABLE(hCard->pllBase);

  return PCIEFD_PLL_IDLE(tmp);
}

static void enable(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned int tmp;

  tmp = IORD_PLL_ENABLE(hCard->pllBase);
  tmp |= PCIEFD_PLL_ENABLE_MSK;

  IOWR_PLL_ENABLE(hCard->pllBase, tmp);
}

static void disable(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;
  unsigned int tmp;

  tmp = IORD_PLL_ENABLE(hCard->pllBase);
  tmp &= ~PCIEFD_PLL_ENABLE_MSK;

  IOWR_PLL_ENABLE(hCard->pllBase, tmp);
}

static int waitForIdle(VCanCardData *vCard)
{
  int loop_max = 100;

  while(loop_max)
    {
      if(isIdle(vCard)) return 1;

      // Wait
      os_if_set_task_uninterruptible();
      os_if_wait_for_event_timeout_simple(1); // Wait 1 ms

      --loop_max;
    }

  DEBUGPRINT(1, "Error: PLL routine waitForIdle reached maximum retry limit");

  return 0;
}

int setPllOffset(VCanCardData *vCard, int ppm)
{
  int delay,min_steps,max_steps,steps,P,best_delay=0,best_steps=0;
  unsigned int actual,best_diff,best_actual=0;
  int percent;
  int found = 0;

  PciCanCardData *hCard = vCard->hwCardData;

  // Get minimum phase shift
  unsigned int min_ps = IORD_PLL_MIN_PHASE_SHIFT(hCard->pllBase);

  // Get scanclk frequency
  unsigned int fscanclk = IORD_PLL_SCANCLK_FREQUENCY(hCard->pllBase);

  // Get number of phase steps for one clock period
  unsigned int n_phase_steps = 8*IORD_PLL_C0_C1_DIV(hCard->pllBase);

  if( ppm < 1 ) {
    // Disable
    IOWR_PLL_ENABLE(hCard->pllBase, 0);
    return 0;
  }

  if( ppm == 1 ) {
    disable(vCard);
    waitForIdle(vCard);
    IOWR_PLL_DELAY(hCard->pllBase, 10000000);
    IOWR_PLL_STEP_SZ(hCard->pllBase, 0);
    enable(vCard);
    return 0;
  }

  DEBUGPRINT(4,"------------------PLL");
  DEBUGPRINT(4,"min_ps:%d fscanclk:%d n_phase_steps:%d ppm:%d",min_ps,fscanclk,n_phase_steps,ppm);
  DEBUGPRINT(4,"4*ppm:%d min_ps*fscanclk:%d",4*ppm,min_ps*fscanclk);

  if((4*ppm) > (min_ps*fscanclk)) { // Phase shifting is enough */

    min_steps = (100*(n_phase_steps*(4*ppm - min_ps*fscanclk)));

    DEBUGPRINT(4,"steps before div:%d",min_steps);
    {
      int tmp = (8*ppm - min_ps*fscanclk);
      DEBUGPRINT(4,"tmp:%d",tmp);

      min_steps /= tmp;

    }

    DEBUGPRINT(4,"steps before ceil:%d",min_steps);

    min_steps /= 100; // Truncate
    if(min_steps <0) min_steps=0;
  }
  else
    {
      min_steps=0;
    }
  max_steps = (n_phase_steps/2-1);

  DEBUGPRINT(4,"steps after truncate:%d",min_steps);
  DEBUGPRINT(4,"--- Check steps from %u to %u",min_steps,max_steps);

  if( min_steps > max_steps )
    {
      DEBUGPRINT(1,"PLL: Out of range");
      return -1;
    }

  best_diff=-1;

  for(steps=min_steps;steps <= max_steps; steps++)
    {
      int diff;
      int min_delay,max_delay;

      P = 1000 + 1000*steps/(n_phase_steps - 2*steps);

      delay = (500+P*min_ps*fscanclk/ppm)/1000 - 4;

      if( delay < 0) delay=0;
      if( steps < 0) steps=0;

      min_delay=delay-10;
      max_delay=delay+10;

      if( min_delay < 0) min_delay=0;

      for(delay=min_delay;delay<=max_delay;delay++)
        {
          actual = min_ps*fscanclk/(delay+4)*(1000+1000*steps/(n_phase_steps-2*steps))/1000;

          diff= actual - ppm;

          if(diff<0)
            diff=-diff;

          if(diff<best_diff) {
            best_delay=delay;
            best_steps=steps;
            best_diff=diff;
            best_actual=actual;
            found = 1;
          }
        }
    }

  if(!found) {
    return -1;
  }

  DEBUGPRINT(4,"Best abs diff:%u",best_diff);
  delay = best_delay;
  steps = best_steps;
  actual = best_actual;

  percent = actual - ppm;
  percent *= 100;
  percent /= ppm;

  DEBUGPRINT(4,"ppm=%d steps=%d P=%d delay=%d "
             "Actual ppm=%d diff=%d (%d%%) "
             "\n",
             ppm,steps,P,delay,
             actual, actual-ppm, percent
             );


  disable(vCard);

  waitForIdle(vCard);

  IOWR_PLL_DELAY(hCard->pllBase, delay);

  IOWR_PLL_STEP_SZ(hCard->pllBase, steps);

  enable(vCard);

  return actual;
}

int getPllOffset(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;

  // Get minimum phase shift
  unsigned int min_ps = IORD_PLL_MIN_PHASE_SHIFT(hCard->pllBase);

  // Get scanclk frequency
  unsigned int fscanclk = IORD_PLL_SCANCLK_FREQUENCY(hCard->pllBase);

  unsigned int n_phase_steps = 8*IORD_PLL_C0_C1_DIV(hCard->pllBase);

  if( (IORD_PLL_ENABLE(hCard->pllBase) & 1) != 0)
    {
      int delay = IORD_PLL_DELAY(hCard->pllBase);
      int steps = IORD_PLL_STEP_SZ(hCard->pllBase);

      return min_ps*fscanclk/(delay+4)*(1000+1000*steps/(n_phase_steps-2*steps))/1000;
    }
  else
    {
      return 0;
    }
}

int setPllDelay(VCanCardData *vCard, int delay)
{
  PciCanCardData *hCard = vCard->hwCardData;

  if(delay < 0) return -1;

  disable(vCard);
  waitForIdle(vCard);

  IOWR_PLL_DELAY(hCard->pllBase, delay);

  enable(vCard);

  return 0;
}

int setPllSteps(VCanCardData *vCard, int steps)
{
  PciCanCardData *hCard = vCard->hwCardData;

  if(steps < 0) return -1;

  disable(vCard);
  waitForIdle(vCard);

  IOWR_PLL_STEP_SZ(hCard->pllBase, steps);

  enable(vCard);

  return 0;
}

int setPllPhase(VCanCardData *vCard, unsigned int phase)
{
  PciCanCardData *hCard = vCard->hwCardData;

  // Get number of phase steps for one clock period
  unsigned int n_phase_steps = 8*IORD_PLL_C0_C1_DIV(hCard->pllBase);

  if (phase > n_phase_steps)
    phase = n_phase_steps;

  disable(vCard);
  waitForIdle(vCard);

  IOWR_PLL_DELAY(hCard->pllBase, 0);
  IOWR_PLL_STEP_SZ(hCard->pllBase, 0);

  // Set forced phase
  IOWR_PLL_ENABLE(hCard->pllBase, (1<<15) | (phase << 8) );

  enable(vCard);

  return 0;
}

int getPllDelay(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;

  return IORD_PLL_DELAY(hCard->pllBase);
}

int getPllSteps(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;

  return IORD_PLL_STEP_SZ(hCard->pllBase);
}

int getPllPhase(VCanCardData *vCard)
{
  PciCanCardData *hCard = vCard->hwCardData;

  return (IORD_PLL_ENABLE(hCard->pllBase) >> 8) & 0xff;
}
