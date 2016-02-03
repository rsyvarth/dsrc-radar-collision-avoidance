#ifndef __PLL_CONTROL_H__
#define __PLL_CONTROL_H__

int setPllOffset(VCanCardData *vCard, int ppm);
int getPllOffset(VCanCardData *vCard);

int setPllDelay(VCanCardData *vCard, int delay);
int setPllSteps(VCanCardData *vCard, int steps);
int setPllPhase(VCanCardData *vCard, unsigned int phase);

int getPllDelay(VCanCardData *vCard);
int getPllSteps(VCanCardData *vCard);
int getPllPhase(VCanCardData *vCard);

#endif
