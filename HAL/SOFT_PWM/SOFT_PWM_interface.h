#ifndef SOFT_PWM_INTERFACE_H
#define SOFT_PWM_INTERFACE_H

#include "SOFT_PWM_private.h"
#include "SOFT_PWM_config.h"
#include "../../SERVICES/STD_TYPES.h"

/* Configure output pin and set it LOW */
void SOFT_PWM_Init(void);

/*
 * Generate one complete PWM period (blocking).
 * HIGH for SOFT_PWM_HIGH_US, then LOW for SOFT_PWM_LOW_US.
 * Call repeatedly in the main loop to sustain PWM output.
 */
void SOFT_PWM_Tick(void);

/* Drive output LOW immediately (disable PWM output) */
void SOFT_PWM_Stop(void);

#endif
