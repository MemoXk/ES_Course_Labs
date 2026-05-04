#ifndef TIMER_1_INTERFACE_H
#define TIMER_1_INTERFACE_H

#include "../../SERVICES/STD_TYPES.h"
#include "../../SERVICES/BIT_MATH.h"
#include "TIMER_1_Private.h"
#include "TIMER_1_Config.h"

/* Configure Timer1 for internal clock and the configured prescaler. */
void TIMER1_Init(void);

/* Clear the 16-bit Timer1 counter. */
void TIMER1_Reset(void);

/* Start/stop Timer1 without changing the configured clock mode. */
void TIMER1_Start(void);
void TIMER1_Stop(void);

/* Read the current 16-bit Timer1 count. */
u16 TIMER1_GetValue(void);

/* Fast high-byte read for overflow/timeout guards. */
u8 TIMER1_GetHighByte(void);

#endif
