#ifndef TIMER0_CONFIG_H
#define TIMER0_CONFIG_H

#include "TIMER0_interface.h"

/*  8 MHz crystal  →  Fosc/4 = 2 MHz instruction clock
    Prescaler 1:256  →  Ttick = 0.5 µs × 256 = 128 µs

    Target: 2.5-second callback period (LED1 toggle interval)
      Total counts  = 2 500 000 µs / 128 µs = 19531
      Full overflows = 19531 / 256 = 76  (19456 counts)
      Remaining      = 19531 − 19456 = 75 counts
      Preload        = 256 − 75  = 181

    Strategy: 76 full overflows (TMR0 reloaded to 0) then
              1 partial overflow (TMR0 reloaded to 181).
              Total = 76 × 256 + 75 = 19531 counts × 128 µs ≈ 2.500 s */

#define TIMER0_PRESCALER       TIMER0_PRESCALER_256
#define TIMER0_PRELOAD         181u   /* reload value for the final partial overflow */
#define TIMER0_OVF_FULL_COUNT  76u    /* number of full overflows before partial     */

#endif
