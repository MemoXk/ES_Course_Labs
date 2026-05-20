#ifndef ULTRASONIC_PRIVATE_H
#define ULTRASONIC_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/*
 * Timer1 setup @ 20 MHz is owned by MCAL/TIMER_1:
 * internal clock Fosc/4 = 5 MHz, prescaler 1:2 -> tick = 0.4 us.
 */

/*
 * Distance formula:
 * sound round trip for 1 cm is about 58 us, so 58 / 0.4 ~= 145 ticks/cm.
 */
#define ULTRASONIC_TICKS_PER_CM     145U

/* Timer1 timeout: 60000 ticks * 0.4 us = 24 ms. */
#define ULTRASONIC_TIMEOUT_TICKS    60000U

/* Reject pulse widths below about 1 cm as glitches. */
#define ULTRASONIC_MIN_WIDTH_TICKS  145U

/* Return value when no valid echo is received. */
#define ULTRASONIC_NO_OBJ           999U

#endif
