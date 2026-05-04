#ifndef ULTRASONIC_PRIVATE_H
#define ULTRASONIC_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/*
 * Timer1 setup @ 20 MHz is owned by MCAL/TIMER_1:
 *   internal clock Fosc/4 = 5 MHz, prescaler 1:2 -> tick = 0.4 us.
 */

/*
 * Distance formula (prescaler 1:2 @ 20 MHz):
 *   1 tick = 0.4 µs
 *   sound round-trip 1 cm = 58.3 µs → 58.3 / 0.4 ≈ 145 ticks/cm
 */
#define ULTRASONIC_TICKS_PER_CM     145U

/* Stop measuring when Timer1 high byte exceeds this (≈ 400 cm) */
#define ULTRASONIC_OVERFLOW_H       0xE3u

/* Return value when no echo is received */
#define ULTRASONIC_NO_OBJ           999U

/* Timeout counter waiting for echo to go HIGH */
#define ULTRASONIC_ECHO_WAIT_MAX    30000U

#endif
