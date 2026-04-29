#ifndef ULTRASONIC_PRIVATE_H
#define ULTRASONIC_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/* ---- Timer1 SFRs (used for echo pulse-width measurement) ---- */
#ifndef TMR1L
#define TMR1L   (*(volatile u8*)0x0E)
#endif

#ifndef TMR1H
#define TMR1H   (*(volatile u8*)0x0F)
#endif

#ifndef T1CON
#define T1CON   (*(volatile u8*)0x10)
#endif

/*
 * T1CON setup @ 20 MHz:
 *   TMR1CS = 0  (internal clock, Fosc/4 = 5 MHz)
 *   T1CKPS = 01 (prescaler 1:2)  → tick = 0.4 µs
 *   TMR1ON = 1
 *
 *   Bit layout: [7:6]=0, [5:4]=T1CKPS, [3]=T1OSCEN, [2]=T1SYNC, [1]=TMR1CS, [0]=TMR1ON
 *   Value: 0b00010001 = 0x11
 */
#define T1CON_START     0x11u   /* prescaler 1:2, internal, ON  */
#define T1CON_STOP      0x10u   /* same but OFF                 */

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
