#ifndef TIMER_0_CONFIG_H
#define TIMER_0_CONFIG_H

/*
 * Timer0 Configuration for PIC16F877A @ 20 MHz
 *
 * Timer0 clock = Fosc/4 = 5 MHz
 * With prescaler 1:256: tick period = 51.2 us
 * Overflow at TMR0=0xFF -> period = 256 * 51.2 us = ~13.1 ms
 *
 * To get ~10 ms overflow: counts = 10000 / 51.2 ~= 195
 *   preload = 256 - 195 = 61
 * Actual period = 195 * 51.2 us = 9984 us ~= 9.98 ms
 */

/* Prescaler select (see TIMER_0_Private.h for values) */
#define TIMER0_PRESCALER    TIMER0_PS_1_256

/* Timer0 preload value (0–255): TMR0 reloaded in ISR each overflow */
#define TIMER0_PRELOAD      61

/* Clock source: use internal oscillator (timer mode) */
#define TIMER0_CLK_INTERNAL 0

#endif
