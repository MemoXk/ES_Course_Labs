#ifndef TIMER_1_CONFIG_H
#define TIMER_1_CONFIG_H

/*
 * Timer1 Configuration for PIC16F877A @ 20 MHz
 *
 * Internal Timer1 clock = Fosc/4 = 5 MHz.
 * Prescaler 1:2 gives a 0.4 us tick, which the ultrasonic HAL uses
 * for pulse-width distance measurement.
 */

#define TIMER1_PRESCALE_1_1  0x00u
#define TIMER1_PRESCALE_1_2  0x10u
#define TIMER1_PRESCALE_1_4  0x20u
#define TIMER1_PRESCALE_1_8  0x30u

#define TIMER1_PRESCALER     TIMER1_PRESCALE_1_2

#endif
