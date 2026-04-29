#ifndef PWM_CONFIG_H
#define PWM_CONFIG_H

/*
 * PWM Configuration for PIC16F877A @ 20 MHz
 *
 * PWM frequency = Fosc / (4 * (PR2 + 1) * prescaler)
 *
 * Target: ~8 kHz PWM with prescaler 1:4
 *   PR2 = 20000000 / (4 * 8000 * 4) - 1 = 155   (~8.013 kHz, fits in u8)
 *
 * Resolution: 10-bit (CCPR1L:DC1B1:DC1B0) — max count = 4*(PR2+1) = 624
 */

/* CPU frequency in Hz (must match system clock) */
#define PWM_FOSC            20000000UL

/* Timer2 prescaler selection (see PWM_Private.h: T2CKPS_1, T2CKPS_4, T2CKPS_16) */
#define PWM_T2_PRESCALER    T2CKPS_4

/* Numeric prescaler divisor matching the selection above */
#define PWM_T2_PRESCALER_VAL   4UL

/* Default PWM frequency in Hz */
#define PWM_DEFAULT_FREQ    8000UL

/*
 * PR2 = (Fosc / (4 * freq * prescaler)) - 1
 * For 8 kHz at 20 MHz with prescaler 4 → PR2 = 155
 */
#define PWM_PR2_VALUE \
    ((u8)((PWM_FOSC / (4UL * PWM_DEFAULT_FREQ * PWM_T2_PRESCALER_VAL)) - 1UL))

#endif