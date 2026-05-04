#ifndef UART_CONFIG_H
#define UART_CONFIG_H

/* CPU Frequency (Hz) — must match _XTAL_FREQ in STD_TYPES.h */
#define FOSC           20000000UL

/* Baud Rate */
#define UART_BAUDRATE  9600UL

/*
 * Speed Mode selection.
 *   1 = High Speed (BRGH=1, divisor 16)
 *   0 = Low  Speed (BRGH=0, divisor 64)
 *
 * High speed has smaller baud-rate error but limits the *minimum*
 * baud (SPBRG is 8-bit, max 255).
 *   At 20 MHz, BRGH=1: min baud ≈ 4882 → 4800 needs BRGH=0
 *   At 20 MHz, BRGH=0: handles 1200..76800 cleanly
 */
#if (UART_BAUDRATE >= 9600UL)
#  define UART_HIGH_SPEED  1
#  define UART_SPBRG_VALUE \
       ((unsigned char)((FOSC / (16UL * UART_BAUDRATE)) - 1UL))
#else
#  define UART_HIGH_SPEED  0
#  define UART_SPBRG_VALUE \
       ((unsigned char)((FOSC / (64UL * UART_BAUDRATE)) - 1UL))
#endif

#endif
