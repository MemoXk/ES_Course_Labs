#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

/*
 * I2C Configuration for PIC16F877A @ 20 MHz
 *
 * I2C clock = Fosc / (4 * (SSPADD + 1))
 *
 * For 100 kHz (standard mode) @ 20 MHz:
 *   SSPADD = (20000000 / (4 * 100000)) - 1 = 50 - 1 = 49
 *
 * For 400 kHz (fast mode) @ 20 MHz:
 *   SSPADD = (20000000 / (4 * 400000)) - 1 = 12.5 - 1 ~= 11
 */

/* CPU frequency */
#define I2C_FOSC            20000000UL

/* I2C bus frequency in Hz */
#define I2C_FREQUENCY       100000UL

/* Computed SSPADD value */
#define I2C_SSPADD_VALUE    ((u8)((I2C_FOSC / (4UL * I2C_FREQUENCY)) - 1UL))

#endif
