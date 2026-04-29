#ifndef I2C_PRIVATE_H
#define I2C_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/*
 * PIC16F877A I2C — uses the MSSP module (shared hardware with SPI)
 *
 * I2C pins (PORTC):
 *   RC3 — SCL (clock)
 *   RC4 — SDA (data)
 *
 * Both pins must be configured as inputs; the MSSP module drives them
 * open-drain through the hardware.
 */

/* MSSP registers (shared with SPI driver via #ifndef guards) */
#ifndef SSPBUF
#define SSPBUF      (*(volatile u8*)0x13)
#endif

#ifndef SSPCON
#define SSPCON      (*(volatile u8*)0x14)
#endif

#ifndef SSPCON2
#define SSPCON2     (*(volatile u8*)0x91)   /* SSP control register 2 (bank 1) */
#endif

#ifndef SSPSTAT
#define SSPSTAT     (*(volatile u8*)0x94)
#endif

#ifndef SSPADD
#define SSPADD      (*(volatile u8*)0x93)   /* I2C slave address / baud rate   */
#endif

/* PIR1 — shared */
#ifndef PIR1
#define PIR1        (*(volatile u8*)0x0C)
#endif

/* ================= SSPCON bit positions ================= */
#ifndef SSPEN_BIT
#define SSPEN_BIT   5
#endif
#define CKP_I2C_BIT 4   /* Release SCL clock (set to release after a hold) */

/*
 * SSPM bits 3:0 for I2C:
 *   1000 = I2C master mode (Fosc/(4*(SSPADD+1)))
 *   1110 = I2C slave, 7-bit address
 *   1111 = I2C slave, 10-bit address
 */
#define I2C_MASTER_MODE     0x08

/* ================= SSPCON2 bit positions ================= */
#define GCEN_BIT    7   /* General call enable (slave) */
#define ACKSTAT_BIT 6   /* ACK status from slave (master read) */
#define ACKDT_BIT   5   /* ACK data to send: 0=ACK, 1=NACK    */
#define ACKEN_BIT   4   /* Initiate ACK sequence               */
#define RCEN_BIT    3   /* Receive enable                      */
#define PEN_BIT     2   /* STOP condition enable               */
#define RSEN_BIT    1   /* Repeated START enable               */
#define SEN_BIT     0   /* START condition enable              */

/* ================= SSPSTAT bit positions ================= */
#ifndef BF_BIT
#define BF_BIT      0   /* Buffer full */
#endif

/* ================= PIR1 bit positions (I2C/SSP) ================= */
#ifndef SSPIF_BIT
#define SSPIF_BIT   3
#endif

#endif