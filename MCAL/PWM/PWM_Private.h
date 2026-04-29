#ifndef PWM_PRIVATE_H
#define PWM_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/*
 * PIC16F877A PWM using CCP1 module + Timer2
 *
 * CCP1 output pin : RC2 (PORTC pin 2, pin 17)
 * PWM period      : set via PR2 (Timer2 period register)
 * Duty cycle      : CCPR1L (upper 8 bits) + CCP1CON<5:4> (lower 2 bits)
 *
 * PWM frequency = Fosc / (4 * (PR2 + 1) * TMR2_prescaler)
 */

/* CCP1 registers */
#ifndef CCPR1L
#define CCPR1L      (*(volatile u8*)0x15)   /* Capture/Compare/PWM register 1 low */
#endif
#ifndef CCP1CON
#define CCP1CON     (*(volatile u8*)0x17)   /* CCP1 control register              */
#endif

/* Timer2 registers (shared between CCP1 and CCP2) */
#ifndef T2CON
#define T2CON       (*(volatile u8*)0x12)   /* Timer2 control register            */
#endif
#ifndef PR2
#define PR2         (*(volatile u8*)0x92)   /* Timer2 period register (bank 1)    */
#endif

/* PIR1 / PIE1 — shared with other drivers */
#ifndef PIR1
#define PIR1        (*(volatile u8*)0x0C)
#endif

#ifndef PIE1
#define PIE1        (*(volatile u8*)0x8C)
#endif

/* ================= CCP1CON bit positions ================= */
#define DC1B0_BIT   4   /* Duty cycle LSB-1 */
#define DC1B1_BIT   5   /* Duty cycle LSB-0 */

/* CCP1M bits 3:0 — PWM mode: 0b1100 */
#define CCP1_PWM_MODE   0x0C

/* ================= T2CON bit positions ================= */
#define TMR2ON_BIT  2   /* Timer2 on/off */

/*
 * T2CKPS (Timer2 prescaler): bits 1:0
 *   00 = 1:1
 *   01 = 1:4
 *   1x = 1:16
 */
#define T2CKPS_1    0x00
#define T2CKPS_4    0x01
#define T2CKPS_16   0x02

#endif
