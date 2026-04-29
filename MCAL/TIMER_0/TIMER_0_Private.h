#ifndef TIMER_0_PRIVATE_H
#define TIMER_0_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/* PIC16F877A Timer0 registers */
#ifndef TMR0
#define TMR0        (*(volatile u8*)0x01)
#endif

/* OPTION_REG (bank 1: 0x81) — shared with EXT_INT */
#ifndef OPTION_REG
#define OPTION_REG  (*(volatile u8*)0x81)
#endif

/* INTCON (bank 0: 0x0B) — shared with EXT_INT, USART */
#ifndef INTCON
#define INTCON      (*(volatile u8*)0x0B)
#endif

/* ================= OPTION_REG bit positions ================= */
#define T0CS_BIT    5   /* Timer0 Clock Source: 0=internal Fosc/4 */
#define T0SE_BIT    4   /* Timer0 Source Edge */
#define PSA_BIT     3   /* Prescaler Assignment: 0=Timer0, 1=WDT */
/* Prescaler select bits: PS2(bit2), PS1(bit1), PS0(bit0) */
#define PS_MASK     0x07

/* ================= INTCON bit positions ================= */
#define T0IF_BIT    2   /* Timer0 Overflow Interrupt Flag */
#define T0IE_BIT    5   /* Timer0 Overflow Interrupt Enable */
#define GIE_BIT     7   /* Global Interrupt Enable */

/* ================= Prescaler values for PS2:PS0 ================= */
#define TIMER0_PS_1_2     0x00
#define TIMER0_PS_1_4     0x01
#define TIMER0_PS_1_8     0x02
#define TIMER0_PS_1_16    0x03
#define TIMER0_PS_1_32    0x04
#define TIMER0_PS_1_64    0x05
#define TIMER0_PS_1_128   0x06
#define TIMER0_PS_1_256   0x07

#endif