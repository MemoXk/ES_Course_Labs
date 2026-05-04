#ifndef TIMER_1_PRIVATE_H
#define TIMER_1_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/* PIC16F877A Timer1 registers */
#ifndef TMR1L
#define TMR1L   (*(volatile u8*)0x0E)
#endif

#ifndef TMR1H
#define TMR1H   (*(volatile u8*)0x0F)
#endif

#ifndef T1CON
#define T1CON   (*(volatile u8*)0x10)
#endif

/* T1CON bit positions */
#define TMR1ON_BIT   0
#define TMR1CS_BIT   1
#define T1SYNC_BIT   2
#define T1OSCEN_BIT  3
#define T1CKPS0_BIT  4
#define T1CKPS1_BIT  5

#define TIMER1_PRESCALE_MASK  0x30u
#define TIMER1_CONTROL_MASK   0x3Fu

#endif
