#ifndef GPIO_PRIVATE_H
#define GPIO_PRIVATE_H

/* Register Definitions for PIC16F877A
 * #ifndef guards let XC8's device headers or other driver SFR definitions
 * take precedence when they are already available.
 */

#ifndef TRISA
#define TRISA   (*((volatile u8*)0x85))
#endif
#ifndef TRISB
#define TRISB   (*((volatile u8*)0x86))
#endif
#ifndef TRISC
#define TRISC   (*((volatile u8*)0x87))
#endif
#ifndef TRISD
#define TRISD   (*((volatile u8*)0x88))
#endif
#ifndef TRISE
#define TRISE   (*((volatile u8*)0x89))
#endif

#ifndef PORTA
#define PORTA   (*((volatile u8*)0x05))
#endif
#ifndef PORTB
#define PORTB   (*((volatile u8*)0x06))
#endif
#ifndef PORTC
#define PORTC   (*((volatile u8*)0x07))
#endif
#ifndef PORTD
#define PORTD   (*((volatile u8*)0x08))
#endif
#ifndef PORTE
#define PORTE   (*((volatile u8*)0x09))
#endif

#endif
