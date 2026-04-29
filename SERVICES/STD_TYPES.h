#ifndef STD_TYPES_H
#define STD_TYPES_H

/* Crystal frequency — required by XC8 __delay_ms() / __delay_us() */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ  20000000UL
#endif

/* XC8: provides __delay_ms, __delay_us and all PIC SFR definitions */
#include <xc.h>

/* Signed Types */
typedef signed char        s8;
typedef signed short int   s16;
typedef signed long int    s32;

/* Unsigned Types */
typedef unsigned char        u8;
typedef unsigned short int   u16;
typedef unsigned long int    u32;

/* Floating Types */
typedef float       f32;
typedef double      f64;
typedef long double f128;

/* Standard Values */
#define NULL_PTR   ((void*)0)

#endif