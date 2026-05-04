#ifndef STD_TYPES_H
#define STD_TYPES_H

/*
 * Crystal frequency — required by XC8 __delay_ms() / __delay_us().
 * Catch the case where MPLAB X has a project-level -D_XTAL_FREQ=...
 * macro that would silently override this value (which would cause
 * UART baud rates and __delay_ms timing to be wrong).
 */
#ifdef _XTAL_FREQ
#  if (_XTAL_FREQ != 20000000UL)
#    error "_XTAL_FREQ is defined elsewhere with a different value. Remove the project-level macro in MPLAB X (Project Properties -> XC8 Compiler -> Preprocessing and messages -> Define macros)."
#  endif
#else
#  define _XTAL_FREQ  20000000UL
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