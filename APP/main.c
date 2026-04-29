/*
 * Device  : PIC16F877A @ 20 MHz HS oscillator
 * Compiler: MPLAB X + XC8
 *
 * Active test: PWM breathing on RC2 (CCP1)
 *   LED fades in (0→100%) then out (100→0%), looping forever.
 *   Connect LED + 330Ω resistor between RC2 (pin 17) and GND.
 */

// CONFIG
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config LVP  = OFF

#include "pwm_test.h"

int main(void)
{
    PWM_Test();
    return 0;
}
